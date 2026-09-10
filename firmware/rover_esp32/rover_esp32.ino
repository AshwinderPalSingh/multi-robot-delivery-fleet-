/* ═══════════════════════════════════════════════════════════════════════════
 * GForce rover node — ESP32 firmware
 *
 * This is the ENTIRE intelligence of a rover, and that is the point. It holds
 * no map, no planner, no costmap, no goal and no notion that other rovers
 * exist. It closes exactly one loop — wheel velocity — and reports what its
 * sensors read. Every decision about where this rover goes is made on the
 * central brain and arrives here as two numbers.
 *
 * Consequences worth stating plainly:
 *   - A rover cannot be "wrong" about the fleet, because it holds no fleet
 *     state to be wrong about.
 *   - Retuning fleet behaviour never means reflashing four rovers.
 *   - A rover that loses the link stops. It does not improvise, because it
 *     has nothing to improvise with.
 *
 * Counterpart: scripts/rover_link.py with mode:=hardware.
 * Protocol and wiring: docs/HARDWARE.md
 *
 * Board: ESP32-WROOM-32   Arduino core 2.x
 * Driver: BTS7960 half-bridge pair per wheel (or any PWM + DIR driver)
 * ═════════════════════════════════════════════════════════════════════════*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ── identity ───────────────────────────────────────────────────────────────
// The only rover-specific constant in the firmware. Everything else that
// distinguishes rover1 from rover4 lives in fleet_brain.yaml on the brain.
#define ROVER_INDEX      1
static const char* WIFI_SSID = "gforce-fleet";
static const char* WIFI_PASS = "changeme";
static const IPAddress BRAIN_IP(192, 168, 4, 1);
static const uint16_t  BRAIN_PORT  = 9100 + ROVER_INDEX;  // rover_link listen_port
static const uint16_t  LOCAL_PORT  = 9001;      // matches esp_endpoint

// ── platform geometry (must match urdf/robot.urdf.xacro) ───────────────────
static const float WHEEL_R   = 0.150f;   // m
static const float TRACK_B   = 0.720f;   // m between wheel contact points
static const float TICKS_REV = 1200.0f;  // quadrature counts per wheel rev
static const float V_MAX     = 0.50f;    // m/s   — Nav2 is capped here too
static const float W_MAX     = 1.50f;    // rad/s

// ── pins ───────────────────────────────────────────────────────────────────
static const int PIN_L_RPWM = 25, PIN_L_LPWM = 26, PIN_L_EN = 27;
static const int PIN_R_RPWM = 32, PIN_R_LPWM = 33, PIN_R_EN = 14;
static const int PIN_L_ENC_A = 34, PIN_L_ENC_B = 35;
static const int PIN_R_ENC_A = 36, PIN_R_ENC_B = 39;
static const int PIN_VBAT    = 4;        // divider: 100k / 15k -> 25.2 V = 3.29 V
static const int PIN_ESTOP   = 13;       // NC mushroom button to GND

// ── timing ─────────────────────────────────────────────────────────────────
static const uint32_t CTRL_HZ      = 100;
static const uint32_t TELEM_HZ     = 50;
// If the brain goes quiet the wheels stop. rover_link enforces the same rule
// from its side; both halves hold it so neither can mask the other's failure.
static const uint32_t LINK_TIMEOUT_MS = 300;

// ── wire format (byte-identical to rover_link.py) ──────────────────────────
static const uint8_t DOWN_MAGIC = 0xA5;
static const uint8_t UP_MAGIC   = 0x5A;

#pragma pack(push, 1)
struct DownFrame {           // 8 bytes
  uint8_t  magic;
  uint8_t  seq;
  int16_t  v_mm_s;
  int16_t  w_mrad_s;
  uint8_t  mode;             // 0 stop, 1 velocity, 2 dock, 3 estop
  uint8_t  crc;
};
struct UpFrame {             // 16 bytes
  uint8_t  magic;
  uint8_t  seq;
  int32_t  ticks_l;
  int32_t  ticks_r;
  uint16_t vbat_mv;
  int16_t  gyro_z_mrad;
  uint8_t  status;           // bit0 estop, bit1 stall, bit2 undervolt
  uint8_t  crc;
};
#pragma pack(pop)

// ── state ──────────────────────────────────────────────────────────────────
WiFiUDP udp;
volatile int32_t tickL = 0, tickR = 0;
float targetVL = 0, targetVR = 0;        // m/s per wheel
float iL = 0, iR = 0, prevErrL = 0, prevErrR = 0;
int32_t lastTickL = 0, lastTickR = 0;
uint32_t lastRxMs = 0, lastCtrlUs = 0, lastTelemMs = 0;
uint8_t  txSeq = 0;
bool     estop = false, stall = false, undervolt = false;

// PID on wheel velocity. Gains are per-platform: drive one wheel open loop,
// log the step response, then tune. These are a sane starting point for a
// geared 24 V motor on a heavy chassis.
static const float KP = 220.0f, KI = 900.0f, KD = 2.0f;
static const int   PWM_MAX = 1023;

// ── CRC-8 Dallas/Maxim, poly 0x31 ──────────────────────────────────────────
static uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
  }
  return c;
}

// ── encoders ───────────────────────────────────────────────────────────────
void IRAM_ATTR isrL() { tickL += digitalRead(PIN_L_ENC_B) ? 1 : -1; }
void IRAM_ATTR isrR() { tickR += digitalRead(PIN_R_ENC_B) ? -1 : 1; }

// ── motor output ───────────────────────────────────────────────────────────
static void drive(int ch_r, int ch_l, float u) {
  int pwm = (int)constrain(fabsf(u), 0.0f, (float)PWM_MAX);
  if (fabsf(u) < 8.0f) pwm = 0;                 // deadband, stops motor whine
  ledcWrite(ch_r, u >= 0 ? pwm : 0);
  ledcWrite(ch_l, u <  0 ? pwm : 0);
}

static void motorsOff() {
  ledcWrite(0, 0); ledcWrite(1, 0); ledcWrite(2, 0); ledcWrite(3, 0);
  iL = iR = 0;                                   // never wind up while stopped
}

// ── setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);
  pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_L_ENC_A, INPUT); pinMode(PIN_L_ENC_B, INPUT);
  pinMode(PIN_R_ENC_A, INPUT); pinMode(PIN_R_ENC_B, INPUT);

  for (int ch = 0; ch < 4; ch++) ledcSetup(ch, 20000, 10);   // 20 kHz, 10-bit
  ledcAttachPin(PIN_L_RPWM, 0); ledcAttachPin(PIN_L_LPWM, 1);
  ledcAttachPin(PIN_R_RPWM, 2); ledcAttachPin(PIN_R_LPWM, 3);
  motorsOff();

  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), isrR, RISING);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // sleep adds tens of ms of jitter to control
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print('.'); }
  udp.begin(LOCAL_PORT);
  Serial.printf("\nrover%d up at %s\n", ROVER_INDEX, WiFi.localIP().toString().c_str());

  lastCtrlUs = micros();
  lastRxMs = millis();
}

// ── downlink ───────────────────────────────────────────────────────────────
static void readLink() {
  uint8_t buf[32];
  int n;
  // Drain the socket: only the newest command matters. An old velocity that
  // queued behind a Wi-Fi retry is worse than no command at all.
  while ((n = udp.parsePacket()) > 0) {
    int got = udp.read(buf, sizeof(buf));
    if (got != (int)sizeof(DownFrame)) continue;
    DownFrame f; memcpy(&f, buf, sizeof(f));
    if (f.magic != DOWN_MAGIC) continue;
    if (crc8((uint8_t*)&f, sizeof(f) - 1) != f.crc) continue;

    lastRxMs = millis();
    if (f.mode == 0 || f.mode == 3) {            // STOP / ESTOP
      targetVL = targetVR = 0;
      if (f.mode == 3) estop = true;
      continue;
    }
    float v = constrain(f.v_mm_s   / 1000.0f, -V_MAX, V_MAX);
    float w = constrain(f.w_mrad_s / 1000.0f, -W_MAX, W_MAX);
    // Differential drive inverse kinematics — the only "planning" on board.
    targetVL = v - w * TRACK_B * 0.5f;
    targetVR = v + w * TRACK_B * 0.5f;
  }
}

// ── control ────────────────────────────────────────────────────────────────
static void controlStep(float dt) {
  noInterrupts();
  int32_t tl = tickL, tr = tickR;
  interrupts();

  float mL = (tl - lastTickL) / TICKS_REV * TWO_PI * WHEEL_R / dt;
  float mR = (tr - lastTickR) / TICKS_REV * TWO_PI * WHEEL_R / dt;
  lastTickL = tl; lastTickR = tr;

  bool linkDead = (millis() - lastRxMs) > LINK_TIMEOUT_MS;
  estop = estop || (digitalRead(PIN_ESTOP) == LOW);

  if (estop || linkDead) { motorsOff(); return; }

  float eL = targetVL - mL, eR = targetVR - mR;
  iL = constrain(iL + eL * dt, -2.0f, 2.0f);     // clamped, not unbounded
  iR = constrain(iR + eR * dt, -2.0f, 2.0f);
  float uL = KP * eL + KI * iL + KD * (eL - prevErrL) / dt;
  float uR = KP * eR + KI * iR + KD * (eR - prevErrR) / dt;
  prevErrL = eL; prevErrR = eR;

  drive(0, 1, uL);
  drive(2, 3, uR);

  // Commanded hard, moving barely: something is jammed. Report it and let the
  // brain decide — factor F5 will quietly stop trusting this rover with the
  // difficult jobs long before a person notices.
  stall = (fabsf(targetVL) > 0.10f && fabsf(mL) < 0.02f) ||
          (fabsf(targetVR) > 0.10f && fabsf(mR) < 0.02f);
}

// ── uplink ─────────────────────────────────────────────────────────────────
static void sendTelemetry() {
  // 100k/15k divider: Vpack * 15/115 = Vpack * 0.1304, so a full 6S pack at
  // 25.2 V reads 3.29 V, just inside the 3.3 V ADC range. Inverse is 7.667.
  uint16_t mv = (uint16_t)(analogReadMilliVolts(PIN_VBAT) * 7.667f);
  undervolt = (mv < 19800);

  UpFrame f;
  f.magic = UP_MAGIC;
  f.seq = txSeq++;
  noInterrupts();
  f.ticks_l = tickL; f.ticks_r = tickR;
  interrupts();
  f.vbat_mv = mv;
  f.gyro_z_mrad = 0;                 // populate if an MPU6050 is fitted
  f.status = (estop ? 0x01 : 0) | (stall ? 0x02 : 0) | (undervolt ? 0x04 : 0);
  f.crc = crc8((uint8_t*)&f, sizeof(f) - 1);

  udp.beginPacket(BRAIN_IP, BRAIN_PORT);
  udp.write((uint8_t*)&f, sizeof(f));
  udp.endPacket();
}

// ── loop ───────────────────────────────────────────────────────────────────
void loop() {
  readLink();

  uint32_t nowUs = micros();
  if (nowUs - lastCtrlUs >= 1000000UL / CTRL_HZ) {
    float dt = (nowUs - lastCtrlUs) * 1e-6f;
    lastCtrlUs = nowUs;
    controlStep(dt);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastTelemMs >= 1000UL / TELEM_HZ) {
    lastTelemMs = nowMs;
    sendTelemetry();
  }

  // A dropped association leaves the rover deaf; stop rather than coast.
  if (WiFi.status() != WL_CONNECTED) {
    motorsOff();
    WiFi.reconnect();
    delay(50);
  }
}
