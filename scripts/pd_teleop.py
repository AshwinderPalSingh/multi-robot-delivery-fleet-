#!/usr/bin/env python3
"""
Custom PD Teleop Controller for Hospital Delivery Robot
Reads keyboard input and applies PD control to smooth velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time

# ── KEY BINDINGS ─────────────────────────────────────────────
KEY_BINDINGS = {
    'w': ( 1.0,  0.0),   # forward
    's': (-1.0,  0.0),   # backward
    'a': ( 0.0,  1.0),   # turn left
    'd': ( 0.0, -1.0),   # turn right
    'q': ( 1.0,  1.0),   # forward + left
    'e': ( 1.0, -1.0),   # forward + right
    'z': (-1.0,  1.0),   # backward + left
    'c': (-1.0, -1.0),   # backward + right
}

SPEED_STEP  = 0.05   # increase/decrease speed by this amount
TURN_STEP   = 0.1

MAX_LINEAR  = 0.60   # m/s
MAX_ANGULAR = 0.40   # rad/s
MIN_SPEED   = 0.05   # below this treat as zero

HELP_MSG = """

  w     : forward                        
  s     : backward                       
  a     : turn left                    
  d     : turn right                   
 q/e   : diagonal forward              
 z/c   : diagonal backward             
  SPACE : emergency STOP               
  +/=   : increase max speed           
 -     : decrease max speed            
 p     : print current PD gains        
  x     : quit                         
 Control smooths acceleration/deceleration.
"""


class PDController:
    """
    PD Controller for a single velocity axis.
    Smoothly drives current velocity toward target velocity.
    
    P gain: how aggressively to chase the target
    D gain: how much to dampen oscillation / overshoot
    """
    def __init__(self, kp=3.0, kd=0.5, dt=0.05):
        self.kp = kp          # Proportional gain
        self.kd = kd          # Derivative gain
        self.dt = dt          # Time step (seconds)
        self.current = 0.0    # Current velocity
        self.prev_error = 0.0 # Previous error for derivative term

    def update(self, target):
        error      = target - self.current
        derivative = (error - self.prev_error) / self.dt

        # PD output = P*error + D*derivative
        output = self.kp * error + self.kd * derivative

        # Apply output to current velocity
        self.current += output * self.dt

        # Clamp to reasonable range
        self.current = max(-MAX_LINEAR, min(MAX_LINEAR, self.current))

        self.prev_error = error
        return self.current

    def reset(self):
        self.current    = 0.0
        self.prev_error = 0.0


class PDTeleop(Node):

    def __init__(self):
        super().__init__('pd_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
        self.pd_linear  = PDController(kp=4.0, kd=0.8, dt=0.05)
        self.pd_angular = PDController(kp=4.0, kd=0.8, dt=0.05)
        self.target_linear  = 0.0
        self.target_angular = 0.0
        self.speed_scale = 1.0
        self.turn_scale  = 1.0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.key_pressed = None
        self.running     = True

        self.get_logger().info('PD Teleop Controller started!')
        print(HELP_MSG)
        print(f'PD Gains → Linear:  Kp={self.pd_linear.kp}  Kd={self.pd_linear.kd}')
        print(f'PD Gains → Angular: Kp={self.pd_angular.kp} Kd={self.pd_angular.kd}')
        print(f'Max speed: {MAX_LINEAR} m/s | Max turn: {MAX_ANGULAR} rad/s\n')

    def get_key(self):
        """Read a single keypress without blocking."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def keyboard_loop(self):
        """Runs in separate thread — reads keys continuously."""
        while self.running:
            key = self.get_key()

            # QUIT
            if key == 'x':
                self.running = False
                print('\nStopping robot and exiting...')
                break

            # EMERGENCY STOP
            elif key == ' ':
                self.target_linear  = 0.0
                self.target_angular = 0.0
                self.pd_linear.reset()
                self.pd_angular.reset()
                print('\r[STOP]                    ', end='', flush=True)

            # SPEED ADJUST
            elif key in ('+', '='):
                self.speed_scale = min(self.speed_scale + 0.1, 2.0)
                print(f'\rSpeed scale: {self.speed_scale:.1f}x    ', end='', flush=True)

            elif key == '-':
                self.speed_scale = max(self.speed_scale - 0.1, 0.1)
                print(f'\rSpeed scale: {self.speed_scale:.1f}x    ', end='', flush=True)

            # PRINT GAINS
            elif key == 'p':
                print(f'\rLinear  PD: Kp={self.pd_linear.kp}  Kd={self.pd_linear.kd}  current={self.pd_linear.current:.3f}')
                print(f'Angular PD: Kp={self.pd_angular.kp} Kd={self.pd_angular.kd}  current={self.pd_angular.current:.3f}')

            # MOVEMENT KEYS
            elif key in KEY_BINDINGS:
                lin_dir, ang_dir = KEY_BINDINGS[key]
                self.target_linear  = lin_dir * MAX_LINEAR  * self.speed_scale
                self.target_angular = ang_dir * MAX_ANGULAR * self.turn_scale

            # Any other key — stop
            else:
                self.target_linear  = 0.0
                self.target_angular = 0.0

    def control_loop(self):
        """
        Runs at 20Hz via ROS timer.
        Applies PD control and publishes smooth velocity.
        """
        # Get PD-smoothed velocities
        smooth_linear  = self.pd_linear.update(self.target_linear)
        smooth_angular = self.pd_angular.update(self.target_angular)

        # Clamp final output
        smooth_linear  = max(-MAX_LINEAR,  min(MAX_LINEAR,  smooth_linear))
        smooth_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, smooth_angular))

        # Zero out very small values
        if abs(smooth_linear)  < 0.001: smooth_linear  = 0.0
        if abs(smooth_angular) < 0.001: smooth_angular = 0.0

        # Publish
        msg = Twist()
        msg.linear.x  = smooth_linear
        msg.angular.z = smooth_angular
        self.pub.publish(msg)

        # Print status
        print(
            f'\rTarget: lin={self.target_linear:+.2f} ang={self.target_angular:+.2f} | '
            f'Actual: lin={smooth_linear:+.2f} ang={smooth_angular:+.2f}    ',
            end='', flush=True
        )

        # Stop if node is shutting down
        if not self.running:
            stop = Twist()
            self.pub.publish(stop)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PDTeleop()

    # Start keyboard reading in a separate thread
    kb_thread = threading.Thread(target=node.keyboard_loop, daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure robot stops
        stop = Twist()
        node.pub.publish(stop)
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        print('\nController stopped.')


if __name__ == '__main__':
    main()
