#!/usr/bin/env python3
"""
task_console — how work gets into the fleet.

Submits FleetTask messages to the central brain. It makes no decisions of its
own: it never picks a rover, never touches Nav2, and has no idea how many
rovers exist. On hardware this is the ward tablet or the nurses'-station
terminal, and it talks to the Raspberry Pi over Wi-Fi exactly as it does here.

Three ways to submit:

  RViz          Drop a "2D Goal Pose". If it lands within snap_radius of a
                known station the task is labelled with that station name;
                otherwise it goes through as a raw pose. Priority comes from
                the default_priority parameter.

  Command line  ros2 topic pub --once /fleet/task \
                    hospital_robot_description/msg/FleetTask \
                    '{station: pharmacy, priority: 2, payload_kg: 3.0}'

  Demo mode     demo:=true submits a scripted round of deliveries that
                deliberately exercises the factors the policy exists for —
                see DEMO below.
"""

import math
import uuid

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from hospital_robot_description.msg import FleetTask

# A run designed to make the policy show its work rather than to look tidy:
#
#   1-2  two ward calls at opposite ends, so the fleet spreads out
#   3    a pharmacy run issued while the west rovers are committed — F7 keeps
#        someone covering the west wing, F3 keeps the two of them out of the
#        same corridor
#   4    an EMERGENCY to the surgical suite, which is allowed to preempt any
#        routine task still running
#   5-6  long hauls across the full 33 m arena, where F4 starts vetoing the
#        rovers with older packs
#
#  (delay from start, station, priority, payload kg)
DEMO = [
    (5.0,   'ward_a_bed',     FleetTask.PRIORITY_NORMAL,    2.0),
    (10.0,  'ward_b_bed',     FleetTask.PRIORITY_NORMAL,    2.0),
    (22.0,  'pharmacy',       FleetTask.PRIORITY_NORMAL,    4.0),
    (40.0,  'surgical_prep',  FleetTask.PRIORITY_EMERGENCY, 1.0),
    (70.0,  'supply_room',    FleetTask.PRIORITY_ROUTINE,   8.0),
    (95.0,  'east_hall',      FleetTask.PRIORITY_NORMAL,    3.0),
]


class TaskConsole(Node):

    def __init__(self):
        super().__init__('task_console')
        self.declare_parameter('snap_radius', 2.0)
        self.declare_parameter('default_priority', int(FleetTask.PRIORITY_NORMAL))
        self.declare_parameter('default_payload_kg', 2.0)
        self.declare_parameter('demo', False)
        self.declare_parameter('station_names', [''])
        self.declare_parameter('station_data', [0.0])

        g = lambda n: self.get_parameter(n).value
        names = list(g('station_names'))
        flat = list(g('station_data'))
        self.stations = {n: tuple(flat[i * 4:i * 4 + 4])
                         for i, n in enumerate(names) if n}
        self.snap = float(g('snap_radius'))
        self.prio = int(g('default_priority'))
        self.payload = float(g('default_payload_kg'))

        self.pub = self.create_publisher(FleetTask, '/fleet/task', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

        self.get_logger().info(
            f'task console ready — {len(self.stations)} stations, '
            f'RViz goals snap within {self.snap:.1f} m')

        if bool(g('demo')):
            self.get_logger().info(f'demo mode: {len(DEMO)} scripted tasks queued')
            for delay, station, prio, kg in DEMO:
                self.create_timer(
                    delay,
                    self._once(lambda s=station, p=prio, k=kg:
                               self._submit(station=s, priority=p, payload=k)))

    def _once(self, fn):
        """One-shot wrapper: ROS timers repeat, these tasks must not."""
        fired = {'v': False}

        def run():
            if fired['v']:
                return
            fired['v'] = True
            fn()
        return run

    def _nearest(self, x, y):
        best, bd = '', 1e9
        for n, (sx, sy, _yaw, _dem) in self.stations.items():
            d = math.hypot(sx - x, sy - y)
            if d < bd:
                best, bd = n, d
        return (best, bd) if bd <= self.snap else ('', bd)

    def _on_goal(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        station, d = self._nearest(x, y)
        if station:
            self.get_logger().info(f'RViz goal snapped to "{station}" ({d:.2f} m)')
            self._submit(station=station)
        else:
            self.get_logger().info(f'RViz goal at ({x:.2f}, {y:.2f}) — raw pose')
            self._submit(x=x, y=y, yaw=yaw)

    def _submit(self, station='', x=0.0, y=0.0, yaw=0.0, priority=None, payload=None):
        t = FleetTask()
        t.task_id = f't{uuid.uuid4().hex[:6]}'
        t.station = station
        if station and station in self.stations:
            sx, sy, syaw, _ = self.stations[station]
            t.goal_x, t.goal_y, t.goal_yaw = sx, sy, syaw
        else:
            t.goal_x, t.goal_y, t.goal_yaw = x, y, yaw
        t.priority = int(self.prio if priority is None else priority)
        t.payload_kg = float(self.payload if payload is None else payload)
        t.stamp = self.get_clock().now().to_msg()
        self.pub.publish(t)
        self.get_logger().info(
            f'submitted [{t.task_id}] -> {station or f"({t.goal_x:.1f},{t.goal_y:.1f})"} '
            f'priority={t.priority} payload={t.payload_kg:.1f} kg')


def main():
    rclpy.init()
    node = TaskConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
