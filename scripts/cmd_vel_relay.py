#!/usr/bin/env python3
"""
cmd_vel relay — bridges /roverX/cmd_vel_nav → /roverX/cmd_vel
Reads 'rover_name' parameter to determine namespace.
Falls back to non-namespaced /cmd_vel_nav → /cmd_vel for backward compat.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('rover_name', '')
        rover_name = self.get_parameter('rover_name').get_parameter_value().string_value

        if rover_name:
            sub_topic = f'/{rover_name}/cmd_vel_nav'
            pub_topic = f'/{rover_name}/cmd_vel'
        else:
            sub_topic = '/cmd_vel_nav'
            pub_topic = '/cmd_vel'

        self.sub = self.create_subscription(
            Twist, sub_topic, self.callback, 10)
        self.pub = self.create_publisher(
            Twist, pub_topic, 10)
        self.get_logger().info(
            f'cmd_vel relay started: {sub_topic} → {pub_topic}')

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()