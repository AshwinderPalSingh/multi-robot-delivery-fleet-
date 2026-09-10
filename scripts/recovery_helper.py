#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class RecoveryHelper(Node):
    def __init__(self):
        super().__init__('recovery_helper')
        self.local_clear = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')
        self.global_clear = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap')
        self.get_logger().info('Recovery helper ready')

    def clear_costmaps(self):
        self.get_logger().info('Clearing costmaps...')
        if self.local_clear.wait_for_service(timeout_sec=1.0):
            self.local_clear.call_async(ClearEntireCostmap.Request())
        if self.global_clear.wait_for_service(timeout_sec=1.0):
            self.global_clear.call_async(ClearEntireCostmap.Request())
        self.get_logger().info('Costmaps cleared')

def main():
    rclpy.init()
    node = RecoveryHelper()
    node.clear_costmaps()
    rclpy.shutdown()

if __name__ == '__main__':
    main()