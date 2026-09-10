#!/usr/bin/env python3
"""
Multi-Robot TF Bridge

Solves the TF namespace split in multi-robot Nav2 setups:
  - EKF/RSP publish to /tf and /tf_static (global)
  - Nav2 in namespace mode reads from /roverX/tf (namespaced)
  - AMCL publishes map→roverX/odom on /roverX/tf (namespaced)
  - RViz reads from /tf (global)

This node bridges the gap WITHOUT creating infinite relay loops by using
frame-based filtering:
  /tf → /roverX/tf   : only forward transforms whose PARENT frame starts with "roverX/"
  /roverX/tf → /tf    : only forward transforms whose PARENT frame is "map"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


class MultiRobotTFBridge(Node):
    def __init__(self):
        super().__init__('multi_robot_tf_bridge')

        self.declare_parameter('rover_names', ['rover1', 'rover2'])
        self.rover_names = (
            self.get_parameter('rover_names')
            .get_parameter_value().string_array_value
        )

        # QoS for TF topics
        tf_qos = QoSProfile(depth=100)
        tf_static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Global /tf subscriber ──
        self.global_tf_sub = self.create_subscription(
            TFMessage, '/tf', self._global_tf_callback, tf_qos)

        # ── Global /tf_static subscriber ──
        self.global_tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._global_tf_static_callback,
            tf_static_qos)

        # Per-rover: namespaced subscribers + publishers
        self.rover_tf_pubs = {}       # publish TO /roverX/tf
        self.rover_static_pubs = {}   # publish TO /roverX/tf_static
        self.global_tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)

        for name in self.rover_names:
            # Publisher to namespaced TF (for Nav2 to read)
            self.rover_tf_pubs[name] = self.create_publisher(
                TFMessage, f'/{name}/tf', tf_qos)
            self.rover_static_pubs[name] = self.create_publisher(
                TFMessage, f'/{name}/tf_static', tf_static_qos)

            # Subscriber from namespaced TF (AMCL publishes here)
            self.create_subscription(
                TFMessage, f'/{name}/tf',
                lambda msg, n=name: self._namespaced_tf_callback(n, msg),
                tf_qos)

        self.get_logger().info(
            f'TF Bridge started for rovers: {self.rover_names}')

    def _global_tf_callback(self, msg):
        """
        /tf → /roverX/tf
        Only forward transforms whose PARENT frame starts with "roverX/".
        This sends EKF's odom→base_link to the namespaced TF that Nav2 reads.
        """
        for name in self.rover_names:
            filtered = TFMessage()
            prefix = f'{name}/'
            for t in msg.transforms:
                if t.header.frame_id.startswith(prefix):
                    filtered.transforms.append(t)
            if filtered.transforms:
                self.rover_tf_pubs[name].publish(filtered)

    def _global_tf_static_callback(self, msg):
        """
        /tf_static → /roverX/tf_static
        Forward static transforms (RSP URDF frames) to namespaced topic.
        """
        for name in self.rover_names:
            filtered = TFMessage()
            prefix = f'{name}/'
            for t in msg.transforms:
                if (t.header.frame_id.startswith(prefix) or
                        t.child_frame_id.startswith(prefix)):
                    filtered.transforms.append(t)
            if filtered.transforms:
                self.rover_static_pubs[name].publish(filtered)

    def _namespaced_tf_callback(self, rover_name, msg):
        """
        /roverX/tf → /tf
        Only forward transforms whose PARENT frame is "map".
        This sends AMCL's map→roverX/odom to global TF that RViz reads.

        Loop prevention: EKF publishes roverX/odom→roverX/base_link to /tf,
        which we relay to /roverX/tf. That message has parent "roverX/odom"
        (not "map"), so it does NOT get relayed back. No loop.
        """
        filtered = TFMessage()
        for t in msg.transforms:
            if t.header.frame_id == 'map':
                filtered.transforms.append(t)
        if filtered.transforms:
            self.global_tf_pub.publish(filtered)


def main():
    rclpy.init()
    node = MultiRobotTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
