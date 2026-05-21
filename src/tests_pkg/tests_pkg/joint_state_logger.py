#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState


class JointStateLogger(Node):

    def __init__(self):

        super().__init__('dynamic_joint_state_logger') 

        self.sub = self.create_subscription(
            DynamicJointState,
            '/j100_0710/platform/dynamic_joint_states',
            self.callback,
            10
        )

        self.latest_msg = None

        #2 second timer
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info("Subscribed to /j100_0710/platform/dynamic_joint_states")

    def callback(self, msg):
        self.latest_msg = msg

    def timer_callback(self):

        if self.latest_msg is None:
            return

        msg = self.latest_msg

        positions = []
        velocities = []
        efforts = []

        arm_names = []

        for i, joint_name in enumerate(msg.joint_names):

            if not joint_name.startswith("arm_"):
                continue

            interfaces = msg.interface_values[i]
            data = dict(zip(interfaces.interface_names, interfaces.values))

            arm_names.append(joint_name)
            positions.append(data.get("position", 0.0))
            velocities.append(data.get("velocity", 0.0))
            efforts.append(data.get("effort", 0.0))

        self.get_logger().info("\n" + "="*80)

        self.get_logger().info(
            "ARM JOINTS: " + " | ".join(arm_names)
        )

        self.get_logger().info(
            "POSITIONS : " + str([round(p, 5) for p in positions])
        )

        self.get_logger().info(
            "VELOCITIES: " + str([round(v, 5) for v in velocities])
        )

        self.get_logger().info(
            "EFFORTS   : " + str([round(e, 5) for e in efforts])
        )

        self.get_logger().info("="*80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()