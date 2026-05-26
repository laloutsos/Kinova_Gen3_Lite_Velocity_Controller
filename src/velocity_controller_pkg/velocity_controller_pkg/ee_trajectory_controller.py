#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from control_msgs.msg import DynamicJointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from velocity_controller_pkg.compute_Jacobian import fk_func, compute_jacobian


class EETrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__("trajectory_controller")

        self.initialized = False
        self.latest_q_angles = None

        # Subsribers
        self.sub = self.create_subscription(
            DynamicJointState,
            '/j100_0710/platform/dynamic_joint_states',
            self.update_q_angles_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

    def update_q_angles_callback(self, msg):

        data_map = {}

        for i, joint_name in enumerate(msg.joint_names):
            interfaces = msg.interface_values[i]
            data = dict(zip(interfaces.interface_names, interfaces.values))

            if "position" in data:
                data_map[joint_name] = data["position"]

        self.latest_q_angles = np.array([
            data_map.get("arm_0_joint_1", -1000.0),
            data_map.get("arm_0_joint_2", -1000.0),
            data_map.get("arm_0_joint_3", -1000.0),
            data_map.get("arm_0_joint_4", -1000.0),
            data_map.get("arm_0_joint_5", -1000.0),
            data_map.get("arm_0_joint_6", -1000.0),
        ])

        valid = np.all(self.latest_q_angles != -1000.0)
        self.initialized = bool(valid)

    def get_ee_pose(self):

        if self.latest_q_angles is None:
            return None, None

        q = self.latest_q_angles
        pos, R = fk_func(*q)

        return np.array(pos), np.array(R)

    def timer_callback(self):

        if not self.initialized:
            self.get_logger().warn("Waiting for valid joint states...")
            return

        pos, R = self.get_ee_pose()

        if pos is None:
            return

        self.get_logger().info(
            f"EE Position: {pos.tolist()}\n"
            f"EE Orientation:\n{R}"
        )

    def inverse_kinematics(target_pos, target_R, q_init, max_iters=100, alpha=0.5):
        q = q_init.copy()

        for _ in range(max_iters):

            pos, R = fk_func(*q)

            # position error
            dp = target_pos - pos

            # orientation error (rotation matrix -> axis-angle approx)
            R_err = target_R @ R.T
            dR = np.array([
                R_err[2,1] - R_err[1,2],
                R_err[0,2] - R_err[2,0],
                R_err[1,0] - R_err[0,1]
            ]) * 0.5

            error = np.hstack((dp, dR))

            if np.linalg.norm(error) < 1e-3:
                break

            J = compute_jacobian(q)

            dq = alpha * np.linalg.pinv(J) @ error
            q += dq

        return q
    

    def send_joint_trajectory(node, q_target):

        msg = JointTrajectory()
        msg.joint_names = [
            "arm_0_joint_1",
            "arm_0_joint_2",
            "arm_0_joint_3",
            "arm_0_joint_4",
            "arm_0_joint_5",
            "arm_0_joint_6",
        ]

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()

        point.time_from_start.sec = 3

        msg.points = [point]

        pub = node.create_publisher(
            JointTrajectory,
            "/j100_0710/manipulators/arm_0_joint_trajectory_controller/joint_trajectory",
            10
        )

        pub.publish(msg)

        node.get_logger().info("Trajectory sent once.")


        

    


def main(args=None):
    rclpy.init(args=args)
    node = EETrajectoryControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()