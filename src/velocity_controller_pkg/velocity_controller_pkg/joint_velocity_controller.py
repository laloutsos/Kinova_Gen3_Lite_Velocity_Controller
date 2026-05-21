#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import DynamicJointState

import numpy as np


class JointVelocityController(Node):

    def __init__(self):

        # constructor
        super().__init__('joint_velocity_controller')

        # cmd_velocities
        self.declare_parameter("v1", 0.0)
        self.declare_parameter("v2", 0.0)
        self.declare_parameter("v3", 0.0)
        self.declare_parameter("v4", 0.0)
        self.declare_parameter("v5", 0.0)
        self.declare_parameter("v6", 0.0)


        # manipulator state
        self.q_cmd = np.zeros(6)
        self.dt = 0.05
        self.initial_q = None
        self.initialized = False 


        # joint names
        self.joint_names = [
            "arm_0_joint_1",
            "arm_0_joint_2",
            "arm_0_joint_3",
            "arm_0_joint_4",
            "arm_0_joint_5",
            "arm_0_joint_6"
        ]

        # publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            "/j100_0710/manipulators/arm_0_joint_trajectory_controller/joint_trajectory",
            10
        )

        # subscriber
        self.create_subscription(
            DynamicJointState,
            "/j100_0710/platform/dynamic_joint_states",
            self.initialize_q_cmd,
            10
        )

        # Frequency of calling the function that performs fake velocity
        self.timer = self.create_timer(self.dt, self.send_velocity)

        self.get_logger().info("FakeVelocityController RUNNING")

    # Callback: Initial position/angle of each joint
    def initialize_q_cmd(self, msg):
        self.initial_q = msg

        data_map = {}

        for i, joint_name in enumerate(msg.joint_names):
            interfaces = msg.interface_values[i]
            data = dict(zip(interfaces.interface_names, interfaces.values))

            if "position" in data:
                data_map[joint_name] = data["position"]

        self.q_cmd = np.array([
            data_map.get("arm_0_joint_1", -1000.0),
            data_map.get("arm_0_joint_2", -1000.0),
            data_map.get("arm_0_joint_3", -1000.0),
            data_map.get("arm_0_joint_4", -1000.0),
            data_map.get("arm_0_joint_5", -1000.0),
            data_map.get("arm_0_joint_6", -1000.0),
        ])
        
        valid = np.all(self.q_cmd != -1000.0)

        self.initialized = bool(valid)


    # loop
    def send_velocity(self):
        
        # We must send trajectory commands if we don't know the initial position of each joint.
        if self.initialized == True:

            v = np.array([
                self.get_parameter("v1").value,
                self.get_parameter("v2").value,
                self.get_parameter("v3").value,
                self.get_parameter("v4").value,
                self.get_parameter("v5").value,
                self.get_parameter("v6").value,
            ])

            # This is for safety, if you want you can commented it out.
            v = np.clip(v, -1.0, 1.0)

            # Through experiments the best velocity gain I found was 10.0
            velocity_gain = 10.0

            self.q_cmd = self.q_cmd + v * self.dt * velocity_gain

            msg = JointTrajectory()
            msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = self.q_cmd.tolist()

             # These are the best values of the parameters for Joint Trajectories I found. 
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 400000000

            msg.points = [point]

            self.pub.publish(msg)

            self.get_logger().info(
                f"\n"
                f"CMD v : {np.round(v, 3)}\n"
                f"q_cmd : {np.round(self.q_cmd, 3)}"
            )


def main():
    rclpy.init()
    node = JointVelocityController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

