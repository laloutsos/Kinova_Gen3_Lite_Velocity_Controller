#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_jackal_interfaces.msg import JointsVelocity
from control_msgs.msg import DynamicJointState


from velocity_controller_pkg.compute_Jacobian import compute_jacobian

import numpy as np

class EEVelocityController(Node): 
    def __init__(self):
        super().__init__("ee_velocity_controller") 

        self.latest_q_angles = None
        self.initialized = False

        # ee velocity as a parameter 
        self.declare_parameter("ee_velocity", [0.0]*6)

        self.q_dot = None

        # Publisher
        self.pub = self.create_publisher(
            JointsVelocity,
            "/j100_0710/manipulator/joints_velocity",
            10
        )

        # Subsribers
        self.sub = self.create_subscription(
            DynamicJointState,
            '/j100_0710/platform/dynamic_joint_states',
            self.update_q_angles_callback,
            10
        )


        # Velocity Sending Frequency
        self.dt = 0.05

        # Calling the function that sends the final joints' velocities based on end effector's desired velocity
        self.timer = self.create_timer(self.dt, self.send_ee_velocity)


    def send_ee_velocity(self):

        if self.initialized == False: return

        q_angles = self.latest_q_angles

        J = compute_jacobian(q_angles)

        lambda_val = 0.1
        I = np.eye(6)

        J_pinv = (
            J.T @ np.linalg.inv(
                J @ J.T +
                (lambda_val ** 2) * I
            )
        )

        v_ee = self.get_parameter("ee_velocity").value

        self.q_dot = J_pinv @ v_ee

        

        ee_reconstructed = J @ self.q_dot


        msg = JointsVelocity()
        msg.joints_velocity = self.q_dot

        self.pub.publish(msg)

        self.get_logger().info(
            "\n"
            "================ IK LOGGER ================\n"
            f"EE desired      : {np.round(v_ee, 4)}\n"
            f"EE achieved     : {np.round(ee_reconstructed, 4)}\n"
            f"Joint vel qdot  : {np.round(self.q_dot, 4)}\n"
            f"Error norm      : {np.linalg.norm(v_ee - ee_reconstructed):.6f}\n"
            "========================================="
        )




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
        


def main(args=None):
    rclpy.init(args=args)
    node = EEVelocityController() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

