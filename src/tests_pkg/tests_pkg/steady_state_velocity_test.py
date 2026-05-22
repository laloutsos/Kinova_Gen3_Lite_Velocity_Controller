#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState

import numpy as np
import time
import matplotlib.pyplot as plt


class SteadyStateVelocityTest(Node):

    def __init__(self):

        super().__init__('steady_state_velocity_test')

        # ---------------------------------
        # PARAMETERS
        # ---------------------------------
        self.declare_parameter("joint", "arm_0_joint_1")
        self.declare_parameter("cmd_velocity", 0.2)
        self.declare_parameter("duration", 10.0)

        self.joint_name = self.get_parameter("joint").value
        self.cmd_velocity = float(self.get_parameter("cmd_velocity").value)
        self.duration = float(self.get_parameter("duration").value)

        # ---------------------------------
        # STATE
        # ---------------------------------
        self.initial_position = None
        self.final_position = None

        self.start_time = None
        self.last_position = None
        self.last_time = None

        self.velocity_samples = []

        # raw + filtered data
        self.t_samples = []
        self.q_samples = []

        self.v_raw_samples = []
        self.v_filtered_samples = []

        # ---------------------------------
        # LOW PASS FILTER PARAMETER (EMA)
        # ---------------------------------
        self.alpha = 0.2   # 0.1-0.3 good range (smaller = smoother)

        self.v_filtered = None

        # ---------------------------------
        # SUBSCRIBER
        # ---------------------------------
        self.sub = self.create_subscription(
            DynamicJointState,
            "/j100_0710/platform/dynamic_joint_states",
            self.callback,
            10
        )

        # ---------------------------------
        # TIMER
        # ---------------------------------
        self.timer = self.create_timer(
            0.1,
            self.monitor_test
        )

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("STEADY STATE VELOCITY TEST STARTED")
        self.get_logger().info(f"Joint          : {self.joint_name}")
        self.get_logger().info(f"Cmd velocity   : {self.cmd_velocity} rad/s")
        self.get_logger().info(f"Duration       : {self.duration} sec")
        self.get_logger().info("=" * 80 + "\n")

    # ---------------------------------------------------------
    # CALLBACK
    # ---------------------------------------------------------
    def callback(self, msg):

        current_time = time.time()

        for i, name in enumerate(msg.joint_names):

            if name != self.joint_name:
                continue

            interfaces = msg.interface_values[i]
            data = dict(zip(interfaces.interface_names, interfaces.values))

            if "position" not in data:
                return

            q = float(data["position"])

            # store position
            self.q_samples.append(q)
            self.t_samples.append(current_time)

            # ---------------------------------
            # INITIALIZATION
            # ---------------------------------
            if self.initial_position is None:

                self.initial_position = q
                self.last_position = q

                self.start_time = current_time
                self.last_time = current_time

                self.get_logger().info(f"Initial position = {q:.5f} rad")

                return

            # ---------------------------------
            # VELOCITY ESTIMATION (RAW)
            # ---------------------------------
            dt = current_time - self.last_time

            if dt <= 0.0:
                return

            dq = q - self.last_position
            v_raw = dq / dt

            # ---------------------------------
            # LOW PASS FILTER (EMA)
            # ---------------------------------
            if self.v_filtered is None:
                self.v_filtered = v_raw
            else:
                self.v_filtered = (
                    self.alpha * v_raw +
                    (1.0 - self.alpha) * self.v_filtered
                )

            # store both
            self.v_raw_samples.append(v_raw)
            self.v_filtered_samples.append(self.v_filtered)

            self.velocity_samples.append(self.v_filtered)

            # update state
            self.last_position = q
            self.last_time = current_time
            self.final_position = q

    # ---------------------------------------------------------
    # PLOT
    # ---------------------------------------------------------
    def plot_results(self):

        if len(self.t_samples) < 2:
            self.get_logger().warn("Not enough samples for plotting.")
            return

        t0 = self.t_samples[0]

        t = np.array(self.t_samples) - t0
        q = np.array(self.q_samples)

        tv = t[1:]

        v_raw = np.array(self.v_raw_samples)
        v_filt = np.array(self.v_filtered_samples)

        # ---------------------------------
        # EXPECTED TRAJECTORIES
        # ---------------------------------
        expected_q = self.initial_position + self.cmd_velocity * t
        expected_v = np.ones_like(tv) * self.cmd_velocity

        plt.figure()

        # -----------------------
        # POSITION
        # -----------------------
        plt.subplot(2, 1, 1)
        plt.plot(t, q, label="real position")
        plt.plot(t, expected_q, '--', label="expected position")
        plt.title("Joint Position (Real vs Expected)")
        plt.xlabel("time [s]")
        plt.ylabel("rad")
        plt.grid(True)
        plt.legend()

        # -----------------------
        # VELOCITY
        # -----------------------
        plt.subplot(2, 1, 2)
        plt.plot(tv, v_raw, alpha=0.4, label="raw velocity")
        plt.plot(tv, v_filt, label="filtered velocity (EMA)")
        plt.plot(tv, expected_v, '--', label="expected velocity")
        plt.title("Joint Velocity (Raw vs Filtered vs Expected)")
        plt.xlabel("time [s]")
        plt.ylabel("rad/s")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

    # ---------------------------------------------------------
    # MONITOR
    # ---------------------------------------------------------
    def monitor_test(self):

        if self.start_time is None:
            return

        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            return

        total_motion = self.final_position - self.initial_position
        avg_velocity = total_motion / elapsed
        expected_motion = self.cmd_velocity * elapsed

        position_error = total_motion - expected_motion
        velocity_error = avg_velocity - self.cmd_velocity

        rms_velocity_error = np.sqrt(
            np.mean(
                (np.array(self.velocity_samples) - self.cmd_velocity) ** 2
            )
        )

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("STEADY STATE TEST RESULTS")
        self.get_logger().info("=" * 80)

        self.get_logger().info(f"Elapsed time          : {elapsed:.4f} sec")
        self.get_logger().info(f"Initial position      : {self.initial_position:.5f} rad")
        self.get_logger().info(f"Final position        : {self.final_position:.5f} rad")
        self.get_logger().info(f"Total motion          : {total_motion:.5f} rad")
        self.get_logger().info(f"Expected motion       : {expected_motion:.5f} rad")
        self.get_logger().info(f"Position error        : {position_error:.5f} rad")
        self.get_logger().info(f"Avg velocity          : {avg_velocity:.5f} rad/s")
        self.get_logger().info(f"Cmd velocity          : {self.cmd_velocity:.5f} rad/s")
        self.get_logger().info(f"Velocity error        : {velocity_error:.5f} rad/s")
        self.get_logger().info(f"RMS velocity error    : {rms_velocity_error:.5f}")

        self.get_logger().info("=" * 80 + "\n")

        self.plot_results()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = SteadyStateVelocityTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()