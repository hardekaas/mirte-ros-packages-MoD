#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os
import sys
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from mirte_msgs.msg import Encoder
from sensor_msgs.msg import Imu


class BEPTestTopics(Node):

    def __init__(self, car_number: int):
        super().__init__('bep_test_topics_' + str(car_number))

        self.test_samples = 150

        # -------------------------
        # ENCODER
        # -------------------------
        self.last_encoder_stamp = None
        self.encoder_intervals = []
        self.encoder_latencies = []

        # -------------------------
        # IMU
        # -------------------------
        self.last_imu_stamp = None
        self.imu_intervals = []
        self.imu_latencies = []

        # -------------------------
        # ODOM
        # -------------------------
        self.last_odom_stamp = None
        self.odom_intervals = []
        self.odom_latencies = []

        # -------------------------
        # SUBSCRIPTIONS
        # -------------------------
        self.create_subscription(
            Encoder,
            '/io/encoder/main',
            self.callback_encoder,
            10
        )

        self.create_subscription(
            Imu,
            '/io/imu/movement/data',
            self.callback_imu,
            10
        )

        self.create_subscription(
            Odometry,
            '/bicycle_steering_controller/odometry',
            self.callback_odom,
            10
        )

    # -------------------------
    # TIME HELPERS
    # -------------------------
    def now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def stamp_to_ms(self, stamp):
        return stamp.sec * 1000.0 + stamp.nanosec / 1e6

    # =========================================================
    # ENCODER CALLBACK
    # =========================================================
    def callback_encoder(self, msg):
        arrival = self.now_ms()
        msg_stamp = self.stamp_to_ms(msg.header.stamp)

        print(f"[ENCODER] received | stamp={msg_stamp:.2f} ms | arrival={arrival:.2f} ms")

        if self.last_encoder_stamp is not None:
            dt = msg_stamp - self.last_encoder_stamp
            if len(self.encoder_intervals) < self.test_samples:
                self.encoder_intervals.append(dt)

        latency = arrival - msg_stamp
        if len(self.encoder_latencies) < self.test_samples:
            self.encoder_latencies.append(latency)

        print(f"[ENCODER] latency={latency:.3f} ms | samples={len(self.encoder_latencies)}/{self.test_samples}")

        self.last_encoder_stamp = msg_stamp
        self.check_and_report()

    # =========================================================
    # IMU CALLBACK
    # =========================================================
    def callback_imu(self, msg):
        arrival = self.now_ms()
        msg_stamp = self.stamp_to_ms(msg.header.stamp)

        print(f"[IMU] received | stamp={msg_stamp:.2f} ms | arrival={arrival:.2f} ms")

        if self.last_imu_stamp is not None:
            dt = msg_stamp - self.last_imu_stamp
            if len(self.imu_intervals) < self.test_samples:
                self.imu_intervals.append(dt)

        latency = arrival - msg_stamp
        if len(self.imu_latencies) < self.test_samples:
            self.imu_latencies.append(latency)

        print(f"[IMU] latency={latency:.3f} ms | samples={len(self.imu_latencies)}/{self.test_samples}")

        self.last_imu_stamp = msg_stamp
        self.check_and_report()

    # =========================================================
    # ODOM CALLBACK
    # =========================================================
    def callback_odom(self, msg: Odometry):
        arrival = self.now_ms()
        msg_stamp = self.stamp_to_ms(msg.header.stamp)

        print(f"[ODOM] received | stamp={msg_stamp:.2f} ms | arrival={arrival:.2f} ms")

        if self.last_odom_stamp is not None:
            dt = msg_stamp - self.last_odom_stamp
            if len(self.odom_intervals) < self.test_samples:
                self.odom_intervals.append(dt)

        latency = arrival - msg_stamp
        if len(self.odom_latencies) < self.test_samples:
            self.odom_latencies.append(latency)

        print(f"[ODOM] latency={latency:.3f} ms | samples={len(self.odom_latencies)}/{self.test_samples}")

        self.last_odom_stamp = msg_stamp
        self.check_and_report()

    # =========================================================
    # REPORTING
    # =========================================================
    def stats(self, arr):
        return sum(arr) / len(arr), min(arr), max(arr)

    def check_and_report(self):
        if (
            len(self.encoder_intervals) >= self.test_samples and
            len(self.imu_intervals) >= self.test_samples and
            len(self.odom_intervals) >= self.test_samples
        ):

            print("\n================ FINAL TEST RESULTS ================\n")

            e_avg, e_min, e_max = self.stats(self.encoder_intervals)
            print(f"ENCODER interval avg={e_avg:.3f} ms min={e_min:.3f} max={e_max:.3f}")
            print(f"ENCODER latency avg={sum(self.encoder_latencies)/len(self.encoder_latencies):.3f} ms")

            print("\n----------------------------------------------------\n")

            i_avg, i_min, i_max = self.stats(self.imu_intervals)
            print(f"IMU interval avg={i_avg:.3f} ms min={i_min:.3f} max={i_max:.3f}")
            print(f"IMU latency avg={sum(self.imu_latencies)/len(self.imu_latencies):.3f} ms")

            print("\n----------------------------------------------------\n")

            o_avg, o_min, o_max = self.stats(self.odom_intervals)
            print(f"ODOM interval avg={o_avg:.3f} ms min={o_min:.3f} max={o_max:.3f}")
            print(f"ODOM latency avg={sum(self.odom_latencies)/len(self.odom_latencies):.3f} ms")

            print("\n====================================================\n")

            rclpy.shutdown()
            sys.exit(0)

    def run(self):
        print("BEP latency test node started")
        rclpy.spin(self)


def main():
    rclpy.init()

    car_number = int(os.environ.get('car_number', '1'))
    node = BEPTestTopics(car_number)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()