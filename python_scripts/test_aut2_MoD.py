#!/usr/bin/env python3

import math
import time
import threading
import copy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


def euler_from_quaternion(q):
    """Zet quaternion [x, y, z, w] om naar euler hoeken (yaw, pitch, roll)."""
    x, y, z, w = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    return yaw, pitch, roll


class BicycleBEPTest(Node):

    def __init__(self):
        super().__init__('bicycle_bep_test')

        # Control reference publisher using TwistStamped
        self.pub_reference = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            10
        )

        # Odometry Subscriber
        self.create_subscription(
            Odometry,
            '/bicycle_steering_controller/odometry',
            self.odom_callback,
            10
        )

        # Custom QoS Profile matching the Best Effort MoCap publisher
        vicon_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Vicon Subscription using the correct Best Effort profile
        self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/JetracerMirte1/pose',
            self.viconpos_callback,
            vicon_qos
        )

        # PD controller settings
        self.Kp = 0.2
        self.Kd = 0.5
        self.r = 0.3  # Target arrival radius clearance
        self.L = 0.176 # wheelbase

        # Odometry tracking state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.w = 0.0

        # Vicon tracking state
        self.viconpos_x = 0.0
        self.viconpos_y = 0.0
        self.viconpos_theta = 0.0

        self.first_vicon_x = 0.0
        self.first_vicon_y = 0.0
        self.first_vicon_theta = 0.0
        self.last_vicon_msg_received = float('inf')
        self.viconpos_received = False

        #vicon speed
        self.vx=0.0
        self.vy=0.0

        # Waypoint coordinate maps
        self.c_relative = [[2, 1], [0, 0], [4, 0], [0, 0]] #vier
        #self.c_relative = [[2, -1], [5, 1], [1, 0], [5, -2], [3, 1], [3, -2], [5, -2], [0, 0]] #acht
        self.c = copy.deepcopy(self.c_relative)
        self.reached_pos_odo = []
        self.reached_pos_vicon = []

        self.get_logger().info('Bicycle BEP test node met Vicon gestart!')

    def run_test(self):
        self.get_logger().info("Wachten op Vicon MoCap frames...")
        
        # Keep waiting until the background spin_thread starts parsing Vicon frames
        while not self.viconpos_received and rclpy.ok():
            time.sleep(0.1)

        self.get_logger().info("Vicon gelocked! Starten van routebeschrijving...")

        # Run through our localized coordinate targets sequence
        for i, target in enumerate(self.c):
            self.get_logger().info(f"Rij naar Target {i}: {self.c_relative[i]}")
            self.drive_to_target(target)

        # Stop completely when paths are fully parsed
        self.get_logger().info("Route voltooid. Stop de robot.")
        self.stop_robot()
        
        self.print_summary()

    def drive_to_target(self, target):
        c_x, c_y = target[0], target[1]
        rate_hz = 50.0  
        period = 1.0 / rate_hz

        # Loop initialization metrics
        d_target = math.sqrt((c_y - self.y)**2 + (c_x - self.x)**2)
        last_time = time.time()

        while d_target > self.r and rclpy.ok():
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            if dt <= 0.0:
                dt = 0.033

            # Recalculate dynamic distance metric and angle errors
            d_target = math.sqrt((c_y - self.y)**2 + (c_x - self.x)**2)
            theta_c = math.atan2((c_y - self.y), (c_x - self.x))
            d_theta = math.atan2(
                math.sin(theta_c - self.theta),
                math.cos(theta_c - self.theta)
            )
            speed = math.sqrt((self.vx)**2+(self.vy)**2)#speed
            #print("speed",speed)

            print(f"Afstand tot doel: {d_target:.3f} m", end='\r')

            # Dynamic speed adjustment based on distance to waypoint
            throttle = 1.3 if d_target > 0.8 else 1.1
            # DIT WERKT!! throttle = 1.8 if d_target > 0.8 else 1.1, kp=0.2
            #original throttle = 2.09 if d_target > 0.5 else 1.65, kp = 0.15, r=0.3
            calculated_speed = speed if speed > 0.05 else float(throttle)

           
            # working conversion: throttle = 1.5 if d_target > 0.5 else 1.3, kp=0.25, r=0.3
            
            #Calculate steering correction (PD loop structure)
            if d_theta > math.radians(40) and d_target >= 0.5:
                p_term = 2.5 * d_theta
            else:
                p_term = self.Kp * d_theta

            target_angular_z = p_term #- d_term
            target_angular_z = max(-1.0, min(target_angular_z, 1.0)) # Kept within your working limits
            omega = calculated_speed * math.tan(target_angular_z)/self.L
            print ("hoek:", self.theta)


            # Build and send the TwistStamped command
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.twist.linear.x = float(throttle)
            msg.twist.angular.z = float(omega)

            self.pub_reference.publish(msg)
            # time.sleep(period)

        print(f"\nTarget bereikt!")
        self.reached_pos_odo.append([self.x, self.y])
        self.reached_pos_vicon.append([self.viconpos_x - self.first_vicon_x, self.viconpos_y - self.first_vicon_y])

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.pub_reference.publish(msg)

    def print_summary(self):
        print("\n=== EINDRESULTAAT ===")
        print(f"Laatste Odometry positie: x: {self.x:.3f} m, y: {self.y:.3f} m, theta: {self.theta:.3f} rad")
        print(f"Eerste Vicon positie:     x: {self.first_vicon_x:.3f} m, y: {self.first_vicon_y:.3f} m")
        print(f"Laatste Vicon positie:    x: {self.viconpos_x:.3f} m, y: {self.viconpos_y:.3f} m")
        
        diff_x = self.viconpos_x - self.first_vicon_x
        diff_y = self.viconpos_y - self.first_vicon_y
        print(f"Vicon Relatief Verschil:  x: {diff_x:.3f} m, y: {diff_y:.3f} m")
        
        print("\nGemeten bereikte posities (Odometry):")
        for i, pos in enumerate(self.reached_pos_odo):
            print(f" Waypoint {i}: [{pos[0]:.3f}, {pos[1]:.3f}]")
        print("\nGemeten bereikte posities (Vicon Relatief):")
        for i, pos in enumerate(self.reached_pos_vicon):
            print(f" Waypoint {i}: [{pos[0]:.3f}, {pos[1]:.3f}]")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        yaw, _, _ = euler_from_quaternion(q)
        self.theta = yaw
        self.w = msg.twist.twist.angular.z

    def viconpos_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        yaw, _, _ = euler_from_quaternion(q)
        time_now = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)

        if time_now < self.last_vicon_msg_received:
            self.first_vicon_x = x
            self.first_vicon_y = y
            self.first_vicon_theta = yaw
            self.viconpos_received = True

            # Offset target arrays relative to local home origins
            # for i in range(len(self.c_relative)):
            #     self.c[i][0] = self.c_relative[i][0] + self.first_vicon_x
            #     self.c[i][1] = self.c_relative[i][1] + self.first_vicon_y
            
            self.last_vicon_msg_received = time_now

        self.viconpos_x = x
        self.viconpos_y = y
        self.viconpos_theta = yaw


       


def main(args=None):
    rclpy.init(args=args)
    node = BicycleBEPTest()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_test()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()