#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import math

class AdaptiveCruiseControlNode(Node):
    def __init__(self):
        super().__init__('adaptive_cruise_control_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5

        # Lidar subscriber
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/laser_controller/out',
            self.lidar_callback,
            10)
        self.subscription_lidar  # prevent unused variable warning

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

        # State variables
        self.obstacle_detected = False
        self.state = "random_straight_motion"
        self.angle_rotated = 0.0  # Variable to track the angle rotated during obstacle avoidance

    def lidar_callback(self, msg):
        # Process Lidar data, e.g., check for obstacles in the front
        # and set a flag if an obstacle is detected
        min_distance = min(msg.ranges)
        if min_distance < 0.5:  # Example threshold, adjust as needed
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        twist_msg = Twist()

        if self.state == "random_straight_motion":
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0

            # Update the paths only when in random_straight_motion state
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

            # Update path_base_link
            if self.path_base_link.poses:
                pose.pose.position.x = self.path_base_link.poses[-1].pose.position.x
                pose.pose.position.y = self.path_base_link.poses[-1].pose.position.y
                pose.pose.orientation = self.path_base_link.poses[-1].pose.orientation
            else:
                # Handle the case when the list is empty (for the first pose)
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

            self.path_base_link.poses.append(pose)
            self.publisher_path_base_link.publish(self.path_base_link)

            # Update path_base_footprint_ekf
            # Modify this part based on the transformation between base_footprint_ekf and odom
            if self.path_base_footprint_ekf.poses:
                pose.header.frame_id = 'base_footprint_ekf'
                self.path_base_footprint_ekf.poses.append(pose)
                self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

            # Update path_base_footprint_noisy
            # Modify this part based on the transformation between base_footprint_noisy and odom
            if self.path_base_footprint_noisy.poses:
                pose.header.frame_id = 'base_footprint_noisy'
                self.path_base_footprint_noisy.poses.append(pose)
                self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

            if self.obstacle_detected:
                # Transition to obstacle avoidance state
                self.state = "obstacle_avoidance"
                self.obstacle_detected = False  # Reset the flag

        elif self.state == "obstacle_avoidance":
            # Rotate for a certain angle (e.g., pi/2) to avoid obstacle
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5  # Rotate to avoid obstacle

            # Update the angle rotated so far
            self.angle_rotated += 0.1  # Adjust this value based on your requirements

            if self.angle_rotated >= math.pi / 2:  # Rotate for pi/2 radians
                # Transition back to random straight motion state
                self.state = "random_straight_motion"
                self.angle_rotated = 0.0  # Reset the angle

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    adaptive_cruise_control_node = AdaptiveCruiseControlNode()
    rclpy.spin(adaptive_cruise_control_node)
    adaptive_cruise_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
