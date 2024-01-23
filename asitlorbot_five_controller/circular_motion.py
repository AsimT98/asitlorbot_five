#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5
        self.target_angle = math.radians(45)  # Target angle in radians
        self.current_angle = 0
        self.turning_phase = True  # True: Turning, False: Moving in a circular path

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

    def timer_callback(self):
        twist_msg = Twist()

        if self.turning_phase:
            twist_msg.angular.z = self.angular_velocity
            self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
            if self.current_angle >= self.target_angle:
                # Switch to circular motion phase
                self.turning_phase = False
                self.current_angle = 0
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update the paths
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

        # Update path_base_link
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

        # Update path_base_footprint_ekf
        # Modify this part based on the transformation between base_footprint_ekf and odom
        pose.header.frame_id = 'base_footprint_ekf'
        self.path_base_footprint_ekf.poses.append(pose)
        self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

        # Update path_base_footprint_noisy
        # Modify this part based on the transformation between base_footprint_noisy and odom
        pose.header.frame_id = 'base_footprint_noisy'
        self.path_base_footprint_noisy.poses.append(pose)
        self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

def main(args=None):
    rclpy.init(args=args)
    circular_motion_node = CircularMotionNode()
    rclpy.spin(circular_motion_node)
    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
