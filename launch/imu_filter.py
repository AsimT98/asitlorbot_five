#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math

class ComplementaryFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('alpha', 0.98),  # Complementary filter factor
            ]
        )

        self.alpha = self.get_parameter('alpha').value

        # Initialize orientation variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Create a publisher for filtered orientation
        self.publisher_orientation = self.create_publisher(PoseStamped, 'filtered_orientation', 10)

        # Subscribe to IMU messages
        self.subscription = self.create_subscription(
            Imu,
            'imu/out',  # Replace with the actual IMU topic
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extract angular velocities from the gyroscope
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Extract accelerations from the accelerometer
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # Convert angular velocities to radians per second
        gyro_x_rad_sec = gyro_x
        gyro_y_rad_sec = gyro_y
        gyro_z_rad_sec = gyro_z

        # Integration of gyro rates to update orientation
        dt = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Time difference in seconds
        self.roll += gyro_x_rad_sec * dt
        self.pitch += gyro_y_rad_sec * dt
        self.yaw += gyro_z_rad_sec * dt

        # Apply accelerometer data to correct drift
        accel_roll = math.atan2(accel_y, accel_z)
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # Apply complementary filter
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

        # Now 'roll', 'pitch', and 'yaw' contain the filtered orientation

        # Publish the filtered orientation
        orientation_msg = PoseStamped()
        orientation_msg.header.stamp = self.get_clock().now().to_msg()
        orientation_msg.header.frame_id = 'odom'  # Replace with the actual frame ID
        orientation_msg.pose.orientation.x = math.sin(self.roll / 2.0)
        orientation_msg.pose.orientation.y = math.sin(self.pitch / 2.0)
        orientation_msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        orientation_msg.pose.orientation.w = math.cos(self.roll / 2.0) * math.cos(self.pitch / 2.0) * math.cos(self.yaw / 2.0)

        self.publisher_orientation.publish(orientation_msg)

def main(args=None):
    rclpy.init(args=args)
    complementary_filter_node = ComplementaryFilterNode()
    rclpy.spin(complementary_filter_node)
    complementary_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
