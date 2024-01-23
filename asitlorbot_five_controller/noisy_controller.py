#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler

class NoisyController(Node):

    def __init__(self):
        super().__init__("simple_controller") #calling the constructor of base class
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.35)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        #PRINT THIS 2 PARAMETER IN THE TERMINAL
        self.get_logger().info("Using wheel radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation_)

        self.left_front_wheel_prev_pos_ = 0.0
        self.right_front_wheel_prev_pos_ = 0.0
        self.left_rear_wheel_prev_pos_ = 0.0
        self.right_rear_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "asitlorbot_five_controller/odom_noisy", 10)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"  
        self.prev_time_ = self.get_clock().now()


    def jointCallback(self, msg):
        
        wheelencoder_left_front = msg.position[1] + np.random.normal(0,0.005)
        wheelencoder_left_rear = msg.position[1] + np.random.normal(0,0.005)
        wheelencoder_right_front = msg.position[0] + np.random.normal(0,0.005)
        wheelencoder_right_rear = msg.position[0] + np.random.normal(0,0.005)
        
        
        # Implements the inverse differential kinematic model for a 4-wheel drive robot
        # Given the position of the wheels, calculate their velocities
        # then calculate the velocity of the robot with respect to the robot frame
        # and then convert it to the global frame and publish the TF

        dp_left_front = wheelencoder_left_front - self.left_front_wheel_prev_pos_
        dp_right_front = wheelencoder_right_front - self.right_front_wheel_prev_pos_
        dp_left_rear = wheelencoder_left_rear - self.left_rear_wheel_prev_pos_
        dp_right_rear = wheelencoder_right_rear - self.right_rear_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Update the previous pose for the next iteration
        self.left_front_wheel_prev_pos_ = msg.position[1]
        self.right_front_wheel_prev_pos_ = msg.position[0]
        self.left_rear_wheel_prev_pos_ = msg.position[1]
        self.right_rear_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Calculate the rotational speed of each wheel
        fi_left_front = dp_left_front / (dt.nanoseconds/S_TO_NS)
        fi_right_front = dp_right_front / (dt.nanoseconds/S_TO_NS)
        fi_left_rear = dp_left_rear / (dt.nanoseconds/S_TO_NS)
        fi_right_rear = dp_right_rear / (dt.nanoseconds/S_TO_NS)

        # Calculate the linear and angular velocity
        linear = (
            (self.wheel_radius_/4)* (fi_left_front +fi_right_front +fi_left_rear +fi_right_rear)
        )
        angular = (
            (self.wheel_radius_/(2*self.wheel_separation_))*(fi_left_front -fi_right_front +fi_left_rear -fi_right_rear)
        )

        # Calculate the position increment
        d_s = (
            (self.wheel_radius_/4)* (dp_left_front +dp_left_rear + dp_right_rear +dp_right_front)
        )
        d_theta = (
            (self.wheel_radius_*(dp_left_front -dp_right_front +dp_left_rear -dp_right_rear))/ (2*self.wheel_separation_)
        )
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)

def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
