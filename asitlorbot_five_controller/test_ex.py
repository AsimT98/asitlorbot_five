#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_ = self.create_publisher(Twist,'diff_cont/cmd_vel_unstamped', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity_ = 0.2
        self.angular_velocity_ = 0.5
        self.target_angle_ = math.radians(45)  # Target angle in radians
        self.current_angle_ = 0
        self.turning_phase_ = True  # True: Turning, False: Moving in a circular path

    def timer_callback(self):
        msg = Twist()

        if self.turning_phase_:
            msg.angular.z = self.angular_velocity_
            self.current_angle_ += abs(self.angular_velocity_) * 0.1  # 0.1 is the timer period
            if self.current_angle_ >= self.target_angle_:
                # Switch to circular motion phase
                self.turning_phase_ = False
                self.current_angle_ = 0
        else:
            msg.linear.x = self.linear_velocity_
            msg.angular.z = self.angular_velocity_

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    circular_motion_node = CircularMotionNode()

    rclpy.spin(circular_motion_node)

    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
