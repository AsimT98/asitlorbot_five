#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

lidar_pub = None

def lidarCallback(lidar):
    global lidar_pub
    lidar.header.frame_id = "base_footprint_ekf"
    lidar_pub.publish(lidar)


def main(args=None):
    global lidar_pub
    rclpy.init(args=args) #start ros2 interface
    node = Node('lidar_republisher_node')
    time.sleep(1)
    lidar_pub = node.create_publisher(LaserScan, "lidar_ekf", 10)
    lidar_sub = node.create_subscription(LaserScan, "laser_controller/out", lidarCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()