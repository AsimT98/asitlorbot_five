#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
import geometry_msgs.msg  # Add this import statement for Point
from geometry_msgs.msg import Point


class LidarFeatureExtraction(Node):
    def __init__(self):
        super().__init__('lidar_feature_extraction')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_topic',  # Change this to your LIDAR topic
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Marker, 'line_markers', 10)
        self.marker_id = 0

    def lidar_callback(self, msg):
        # Convert polar coordinates to Cartesian coordinates
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        points_x = msg.ranges * np.cos(angles)
        points_y = msg.ranges * np.sin(angles)

        # Stack the Cartesian coordinates into a 2D array
        point_cloud = np.column_stack((points_x, points_y))

        # Apply DBSCAN clustering to identify line segments
        db = DBSCAN(eps=0.2, min_samples=5).fit(point_cloud)
        labels = db.labels_

        # Extract unique labels (ignoring noise, which is labeled as -1)
        unique_labels = set(labels) - {-1}

        # Create Marker message for visualization
        marker_msg = Marker()
        marker_msg.header = msg.header
        marker_msg.id = self.marker_id
        self.marker_id += 1
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.05  # Line width

        # Iterate through each cluster and publish line segments
        for label in unique_labels:
            cluster_points = point_cloud[labels == label]
            cluster_points = np.vstack((cluster_points, cluster_points[0]))  # Close the loop
            for point in cluster_points:
                p = Point()  # Use geometry_msgs.msg.Point
                p.x, p.y = point
                marker_msg.points.append(p)

        # Publish the Marker message
        self.publisher.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)

    lidar_feature_extraction = LidarFeatureExtraction()

    rclpy.spin(lidar_feature_extraction)

    lidar_feature_extraction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
