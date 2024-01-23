#!/usr/bin/env python3
import rclpy
import tf2_ros
import csv
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        qos_profile = QoSProfile(depth=10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer = self.create_timer(2, self.timer_callback)

        self.get_logger().info('Starting {}'.format(self.get_name()))
        self.i = 0
        self.poses = []

    def calculate_pose(self, trans, ground_truth_pose=None):
        # Extract pose information
        pose = Pose()

        if trans is not None and trans.transform is not None:
            pose.position = Point(x=trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z)

            # Convert quaternion to euler
            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            euler = euler_from_quaternion(quaternion)

            # Assign Euler angles to the orientation field
            pose.orientation.x = euler[0]
            pose.orientation.y = euler[1]
            pose.orientation.z = euler[2]
            pose.orientation.w = 1.0  # Assuming a unit quaternion

            if ground_truth_pose:
                # Calculate relative pose error for translation
                translation_error = np.linalg.norm(np.array([
                    pose.position.x - ground_truth_pose.position.x,
                    pose.position.y - ground_truth_pose.position.y,
                    pose.position.z - ground_truth_pose.position.z
                ]))

                # Calculate relative pose error for rotation
                rotation_error = np.arccos(np.dot(
                    np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z]),
                    np.array([ground_truth_pose.orientation.x, ground_truth_pose.orientation.y, ground_truth_pose.orientation.z])
                ))
            else:
                translation_error = 0.0
                rotation_error = 0.0

            return translation_error, rotation_error

        else:
            self.get_logger().warning('Transform is None')
            return 0.0, 0.0

    def timer_callback(self):
        try:
            # Lookup the transform between base_footprint_ekf and odom
            trans_ekf = self.buffer.lookup_transform('base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0))
            pose_ekf = self.calculate_pose(trans_ekf)

            # Lookup the transform between base_footprint and odom
            trans_base = self.buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time(seconds=0))
            pose_base = self.calculate_pose(trans_base)

            # Log the poses
            self.get_logger().info('{}: Pose base_footprint: {}'.format(self.i, pose_base))
            self.get_logger().info('{}: Pose base_footprint_ekf: {}'.format(self.i, pose_ekf))

            # Append poses to the list
            self.poses.append((pose_ekf, pose_base))

            # If 20 poses are collected, save them to a CSV file and exit
            if len(self.poses) == 20:
                self.save_to_csv()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Error getting transforms: {}'.format(str(e)))

    def save_to_csv(self):
        csv_filename = 'relative_pose_error.csv'
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Frame ID', 'Translation Error', 'Rotation Error'])

            for i, ((translation_ekf, rotation_ekf), (translation_base, rotation_base)) in enumerate(self.poses):
                writer.writerow([
                    f'Frame {i}',
                    translation_ekf,
                    rotation_ekf,
                ])

                writer.writerow([
                    f'Frame {i}',
                    translation_base,
                    rotation_base,
                ])

        self.get_logger().info('Relative Pose Errors saved to {}'.format(csv_filename))

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()

if __name__ == "__main__":
    main()
