#!/usr/bin/env python3

import os
import rclpy
import tf2_ros
import yaml
from tf2_ros import TransformListener
from geometry_msgs.msg import Pose, Point
from rclpy.node import Node
import numpy as np
import pandas as pd  # Import pandas for working with Excel
import matplotlib.pyplot as plt

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer = self.create_timer(2, self.timer_callback)

        self.ground_truth_poses = []  # Store ground truth poses
        self.estimated_poses = []  # Store estimated poses
        self.NIS_values = []  # Store NIS values
        self.Mahalanobis_distances = []  # Store Mahalanobis distances

        # Load process noise covariance matrix from ekf.yaml
        self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

    def load_process_noise_covariance_from_yaml(self):
        try:
            # Get the path to the ekf.yaml file
            package_name = 'asitlorbot_five_localization'
            config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

            # Check if the file exists before attempting to open
            if not os.path.isfile(config_file_path):
                self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
                return None

            # Load the YAML file
            with open(config_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)

            # Extract the process noise covariance from the loaded YAML data
            process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

            if process_noise_covariance is not None:
                # Convert the list elements to float
                process_noise_covariance = [float(val) for val in process_noise_covariance]

                # Convert the 1D array to a 15x15 matrix
                process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

                # Ensure the matrix is symmetric (if required)
                process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

                return process_noise_covariance_matrix
            else:
                self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
                return None
        except Exception as e:
            self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
            return None

    def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
        try:
            # Compute the difference between ground truth and estimated poses
            pose_diff = np.array([
                pose_gt.position.x - pose_est.position.x,
                pose_gt.position.y - pose_est.position.y,
                pose_gt.position.z - pose_est.position.z,
                pose_gt.orientation.x - pose_est.orientation.x,
                pose_gt.orientation.y - pose_est.orientation.y,
                pose_gt.orientation.z - pose_est.orientation.z,
                pose_gt.orientation.w - pose_est.orientation.w,
            ])

            # Reshape explicitly to a column vector
            pose_diff = pose_diff.reshape(7, 1)

            # Reshape covariance_matrix to (7, 7)
            covariance_matrix = covariance_matrix[:7, :7]

            # Compute the NIS value and use item() to convert to scalar
            NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()

            return NIS
        except Exception as e:
            self.get_logger().error(f'Error calculating NIS: {str(e)}')
            return None

    def calculate_Mahalanobis_distance(self, pose_gt, pose_est, covariance_matrix):
        try:
            # Compute the difference between ground truth and estimated poses
            pose_diff = np.array([
                pose_gt.position.x - pose_est.position.x,
                pose_gt.position.y - pose_est.position.y,
                pose_gt.position.z - pose_est.position.z,
                pose_gt.orientation.x - pose_est.orientation.x,
                pose_gt.orientation.y - pose_est.orientation.y,
                pose_gt.orientation.z - pose_est.orientation.z,
                pose_gt.orientation.w - pose_est.orientation.w,
            ])

            # Reshape explicitly to a column vector
            pose_diff = pose_diff.reshape(7, 1)

            # Reshape covariance_matrix to (7, 7)
            covariance_matrix = covariance_matrix[:7, :7]

            # Compute the Mahalanobis distance
            mahalanobis_distance = np.sqrt(pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff)

            return mahalanobis_distance.item()
        except Exception as e:
            self.get_logger().error(f'Error calculating Mahalanobis distance: {str(e)}')
            return None

    def timer_callback(self):
        try:
            # Get transforms directly
            trans_base_footprint = self.buffer.lookup_transform(
                'base_footprint', 'odom', rclpy.time.Time(seconds=0)
            )
            trans_base_footprint_ekf = self.buffer.lookup_transform(
                'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
            )

            # Extract poses 
            pose_base_footprint = Pose()
            pose_base_footprint.position = Point(
                x=trans_base_footprint.transform.translation.x,
                y=trans_base_footprint.transform.translation.y,
                z=trans_base_footprint.transform.translation.z
            )
            pose_base_footprint.orientation = trans_base_footprint.transform.rotation

            pose_base_footprint_ekf = Pose()
            pose_base_footprint_ekf.position = Point(
                x=trans_base_footprint_ekf.transform.translation.x,
                y=trans_base_footprint_ekf.transform.translation.y,
                z=trans_base_footprint_ekf.transform.translation.z
            )
            pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

            # Store poses for later saving to Excel
            self.ground_truth_poses.append(pose_base_footprint)
            self.estimated_poses.append(pose_base_footprint_ekf)

            # Print out the collected poses for debugging
            self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
            self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

            # Compute and store NIS value
            NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
            self.NIS_values.append(NIS_value)

            # Compute and store Mahalanobis distance
            mahalanobis_distance = self.calculate_Mahalanobis_distance(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
            self.Mahalanobis_distances.append(mahalanobis_distance)

            # Check if enough poses collected
            if len(self.ground_truth_poses) == 70:  # Assuming you want 25 poses
                # Print out collected poses before saving to Excel
                self.get_logger().info('Collected Ground Truth Poses:')
                for i, gt_pose in enumerate(self.ground_truth_poses):
                    self.get_logger().info(f'Frame {i}: {gt_pose}')

                self.get_logger().info('Collected Estimated Poses:')
                for i, est_pose in enumerate(self.estimated_poses):
                    self.get_logger().info(f'Frame {i}: {est_pose}')

                # Save poses, NIS values, and Mahalanobis distances to Excel
                self.save_to_excel()

                # Plot Mahalanobis distances
                self.plot_mahalanobis_distances()

                # Shut down the node after saving poses to Excel
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Error getting transforms: {}'.format(str(e)))

    def save_to_excel(self):
        excel_filename = 'pose_and_nis_and_mahalanobis_values.xlsx'

        # Convert poses to DataFrame
        ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
                                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                                       for pose in self.ground_truth_poses],
                                      columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

        estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
                                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                                    for pose in self.estimated_poses],
                                   columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

        # Save NIS values to DataFrame
        nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])

        # Save Mahalanobis distances to DataFrame
        mahalanobis_df = pd.DataFrame(self.Mahalanobis_distances, columns=['Mahalanobis_Distance'])

        # Save DataFrames to Excel
        with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
            ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
            estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)
            nis_df.to_excel(writer, sheet_name='NIS_Values', index=False)
            mahalanobis_df.to_excel(writer, sheet_name='Mahalanobis_Distances', index=False)

        self.get_logger().info('Poses, NIS values, and Mahalanobis distances saved to {}'.format(excel_filename))

    def plot_mahalanobis_distances(self):
        # Plot Mahalanobis distances
        plt.plot(range(1, len(self.Mahalanobis_distances) + 1), self.Mahalanobis_distances, label='Mahalanobis Distance')
        
        # Plot a line for NIS = 7.815
        plt.axhline(y=7.815, color='r', linestyle='--', label='NIS = 7.815')

        plt.xlabel('Number of Cycles')
        plt.ylabel('Mahalanobis Distance')
        plt.title('Mahalanobis Distance over Cycles')
        plt.legend()
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

if __name__ == "__main__":
    main()











## NIS and Mahalanobis distance
# import os
# import rclpy
# import tf2_ros
# import yaml
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.NIS_values = []  # Store NIS values
#         self.Mahalanobis_distances = []  # Store Mahalanobis distances

#         # Load process noise covariance matrix from ekf.yaml
#         self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

#     def load_process_noise_covariance_from_yaml(self):
#         try:
#             # Get the path to the ekf.yaml file
#             package_name = 'asitlorbot_five_localization'
#             config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

#             # Check if the file exists before attempting to open
#             if not os.path.isfile(config_file_path):
#                 self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
#                 return None

#             # Load the YAML file
#             with open(config_file_path, 'r') as file:
#                 yaml_data = yaml.safe_load(file)

#             # Extract the process noise covariance from the loaded YAML data
#             process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

#             if process_noise_covariance is not None:
#                 # Convert the list elements to float
#                 process_noise_covariance = [float(val) for val in process_noise_covariance]

#                 # Convert the 1D array to a 15x15 matrix
#                 process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

#                 # Ensure the matrix is symmetric (if required)
#                 process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

#                 return process_noise_covariance_matrix
#             else:
#                 self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
#                 return None
#         except Exception as e:
#             self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
#             return None

#     def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             print("Shape of pose_diff before reshape:", pose_diff.shape)

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             print("Shape of pose_diff after reshape:", pose_diff.shape)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             print("Shape of covariance_matrix after reshape:", covariance_matrix.shape)

#             # Compute the NIS value and use item() to convert to scalar
#             NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()

#             return NIS
#         except Exception as e:
#             self.get_logger().error(f'Error calculating NIS: {str(e)}')
#             return None
#     def calculate_Mahalanobis_distance(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             # Compute the Mahalanobis distance
#             mahalanobis_distance = np.sqrt(pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff)

#             return mahalanobis_distance.item()
#         except Exception as e:
#             self.get_logger().error(f'Error calculating Mahalanobis distance: {str(e)}')
#             return None

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Compute and store NIS value
#             NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.NIS_values.append(NIS_value)

#             # Compute and store Mahalanobis distance
#             mahalanobis_distance = self.calculate_Mahalanobis_distance(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.Mahalanobis_distances.append(mahalanobis_distance)

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 25:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses and NIS values to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_and_nis_and_mahalanobis_values.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save NIS values to DataFrame
#         nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])

#         # Save Mahalanobis distances to DataFrame
#         mahalanobis_df = pd.DataFrame(self.Mahalanobis_distances, columns=['Mahalanobis_Distance'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)
#             nis_df.to_excel(writer, sheet_name='NIS_Values', index=False)
#             mahalanobis_df.to_excel(writer, sheet_name='Mahalanobis_Distances', index=False)

#         self.get_logger().info('Poses, NIS values, and Mahalanobis distances saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()









# STORE THE POSE IN EXCEL

# import os
# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 50:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_gf.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)

#         self.get_logger().info('Poses saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()





## NIS only

# import os
# import rclpy
# import tf2_ros
# import yaml
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# import pandas as pd  # Import pandas for working with Excel

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.NIS_values = []  # Store NIS values

#         # Load process noise covariance matrix from ekf.yaml
#         self.process_noise_covariance = self.load_process_noise_covariance_from_yaml()

#     def load_process_noise_covariance_from_yaml(self):
#         try:
#             # Get the path to the ekf.yaml file
#             package_name = 'asitlorbot_five_localization'
#             config_file_path = f'/home/asimkumar/asitlor_ws/src/{package_name}/config/ekf.yaml'

#             # Check if the file exists before attempting to open
#             if not os.path.isfile(config_file_path):
#                 self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: File not found at {os.path.abspath(config_file_path)}')
#                 return None

#             # Load the YAML file
#             with open(config_file_path, 'r') as file:
#                 yaml_data = yaml.safe_load(file)

#             # Extract the process noise covariance from the loaded YAML data
#             process_noise_covariance = yaml_data.get('ekf_filter_node', {}).get('ros__parameters', {}).get('process_noise_covariance', None)

#             if process_noise_covariance is not None:
#                 # Convert the list elements to float
#                 process_noise_covariance = [float(val) for val in process_noise_covariance]

#                 # Convert the 1D array to a 15x15 matrix
#                 process_noise_covariance_matrix = np.array(process_noise_covariance).reshape((15, 15))

#                 # Ensure the matrix is symmetric (if required)
#                 process_noise_covariance_matrix = 0.5 * (process_noise_covariance_matrix + process_noise_covariance_matrix.T)

#                 return process_noise_covariance_matrix
#             else:
#                 self.get_logger().error('Error loading process_noise_covariance from ekf.yaml: Missing or invalid data.')
#                 return None
#         except Exception as e:
#             self.get_logger().error(f'Error loading process_noise_covariance from ekf.yaml: {str(e)}')
#             return None


#     def calculate_NIS(self, pose_gt, pose_est, covariance_matrix):
#         try:
#             # Compute the difference between ground truth and estimated poses
#             pose_diff = np.array([
#                 pose_gt.position.x - pose_est.position.x,
#                 pose_gt.position.y - pose_est.position.y,
#                 pose_gt.position.z - pose_est.position.z,
#                 pose_gt.orientation.x - pose_est.orientation.x,
#                 pose_gt.orientation.y - pose_est.orientation.y,
#                 pose_gt.orientation.z - pose_est.orientation.z,
#                 pose_gt.orientation.w - pose_est.orientation.w,
#             ])

#             print("Shape of pose_diff before reshape:", pose_diff.shape)

#             # Reshape explicitly to a column vector
#             pose_diff = pose_diff.reshape(7, 1)

#             print("Shape of pose_diff after reshape:", pose_diff.shape)

#             # Reshape covariance_matrix to (7, 7)
#             covariance_matrix = covariance_matrix[:7, :7]

#             print("Shape of covariance_matrix after reshape:", covariance_matrix.shape)

#             # Compute the NIS value and use item() to convert to scalar
#             NIS = (pose_diff.T @ np.linalg.pinv(covariance_matrix) @ pose_diff).item()

#             return NIS
#         except Exception as e:
#             self.get_logger().error(f'Error calculating NIS: {str(e)}')
#             return None


#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later saving to Excel
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Compute and store NIS value
#             NIS_value = self.calculate_NIS(pose_base_footprint, pose_base_footprint_ekf, self.process_noise_covariance)
#             self.NIS_values.append(NIS_value)

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 25:  # Assuming you want 50 poses
#                 # Print out collected poses before saving to Excel
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Save poses and NIS values to Excel
#                 self.save_to_excel()

#                 # Shut down the node after saving poses to Excel
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_excel(self):
#         excel_filename = 'pose_and_nis_values.xlsx'

#         # Convert poses to DataFrame
#         ground_truth_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                        for pose in self.ground_truth_poses],
#                                       columns=['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z', 'GT_Orient_X', 'GT_Orient_Y', 'GT_Orient_Z', 'GT_Orient_W'])

#         estimated_df = pd.DataFrame([[pose.position.x, pose.position.y, pose.position.z,
#                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
#                                     for pose in self.estimated_poses],
#                                    columns=['Est_Pos_X', 'Est_Pos_Y', 'Est_Pos_Z', 'Est_Orient_X', 'Est_Orient_Y', 'Est_Orient_Z', 'Est_Orient_W'])

#         # Save NIS values to DataFrame
#         nis_df = pd.DataFrame(self.NIS_values, columns=['NIS'])

#         # Save DataFrames to Excel
#         with pd.ExcelWriter(excel_filename, engine='xlsxwriter') as writer:
#             ground_truth_df.to_excel(writer, sheet_name='Ground_Truth_Poses', index=False)
#             estimated_df.to_excel(writer, sheet_name='Estimated_Poses', index=False)
#             nis_df.to_excel(writer, sheet_name='NIS_Values', index=False)

#         self.get_logger().info('Poses and NIS values saved to {}'.format(excel_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()


########################################

# import os
# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )
 
#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 50:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'

#         # Check if the CSV file already exists
#         file_exists = os.path.isfile(csv_filename)

#         with open(csv_filename, mode='a', newline='') as file:
#             writer = csv.writer(file)

#             # If the file doesn't exist, write the header
#             if not file_exists:
#                 writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             # Write the new RPE values
#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values appended to {}'.format(csv_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()






########################

# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np
# from std_msgs.msg import String
# import time
# import signal

# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )

#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 5:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'
#         with open(csv_filename, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values saved to {}'.format(csv_filename))


# def handler(signum, frame):
#     print(f"Received signal {signum}. Shutting down gracefully...")
#     rclpy.shutdown()

# def main(args=None):
#     signal.signal(signal.SIGINT, handler)

#     rclpy.init(args=args)
#     node = Node('rmse_node')
#     subscriber = node.create_subscription(String, 'topic', lambda msg: print(msg.data), 10)
#     rclpy.spin(node)
#     node.destroy_subscription(subscriber)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

###########################




# import rclpy
# import tf2_ros
# import csv
# from tf2_ros import TransformListener
# from geometry_msgs.msg import Pose, Point
# from rclpy.node import Node
# import numpy as np


# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('subscriber')

#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.timer = self.create_timer(2, self.timer_callback)

#         self.ground_truth_poses = []  # Store ground truth poses
#         self.estimated_poses = []  # Store estimated poses
#         self.rpe_trans = []  # Store translation RPE
#         self.rpe_rot = []  # Store rotation RPE

#     def calculate_translation_rpe(self, pose_gt, pose_est):
#         # Extract and convert translation vectors
#         position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
#         position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

#         # Calculate squared Euclidean norm and accumulate for all poses
#         translation_diff = position_gt - position_est
#         squared_norm = np.linalg.norm(translation_diff) ** 2

#         return squared_norm

#     def calculate_rotation_rpe(self, pose_gt, pose_est):
#         # Extract and convert rotations (assuming quaternions)
#         quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
#         quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

#         # Ensure quaternions are normalized
#         quaternion_gt /= np.linalg.norm(quaternion_gt)
#         quaternion_est /= np.linalg.norm(quaternion_est)

#         # Calculate difference in angle using arccos for quaternions
#         angle_diff = np.arccos(
#             np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
#         )
 
#         return angle_diff

#     def timer_callback(self):
#         try:
#             # Get transforms directly
#             trans_base_footprint = self.buffer.lookup_transform(
#                 'base_footprint', 'odom', rclpy.time.Time(seconds=0)
#             )
#             trans_base_footprint_ekf = self.buffer.lookup_transform(
#                 'base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0)
#             )

#             # Extract poses 
#             pose_base_footprint = Pose()
#             pose_base_footprint.position = Point(
#                 x=trans_base_footprint.transform.translation.x,
#                 y=trans_base_footprint.transform.translation.y,
#                 z=trans_base_footprint.transform.translation.z
#             )
#             pose_base_footprint.orientation = trans_base_footprint.transform.rotation

#             pose_base_footprint_ekf = Pose()
#             pose_base_footprint_ekf.position = Point(
#                 x=trans_base_footprint_ekf.transform.translation.x,
#                 y=trans_base_footprint_ekf.transform.translation.y,
#                 z=trans_base_footprint_ekf.transform.translation.z
#             )
#             pose_base_footprint_ekf.orientation = trans_base_footprint_ekf.transform.rotation

#             # Store poses for later RPE calculation
#             self.ground_truth_poses.append(pose_base_footprint)
#             self.estimated_poses.append(pose_base_footprint_ekf)

#             # Print out the collected poses for debugging
#             self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
#             self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

#             # Check if enough poses collected
#             if len(self.ground_truth_poses) == 5:  # Assuming you want 10 poses
#                 # Print out collected poses before calculating RPE
#                 self.get_logger().info('Collected Ground Truth Poses:')
#                 for i, gt_pose in enumerate(self.ground_truth_poses):
#                     self.get_logger().info(f'Frame {i}: {gt_pose}')

#                 self.get_logger().info('Collected Estimated Poses:')
#                 for i, est_pose in enumerate(self.estimated_poses):
#                     self.get_logger().info(f'Frame {i}: {est_pose}')

#                 # Continue with RPE calculation
#                 m = len(self.ground_truth_poses)
#                 n = len(self.estimated_poses)

#                 # Calculate translation RPE
#                 rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_trans = np.sqrt(rpe_trans_sum / m)
#                 self.rpe_trans.append(rpe_trans)

#                 # Calculate rotation RPE
#                 rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
#                 rpe_rot = np.mean(rpe_rot_sum / n)
#                 self.rpe_rot.append(rpe_rot)

#                 # Save RPE values to a CSV file
#                 self.save_to_csv()

#                 # Shut down the node after saving RPE values
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error('Error getting transforms: {}'.format(str(e)))

#     def save_to_csv(self):
#         csv_filename = 'rpe_values.csv'
#         with open(csv_filename, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['Frame ID', 'Translation RPE', 'Rotation RPE'])

#             for i, (translation_rpe, rotation_rpe) in enumerate(zip(self.rpe_trans, self.rpe_rot)):
#                 writer.writerow([
#                     f'Frame {i}',
#                     translation_rpe,
#                     rotation_rpe,
#                 ])

#         self.get_logger().info('RPE Values saved to {}'.format(csv_filename))


# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

# if __name__ == "__main__":
#     main()
    