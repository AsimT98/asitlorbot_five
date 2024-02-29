
#!/usr/bin/env python3
import rclpy
import tf2_ros
import csv
from tf2_ros import TransformListener
from geometry_msgs.msg import Pose, Point
from rclpy.node import Node
import numpy as np
import yaml

class EkfUpdater(Node):

    def __init__(self, config_file_path):
        super().__init__('ekf_updater')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.config_file_path = config_file_path
        self.rpe_trans = []  # Store translation RPE
        self.rpe_rot = []  # Store rotation RPE
        self.ground_truth_poses = []  # Initialize ground truth poses list
        self.estimated_poses = []  # Initialize estimated poses list

    def calculate_translation_rpe(self, pose_gt, pose_est):
        # Extract and convert translation vectors
        position_gt = np.array([pose_gt.position.x, pose_gt.position.y, pose_gt.position.z])
        position_est = np.array([pose_est.position.x, pose_est.position.y, pose_est.position.z])

        # Calculate squared Euclidean norm and accumulate for all poses
        translation_diff = position_gt - position_est
        squared_norm = np.linalg.norm(translation_diff) ** 2

        return squared_norm

    def calculate_rotation_rpe(self, pose_gt, pose_est):
        # Extract and convert rotations (assuming quaternions)
        quaternion_gt = np.array([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])
        quaternion_est = np.array([pose_est.orientation.x, pose_est.orientation.y, pose_est.orientation.z, pose_est.orientation.w])

        # Ensure quaternions are normalized
        quaternion_gt /= np.linalg.norm(quaternion_gt)
        quaternion_est /= np.linalg.norm(quaternion_est)

        # Calculate difference in angle using arccos for quaternions
        angle_diff = np.arccos(
            np.clip(2 * np.dot(quaternion_gt, quaternion_est) ** 2 - 1, -1.0, 1.0)
        )

        return angle_diff

    def modify_matrix_and_run_rpe(self, initial, final, step):
        try:
            for factor in np.logspace(np.log10(initial), np.log10(final), num=int(np.log10(final/initial)/np.log10(step))+1):
                # Read EKF configuration from file
                with open(self.config_file_path, 'r') as file:
                    ekf_config = yaml.safe_load(file)
                    

                if ekf_config and 'process_noise_covariance' in ekf_config:
                    process_noise_covariance = np.array(ekf_config['process_noise_covariance']).reshape((15, 15))

                    # Indices of the diagonal elements to be modified
                    indices_to_modify = [5, 6, 7, 11]

                    for index in indices_to_modify:
                        process_noise_covariance[index, index] = factor

                    ekf_config['process_noise_covariance'] = process_noise_covariance.flatten().tolist()

                    # Save modified configuration
                    with open(self.config_file_path, 'w') as file:
                        yaml.dump(ekf_config, file)

                    # Reset poses for RPE calculation
                    self.rpe_trans = []
                    self.rpe_rot = []

                    # Run RPE calculation
                    self.run_rpe()

        except Exception as e:
            self.get_logger().error('Error modifying matrix and running RPE: {}'.format(str(e)))

    def run_rpe(self):
        try:
            for _ in range(30):  # Assuming you want 30 poses
                # Get transforms directly
                trans_base_footprint = self.buffer.lookup_transform(
                    'base_footprint', 'odom', rclpy.time.Time()
                )
                trans_base_footprint_ekf = self.buffer.lookup_transform(
                    'base_footprint_ekf', 'odom', rclpy.time.Time()
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

                # Store poses for later RPE calculation
                self.ground_truth_poses.append(pose_base_footprint)
                self.estimated_poses.append(pose_base_footprint_ekf)

                # Print out the collected poses for debugging
                self.get_logger().info('Ground Truth Pose: {}'.format(pose_base_footprint))
                self.get_logger().info('Estimated Pose: {}'.format(pose_base_footprint_ekf))

            # Continue with RPE calculation
            m = len(self.ground_truth_poses)
            n = len(self.estimated_poses)

            # Calculate translation RPE
            rpe_trans_sum = sum(self.calculate_translation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
            rpe_trans = np.sqrt(rpe_trans_sum / m)
            self.rpe_trans.append(rpe_trans)

            # Calculate rotation RPE
            rpe_rot_sum = sum(self.calculate_rotation_rpe(gt, est) for gt, est in zip(self.ground_truth_poses, self.estimated_poses))
            rpe_rot = np.mean(rpe_rot_sum / n)
            self.rpe_rot.append(rpe_rot)

            # Save RPE values to a CSV file
            self.save_to_csv()

        except Exception as e:
            self.get_logger().error('Error running RPE: {}'.format(str(e)))

    def save_to_csv(self):
        csv_filename = 'rpe_values.csv'
        with open(csv_filename, mode='a', newline='') as file:  # Append to the file
            writer = csv.writer(file)
            writer.writerow(['Translation RPE', 'Rotation RPE'])

            for translation_rpe, rotation_rpe in zip(self.rpe_trans, self.rpe_rot):
                writer.writerow([
                    translation_rpe,
                    rotation_rpe,
                ])
            file.close()  # Close the file after writing

        self.get_logger().info('RPE Values appended to {}'.format(csv_filename))


def main(args=None):
    rclpy.init(args=args)

    config_file_path = '/home/asimkumar/asitlor_ws/src/asitlorbot_five_localization/config/ekf.yaml'  # Replace with the actual path

    ekf_updater = EkfUpdater(config_file_path)

    # Define the modification factors for the design of experiment
    initial_factor = 0.01
    final_factor = 100
    step_factor = 10

    # Modify the matrix and run RPE for each factor
    ekf_updater.modify_matrix_and_run_rpe(initial_factor, final_factor, step_factor)

    rclpy.spin(ekf_updater)
    rclpy.shutdown()

if __name__ == "__main__":
    main()














#################################33
# import numpy as np
# from skopt import BayesSearchCV
# from skopt.space import Real
# from skopt.utils import use_named_args

# # Define the search space for Q values (adjust bounds as needed)
# space = [Real(1e-6, 1e2, name=f'q_{i}') for i in range(15)]  # Assuming a 15x15 matrix for state variables

# # Define the function to optimize (the negative of the performance metric)
# @use_named_args(space)
# def objective_function(**params):
#     Q_matrix = np.diag([params[f'q_{i}'] for i in range(15)])  # Assuming a diagonal covariance matrix
#     performance_metric = evaluate_filter(Q_matrix)  # Implement your filtering algorithm
#     return -performance_metric  # Minimize by maximizing the negative performance metric

# # Perform Bayesian optimization
# optimizer = BayesSearchCV(
#     objective_function,
#     search_spaces=space,
#     n_iter=50,  # Adjust the number of iterations as needed
#     random_state=42,
#     n_jobs=-1
# )

# # Run the optimization
# optimizer.fit(None)  # The first argument is not used in this case

# # Get the optimal parameters
# optimal_params = optimizer.best_params_
# optimal_Q_matrix = np.diag([optimal_params[f'q_{i}'] for i in range(15)])

# # Output the tuned process noise covariance matrix
# print("Optimal Q matrix:")
# print(optimal_Q_matrix)
