#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import yaml

# Load the YAML configuration
with open("ekf.yaml", "r") as file:
    ekf_config = yaml.safe_load(file)

# Extract the covariance matrices from the YAML configuration
process_noise_covariance = np.array(ekf_config['ekf_filter_node']['ros__parameters']['process_noise_covariance'])
initial_estimate_covariance = np.array(ekf_config['ekf_filter_node']['ros__parameters']['initial_estimate_covariance'])

# RMSE calculation function
def calculate_rmse(estimated, true):
    return np.sqrt(np.mean(np.square(estimated - true)))

# Objective function for GWO optimization
def objective_function(params):
    # Update the covariance matrices with the parameters from GWO
    ekf_config['ekf_filter_node']['ros__parameters']['process_noise_covariance'] = params[:len(process_noise_covariance)]
    ekf_config['ekf_filter_node']['ros__parameters']['initial_estimate_covariance'] = params[len(process_noise_covariance):]

    # TODO: Run your EKF with the updated parameters and evaluate its performance
    # Return a performance metric that GWO aims to minimize
    # Run your EKF with the updated parameters
    estimated_values = run_ekf_and_get_estimated_values(ekf_config)
    true_values = get_true_values()

    # Calculate RMSE
    rmse = calculate_rmse(estimated_values, true_values)

    # Return the RMSE (performance metric that GWO aims to minimize)
    return rmse

# Grey Wolf Optimization algorithm
def grey_wolf_optimization(obj_function, num_dimensions, num_wolves, num_iterations, lb, ub):
    # Initialization
    positions = np.random.uniform(lb, ub, size=(num_wolves, num_dimensions))

    for iteration in range(num_iterations):
        # Evaluate the objective function for each wolf
        fitness = np.array([obj_function(pos) for pos in positions])

        # Update alpha, beta, and delta positions
        alpha_pos = positions[np.argmin(fitness)]
        beta_pos = positions[np.argsort(fitness)[1]]
        delta_pos = positions[np.argsort(fitness)[2]]

        # Update positions of wolves
        a = 2 - 2 * iteration / num_iterations
        for i in range(num_wolves):
            r1, r2 = np.random.random(size=(2, num_dimensions))
            A1 = 2 * a * r1 - a
            C1 = 2 * r2
            D_alpha = abs(C1 * alpha_pos - positions[i])
            X1 = alpha_pos - A1 * D_alpha

            r1, r2 = np.random.random(size=(2, num_dimensions))
            A2 = 2 * a * r1 - a
            C2 = 2 * r2
            D_beta = abs(C2 * beta_pos - positions[i])
            X2 = beta_pos - A2 * D_beta

            r1, r2 = np.random.random(size=(2, num_dimensions))
            A3 = 2 * a * r1 - a
            C3 = 2 * r2
            D_delta = abs(C3 * delta_pos - positions[i])
            X3 = delta_pos - A3 * D_delta

            positions[i] = (X1 + X2 + X3) / 3.0

        # Clip positions to ensure they are within the search space bounds
        positions = np.clip(positions, lb, ub)

    # Return the best position found (minimized objective)
    best_position = positions[np.argmin(fitness)]
    return best_position

# Number of dimensions (length of concatenated process_noise_covariance and initial_estimate_covariance)
num_dimensions = len(process_noise_covariance) + len(initial_estimate_covariance)

# Number of wolves in the pack
num_wolves = 3

# Number of iterations
num_iterations = 100

# Bounds for each dimension (can be adjusted based on your problem)
lb = np.zeros(num_dimensions)
ub = np.ones(num_dimensions)  # Assuming normalized values between 0 and 1

# Run GWO optimization
optimized_params = grey_wolf_optimization(objective_function, num_dimensions, num_wolves, num_iterations, lb, ub)

# Extract the optimized parameters
optimized_process_noise_covariance = optimized_params[:len(process_noise_covariance)]
optimized_initial_estimate_covariance = optimized_params[len(process_noise_covariance):]

# Update the YAML configuration with the optimized parameters
ekf_config['ekf_filter_node']['ros__parameters']['process_noise_covariance'] = optimized_process_noise_covariance.tolist()
ekf_config['ekf_filter_node']['ros__parameters']['initial_estimate_covariance'] = optimized_initial_estimate_covariance.tolist()

# Save the updated YAML configuration
with open("optimized_ekf_config.yaml", "w") as file:
    yaml.dump(ekf_config, file)
