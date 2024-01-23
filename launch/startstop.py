#!/usr/bin/env python3
import os
import time
import subprocess

def launch_nodes():
    subprocess.Popen(["ros2", "launch", "asitlorbot_five_localization","local_localization.launch.py"])

def main():
    while True:
        # Start nodes
        print("Starting nodes for 10 seconds...")
        launch_nodes()

        # Run nodes for 10 seconds
        time.sleep(10)

        # Stop nodes
        print("Stopping nodes...")
        subprocess.Popen(["ros2", "lifecycle", "set", "static_transform_publisher", "shutdown"])
        subprocess.Popen(["ros2", "lifecycle", "set", "robot_localization", "shutdown"])
        subprocess.Popen(["ros2", "lifecycle", "set", "imu_republisher_py", "shutdown"])
        subprocess.Popen(["ros2", "lifecycle", "set", "rmse", "shutdown"])

        # Wait for 5 seconds before restarting the cycle
        print("Waiting for 5 seconds...")
        time.sleep(5)

if __name__ == "__main__":
    main()
