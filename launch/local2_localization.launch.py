from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    rmse = Node(
        package="asitlorbot_five_localization",
        executable="fine_tune.py"
    )
    return LaunchDescription([
     
        rmse
    ])