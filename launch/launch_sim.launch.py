import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='asitlorbot_five' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time':'true'}.items()
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name='laser_filter_node',
        output='screen',
        parameters=[{'config_file': os.path.join(get_package_share_directory('asitlorbot_five'), 'my_laser_filter.yaml')}],
        # remappings=[('/scan', '/base_scan')],
        # arguments=["scan_filter_chain"],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("asitlorbot_five"), "config","drive_bot.rviz")]
    )
    imu_filter = Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(get_package_share_directory("asitlorbot_five"), "config", "imu_filter.yaml")],
                arguments=["imu_filter"],
                remappings=[
                    ('/imu/mag', '/imu/out'),
                ]
            )
    imu_filter2 = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
                remappings=[
                    ('/imu/data_raw', '/imu/out'),
                ]
            )
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        imu_filter2
    ])