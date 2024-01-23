from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('database_path', default_value='rtabmap.db'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/rgb/camera_info'),

        # Mapping Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=[{'database_path': LaunchConfiguration('database_path')},
                        {'frame_id': 'map'},
                        {'odom_frame_id': 'odom'},
                        {'subscribe_depth': True},
                        {'subscribe_scan': True}],
            remappings=[('scan', '/laser_controller/out'),
                        ('rgb/image', LaunchConfiguration('rgb_topic')),
                        ('depth/image', LaunchConfiguration('depth_topic')),
                        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                        ('grid_map', '/map')],
        ),

        # Visualization with rtabmapviz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz',
            output='screen',
            arguments=['-d', '$(find rtabmap_ros)/launch/config/rgbd_gui.ini'],
            parameters=[{'subscribe_depth': True},
                        {'subscribe_scan': True},
                        {'frame_id': 'map'}],
            remappings=[('rgb/image', LaunchConfiguration('rgb_topic')),
                        ('depth/image', LaunchConfiguration('depth_topic')),
                        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                        ('scan', '/laser_controller/out'),
                        ('odom', '/odom')],
        )
    ])
