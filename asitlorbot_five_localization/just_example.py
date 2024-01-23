#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry, "asitlorbot_five_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "asitlorbot_five_controller/odom_kalman", 10)
        
        # Initially the robot has no idea about how fast is going
        self.mean_ = 0.0
        self.variance_ = 1000.0

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

        # Store the messages - only for the orientation
        self.imu_angular_z_ = 0.0

        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()


    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpdate()

        # Update for the next iteration
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Update and publish the filtered odom message
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


    def imuCallback(self, imu):
        # Store the measurement update
        self.imu_angular_z_ = imu.angular_velocity.z


    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) \
                   / (self.variance_ + self.measurement_variance_)
                     
        self.variance_ = (self.variance_ * self.measurement_variance_) \
                       / (self.variance_ + self.measurement_variance_)


    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_


def main():
    rclpy.init()

    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.15",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("asitlorbot_five_localization"), "config", "ekf.yaml")],
    )
    
    lidar_republisher_py = Node(
        package="asitlorbot_five_localization",
        executable="lidar_republisher.py"
    )
   
    imu_republisher_py = Node(
        package="asitlorbot_five_localization",
        executable="imu_republisher.py"
    )
    rmse = Node(
        package="asitlorbot_five_localization",
        executable="rmse.py"
    )
    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        # lidar_republisher_py,
        imu_republisher_py,
        rmse
        
    ])

#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        qos_profile = QoSProfile(depth=10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer_ekf = self.create_timer(2, self.timer_callback_ekf)
        self.timer_base = self.create_timer(2, self.timer_callback_base)

        self.get_logger().info('Starting {}'.format(self.get_name()))
        self.i = 0

    def calculate_pose_error(self, trans):
        # Extract pose information
        pose = Pose()
        pose.position = Point(x=trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z)
        pose.orientation = Quaternion(x=trans.transform.rotation.x, y=trans.transform.rotation.y,
                                      z=trans.transform.rotation.z, w=trans.transform.rotation.w)
        return pose

    def timer_callback_ekf(self):
        try:
            trans_ekf = self.buffer.lookup_transform('base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0))
            pose_ekf = self.calculate_pose_error(trans_ekf)
            self.get_logger().info('{}: Pose Error EKF: {}'.format(self.i, pose_ekf))
        except Exception as e:
            self.get_logger().error('Error getting EKF transform: {}'.format(str(e)))

    def timer_callback_base(self):
        try:
            trans_base = self.buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time(seconds=0))
            pose_base = self.calculate_pose_error(trans_base)
            self.get_logger().info('{}: Pose Error Base: {}'.format(self.i, pose_base))
        except Exception as e:
            self.get_logger().error('Error getting Base transform: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5
        self.target_angle = math.radians(45)  # Target angle in radians
        self.current_angle = 0
        self.turning_phase = True  # True: Turning, False: Moving in a circular path

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

    def timer_callback(self):
        twist_msg = Twist()

        if self.turning_phase:
            twist_msg.angular.z = self.angular_velocity
            self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
            if self.current_angle >= self.target_angle:
                # Switch to circular motion phase
                self.turning_phase = False
                self.current_angle = 0
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update the paths
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

        # Update path_base_link
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

        # Update path_base_footprint_ekf
        # Modify this part based on the transformation between base_footprint_ekf and odom
        pose.header.frame_id = 'base_footprint_ekf'
        self.path_base_footprint_ekf.poses.append(pose)
        self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

        # Update path_base_footprint_noisy
        # Modify this part based on the transformation between base_footprint_noisy and odom
        pose.header.frame_id = 'base_footprint_noisy'
        self.path_base_footprint_noisy.poses.append(pose)
        self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

def main(args=None):
    rclpy.init(args=args)
    circular_motion_node = CircularMotionNode()
    rclpy.spin(circular_motion_node)
    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5
        self.target_angle = math.radians(45)  # Target angle in radians
        self.current_angle = 0
        self.turning_phase = True  # True: Turning, False: Moving in a circular path

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

    def timer_callback(self):
        twist_msg = Twist()

        if self.turning_phase:
            twist_msg.angular.z = self.angular_velocity
            self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
            if self.current_angle >= self.target_angle:
                # Switch to circular motion phase
                self.turning_phase = False
                self.current_angle = 0
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update the paths
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

        # Update path_base_link
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

        # Update path_base_footprint_ekf
        # Modify this part based on the transformation between base_footprint_ekf and odom
        pose.header.frame_id = 'base_footprint_ekf'
        self.path_base_footprint_ekf.poses.append(pose)
        self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

        # Update path_base_footprint_noisy
        # Modify this part based on the transformation between base_footprint_noisy and odom
        pose.header.frame_id = 'base_footprint_noisy'
        self.path_base_footprint_noisy.poses.append(pose)
        self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

def main(args=None):
    rclpy.init(args=args)
    circular_motion_node = CircularMotionNode()
    rclpy.spin(circular_motion_node)
    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5
        self.target_angle = math.radians(45)  # Target angle in radians
        self.current_angle = 0
        self.turning_phase = True  # True: Turning, False: Moving in a circular path

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

    def timer_callback(self):
        twist_msg = Twist()

        if self.turning_phase:
            twist_msg.angular.z = self.angular_velocity
            self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
            if self.current_angle >= self.target_angle:
                # Switch to circular motion phase
                self.turning_phase = False
                self.current_angle = 0
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update the paths
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

        # Update path_base_link
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

        # Update path_base_footprint_ekf
        # Modify this part based on the transformation between base_footprint_ekf and odom
        pose.header.frame_id = 'base_footprint_ekf'
        self.path_base_footprint_ekf.poses.append(pose)
        self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

        # Update path_base_footprint_noisy
        # Modify this part based on the transformation between base_footprint_noisy and odom
        pose.header.frame_id = 'base_footprint_noisy'
        self.path_base_footprint_noisy.poses.append(pose)
        self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

def main(args=None):
    rclpy.init(args=args)
    circular_motion_node = CircularMotionNode()
    rclpy.spin(circular_motion_node)
    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class CircularMotionNode(Node):
    def __init__(self):
        super().__init__('circular_motion_node')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Publishers for paths in different frames
        self.publisher_path_base_link = self.create_publisher(Path, 'path_base_footprint', 10)
        self.publisher_path_base_footprint_ekf = self.create_publisher(Path, 'path_base_footprint_ekf', 10)
        self.publisher_path_base_footprint_noisy = self.create_publisher(Path, 'path_base_footprint_noisy', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5
        self.target_angle = math.radians(45)  # Target angle in radians
        self.current_angle = 0
        self.turning_phase = True  # True: Turning, False: Moving in a circular path

        # Paths for different frames
        self.path_base_link = Path()
        self.path_base_link.header.frame_id = 'odom'

        self.path_base_footprint_ekf = Path()
        self.path_base_footprint_ekf.header.frame_id = 'odom'

        self.path_base_footprint_noisy = Path()
        self.path_base_footprint_noisy.header.frame_id = 'odom'

    def timer_callback(self):
        twist_msg = Twist()

        if self.turning_phase:
            twist_msg.angular.z = self.angular_velocity
            self.current_angle += abs(self.angular_velocity) * 0.1  # 0.1 is the timer period
            if self.current_angle >= self.target_angle:
                # Switch to circular motion phase
                self.turning_phase = False
                self.current_angle = 0
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

        # Publish cmd_vel
        self.publisher_cmd_vel.publish(twist_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = twist_msg
        self.publisher_odom.publish(odom_msg)

        # Update the paths
        pose = PoseStamped()
        pose.header.stamp = odom_msg.header.stamp
        pose.header.frame_id = 'base_link'  # Use 'base_link' frame for PoseStamped

        # Update path_base_link
        pose.pose.position.x = odom_msg.pose.pose.position.x
        pose.pose.position.y = odom_msg.pose.pose.position.y
        pose.pose.orientation = odom_msg.pose.pose.orientation
        self.path_base_link.poses.append(pose)
        self.publisher_path_base_link.publish(self.path_base_link)

        # Update path_base_footprint_ekf
        # Modify this part based on the transformation between base_footprint_ekf and odom
        pose.header.frame_id = 'base_footprint_ekf'
        self.path_base_footprint_ekf.poses.append(pose)
        self.publisher_path_base_footprint_ekf.publish(self.path_base_footprint_ekf)

        # Update path_base_footprint_noisy
        # Modify this part based on the transformation between base_footprint_noisy and odom
        pose.header.frame_id = 'base_footprint_noisy'
        self.path_base_footprint_noisy.poses.append(pose)
        self.publisher_path_base_footprint_noisy.publish(self.path_base_footprint_noisy)

def main(args=None):
    rclpy.init(args=args)
    circular_motion_node = CircularMotionNode()
    rclpy.spin(circular_motion_node)
    circular_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

############
    #   process_noise_covariance: [0.05,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.05,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.06,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.03,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.03,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.06,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,   0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.04,   0.0,    0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
    #                              0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]
############
#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        qos_profile = QoSProfile(depth=10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer_ekf = self.create_timer(2, self.timer_callback_ekf)
        self.timer_base = self.create_timer(2, self.timer_callback_base)

        self.get_logger().info('Starting {}'.format(self.get_name()))
        self.i = 0

    def calculate_pose_error(self, trans):
        # Extract pose information
        pose = Pose()
        pose.position = Point(x=trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z)
        pose.orientation = Quaternion(x=trans.transform.rotation.x, y=trans.transform.rotation.y,
                                      z=trans.transform.rotation.z, w=trans.transform.rotation.w)
        return pose

    def timer_callback_ekf(self):
        try:
            trans_ekf = self.buffer.lookup_transform('base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0))
            pose_ekf = self.calculate_pose_error(trans_ekf)
            self.get_logger().info('{}: Pose Error EKF: {}'.format(self.i, pose_ekf))
        except Exception as e:
            self.get_logger().error('Error getting EKF transform: {}'.format(str(e)))

    def timer_callback_base(self):
        try:
            trans_base = self.buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time(seconds=0))
            pose_base = self.calculate_pose_error(trans_base)
            self.get_logger().info('{}: Pose Error Base: {}'.format(self.i, pose_base))
        except Exception as e:
            self.get_logger().error('Error getting Base transform: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
######################################################################################################
#!/usr/bin/env python3
import rclpy
import tf2_ros
import csv
from geometry_msgs.msg import Pose, Quaternion, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        qos_profile = QoSProfile(depth=10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.timer = self.create_timer(2, self.timer_callback)

        self.get_logger().info('Starting {}'.format(self.get_name()))
        self.i = 0
        self.pose_errors = []

    def calculate_pose_error(self, trans):
        # Extract pose information
        pose = Pose()
        pose.position = Point(x=trans.transform.translation.x, y=trans.transform.translation.y, z=trans.transform.translation.z)
        pose.orientation = Quaternion(x=trans.transform.rotation.x, y=trans.transform.rotation.y,
                                      z=trans.transform.rotation.z, w=trans.transform.rotation.w)
        return pose

    def timer_callback(self):
        try:
            trans_ekf = self.buffer.lookup_transform('base_footprint_ekf', 'odom', rclpy.time.Time(seconds=0))
            pose_error = self.calculate_pose_error(trans_ekf)

            # Subtract the pose of base_footprint from the pose of base_footprint_ekf
            trans_base = self.buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time(seconds=0))
            pose_base = self.calculate_pose_error(trans_base)

            pose_error_base_ekf = Pose()
            pose_error_base_ekf.position.x = pose_error.position.x - pose_base.position.x
            pose_error_base_ekf.position.y = pose_error.position.y - pose_base.position.y
            pose_error_base_ekf.position.z = pose_error.position.z - pose_base.position.z
            pose_error_base_ekf.orientation.x = pose_error.orientation.x - pose_base.orientation.x
            pose_error_base_ekf.orientation.y = pose_error.orientation.y - pose_base.orientation.y
            pose_error_base_ekf.orientation.z = pose_error.orientation.z - pose_base.orientation.z
            pose_error_base_ekf.orientation.w = pose_error.orientation.w - pose_base.orientation.w

            # Append pose_error_base_ekf to the list
            self.pose_errors.append(pose_error_base_ekf)
            self.get_logger().info('{}: Pose Error Base: {}'.format(self.i, pose_base))
            self.get_logger().info('{}: Pose Error EKF: {}'.format(self.i, pose_error))
            self.get_logger().info('{}: Pose Error Between base_footprint and base_footprint_ekf: {}'.format(self.i, pose_error_base_ekf))

            # If 50 pose errors are collected, save them to a CSV file and exit
            if len(self.pose_errors) == 20:
                self.save_to_csv()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Error getting transforms: {}'.format(str(e)))

    def save_to_csv(self):
        csv_filename = 'pose_errors.csv'
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y', 'Z', 'Quaternion_X', 'Quaternion_Y', 'Quaternion_Z', 'Quaternion_W'])

            for pose_error in self.pose_errors:
                writer.writerow([
                    pose_error.position.x, pose_error.position.y, pose_error.position.z,
                    pose_error.orientation.x, pose_error.orientation.y, pose_error.orientation.z, pose_error.orientation.w
                ])
        self.get_logger().info('Pose errors saved to {}'.format(csv_filename))

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()

if __name__ == "__main__":
    main()
####################################333333#######################################################