cmake_minimum_required(VERSION 3.5)
project(asitlorbot_five_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  asitlorbot_five_controller/s.py
  asitlorbot_five_controller/noisy_controller.py
  asitlorbot_five_controller/circular_motion.py
  asitlorbot_five_controller/straight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()