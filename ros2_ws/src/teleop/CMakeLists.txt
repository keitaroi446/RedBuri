cmake_minimum_required(VERSION 3.8)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(redburi_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(joy_base_node src/joy_base_node.cpp)
ament_target_dependencies(joy_base_node rclcpp sensor_msgs redburi_msgs)

add_executable(joy_arm_joint_node src/joy_arm_joint_node.cpp)
ament_target_dependencies(joy_arm_joint_node control_msgs rclcpp sensor_msgs redburi_msgs std_msgs)

add_executable(joy_arm_cartesian_node src/joy_arm_cartesian_node.cpp)
ament_target_dependencies(joy_arm_cartesian_node geometry_msgs rclcpp sensor_msgs std_msgs)

add_executable(joy_mode_node src/joy_mode_node.cpp)
ament_target_dependencies(joy_mode_node rclcpp sensor_msgs)

add_executable(servo_arm_motor_node src/servo_arm_motor_node.cpp)
ament_target_dependencies(servo_arm_motor_node rclcpp redburi_msgs std_msgs trajectory_msgs)

add_executable(arm_joint_state_estimator_node src/arm_joint_state_estimator_node.cpp)
ament_target_dependencies(arm_joint_state_estimator_node rclcpp redburi_msgs sensor_msgs)

install(TARGETS
  joy_base_node
  joy_arm_joint_node
  joy_arm_cartesian_node
  joy_mode_node
  servo_arm_motor_node
  arm_joint_state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
