cmake_minimum_required(VERSION 3.8)
project(xarm_vr_teleop)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build C++ executables
add_executable(robot_control_moveit src/robot_control_moveit.cpp)
add_executable(diagnose_moveit src/diagnose_moveit.cpp)
add_executable(simple_moveit_test src/simple_moveit_test.cpp)
add_executable(vr_teleop_direct src/vr_teleop_direct.cpp)
add_executable(vr_teleop_simple src/vr_teleop_simple.cpp)
add_executable(vr_teleop_tf src/vr_teleop_tf.cpp)
add_executable(vr_teleop_correct_mapping src/vr_teleop_correct_mapping.cpp)
add_executable(test_simple_movement src/test_simple_movement.cpp)
add_executable(vr_teleop_manual_bridge src/vr_teleop_manual_bridge.cpp)

# Dependencies
ament_target_dependencies(robot_control_moveit
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(diagnose_moveit
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

ament_target_dependencies(simple_moveit_test
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
)

ament_target_dependencies(vr_teleop_direct
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(vr_teleop_simple
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(vr_teleop_tf
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(vr_teleop_correct_mapping
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(test_simple_movement
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(vr_teleop_manual_bridge
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install executables
install(TARGETS robot_control_moveit diagnose_moveit simple_moveit_test vr_teleop_direct vr_teleop_simple vr_teleop_tf vr_teleop_correct_mapping test_simple_movement vr_teleop_manual_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/robot_control.py
  scripts/robot_control_moveit.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()