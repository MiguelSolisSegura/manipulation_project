cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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

add_executable(pick_and_place
    src/pick_and_place.cpp)

ament_target_dependencies(pick_and_place
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	geometric_shapes
	moveit_msgs)

install(TARGETS 
	pick_and_place
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME})

ament_package()