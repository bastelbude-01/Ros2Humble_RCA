cmake_minimum_required(VERSION 3.8)
project(rca_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_executable(moveit_interface src/moveit_interface.cpp)
ament_target_dependencies(moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(demo_lifecycle_node src/lifecycle_node.cpp)
ament_target_dependencies(demo_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

install(TARGETS
  moveit_interface
  demo_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
