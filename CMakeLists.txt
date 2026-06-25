cmake_minimum_required(VERSION 3.8)
project(prob_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(pf_node src/pf_node.cpp)
ament_target_dependencies(pf_node rclcpp nav_msgs geometry_msgs visualization_msgs)

add_executable(s_curve_node src/s_curve_node.cpp)
ament_target_dependencies(s_curve_node rclcpp geometry_msgs)

install(TARGETS
  kf_node
  ekf_node
  pf_node
  s_curve_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
