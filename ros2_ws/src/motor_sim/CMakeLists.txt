cmake_minimum_required(VERSION 3.8)
project(motor_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
add_executable(motor_sim_node src/motor_sim_node.cpp)
ament_target_dependencies(motor_sim_node rclcpp std_msgs)

add_executable(torque_pub_node src/torque_pub_node.cpp)
ament_target_dependencies(torque_pub_node rclcpp std_msgs)
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs)

install(TARGETS
  motor_sim_node
  torque_pub_node
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
