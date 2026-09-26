cmake_minimum_required(VERSION 3.20)
project(trajectory_creation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(trajectory_creation_node src/trajectory_creation.cpp)
target_link_libraries(trajectory_creation_node PUBLIC
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  tf2::tf2
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
  control_msgs::control_msgs
  ${OpenCV_LIBRARIES}
)

install(TARGETS
trajectory_creation_node
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
