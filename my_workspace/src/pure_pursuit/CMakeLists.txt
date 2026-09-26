cmake_minimum_required(VERSION 3.20)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# 2. Crea l'eseguibile
add_executable(pure_pursuit_node src/pure_pursuit.cpp)

# 3. Collega le directory e le librerie di OpenCV (Non è un pacchetto ament)
target_include_directories(pure_pursuit_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(pure_pursuit_node PUBLIC
  ${OpenCV_LIBRARIES}
)

target_link_libraries(pure_pursuit_node PUBLIC
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  tf2::tf2
  geometry_msgs::geometry_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
  std_msgs::std_msgs
)

# 5. Regola di installazione FONDAMENTALE per poter fare 'ros2 run'
install(TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
