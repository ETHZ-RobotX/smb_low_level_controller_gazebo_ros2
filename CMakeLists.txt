cmake_minimum_required(VERSION 3.8)
project(smb_low_level_controller_gazebo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include
  /usr/include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(smb_low_level_controller_gazebo_ros2_node src/smb_low_level_controller_gazebo_ros2_node.cpp)
target_include_directories(smb_low_level_controller_gazebo_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(smb_low_level_controller_gazebo_ros2_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(smb_low_level_controller_gazebo_ros2_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  pcl_ros 
  pcl_conversions
)

target_link_libraries(smb_low_level_controller_gazebo_ros2_node ${GZ-MSGS_LIBRARIES} ${GZ-TRANSPORT_LIBRARIES})

install(TARGETS smb_low_level_controller_gazebo_ros2_node
  DESTINATION lib/${PROJECT_NAME})

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
