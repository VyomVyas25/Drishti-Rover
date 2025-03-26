cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
# find_package(cv_bridge)
# find_package(opencv2)

include_directories(include)

# add_executable(tsmNode src/ttlsim.cpp)
# ament_target_dependencies(tsmNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(tb3Node src/first_node.cpp)
ament_target_dependencies(tb3Node rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(ps4Node src/ps4.cpp)
ament_target_dependencies(ps4Node rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(pstryNode src/ps4_try.cpp)
ament_target_dependencies(pstryNode rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(xbox_node src/xbox_node.cpp)
ament_target_dependencies(xbox_node rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(autoNode src/autonomous.cpp)
ament_target_dependencies(autoNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

# add_executable(t2Node src/try2.cpp)
# ament_target_dependencies(t2Node rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(finalNode src/g2g_obs_avoidance_software.cpp)
ament_target_dependencies(finalNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

add_executable(gpsNode src/gps.cpp)
ament_target_dependencies(gpsNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nmea_msgs geometry_msgs std_msgs)

add_executable(dNode src/g2g_obs_avoidance_hardware.cpp)
ament_target_dependencies(dNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs geometry_msgs std_msgs)

add_executable(armNode src/arm.cpp)
ament_target_dependencies(armNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nmea_msgs geometry_msgs std_msgs)

 add_executable(ircNode src/autex.cpp)
 ament_target_dependencies(ircNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)
 add_executable(IrcNode src/autex_hardware.cpp)
 ament_target_dependencies(IrcNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs geometry_msgs std_msgs)

 add_executable(CamNode src/3d_cam.cpp)
 ament_target_dependencies(CamNode rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)
 add_executable(combine src/combined_drive_arm.cpp)
 ament_target_dependencies(combine rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

 add_executable(lidar src/lidar.cpp)
 ament_target_dependencies(lidar rclcpp turtlesim tf2_geometry_msgs tf2 sensor_msgs nav_msgs geometry_msgs std_msgs)

install(TARGETS
  #subNode
  xbox_node
  finalNode
  #tsmNode
  tb3Node
  ps4Node
  pstryNode
  autoNode
  #t2Node
  gpsNode
  dNode
  armNode
  ircNode
  IrcNode
  CamNode
  combine
  lidar
  DESTINATION lib/${PROJECT_NAME})


# uncomment the followinps4g section in order to fill in
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

ament_package()
