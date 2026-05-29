cmake_minimum_required(VERSION 3.8)
project(puzzlebot_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(odom_node
  src/localisation_node.cpp
  src/localisation.cpp
)

add_executable(kalman_node
  src/kalman_node.cpp
  src/kalman.cpp
)

add_executable(mcl_node
  src/montecarlo_node.cpp
  src/montecarlo.cpp
)

add_executable(route_node
  src/route_node.cpp
  src/utils.cpp
)

add_executable(map_publisher
  src/map_publisher.cpp
  src/utils.cpp
)

target_compile_features(odom_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(kalman_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(mcl_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(route_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(map_publisher PUBLIC c_std_99 cxx_std_17)

target_include_directories(odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(kalman_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mcl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(route_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(map_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odom_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(kalman_node
  rclcpp
  Eigen3
  std_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(mcl_node
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  cv_bridge
)

ament_target_dependencies(route_node
  rclcpp
  nav_msgs
  geometry_msgs 
)

ament_target_dependencies(map_publisher
  rclcpp
  nav_msgs
)

install(TARGETS
  odom_node
  kalman_node
  mcl_node
  route_node
  map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

file(GLOB python_scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
install(PROGRAMS ${python_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
