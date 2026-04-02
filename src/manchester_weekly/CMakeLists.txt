cmake_minimum_required(VERSION 3.8)
project(manchester_weekly)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

add_executable(static_tf_week_1
  src/week1/static_tf.cpp
)
add_executable(dynamic_tf_week_1
  src/week1/dyn_tf.cpp
)

add_executable(testing_node 
 src/utils/testing_node.cpp
 src/utils/testing.cpp
)

add_executable(mini_challenge_week_1
  src/week1/mini_challenge.cpp
)

target_compile_features(static_tf_week_1 PUBLIC c_std_99 cxx_std_17)
target_compile_features(dynamic_tf_week_1 PUBLIC c_std_99 cxx_std_17)
target_compile_features(mini_challenge_week_1 PUBLIC c_std_99 cxx_std_17)
target_compile_features(testing_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(static_tf_week_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(dynamic_tf_week_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mini_challenge_week_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(testing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(static_tf_week_1
  rclcpp
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(dynamic_tf_week_1
  rclcpp
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(mini_challenge_week_1
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(testing_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  static_tf_week_1
  dynamic_tf_week_1
  mini_challenge_week_1
  testing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
 DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY rviz/
 DESTINATION share/${PROJECT_NAME}/rviz
)

file(GLOB python_scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
install(PROGRAMS ${python_scripts}
  DESTINATION lib/${PROJECT_NAME}
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
