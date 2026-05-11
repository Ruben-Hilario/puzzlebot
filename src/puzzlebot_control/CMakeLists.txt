cmake_minimum_required(VERSION 3.8)
project(puzzlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(pid_node
  src/pid_node.cpp
  src/control.cpp
)

target_compile_features(pid_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pid_node
  rclcpp 
  geometry_msgs 
  nav_msgs 
  std_srvs
)

# Install executable
install(TARGETS
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)
# Install header files
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

file(GLOB python_scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/nodes/*.py")
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
