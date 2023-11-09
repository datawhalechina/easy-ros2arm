cmake_minimum_required(VERSION 3.5)
project(dofbot_moveit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(kdl_parser REQUIRED)
# https://github.com/ros2/orocos_kdl_vendor
find_package(orocos_kdl REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# 机械臂在Rviz中随机运动
add_executable(random_move src/random_move.cpp)
ament_target_dependencies(random_move rclcpp moveit_ros_planning_interface)
#target_link_libraries(random_move ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS
random_move
DESTINATION lib/${PROJECT_NAME})

ament_package()
