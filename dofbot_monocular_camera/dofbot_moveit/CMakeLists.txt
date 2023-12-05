cmake_minimum_required(VERSION 3.8)
project(dofbot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

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

add_executable(randomMove src/random_move.cpp)
ament_target_dependencies(randomMove moveit_ros_planning_interface rclcpp)

add_executable(motionPlaning src/motion_planing.cpp)
ament_target_dependencies(motionPlaning 
                    rclcpp 
                    moveit_ros_planning_interface 
                    geometry_msgs)

add_executable(collisionChecking src/attached_object.cpp)
ament_target_dependencies(collisionChecking
                          rclcpp
                          moveit_ros_planning_interface
                          moveit_msgs
                          shape_msgs
                          std_msgs
                          moveit_visual_tools)

add_executable(cartesianRoute src/cartesian_route.cpp)
ament_target_dependencies(cartesianRoute
                          rclcpp
                          moveit_ros_planning_interface
                          moveit_msgs
                          shape_msgs
                          std_msgs
                          moveit_visual_tools)

install(TARGETS
      randomMove
      motionPlaning
      collisionChecking
      cartesianRoute
      DESTINATION lib/${PROJECT_NAME})

ament_package()
