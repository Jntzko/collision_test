cmake_minimum_required(VERSION 2.8.3)
project(collision_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  moveit_ros_planning_interface
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_test src/collision_test.cpp)
target_link_libraries(collision_test ${catkin_LIBRARIES})
add_dependencies(collision_test ${catkin_EXPORTED_TARGETS})
