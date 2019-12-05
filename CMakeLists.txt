cmake_minimum_required(VERSION 2.8.3)
project(docking_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS 
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf pcl_ros
 DEPENDS
)

add_executable(docking_planner src/docking_planner.cpp)
add_dependencies(docking_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_planner ${catkin_LIBRARIES})

add_executable(path_tester src/path_tester.cpp)
add_dependencies(path_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tester ${catkin_LIBRARIES})



