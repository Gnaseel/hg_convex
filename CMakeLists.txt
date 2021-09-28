cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(register src/grad_register.cpp)
add_dependencies(register ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(register
  ${catkin_LIBRARIES}
)
add_executable(convex src/convex.cpp)
add_dependencies(convex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(convex
  ${catkin_LIBRARIES}
)