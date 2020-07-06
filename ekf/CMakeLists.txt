cmake_minimum_required(VERSION 2.8.3)
project(ekf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf
    nav_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf nav_msgs  geometry_msgs sensor_msgs std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_node src/base.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})
