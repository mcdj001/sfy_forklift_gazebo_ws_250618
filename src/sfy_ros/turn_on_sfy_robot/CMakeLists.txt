cmake_minimum_required(VERSION 2.8.3)
project(turn_on_sfy_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
  apriltag_ros
  sfy_forklift_msgs
  dynamic_reconfigure
)
# find_package(Eigen REQUIRED)

add_compile_options(-g -std=c++11)    #-g -> core dumped

# generate_dynamic_reconfigure_options(
#   cfg/SfyForkliftTestReconfigure.cfg
# )

catkin_package(
  #INCLUDE_DIRS include
  #CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
  #DEPENDS system_lib
)
# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})
add_executable(sfy_robot_node src/sfy_robot.cpp)
target_link_libraries(sfy_robot_node ${catkin_LIBRARIES})


# add_library(turn_on_sfy_robot 
# src/sfy_robot.cpp
# )

# add_dependencies(turn_on_sfy_robot ${PROJECT_NAME}_gencfg)

# add_dependencies(turn_on_sfy_robot ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(turn_on_sfy_robot
# ${EXTERNAL_LIBS}
# ${catkin_LIBRARIES}
# )

# install(DIRECTORY
#   cfg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
 