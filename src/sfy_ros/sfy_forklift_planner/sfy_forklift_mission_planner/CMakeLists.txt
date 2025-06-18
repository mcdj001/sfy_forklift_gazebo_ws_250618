cmake_minimum_required(VERSION 3.0.2)
project(sfy_forklift_mission_planner)

# set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11 -O3)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  message_generation
  move_base
  base_local_planner
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_filters
  cv_bridge
  nav_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  apriltag_ros
  sfy_forklift_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
# generate_dynamic_reconfigure_options(
#   cfg/SfyForkliftLocalPlannerReconfigure.cfg
# )

add_action_files(
  DIRECTORY action
  FILES SfyForkliftMoveBasePlugin.action
) 

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sfy_forklift_mission_planner
   CATKIN_DEPENDS actionlib actionlib_msgs costmap_2d dynamic_reconfigure geometry_msgs move_base roscpp rospy std_msgs tf visualization_msgs pluginlib nav_core tf2 tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(/usr/include)
link_directories(/usr/lib)


add_library(sfy_forklift_mission_planner 
src/mytools.cpp
src/trajectory.cpp
)

target_link_libraries(sfy_forklift_mission_planner
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)  

add_executable(sfy_forklift_path_planner src/sfy_forklift_path_planning.cpp)
target_link_libraries(sfy_forklift_path_planner ${catkin_LIBRARIES} sfy_forklift_mission_planner)
add_dependencies(sfy_forklift_path_planner ${catkin_EXPORTED_TARGETS})

add_executable(sfy_forklift_collect_planner src/sfy_forklift_collect_planning.cpp)
target_link_libraries(sfy_forklift_collect_planner ${catkin_LIBRARIES} sfy_forklift_mission_planner)
add_dependencies(sfy_forklift_collect_planner ${catkin_EXPORTED_TARGETS})

add_executable(sfy_forklift_mission src/sfy_forklift_mission_planning.cpp)
target_link_libraries(sfy_forklift_mission ${catkin_LIBRARIES} sfy_forklift_mission_planner)
add_dependencies(sfy_forklift_mission ${catkin_EXPORTED_TARGETS})

add_executable(sfy_forklift_move_base_plugin_server src/sfy_forklift_move_base_plugin_server.cpp)
target_link_libraries(sfy_forklift_move_base_plugin_server ${catkin_LIBRARIES} sfy_forklift_mission_planner)
add_dependencies(sfy_forklift_move_base_plugin_server ${catkin_EXPORTED_TARGETS})

add_executable(client_test src/client_test.cpp)
target_link_libraries(client_test ${catkin_LIBRARIES} sfy_forklift_mission_planner)
add_dependencies(client_test ${catkin_EXPORTED_TARGETS})



#############
## Install ##
#############
# install(DIRECTORY
#   cfg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
