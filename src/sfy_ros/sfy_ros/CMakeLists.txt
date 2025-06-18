cmake_minimum_required(VERSION 3.5)
project(sfy_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11 -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# options for build configuration
# option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF) 

find_package(catkin REQUIRED COMPONENTS
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
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS} include)
include_directories(/usr/include)
link_directories(/usr/lib)


# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/MPCPlanner.cfg
# )


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sfy_ros
   CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs move_base roscpp rospy std_msgs tf visualization_msgs pluginlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( sfy_ros 
  src/grid_map.cpp 
  src/raycast.cpp
  )
target_link_libraries(sfy_ros
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)  

add_executable(gridmap_node src/node.cpp)
target_link_libraries(gridmap_node ${catkin_LIBRARIES} sfy_ros)
add_dependencies(gridmap_node ${catkin_EXPORTED_TARGETS})

