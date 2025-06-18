cmake_minimum_required(VERSION 3.5)
project(sfy_motion_controller)

add_compile_definitions(HAVE_CSTDDEF HAVE_STDDEF_H)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF) 


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
  cmake_modules
  tf
  tf2
  visualization_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
# include_directories(
#         ${PYTHON_INCLUDE_DIRS}
#         ${Eigen3_INCLUDE_DIRS} 
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories( ${catkin_INCLUDE_DIRS} include )
include_directories(/usr/include)
link_directories(/usr/lib)


# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/MPCPlanner.cfg
# )

catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

set(SFY_Motion_Controller_HEADERS
    include/constants.h
    include/mytools.h
    include/mypath.h
    include/cubic_spline.hpp
    include/trajectory.h
    include/pure_pursuit.h
    include/lqr_dynamic_control.h
    include/lqr_dynamic_node.h
    include/lqr_kinematic_control.h
    include/lqr_kinematic_node.h
    include/track_mpc_node.h
    include/track_mpc.h
)

set(SFY_Motion_Controller_SOURCES
    src/mytools.cpp
    src/mypath.cpp
    src/trajectory.cpp
    src/lqr_dynamic_control.cpp
    src/lqr_kinematic_control.cpp
)

add_library(SFY_Motion_Controller_LIBRARIES ${SFY_Motion_Controller_HEADERS} ${SFY_Motion_Controller_SOURCES})
target_link_libraries(SFY_Motion_Controller_LIBRARIES  ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})


link_directories(
  ${catkin_LIB_DIRS}
)

add_executable(path_planning src/path_planning.cpp)
add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES)

add_executable(Pure_Pursuit_Node src/pure_pursuit.cpp)
add_dependencies(Pure_Pursuit_Node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Pure_Pursuit_Node ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES)

add_executable(LQR_Dynamic_Node src/lqr_dynamic_node.cpp)
add_dependencies(LQR_Dynamic_Node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LQR_Dynamic_Node ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES)

add_executable(LQR_Kinematic_Node src/lqr_kinematic_node.cpp)
add_dependencies(LQR_Kinematic_Node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LQR_Kinematic_Node ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES)

add_executable(tracking_mpc  src/track_mpc_node.cpp src/track_mpc.cpp )
add_dependencies(tracking_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_mpc ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES ipopt)

add_executable(path_trajectory_generation  src/path_trajectory_generation.cpp )
add_dependencies(path_trajectory_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_trajectory_generation ${catkin_LIBRARIES} SFY_Motion_Controller_LIBRARIES  )


#############
## Example ##
#############
if(BUILD_EXAMPLE)
    ADD_EXECUTABLE( CppAD_started example/CppAD_started.cpp )
    TARGET_LINK_LIBRARIES(CppAD_started)
    
    ADD_EXECUTABLE( CppAD_Ipopt example/CppAD_Ipopt.cpp )
    TARGET_LINK_LIBRARIES(CppAD_Ipopt ipopt)
endif(BUILD_EXAMPLE)