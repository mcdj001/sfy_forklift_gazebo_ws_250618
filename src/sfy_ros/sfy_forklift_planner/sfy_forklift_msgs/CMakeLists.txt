cmake_minimum_required(VERSION 3.5)
project(sfy_forklift_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib
  message_generation
  message_runtime
  roscpp
  rospy
)

add_message_files(
  FILES
  ForkliftApriltagInfo.msg
  ForkliftCmdVel.msg
  ForkliftFlag.msg
  ForkliftInfo.msg
  ForkliftInitImu.msg
  ForkliftInitInfo.msg
  ForkliftLocation.msg
  ForkliftMissionTargetPoint.msg
  ForkliftPathPlanningResult.msg
  ForkliftPathPlanningStatus.msg
  ForkliftPathTrackingStatus.msg
  ForkliftPose.msg
  ForkliftPurePursuit.msg
  ForkliftStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_generation message_runtime
)


# add_service_files(
#   DIRECTORY srv
#   FILES
# )

# add_action_files(
#   DIRECTORY action
#   FILES
# )

