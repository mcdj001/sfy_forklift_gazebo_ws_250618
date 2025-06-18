/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-12 12:54:24
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-03-05 14:26:52
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_mission_planner/include/collect_planning/sfy_forklift_collect_planning.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef __SFY_FORKLIFT_COLLECT_PLANNING_H__
#define __SFY_FORKLIFT_COLLECT_PLANNING_H__

#include <ros/ros.h>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <math.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <unordered_map>
#include <limits>
#include <chrono>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "apriltag_ros/AprilTagDetectionArray.h"

#include "path_planning/mytools.h"
#include "sfy_forklift_msgs/ForkliftLocation.h"
#include "sfy_forklift_msgs/ForkliftFlag.h"
#include "sfy_forklift_msgs/ForkliftPurePursuit.h"
#include "sfy_forklift_msgs/ForkliftInfo.h"
#include "sfy_forklift_msgs/ForkliftInitImu.h"

class SfyForkliftCollectPlanning{
public:
    double frequency_;
    string desired_path_frame_id_;
    string pp_controller_csv_path_;
    string line_csv_path_;

    SfyForkliftCollectPlanning();
    ~SfyForkliftCollectPlanning();
    void control();

private:
    SfyMissionPlanner::MyTools* mytools;
    
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry odom_;

    ros::Subscriber globalpath_sub_;
    nav_msgs::Path global_path_;

    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist cmd_vel_;

    ros::Subscriber forklift_location_odom_sub_;
    sfy_forklift_msgs::ForkliftLocation forklift_location_odom_;

    ros::Subscriber move_base_goal_sub_;
    bool move_base_simple_goal_bool_ = false;

    ros::Subscriber local_plan_goal_reached_sub_;
    bool local_plan_goal_reached_;
 
    ros::Subscriber collect_pp_control_sub_;
    sfy_forklift_msgs::ForkliftPurePursuit collect_pp_control_;

    ros::Subscriber forklift_location_ekf_sub;
    geometry_msgs::PoseWithCovarianceStamped forklift_location_ekf_;

    ros::Subscriber Apriltag_Positon_Sub;
    bool use_apriltag;

    ros::Subscriber forklift_info_sub_;
    sfy_forklift_msgs::ForkliftInfo forklift_info_;

    ros::Subscriber  forklift_init_imu_sub_;
    sfy_forklift_msgs::ForkliftInitImu forklift_init_imu_;

    std::ofstream file_pp_controller;
    std::ofstream file_pid_controller;
    std::ofstream file_line;
    bool is_collect_pp_;
    bool is_collect_line_;

    double last_global_path_yaw_;

    void collectDataToCsv();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& globalPathMsg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVelMsg);
    void locationOdomCallback(const sfy_forklift_msgs::ForkliftLocation::ConstPtr& locationOdomMsg);
    void moveBaseGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void localPlanGoalCB(const sfy_forklift_msgs::ForkliftFlag::ConstPtr& goalMsg);
    void collectPPControlCB(const sfy_forklift_msgs::ForkliftPurePursuit::ConstPtr& Msg);
    void locationEkfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Msg);
    void ApriltagPositonCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr&  Msg);
    void forkliftInfoCallback(const sfy_forklift_msgs::ForkliftInfo::ConstPtr&  Msg);
    void forkliftInitImuCB(const sfy_forklift_msgs::ForkliftInitImu::ConstPtr&  Msg);
       
};


#endif