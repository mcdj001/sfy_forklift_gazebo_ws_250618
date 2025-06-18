/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-10 13:47:41
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-05-26 10:23:44
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_global_planner/include/sfy_forklift_global_planner/sfy_forklift_global_planner_ros.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef SFY_FORKLIFT_GLOBAL_PLANNER_ROS_H_
#define SFY_FORKLIFT_GLOBAL_PLANNER_ROS_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include <map>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#include <limits>

#include "sfy_forklift_msgs/ForkliftPathPlanningResult.h"
using namespace std;

namespace sfy_forklift_global_planner {

 class SfyForkliftGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        SfyForkliftGlobalPlanner();
        ~SfyForkliftGlobalPlanner();

        SfyForkliftGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan
                );

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void getCmdCB(const geometry_msgs::Twist&);
        void calError(const ros::TimerEvent&);
        

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_globalpath_;
        ros::Subscriber sub_odom_, sub_get_path_, sub_goal_, sub_cmd_;
        nav_msgs::Odometry odom_;
        nav_msgs::Path odom_path_;
        nav_msgs::Path desired_path_;
        ros::Timer errtimer_;
        int controller_freq_;
        bool goal_received_;           // 是否接收到goal
        double linear_vel_;
        double angular_vel_;

        int min_idx_; //nearest point
        double pathLength_;

        // makePlan()
        double waypointsDist_;  //minimum distance between points of path
        tf::TransformListener tf_listener_;
        
        double polyeval(Eigen::VectorXd coeffs, double x);        
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
        unsigned int idx;

        ros::Subscriber forklift_path_planning_result_sub_; 
        sfy_forklift_msgs::ForkliftPathPlanningResult forklift_path_planning_result_;
        bool is_forklift_path_planning_result_sub_;
        void forkliftPathPlanningResultCB(const sfy_forklift_msgs::ForkliftPathPlanningResult::ConstPtr& Msg);
        

        void checkSubscriberStatus();
 };

};

# endif