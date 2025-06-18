/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-10 13:47:28
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 16:14:05
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_global_planner/src/sfy_forklift_global_planner_ros.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include <pluginlib/class_list_macros.h>
#include "sfy_forklift_global_planner/sfy_forklift_global_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(sfy_forklift_global_planner::SfyForkliftGlobalPlanner, nav_core::BaseGlobalPlanner)


namespace sfy_forklift_global_planner {

    SfyForkliftGlobalPlanner::SfyForkliftGlobalPlanner(){
        sub_odom_   = nh_.subscribe("/odom", 1, &SfyForkliftGlobalPlanner::odomCB, this);
        sub_get_path_   = nh_.subscribe( "desired_path", 1, &SfyForkliftGlobalPlanner::desiredPathCB, this);  
        sub_goal_   = nh_.subscribe( "/move_base_simple/goal", 1, &SfyForkliftGlobalPlanner::goalCB, this); 
        sub_cmd_   = nh_.subscribe("/cmd_vel", 5, &SfyForkliftGlobalPlanner::getCmdCB, this);
        controller_freq_ = 10;
        errtimer_ = nh_.createTimer(ros::Duration((1.0)/controller_freq_), &SfyForkliftGlobalPlanner::calError, this);
        pub_globalpath_  = nh_.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1);

        forklift_path_planning_result_sub_ = nh_.subscribe("/forklift_path_planning_result", 1, 
                &SfyForkliftGlobalPlanner::forkliftPathPlanningResultCB, this);
                
        min_idx_ = 0;
        pathLength_ = 3;
        waypointsDist_ = 0;
        goal_received_ = false;
    }
    
    SfyForkliftGlobalPlanner::~SfyForkliftGlobalPlanner(){

    }

    SfyForkliftGlobalPlanner::SfyForkliftGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void SfyForkliftGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {

    }   

    bool SfyForkliftGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
    {   
            plan.clear();
            // if(is_forklift_path_planning_result_sub_ && forklift_path_planning_result_.is_start_control){
            nav_msgs::Path global_path = nav_msgs::Path();  
            geometry_msgs::PoseStamped tempPose;
            nav_msgs::Odometry odom = odom_; 
            
            try
            {
                double total_length = 0.0;
                double gap_x = desired_path_.poses[1].pose.position.x - desired_path_.poses[0].pose.position.x;
                double gap_y = desired_path_.poses[1].pose.position.y - desired_path_.poses[0].pose.position.y;
                waypointsDist_ = sqrt(gap_x*gap_x + gap_y*gap_y); 

                int min_val = 100;      
                int N = desired_path_.poses.size(); // Number of waypoints        
                const double px = odom.pose.pose.position.x; 
                const double py = odom.pose.pose.position.y;
                const double ptheta = odom.pose.pose.position.y;

                double dx, dy; // difference distance
                double pre_yaw = 0;
                double roll, pitch, yaw = 0;

                for(int i = min_idx_; i < N; i++) 
                {
                    dx = desired_path_.poses[i].pose.position.x - px;
                    dy = desired_path_.poses[i].pose.position.y - py;
                            
                    tf::Quaternion q(
                        desired_path_.poses[i].pose.orientation.x,
                        desired_path_.poses[i].pose.orientation.y,
                        desired_path_.poses[i].pose.orientation.z,
                        desired_path_.poses[i].pose.orientation.w);
                    tf::Matrix3x3 m(q);
                    m.getRPY(roll, pitch, yaw);

                    if(abs(pre_yaw - yaw) > 5)
                        pre_yaw = yaw;
            
                    if(min_val > sqrt(dx*dx + dy*dy)   && abs(i - min_idx_) < 50)   //  && abs(i - min_idx_) < 50
                    {
                        min_val = sqrt(dx*dx + dy*dy);
                        min_idx_ = i;
                    }
                }

                // 发布当前叉车位置到终点的全局路径
                for(int i = min_idx_; i < N ; i++)
                {
                    // if(total_length > pathLength_) break;

                    tf_listener_.transformPose("map", ros::Time(0) , 
                                                    desired_path_.poses[i], "map", tempPose);                     
                    global_path.poses.push_back(tempPose);                          
                    total_length = total_length + waypointsDist_; 
                    
                    //cout << " tempPose: " <<  tempPose.pose.position.x << ", " <<  tempPose.pose.position.y << endl; 
                    plan.push_back(tempPose);          
                }

                // cout << "global_path.poses.size(): " << global_path.poses.size() << endl;
                // if(global_path.poses.size() >= pathLength_ )
                // {
                        // odom_path_ = global_path; // Path waypoints in odom frame
                        // // publish global path
                        // global_path.header.frame_id = "odom";
                        // global_path.header.stamp = ros::Time::now();
                        // pub_globalpath_.publish(global_path);        
                // }
                // else
                //     cout << "Failed to path generation" << endl;                  

            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            // cout << "global planner: 生成路径成功" << endl;
            
            return true;        
        // }else{
        //     plan.clear();
        //     return true; 
        // }

    }

    // 接收里程计信息
    void SfyForkliftGlobalPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // 将里程计信息赋值给odom_
        odom_ = *odomMsg;        
    }

    void SfyForkliftGlobalPlanner::desiredPathCB(const nav_msgs::Path::ConstPtr& totalPathMsg)
    {
        desired_path_ = *totalPathMsg;
    }

    void SfyForkliftGlobalPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
    {
        goal_received_ = true;
        ROS_INFO("Goal Received :goalCB!");
    }
    
    void SfyForkliftGlobalPlanner::getCmdCB(const geometry_msgs::Twist& cmdMsg)
    {
        linear_vel_ = cmdMsg.linear.x;
        angular_vel_ = cmdMsg.angular.z;
    }

    void SfyForkliftGlobalPlanner::calError(const ros::TimerEvent&)
    { 
        checkSubscriberStatus();  
       

        if(goal_received_) //received goal & goal not reached    
        {  
            geometry_msgs::PoseStamped current_start_pose;
            geometry_msgs::PoseStamped current_goal_pose;
            vector<geometry_msgs::PoseStamped> current_plan;
        
            bool test_make_plan  = makePlan(current_start_pose, current_goal_pose, current_plan);
            cout << "test_make_plan: " << test_make_plan << endl;
            // cout << "goal_received  true" << endl;
            nav_msgs::Odometry odom = odom_; 
            nav_msgs::Path odom_path = odom_path_;   

            // Update system states: X=[x, y, theta, v]
            const double px = odom.pose.pose.position.x; //pose: odom frame
            const double py = odom.pose.pose.position.y;
            tf::Pose pose;
            tf::poseMsgToTF(odom.pose.pose, pose);
            const double theta = tf::getYaw(pose.getRotation());

            // Waypoints related parameters
            const int N = odom_path.poses.size(); // Number of waypoints
            const double costheta = cos(theta);
            const double sintheta = sin(theta);

            // Convert to the vehicle coordinate system
            Eigen::VectorXd x_veh(N);
            Eigen::VectorXd y_veh(N);
            for(int i = 0; i < N; i++) 
            {
                const double dx = odom_path.poses[i].pose.position.x - px;
                const double dy = odom_path.poses[i].pose.position.y - py;
                x_veh[i] = dx * costheta + dy * sintheta;
                y_veh[i] = dy * costheta - dx * sintheta;
            }
            
            // Fit waypoints
            auto coeffs = polyfit(x_veh, y_veh, 2); 

            const double cte  = polyeval(coeffs, 0.0);
            const double etheta = atan(coeffs[1]);

            // cout << "============global: " << idx << "===============" << endl;    
            // cout << "cte: " << cte << endl;
            // cout << "etheta: " << etheta << endl;
            // cout << "linear_vel: " << linear_vel_ << endl;
            // cout << "angular_vel: " << angular_vel_ << endl;
            idx++;
        }
    }

    double SfyForkliftGlobalPlanner::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }    
    
    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd SfyForkliftGlobalPlanner::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) 
        {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void SfyForkliftGlobalPlanner::forkliftPathPlanningResultCB(const sfy_forklift_msgs::ForkliftPathPlanningResult::ConstPtr& Msg)
    {
        forklift_path_planning_result_ = *Msg;
    } 

    void SfyForkliftGlobalPlanner::checkSubscriberStatus() {
        // 1.获取当前所有活跃话题
        ros::master::V_TopicInfo topic_infos;
        ros::master::getTopics(topic_infos);

        // 2. 将活跃话题名称存入哈希集合（快速查找）
        std::unordered_set<std::string> active_topics;
        for (const auto& topic_info : topic_infos) {
            active_topics.insert(topic_info.name);
        }
        if (active_topics.count("/forklift_path_planning_result") > 0) {
            is_forklift_path_planning_result_sub_ = true;
        } else {
            is_forklift_path_planning_result_sub_ = false;
        }


    }
}