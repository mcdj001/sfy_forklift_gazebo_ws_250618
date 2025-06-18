/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 14:56:56
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-15 14:54:43
 */
#ifndef  SFY_MOTION_CONTROLLER_PURE_PURSUIT_H
#define SFY_MOTION_CONTROLLER_PURE_PURSUIT_H


#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <chrono>
#include <thread>
#include "mypath.h"
#include "mytools.h"

using namespace SfyMotionController;

#define T  1 / freq 

double freq  , v_expect;//采样频率，期望速度
double v_max;  //最大速度
double max_steer_state;  // 最大转角
bool limit_v_and_kesi;  //轮子转角是否限幅
double initial_x, initial_y, initial_yaw, initial_v,initial_kesi;  //初始化车辆位姿，速度和前轮转角
double slow_LEVE1_DISTANCE, slow_LEVE2_DISTANCE, slow_LEVE1_V,  slow_LEVE2_V, goal_tolerance_DISTANCE;  //定义二级减速距离和速度
vehicleDynamicParam  vehicle_dynamic_param;
double front_view_distance_law , front_view_distance ;  // 前视距离系数  前视距离
bool delay_model , steer_vel_delay_pid_model;
double steer_vel_delay_pid_kp , steer_vel_delay_pid_ki , steer_vel_delay_pid_kd;
std::ofstream file;
MyTools* mytools;

class Pure_Pursuit {
    private:
		// forklift
		vehicleState forklift , forklift_odom;  //小车状态
		U control; //小车控制量[v,kesi]
		int lastIndex; //最后一个点索引值
		int preIndex; //之前的下标
		waypoint lastPoint; //最后一个点信息
		string action;  //小车目前动作：跟踪或跟踪完成(tracking or reach goal!)

		//ROS
		ros::Subscriber path_sub;    //订阅路径，消息类型为nav_msgs::Path
		ros::Publisher vel_pub;    //发布速度信息，消息类型为geometry_msgs::Twist
		ros::Publisher actual_state_pub;    //发布小车实际位姿，消息类型为geometry_msgs::Pose2D
		ros::Publisher visual_state_pub;    //向rviz发布小车虚拟轨迹，消息类型为visualization_msgs::Marker
		geometry_msgs::Point visual_state_pose;
		visualization_msgs::Marker visual_state_trajectory;
		geometry_msgs::Pose2D actual_pose;
		geometry_msgs::Twist vel_msg;
		int temp;  //计数，达到终点时，用于判断控制器是否需要关闭
		// ros::Time start_time , current_time; // 这个时间有问题
		std::chrono::steady_clock::time_point  start_time , current_time; 
		ros::Subscriber odomSub;
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

		MyPath*  path;
		void addpointcallback(const nav_msgs::Path::ConstPtr& msg);
        double slow_judge(double distance , int cur_index , int all_index);
        //控制器流程
        void PP_Track();
        // 发布话题函数
        void PUB();
        //关闭控制器
        void shutdown_controller();
        void Marker_set();
        U v_and_kesi_limit(U control_value);
		vehicleState update_state(U control, vehicleState forklift);
		vehicleState delay_odom_postion();
		// 角速度pid控制器
		float z_vel_pid_controller(float pv,float sp);

    public:      
        Pure_Pursuit(ros::NodeHandle& nh);
        ~Pure_Pursuit();
		//控制启停函数
	    void node_control();
};

#endif   //SFY_MOTION_CONTROLLER_PURE_PURSUIT_H