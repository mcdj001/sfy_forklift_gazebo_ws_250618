/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-13 16:31:21
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-21 12:09:09
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "cubic_spline.hpp"
#include "mytools.h"
using namespace std;
using namespace SfyMotionController;
MyTools* mytools;

nav_msgs::Path odom_path;
nav_msgs::Path error_path;

nav_msgs::Odometry robot_odom;

int odom_count = 0;
double sum_error = 0;
double error_x = 0;
vector<double> wx , wy;

std::string id = "odom";
std::string trajectory_type = "";

double current_path_x = 0, current_path_y = 0, current_path_theta = 0;

ros::Publisher odom_path_pub;
ros::Publisher desired_path_pub;

void generation_desired_path() {
    ROS_INFO("generation_desired_path()");
    nav_msgs::Path desired_path;

    int iter = 1000;
    if (trajectory_type == "circle") {
        double  radius = 5;
        double  period = 1000;
        for (int t = 0; t < iter; t++) {
            desired_path.header.stamp = ros::Time::now();
            desired_path.header.frame_id = id;
            desired_path.header.seq = t;

            geometry_msgs::PoseStamped pose;
            pose.header.seq = t;
            pose.header.frame_id = id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = radius * sin(2 * M_PI * t / period); 
            pose.pose.position.y = -radius * cos(2 * M_PI * t / period); 
            double grad = atan2((-radius * cos(2 * M_PI * (t + 1) / period) - pose.pose.position.y), (radius * sin(2 * M_PI * (t + 1) / period)) - (pose.pose.position.x + 1e-5));
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(grad);
            pose.pose.orientation.x = goal_quat.x;
            pose.pose.orientation.y = goal_quat.y;
            pose.pose.orientation.z = goal_quat.z;
            pose.pose.orientation.w = goal_quat.w;
            desired_path.poses.push_back(pose);
        }
    } else if (trajectory_type == "mypath") {
        vector<waypoint> waypoints;
        waypoint PP;
        Spline2D csp_obj(wx, wy);
        for(double i=0; i < csp_obj.s.back(); i += 1.0){
            vector<double> point_= csp_obj.calc_postion(i);

            desired_path.header.stamp = ros::Time::now();
            desired_path.header.frame_id = id;
            desired_path.header.seq = i;

            geometry_msgs::PoseStamped pose;
            pose.header.seq = i;
            pose.header.frame_id = id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = point_[0]; 
            pose.pose.position.y = point_[1]; 
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(mytools->normalizeAngle(csp_obj.calc_yaw(i)));
            pose.pose.orientation.x = goal_quat.x;
            pose.pose.orientation.y = goal_quat.y;
            pose.pose.orientation.z = goal_quat.z;
            pose.pose.orientation.w = goal_quat.w;
            desired_path.poses.push_back(pose);
        }
    }else if (trajectory_type == "epitrochoid"){
        double R = 5;
        double r = 1;
        double d = 3;
        double period = 1000;
        double scale_factor = 1;
        for (int t = 0; t < iter; t++) {
            desired_path.header.stamp = ros::Time::now();
            desired_path.header.frame_id = id;
            desired_path.header.seq = t;

            geometry_msgs::PoseStamped pose;
            pose.header.seq = t;
            pose.header.frame_id = id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = scale_factor * ((R + r) * cos(2 * M_PI * t/ period) - d * cos(((R + r) / r) * 2 * M_PI * t / period)); 
            pose.pose.position.y = scale_factor * ((R + r) * sin(2 * M_PI * t/ period) - d * sin(((R + r) / r) * 2 * M_PI * t / period)); 
            double grad = atan2((5 * sin(2 * M_PI* (t+1) / period) * cos(2 * M_PI* (t+1) / period) / (pow(sin(2 * M_PI * (t+1) / period), 2) + 1)- pose.pose.position.y) , (5 * cos(2 * M_PI* (t+1) / period) / (pow(sin(2 * M_PI * (t+1) / period), 2) + 1)-pose.pose.position.x + 1e-5));
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(grad);
            pose.pose.orientation.x = goal_quat.x;
            pose.pose.orientation.y = goal_quat.y;
            pose.pose.orientation.z = goal_quat.z;
            pose.pose.orientation.w = goal_quat.w;
            desired_path.poses.push_back(pose);
        }
    }else if (trajectory_type == "infinite"){
        double period = 1000;
        double scale_factor = 1;
        for (int t = 0; t < iter; t++) {
            desired_path.header.stamp = ros::Time::now();
            desired_path.header.frame_id = id;
            desired_path.header.seq = t;

            geometry_msgs::PoseStamped pose;
            pose.header.seq = t;
            pose.header.frame_id = id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 10 * cos(2 * M_PI* t / period) / (pow(sin(2 * M_PI * (t) / period), 2) + 1); 
            pose.pose.position.y = 10 * sin(2 * M_PI* t / period) * cos(2 * M_PI* t / period) / (pow(sin(2 * M_PI * (t) / period), 2) + 1); 
            double grad = atan2((10 * cos(2 * M_PI* (t+1) / period) / (pow(sin(2 * M_PI * (t+1) / period), 2) + 1)- pose.pose.position.y)\
                        ,10 * sin(2 * M_PI* (t+1) / period) * cos(2 * M_PI* (t+1) / period) /  (pow(sin(2 * M_PI * (t+1) / period), 2) + 1) - pose.pose.position.x + 1e-5);
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(grad);
            pose.pose.orientation.x = goal_quat.x;
            pose.pose.orientation.y = goal_quat.y;
            pose.pose.orientation.z = goal_quat.z;
            pose.pose.orientation.w = goal_quat.w;
            desired_path.poses.push_back(pose);
        }        
    }else if (trajectory_type == "mypath_new"){
        
    }
    
    desired_path_pub.publish(desired_path);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_odom = *msg;
    odom_count++;

    if (odom_count % 3 == 0) {
        odom_path.header = msg->header;
        odom_path.header.frame_id = id;

        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        pose.header.frame_id = id;
        odom_path.poses.push_back(pose);

        odom_path_pub.publish(odom_path);
        generation_desired_path();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_trajectory_generation");
    ros::NodeHandle nh;

    ROS_INFO("path_trajectory_generation is started!!");
    ros::NodeHandle pn("~");
    pn.param<std::string>("trajectory_type", trajectory_type, "circle" );
    pn.param("wx", wx, wx);
	pn.param("wy", wy , wy);

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_cb);
    desired_path_pub = nh.advertise<nav_msgs::Path>("/desired_path", 500);
    odom_path_pub = nh.advertise<nav_msgs::Path>("/recorded_path", 10);

    ros::spin();


    return 0;
}
