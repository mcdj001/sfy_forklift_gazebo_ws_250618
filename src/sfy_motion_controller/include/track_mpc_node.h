/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-13 13:54:12
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-26 18:23:33
 */
#ifndef  SFY_MOTION_CONTROLLER_TRACK_MPC_NODE_H
#define SFY_MOTION_CONTROLLER_TRACK_MPC_NODE_H

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>
#include <fstream>
#include <string>

#include "track_mpc.h"

using namespace std;
using namespace Eigen;


class MPCNode
{
    public:
        MPCNode();
        ~MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_gen_path, _sub_path, _sub_goal, _sub_amcl;
        ros::Publisher _pub_totalcost, _pub_ctecost, _pub_ethetacost,_pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj; 
	    //ackermann_msgs::AckermannDriveStamped _ackermann_msg;
        geometry_msgs::Twist _twist_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
               _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

        //double _Lf; 
        double _dt, _w, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius, _waypointsDist;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);

        double steer_vel_delay_pid_kp , steer_vel_delay_pid_ki , steer_vel_delay_pid_kd;
        bool steer_vel_delay_pid_model;
        float  z_vel_pid_controller(float pv,  float sp);

        //For making global planner
        nav_msgs::Path _gen_path;
        unsigned int min_idx;
        
        double _mpc_etheta;
        double _mpc_cte;
        fstream file;
        unsigned int idx;
}; 


#endif