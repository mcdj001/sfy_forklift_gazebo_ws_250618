/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-12 12:31:06
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-03-05 14:41:36
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/src/sfy_forklift_collect_planning.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include "collect_planning/sfy_forklift_collect_planning.h"

void SfyForkliftCollectPlanning::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom_ = *odomMsg;        
}

void SfyForkliftCollectPlanning::globalPathCallback(const nav_msgs::Path::ConstPtr& globalPathMsg)
{
    global_path_ = *globalPathMsg;
}

void SfyForkliftCollectPlanning::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVelMsg)
{
    cmd_vel_ = *cmdVelMsg;
}

void SfyForkliftCollectPlanning::locationOdomCallback(const sfy_forklift_msgs::ForkliftLocation::ConstPtr& locationOdomMsg)
{
    forklift_location_odom_ = *locationOdomMsg;
}

void SfyForkliftCollectPlanning::locationEkfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Msg)
{
    forklift_location_ekf_ = *Msg;
}

void SfyForkliftCollectPlanning::moveBaseGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    move_base_simple_goal_bool_ = true;
}

void SfyForkliftCollectPlanning::localPlanGoalCB(const sfy_forklift_msgs::ForkliftFlag::ConstPtr& goalMsg)
{
    sfy_forklift_msgs::ForkliftFlag temp = *goalMsg;
    local_plan_goal_reached_ = temp.flag;
}

void SfyForkliftCollectPlanning::collectPPControlCB(const sfy_forklift_msgs::ForkliftPurePursuit::ConstPtr& Msg)
{
    collect_pp_control_ = *Msg;
}

void SfyForkliftCollectPlanning::ApriltagPositonCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr&  Msg)
{
    if(Msg->detections.size()>0){
        use_apriltag = true;
    }else{
        use_apriltag = false;
    }
}

void SfyForkliftCollectPlanning::forkliftInfoCallback(const sfy_forklift_msgs::ForkliftInfo::ConstPtr&  Msg)
{
    forklift_info_ = *Msg;
}

void SfyForkliftCollectPlanning::forkliftInitImuCB(const sfy_forklift_msgs::ForkliftInitImu::ConstPtr&  Msg)
{
  forklift_init_imu_ = *Msg;
}


void SfyForkliftCollectPlanning::collectDataToCsv(){

    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = std::localtime(&now_time_t);

    ros::Time current_ros_time = ros::Time::now();

    long long mill_ros_time = current_ros_time.toSec();

    const double forklift_ekf_x = forklift_location_ekf_.pose.pose.position.x;
    const double forklift_ekf_y = forklift_location_ekf_.pose.pose.position.y;
    double x =  forklift_location_ekf_.pose.pose.orientation.x;
    double y = forklift_location_ekf_.pose.pose.orientation.y;
    double z = forklift_location_ekf_.pose.pose.orientation.z;
    double w =  forklift_location_ekf_.pose.pose.orientation.w;
    tf::Quaternion q(x, y, z, w);
    const double forklift_ekf_yaw = tf::getYaw(q);

    const double odom_x = odom_.pose.pose.position.x;
    const double odom_y = odom_.pose.pose.position.y;

    x =  odom_.pose.pose.orientation.x;
    y = odom_.pose.pose.orientation.y;
    z =  odom_.pose.pose.orientation.z;
    w = odom_.pose.pose.orientation.w;
    tf::Quaternion q1(x, y, z, w);

    const double odom_yaw = mytools->normalizeAngle(tf::getYaw(q1));
    // const double odom_yaw = 0;
    const double odom_vel_linear_x = odom_.twist.twist.linear.x;
    const double odom_vel_angular_z = odom_.twist.twist.angular.z;
    const double cmd_vel_linear_x = cmd_vel_.linear.x;
    const double cmd_vel_angular_z = cmd_vel_.angular.z;

    // 在发布目标点后 以及  局部路径规划没有到达目标点时  才收集数据
    if(is_collect_pp_ && move_base_simple_goal_bool_ && !local_plan_goal_reached_){
        file_pp_controller << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "," << mill_ros_time << ","
        <<  odom_x << "," << odom_y <<  "," << odom_yaw <<  "," 
        <<  forklift_ekf_x << "," << forklift_ekf_y <<  "," << forklift_ekf_yaw <<  "," 
        << collect_pp_control_.e_lat << "," <<  collect_pp_control_.e_yaw << "," 
        << collect_pp_control_.k_fuzzy_distance <<  ","<< collect_pp_control_.l_d <<  ","
        <<  collect_pp_control_.pp_need_x << "," << collect_pp_control_.pp_need_y <<  "," 
        <<  collect_pp_control_.pp_next_x << "," << collect_pp_control_.pp_next_y <<  "," 
        <<  collect_pp_control_.alpha_pp << "," << collect_pp_control_.delta_pp <<  ","
        <<  collect_pp_control_.expect_steer_vel << ","
        <<  odom_vel_linear_x << "," << odom_vel_angular_z <<  "," 
        <<  cmd_vel_linear_x << "," << cmd_vel_angular_z  <<  "," 
        <<  use_apriltag  << "," << forklift_init_imu_.init_yaw_error  <<  "," << forklift_init_imu_.current_yaw_error
        << "\n" ;
    }

    if(is_collect_line_ &&  move_base_simple_goal_bool_){
        file_line << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "," << mill_ros_time << ","
        <<  odom_x << "," << odom_y <<  "," << odom_yaw <<  "," 
        <<  forklift_ekf_x << "," << forklift_ekf_y <<  "," << forklift_ekf_yaw <<  "," 
        <<  odom_vel_linear_x << "," << odom_vel_angular_z <<  "," 
        <<  cmd_vel_linear_x << "," << cmd_vel_angular_z  <<  "," 
        <<  use_apriltag  << "," <<  forklift_info_.forklift_fcode
        << "\n" ;
    }  
}

void SfyForkliftCollectPlanning::control(){
    ros::Rate loop_rate(frequency_);

    while(ros::ok()){
        collectDataToCsv();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

SfyForkliftCollectPlanning::~SfyForkliftCollectPlanning(){
    file_pp_controller.close();
    file_line.close();
}

SfyForkliftCollectPlanning::SfyForkliftCollectPlanning(){
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    nh.param<string>("pp_controller_csv_path", pp_controller_csv_path_,"none");
    nh.param<string>("line_csv_path", line_csv_path_,"none");
    nh.param<double>("frequency", frequency_ , 10);
    nh.param<bool>("is_collect_pp", is_collect_pp_ , false);
    nh.param<bool>("is_collect_line", is_collect_line_ , false);

    Apriltag_Positon_Sub = n.subscribe("/tag_detections", 1, 
      &SfyForkliftCollectPlanning::ApriltagPositonCallback, this, ros::TransportHints().tcpNoDelay(true));

    // /move_base/SfyForkliftLocalPlanner/local_plan
    odom_sub_ = n.subscribe("odom", 5, 
        &SfyForkliftCollectPlanning::odomCallback, this, ros::TransportHints().tcpNoDelay(true));
    
    cmd_vel_sub_   = n.subscribe("/cmd_vel", 5, 
        &SfyForkliftCollectPlanning::cmdVelCallback, this, ros::TransportHints().tcpNoDelay(true));

    globalpath_sub_  = n.subscribe("/move_base/SfyForkliftLocalPlanner/global_plan", 1 ,
        &SfyForkliftCollectPlanning::globalPathCallback,this, ros::TransportHints().tcpNoDelay(true));

    forklift_location_odom_sub_ = n.subscribe("/move_base/SfyForkliftLocalPlanner/forklift_location_odom", 1, 
        &SfyForkliftCollectPlanning::locationOdomCallback, this, ros::TransportHints().tcpNoDelay(true));

    forklift_location_ekf_sub = n.subscribe("robot_pose_ekf/odom_combined", 1, 
        &SfyForkliftCollectPlanning::locationEkfCallback, this, ros::TransportHints().tcpNoDelay(true));

    move_base_goal_sub_   = n.subscribe( "/move_base_simple/goal", 1, 
        &SfyForkliftCollectPlanning::moveBaseGoalCB, this);

    local_plan_goal_reached_sub_ = n.subscribe("/local_plan_goal_reached", 1 ,
        &SfyForkliftCollectPlanning::localPlanGoalCB, this);

    collect_pp_control_sub_ = n.subscribe("/collect_pp_control", 1 ,
        &SfyForkliftCollectPlanning::collectPPControlCB, this);

    // AGV叉车信息收集
    forklift_info_sub_ = n.subscribe("/forklift_info", 1 ,
        &SfyForkliftCollectPlanning::forkliftInfoCallback, this);

    forklift_init_imu_sub_ = n.subscribe("sfy/imu_init", 1,  &SfyForkliftCollectPlanning::forkliftInitImuCB, this, 
      ros::TransportHints().tcpNoDelay(true));
    
    file_pp_controller.open(pp_controller_csv_path_ , std::ios::trunc);
    file_pp_controller << "Date"<< "," << "Ros_Time" << "," 
            << "forklift_odom_x" << "," << "forklift_odom_y" << "," << "forklift_odom_yaw" << ","
            << "forklift_ekf_x" << "," << "forklift_ekf_y" <<  "," << "forklift_ekf_yaw" <<  "," 
            << "e_lat" << "," << "e_yaw" << "," 
            << "k_fuzzy_distance" << "," << "l_d" << ","
            << "pp_need_x" << "," << "pp_need_y" << ","
            << "pp_next_x" << "," << "pp_next_y" << ","
            << "alpha_pp" << "," << "delta_pp" << ","
            << "expect_steer_vel" << ","
            << "odom_vel_linear_x" << "," << "odom_vel_angular_z" << ","
            << "cmd_vel_linear_x" << "," << "cmd_vel_angular_z"  << ","
            << "use_apriltag" << ","  << "init_imu_yaw_error"  <<  "," << "current_imu_yaw_error"
            << "\n" ;

    file_line.open(line_csv_path_ , std::ios::trunc);
    file_line << "Date"<< "," << "Ros_Time" << "," 
            << "forklift_odom_x" << "," << "forklift_odom_y" << "," << "forklift_odom_yaw" << ","
            << "forklift_ekf_x" << "," << "forklift_ekf_y" <<  "," << "forklift_ekf_yaw" <<  "," 
            << "odom_vel_linear_x" << "," << "odom_vel_angular_z" << ","
            << "cmd_vel_linear_x" << "," << "cmd_vel_angular_z"  << ","
            << "use_apriltag" << "," << "Fcode"
            << "\n" ;
}

int main(int argc, char** argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "sfy_forklift_collect_planner");
    SfyForkliftCollectPlanning sfy_forklift_collect_planner;
    sfy_forklift_collect_planner.control();
    return 0;
}
