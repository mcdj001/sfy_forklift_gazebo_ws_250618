/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-07 13:46:35
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 15:45:06
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_local_planner/src/sfy_forklift_local_planner_ros.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include <pluginlib/class_list_macros.h>
#include "sfy_forklift_local_planner/sfy_forklift_local_planner_ros.h"


PLUGINLIB_EXPORT_CLASS(sfy_forklift_local_planner::SfyForkliftLocalPlanner, nav_core::BaseLocalPlanner)

namespace sfy_forklift_local_planner
{

    SfyForkliftLocalPlanner::SfyForkliftLocalPlanner() : tf_(NULL), dynamic_recfg_(NULL),
                                               goal_reached_(false), world_model_(NULL), initialized_(false)
    {
        myTimer_ = nt_.createTimer(ros::Duration((1.0)/20), &SfyForkliftLocalPlanner::calTimer, this);
        local_plan_goal_reached_pub_ = nt_.advertise<sfy_forklift_msgs::ForkliftFlag>("/local_plan_goal_reached", 1);
        forklift_path_tracking_status_pub_ = nt_.advertise<sfy_forklift_msgs::ForkliftPathTrackingStatus>("/forklift_path_tracking_status", 1);

        forklift_path_planning_result_sub_ = nt_.subscribe("/forklift_path_planning_result", 1, 
                &SfyForkliftLocalPlanner::forkliftPathPlanningResultCB, this);
    }

    SfyForkliftLocalPlanner::~SfyForkliftLocalPlanner() {}

    void SfyForkliftLocalPlanner::reconfigureCB(SfyForkliftLocalPlannerReconfigureConfig &config, uint32_t level)
    {
        cfg_.reconfigure(config);     
        ROS_INFO("Updated expected_vel_x: %f", cfg_.agv.expected_vel_x);  // 打印更新后的期望速度
        ROS_INFO("Updated debug_model: %d", cfg_.forklift_chose.debug_model);   
    }

    void SfyForkliftLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);
            cfg_.loadRosParamFromNodeHandle(nh);
            obstacles_.reserve(500);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();

            visualization_ = SfyForkliftVisualizationPtr(new SfyForkliftVisualization(nh, global_frame_));
            dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<SfyForkliftLocalPlannerReconfigureConfig>>(nh);
            // 读取cfg中的动态参数
            dynamic_reconfigure::Server<SfyForkliftLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&SfyForkliftLocalPlanner::reconfigureCB, this, _1, _2);
            dynamic_recfg_->setCallback(cb);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
            cout << "odom_topic===========: " << cfg_.odom_topic << endl;
            cout << "debug_model===========: " << cfg_.forklift_chose.debug_model << endl;
            cout << "debug_model: " << cfg_.forklift_chose.debug_model << endl;
            cout << "expected_vel_x: " << cfg_.agv.expected_vel_x << endl;
            odom_helper_.setOdomTopic(cfg_.odom_topic);

            ros::NodeHandle nh_move_base("~");
            // double controller_frequency = 20;
            // nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
            
            std::string controller_frequency_param_name;
            double controller_frequency = 0;
            if(!nh_move_base.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
                ROS_WARN("controller_frequency_param_name doesn't exits");
            } else {
                nh_move_base.param(controller_frequency_param_name, controller_frequency, 20.0);
                if(controller_frequency > 0) {
                } else {
                    ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                }
            }
            control_duration_ = 1.0 / controller_frequency;

            
            collect_pp_control_pub_ = nh.advertise<sfy_forklift_msgs::ForkliftPurePursuit>("/collect_pp_control", 1);
            forklift_cmd_vel_pub_ =  nh.advertise<sfy_forklift_msgs::ForkliftCmdVel>("/forklift_cmd_vel", 1);
            
            forklift_info_sub_   = nh.subscribe( "/forklift_info", 1, &SfyForkliftLocalPlanner::forkliftInfoCB, this);
            forklift_location_ekf_sub_ = nh.subscribe("robot_pose_ekf/odom_combined", 1, 
                     &SfyForkliftLocalPlanner::locationEkfCallback, this);

            initialized_ = true;
        }
    }

    bool SfyForkliftLocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("sfy_forklift_local_planner has not been initialized");
            return false;
        }
        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        std::cout << "sfy_local_plan_test" << std::endl;
        goal_reached_ = false;
        xy_reached_ = false;
        last_back_ = false;
        is_goal_slow_ = false;
        slow_mini_goal_dist_ = numeric_limits<double>::infinity();
        omega_filter_.filterCall(ros::Time::now(), last_cmd_.angular.z);
        // 可以暂时先不要平滑
        // smoothPlan2d(global_plan_); 
        visualization_->publishGlobalPlan(global_plan_);
        return true;
    }

    bool SfyForkliftLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        cout << "debug_model: " << cfg_.forklift_chose.debug_model << endl;
        cout << "expected_vel_x: " << cfg_.agv.expected_vel_x << endl;
        if(cfg_.forklift_chose.debug_model){
            std::cout << "===== Forklift BASE_LOCAL computeVelocityCommands =====" << std::endl;
            cout << "control_duration_: " << control_duration_ << endl;    
        }
       
        if (!initialized_)
        {
            ROS_ERROR("sfy_forklift_local_planner has not been initialized");
            return false;
        }

        // 获取AGV的状态
        geometry_msgs::PoseStamped robot_vel_tf;
        odom_helper_.getRobotVel(robot_vel_tf);
        robot_vel_.linear.x = robot_vel_tf.pose.position.x;
        robot_vel_.linear.y = robot_vel_tf.pose.position.y;
        robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

        goal_reached_ = false;

        // 拿到AGV当前的位姿  这个是/odom的姿态
        costmap_ros_->getRobotPose(robot_pose_);
        if(cfg_.forklift_chose.use_forklift_odom_combined){      //  使用融合后的数据与期望路径进行计算
            forklift_odom_.x = forklift_location_ekf_.pose.pose.position.x;
            forklift_odom_.y = forklift_location_ekf_.pose.pose.position.y;
            double x =  forklift_location_ekf_.pose.pose.orientation.x;
            double y = forklift_location_ekf_.pose.pose.orientation.y;
            double z = forklift_location_ekf_.pose.pose.orientation.z;
            double w =  forklift_location_ekf_.pose.pose.orientation.w;
            tf::Quaternion q(x, y, z, w);
            double forklift_ekf_yaw = tf::getYaw(q);
            forklift_odom_.yaw = mytools_->normalizeAngle(forklift_ekf_yaw);
        }else{
            forklift_odom_.x = robot_pose_.pose.position.x;
            forklift_odom_.y = robot_pose_.pose.position.y;
            double roll,pitch,yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(robot_pose_.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            forklift_odom_.yaw = mytools_->normalizeAngle(yaw);
        }
    
        forklift_odom_.v =  robot_vel_.linear.x;
        forklift_odom_.kesi = robot_vel_.angular.z;

        // 发布AGV的odom的x y yaw 用作数据收集
        forklift_location_odom_.x = forklift_odom_.x;
        forklift_location_odom_.y = forklift_odom_.y;
        forklift_location_odom_.yaw = forklift_odom_.yaw;
        
        // 裁剪全局路径，即去除全局路径中那些在AGV之前的位置
        pruneGlobalPlan(*tf_, robot_pose_, global_plan_, cfg_.trajectory.global_plan_prune_distance);

        // 将给定的全局路径转换为局部路径
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        int goal_idx;
        geometry_msgs::TransformStamped tf_plan_to_global;
        if (!transformGlobalPlan(*tf_, global_plan_, robot_pose_, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                                 transformed_plan, &goal_idx, &tf_plan_to_global))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }
        if (transformed_plan.empty())
            return false;

        // 获取到目标点
        const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();

        //判断是否到达终点
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;
        const double goal_th = tf2::getYaw(goal_point.pose.orientation);
        double dx = goal_x - forklift_odom_.x;
        double dy = goal_y - forklift_odom_.y;
        double dyaw;
        //dyaw = fmod(goal_th - tf2::getYaw(robot_pose_.pose.orientation) - M_PI, 2 * M_PI);
        
        // global_plan_viapoint_sep  从全局规划中提取的每两个连续中途点之间的最小间隔 
        // global_plan_goal_sep  用于修剪的全局规划中机器人与中途点之间的距离
        updateViaPointsContainer(transformed_plan,
                                 cfg_.trajectory.global_plan_viapoint_sep, cfg_.trajectory.global_plan_goal_sep);
      
        via_points_.push_back(Eigen::Vector3d(goal_x, goal_y, goal_th));
        local_plan_.push_back(transformed_plan.back());

        obstacles_.clear();   // 障碍物清空
        
        // 更新障碍物的局部代价地图std::vector<geometry_msgs::PoseStamped> local_plan_;
        updateObstacleContainerWithCostmap();

        // 检查给定路径上是否存在潜在的碰撞，并找到距离AGV最近的碰撞点
        // max_global_plan_lookahead_dist 指定用于优化的全局规划子集的最大长度（累积欧几里得距离）3
        double lethal_distance_ = checkCollision(transformed_plan, cfg_.trajectory.max_global_plan_lookahead_dist);
        dec_ratio_ = std::min(lethal_distance_ / cfg_.agv.dec_dist, 1.0);
       

        // 计算AGV和世界坐标系之间的旋转矩阵和位移矩阵
        Eigen::Quaterniond quat_world_robot(robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
                                            robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);
        Eigen::Matrix3d rot_mat_world_robot = quat_world_robot.toRotationMatrix();
        Eigen::Translation3d trans_world_robot(robot_pose_.pose.position.x, robot_pose_.pose.position.y, 0.0);
        Eigen::Affine3d T_world_robot = trans_world_robot * rot_mat_world_robot;

        // 计算目标点和世界坐标系之间的旋转矩阵和位移矩阵
        Eigen::Vector3d euler_world_target(via_points_[0][2], 0, 0);
        // 相当于欧拉角转四元数 依次旋转yaw, pitch, roll
        Eigen::Matrix3d rot_mat_world_target;
        rot_mat_world_target = Eigen::AngleAxisd(euler_world_target(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler_world_target(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler_world_target(2), Eigen::Vector3d::UnitX());
        Eigen::Translation3d trans_world_target(via_points_[0][0], via_points_[0][1], 0.0);
        Eigen::Affine3d T_world_target = trans_world_target * rot_mat_world_target;

        // 计算机器人和目标点之间的位移矩阵、旋转矩阵和欧拉角
        Eigen::Affine3d T_robot_target = T_world_robot.inverse() * T_world_target;
        // 单独把T_robot_target的旋转矩阵拿出来
        Eigen::Matrix3d rot_mat_robot_target = T_robot_target.rotation();
        Eigen::Vector3d euler_robot_target = R2ypr(rot_mat_robot_target);
       
        double dx1 = T_robot_target.matrix()(0, 3);
        double dy1 = T_robot_target.matrix()(1, 3);
        double dyaw1 = euler_robot_target(0);
        
        double rho, alpha, phi, v, omega,expected_steer;
        // 已知目标点 x y 偏航角  推出距离(速度×系数) 叉车与目标斜率 偏航角  
        poseError( dx1, dy1, dyaw1, rho, alpha, phi);

        if(cfg_.forklift_chose.debug_model){
            // std::cout << "rot_mat_world_robot :\n" << rot_mat_world_robot << std::endl;
            // std::cout << "trans_world_robot :\n" << trans_world_robot.vector() << std::endl;
            // std::cout << "T_world_robot:\n" << T_world_robot.matrix() << std::endl;

            // std::cout << "euler_world_target:\n" << euler_world_target << std::endl;
            // std::cout << "rot_mat_world_target:\n" << rot_mat_world_target << std::endl;
            // std::cout << "T_world_target:\n" << T_world_target.matrix() << std::endl;
            
            // std::cout << "T_robot_target:\n" << T_robot_target.matrix() << std::endl;
            // std::cout << "rot_mat_robot_target:\n" << rot_mat_robot_target << std::endl;
            // std::cout << "euler_robot_target:\n" << euler_robot_target << std::endl;
            cout << "lethal_distance_: " << lethal_distance_ << " dec_ratio_: " << dec_ratio_ << endl;
            std::cout << "误差1 dx1: " << dx1 << " dy1: " << dy1 << " dyaw1: " << dyaw1 << std::endl;
            std::cout << "rho: " << rho << " alpha: " << alpha << " phi: " << phi << std::endl;
            cout << endl;
            std::cout << "当前AGV的控制算法为: " << cfg_.forklift_chose.control_model << std::endl;
        }   
        
        // 选择控制模型为PP
        if (cfg_.forklift_chose.control_model == "pp"){
            localPPControl(rho, alpha, v, omega,expected_steer);
        // 选择控制模型为PID
        }else if (cfg_.forklift_chose.control_model == "pid"){
            localPidControl(rho, alpha, v, omega,expected_steer);
        // 选择控制模型为MPC
        }else if(cfg_.forklift_chose.control_model == "mpc"){
            localMpcControl(rho, alpha, v, omega,expected_steer);
        }else if(cfg_.forklift_chose.control_model == "lqr_dynamic"){
            localLqrDynamicControl(rho, alpha, v, omega,expected_steer);
        }else if(cfg_.forklift_chose.control_model == "lqr_kinematic"){
            localLqrKinematicControl(rho, alpha, v, omega,expected_steer);
        }

        // 经过控制算法后  统一做限制处理
        v = clip(v, cfg_.agv.max_vel_x * (-1.0) * dec_ratio_, cfg_.agv.max_vel_x * dec_ratio_);
        omega = clip(omega, cfg_.agv.max_vel_theta * (-1.0) * dec_ratio_, cfg_.agv.max_vel_theta * dec_ratio_);
        expected_steer = clip(expected_steer, 1.2 * (-1.0) ,  1.2);
        // 线速度和角速度限制条件
        // if (cfg_.agv.min_turn_radius > 0)
        // {
        //     double omega_max = fabs(v / cfg_.agv.min_turn_radius);
        //     omega = clip(omega, omega_max * (-1.0), omega_max);
        // }
        // else
        // {
        //     double d_atan = std::atan2(dy1, dx1);
        //     double d_atan_phi = fabs(d_atan - phi
        //     if ((xy_reached_) or
        //         (cfg_.agv.turn_around_priority and fabs(phi) > 0.75 and
        //          (d_atan_phi < 0.5 or fmod(d_atan_phi, M_PI) < 0.5 or fmod(d_atan_phi, M_PI) > 2.6)))
        //     {
        //         v = 0;
        //         omega = clip(phi, cfg_.agv.max_vel_theta * (-1.0), cfg_.agv.max_vel_theta);
        //     }
        // }
        int last_global_index = global_plan_.size() - 1;
        double global_last_distance = sqrt(pow(global_plan_[last_global_index].pose.position.x - forklift_odom_.x, 2) 
                + pow(global_plan_[last_global_index].pose.position.y - forklift_odom_.y, 2));
         
        // if( (lethal_distance_ > cfg_.agv.stop_dist && lethal_distance_ < cfg_.agv.slow_level1_distance)
        //     // || 
        //     // (global_last_distance > cfg_.agv.stop_dist && global_last_distance < cfg_.agv.slow_level1_distance) 
        //     ){
        //     if(last_back_){
        //         v = -1 * cfg_.agv.slow_level1_vel;
        //     }else{
        //         v = cfg_.agv.slow_level1_vel;
        //     }

        if((global_last_distance > cfg_.agv.stop_dist && global_last_distance < cfg_.agv.slow_level1_distance) || is_goal_slow_){
            if(is_reverse_){
                v = -1 * cfg_.agv.slow_level1_vel;
            }else{
                v = cfg_.agv.slow_level1_vel;
            }
            is_goal_slow_ = true;
        }

        // if(global_last_distance < 0.3){
        //     expected_steer = clip(expected_steer, 0.1 * (-1.0) ,  0.1);
        // }

        if(lethal_distance_ <= cfg_.agv.stop_dist)
        {
            v = 0;
            omega = 0;
            expected_steer = 0;
        }

        if(forklift_odom_.x == 0 && forklift_odom_.y == 0){
            v = 0;
            omega = 0;
            expected_steer = 0;
        }    
    
        // 保证每次传输的角速度和线速度都在安全范围内
        // const double min_feasible_angular_speed = robot_vel_.angular.z - cfg_.agv.acc_lim_theta * control_duration_;
        // const double max_feasible_angular_speed = robot_vel_.angular.z + cfg_.agv.acc_lim_theta * control_duration_;
        // omega = clip(omega, min_feasible_angular_speed, max_feasible_angular_speed);

        // const double min_feasible_linear_speed = robot_vel_.linear.x - cfg_.agv.acc_lim_x * control_duration_;
        // const double max_feasible_linear_speed = robot_vel_.linear.x + cfg_.agv.acc_lim_x * control_duration_;
        // v = clip(v, min_feasible_linear_speed, max_feasible_linear_speed);
        
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = omega;

        last_cmd_ = cmd_vel;
        visualization_->publishViaPoints(via_points_);
        visualization_->publishViaPoints(collision_points_, "CollsionPoints", visualization_->toColorMsg(1.0, 1.0, 0.65, 0.0));
        visualization_->publishLocalPlan(local_plan_);
        visualization_->publishObstacles(obstacles_, costmap_->getResolution());
        visualization_->publishForkliftLocationOdom(forklift_location_odom_);

        if(is_reverse_){
            // goal_th 局部路径的终点
            dyaw = mytools_->normalizeAngle(goal_th - forklift_odom_.yaw + M_PI);
        }else{
            dyaw = mytools_->normalizeAngle(goal_th - forklift_odom_.yaw);
        }

        cout << "dyaw: " << dyaw << endl;
        cout << "distance:" << std::sqrt(dx * dx + dy * dy) << endl;

        // 保证叉车的位置和斜率都到达目标点  误差为0.1 可人为设置
        if ((fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance) && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance)){
            goal_reached_ = true;
            xy_reached_ = true;
        }
        else if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance)
            xy_reached_ = true;
        else if (xy_reached_ && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance))
            goal_reached_ = true;

        if(is_goal_slow_){
            if(slow_mini_goal_dist_ > global_last_distance) {
                slow_mini_goal_dist_ = global_last_distance;
            }else{
                goal_reached_ = true;
                xy_reached_ = true;
            }
        }
        
        // 发布叉车需要的cmd_vel   不使用move_base里的信息了
        forklift_cmd_vel_.forklift_id = forklift_info_.forklift_id;
        forklift_cmd_vel_.control_model = cfg_.forklift_chose.control_model;
        forklift_cmd_vel_.linear_x = v;
        forklift_cmd_vel_.angular_z = omega;
        forklift_cmd_vel_.expected_steer = expected_steer;
        forklift_cmd_vel_.pid_steer   = 0;
        forklift_cmd_vel_pub_.publish(forklift_cmd_vel_);

        if(cfg_.forklift_chose.debug_model){
            cout << "局部地图当前终点位置 goal_x: " << goal_x << " goal_y: " << goal_y << " goal_th: " << goal_th << endl;
            cout << "global_plan_.size: " << global_plan_.size() << endl;
            cout << "global_plan_终点位置 x: " << global_plan_[last_global_index].pose.position.x << " y: " << global_plan_[last_global_index].pose.position.y  << endl;
            cout << "叉车当前位置 x: " << forklift_odom_.x << " y: " << forklift_odom_.y << " yaw: " << forklift_odom_.yaw << endl;
            cout << "上述误差 dx: " << dx << " dy: " << dy << " dyaw: " << dyaw << endl;
            cout << "is_reverse_" << is_reverse_ << endl;
            cout << "xy_reached_: " << xy_reached_ << endl;
            cout << "goal_reached_: " << goal_reached_ << endl;
            cout << "local_plan_.size: " << local_plan_.size() << endl;
            cout << "global_last_distance：" << global_last_distance << endl;


            // std::cout << "via_points_:" << std::endl;
            // for (const Eigen::Vector3d& vec : via_points_) {
            //     std::cout << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]" << std::endl;
            // }
            // std::cout << std::endl;

            // std::cout << "local_plan_:" << std::endl;
            // for (const auto& pose : local_plan_) {
            //     std::cout << "Position: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ")" << std::endl;
            // }
            // std::cout << std::endl;

            // std::cout << "global_plan_:" << std::endl;
            // for (const auto& pose : global_plan_) {
            //     std::cout << "Position: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ")" << std::endl;
            // }
            // std::cout << std::endl;
            std::cout << "期望steer: " << expected_steer  << std::endl;
            std::cout << "本次控制线速度: " << v << ", 角速度: " << omega << std::endl;
        }

        return true;
    }

    bool SfyForkliftLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (goal_reached_)
        {
            // local_plan_goal_reached_pub_.publish(local_plan_goal_reached_);
            pid_controller_.reset();
            ROS_INFO("Base Local Planner Goal Reached!");
            // planner_->clearPlanner();
            return true;
        }
        
        return false;
    }

    void SfyForkliftLocalPlanner::smoothPlan2d(std::vector<geometry_msgs::PoseStamped> &global_plan)
    {
        // 如果global_plan为空，则返回
        if (global_plan.empty())
            return;
        
        simplifyGlobalPlan(global_plan, 0.03);
        // 创建smoothed_global_plan
        std::vector<geometry_msgs::PoseStamped> smoothed_global_plan(global_plan);
        // 权重
        double weight_data = 0.5;
        double weight_smooth = 1.0 - weight_data;
        // 容差
        double tolerance = 0.007;
        double change = tolerance;
        double aux1, aux2;
        // 当change大于容差时，循环
        while (change >= tolerance)
        {
            change = 0;
            for (std::size_t i = 1; i < global_plan.size() - 1; ++i)
            {
                // 计算当前点的x坐标
                aux1 = smoothed_global_plan[i].pose.position.x;
                // 计算smoothed_global_plan的x坐标
                smoothed_global_plan[i].pose.position.x += weight_data * (global_plan[i].pose.position.x - smoothed_global_plan[i].pose.position.x) 
                + weight_smooth * (smoothed_global_plan[i - 1].pose.position.x + smoothed_global_plan[i + 1].pose.position.x - 2 * smoothed_global_plan[i].pose.position.x);

                // 计算当前点的y坐标
                aux2 = smoothed_global_plan[i].pose.position.y;
                // 计算smoothed_global_plan的y坐标
                smoothed_global_plan[i].pose.position.y += weight_data * (global_plan[i].pose.position.y - smoothed_global_plan[i].pose.position.y) + weight_smooth * (smoothed_global_plan[i - 1].pose.position.y + smoothed_global_plan[i + 1].pose.position.y - 2 * smoothed_global_plan[i].pose.position.y);
                // 计算change
                change += (fabs(aux1 - smoothed_global_plan[i].pose.position.x) + fabs(aux2 - smoothed_global_plan[i].pose.position.y));
            }
        }
        global_plan = smoothed_global_plan;
    } 

    void SfyForkliftLocalPlanner::simplifyGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan, double simplify_separation)
    {
        if (plan.empty())
            return;
        // 创建一个简单的计划
        std::vector<geometry_msgs::PoseStamped> simple_plan;
        simple_plan.push_back(plan[0]);
        // 记录上一个点的索引
        std::size_t prev_idx = 0;
        // 遍历计划
        for (std::size_t i = 1; i < plan.size(); ++i)
        {
            // 如果当前点和上一个点的距离小于简化距离，或者当前点和最后一个点的距离小于简化距离，则跳过
            if ((distancePoints2d(plan[prev_idx].pose.position, plan[i].pose.position) < simplify_separation) ||
                (distancePoints2d(plan.back().pose.position, plan[i].pose.position) < simplify_separation))
                continue;
            simple_plan.push_back(plan[i]);
            // 更新上一个点的索引
            prev_idx = i;
        }
        
        simple_plan.push_back(plan.back());
        plan = simple_plan;
    }

    bool SfyForkliftLocalPlanner::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;

        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform);

            double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
                double dx = robot.pose.position.x - it->pose.position.x;
                double dy = robot.pose.position.y - it->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < dist_thresh_sq)
                {
                    erase_end = it;
                    break;
                }
                ++it;
            }
            if (erase_end == global_plan.end())
                return false;

            if (erase_end != global_plan.begin())
                global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }
    

    /**
     * @brief 此函数用于将给定的全局路径转换为局部路径。
     * 
     * @param[in] tf tf2_ros::Buffer对象，用于存储和查询变换。
     * @param[in] global_plan 存储给定的全局路径的std::vector<geometry_msgs::PoseStamped>对象。
     * @param[in] global_pose 当前机器人的geometry_msgs::PoseStamped对象。
     * @param[in] costmap costmap_2d::Costmap2D对象，表示局部地图。
     * @param[in] global_frame 全局路径的坐标系。
     * @param[in] max_plan_length 允许的最大计划长度（如果为负数，则没有限制）。
     * @param[out] transformed_plan 存储转换后的局部路径的std::vector<geometry_msgs::PoseStamped>对象。
     * @param[out] current_goal_idx 如果找到目标点，则存储当前目标点的索引。
     * @param[out] tf_plan_to_global 如果需要，存储全局路径到局部路径的变换的geometry_msgs::TransformStamped对象。
     * 
     * @return 转换成功返回true，否则返回false。
     */
    bool SfyForkliftLocalPlanner::transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                                 const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                                                 std::vector<geometry_msgs::PoseStamped> &transformed_plan, int *current_goal_idx, geometry_msgs::TransformStamped *tf_plan_to_global) const
    {
        const geometry_msgs::PoseStamped &plan_pose = global_plan[0];
        transformed_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                          plan_pose.header.frame_id, ros::Duration(0.5));

            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                             costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85; // 只考虑 85% 的代价地图大小，以便更好地整合位于局部代价地图边界上的点障碍物

            int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            bool robot_reached = false;
            for (int j = 0; j < (int)global_plan.size(); ++j)
            {
                double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
                double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

                if (robot_reached && new_sq_dist > sq_dist)
                    break;

                if (new_sq_dist < sq_dist) // find closest distance
                {
                    sq_dist = new_sq_dist;
                    i = j;
                    if (sq_dist < 0.05)       // 2.5 cm to the robot; take the immediate local minima; if it's not the global
                        robot_reached = true; // minima, probably means that there's a loop in the path, and so we prefer this
                }
            }

            geometry_msgs::PoseStamped newer_pose;
            double plan_length = 0; // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped &pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_global_transform);
                newer_pose.header.frame_id = global_frame;
                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose
                if (i > 0 && max_plan_length > 0)
                    plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);
                newer_pose.header.frame_id = global_frame;
                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
            }

            // Return the transformation from the global plan to the global planning frame if desired
            if (tf_plan_to_global)
                *tf_plan_to_global = plan_to_global_transform;
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }


    void SfyForkliftLocalPlanner::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation_via, double min_separation_goal)
    {
        via_points_.clear();
        local_plan_.clear();

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time::now();
        pose.pose = robot_pose_.pose;
        local_plan_.push_back(pose);

        if (min_separation_via <= 0)
            return;

        std::size_t prev_idx = 0;
        double via_point_yaw;
        for (std::size_t i = 1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
        {
            // check separation to the previous via-point inserted
            if ((distancePoints2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation_via) ||
                (distancePoints2d(transformed_plan.back().pose.position, transformed_plan[i].pose.position) < min_separation_goal) ||
                (distancePoints2d(robot_pose_.pose.position, transformed_plan[i].pose.position) < min_separation_via))
                continue;

            // add via-point
            via_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[prev_idx].pose.position.y,
                                       transformed_plan[i].pose.position.x - transformed_plan[prev_idx].pose.position.x);
            via_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, via_point_yaw));
            geometry_msgs::PoseStamped local_plan_pose = transformed_plan[i];
            local_plan_pose.pose.orientation = tf::createQuaternionMsgFromYaw(via_point_yaw);
            local_plan_.push_back(local_plan_pose);
            prev_idx = i;
        }

        
    }

    void SfyForkliftLocalPlanner::updateObstacleContainerWithCostmap()
    {
        // Add costmap obstacles if desired
        // if (cfg_.obstacles.include_costmap_obstacles)
        if (true)
        {
            double theta = tf2::getYaw(robot_pose_.pose.orientation);
            Eigen::Vector2d robot_orient = Eigen::Vector2d(std::cos(theta), std::sin(theta));

            for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
            {
                for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
                {
                    if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
                    {
                        Eigen::Vector2d obs;
                        costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                        // check if obstacle is interesting (e.g. not far behind the robot)
                        Eigen::Vector2d obs_dir = obs - Eigen::Vector2d(robot_pose_.pose.position.x, robot_pose_.pose.position.y);
                        // if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist)
                        //     continue;
                        if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > 1.0)
                            continue;

                        // if (obs_dir.norm() < 1.0)
                        //     std::cout << obs_dir.norm() << std::endl;
                        obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
                    }
                }
            }
        }
    }

    double SfyForkliftLocalPlanner::checkCollision(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double check_dist)
    {
        // 清除之前保存的碰撞点
        collision_points_.clear();
        // 存储路径点的偏航角
        double path_point_yaw;
        // 存储路径长度
        double plan_length = 0;
        // 存储最危险点的距离
        double lethal_point_distance = 999;
        // 存储当前路径点的位置
        std::size_t i = 1;
        // 遍历路径点
        while (i < transformed_plan.size() && plan_length < check_dist)
        {
            // 计算路径点与前一个路径点的偏航角
            path_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[i - 1].pose.position.y,
                                        transformed_plan[i].pose.position.x - transformed_plan[i - 1].pose.position.x);
            // 计算路径点与机器人的碰撞成本
            double footprint_cost = world_model_->footprintCost(transformed_plan[i].pose.position.x,
                                                                transformed_plan[i].pose.position.y, path_point_yaw, costmap_ros_->getRobotFootprint());

            // 计算路径长度
            plan_length += distance_points2d(transformed_plan[i].pose.position, transformed_plan[i - 1].pose.position);
            // 如果路径点与机器人的碰撞成本为-1，则将路径点添加到碰撞点列表中
            if (footprint_cost == -1)
            {
                collision_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, path_point_yaw));

                // 如果路径长度小于最危险点的距离，则更新最危险点的距离
                if (plan_length < lethal_point_distance)
                    lethal_point_distance = plan_length;
            }
            i++;
        }

        return lethal_point_distance;
    }
        
    Eigen::Vector3d SfyForkliftLocalPlanner::R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);
        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;
        return ypr;
    }

    void SfyForkliftLocalPlanner::cart2Pol(double x, double y, double &deg, double &dist)
    {
        dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        deg = std::atan2(y, x);
    }

    double SfyForkliftLocalPlanner::angDiff(double alpha, double beta)
    {
        double delta;
        delta = fmod((alpha - beta), 2.0 * M_PI);
        if (delta > M_PI)
            delta = delta - 2 * M_PI;
        else if (delta < M_PI * (-1.0))
            delta = delta + 2 * M_PI;
        return delta;
    }

    void SfyForkliftLocalPlanner::poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi)
    {
        cart2Pol(dx, dy, alpha, rho);
        alpha = angDiff(0, alpha);
        phi = dyaw;
    }

    double SfyForkliftLocalPlanner::clip(double value, double lower, double upper)
    {
        if (value < lower)
            return lower;
        else if (value > upper)
            return upper;
        else
            return value;
    }

    float SfyForkliftLocalPlanner::z_vel_pid_controller(float pv,  float sp)
    {
        // k_z_vel_pid_kp
        float Kp = cfg_.optimization.pp_front_k_p;
        float Kd = cfg_.optimization.pp_front_k_d;
        float Kd2 = cfg_.optimization.pp_front_k_i; 
        // 角速度最大输出   rad
        float max_output_pid = 3 , min_output_pid = -3 ;
        // 误差最大不超过
        float max_output_i = 5,min_output_i = -5 ;
        static float error = 0,error_last=0,error_last_last=0;
        static float output_p,output_i,output_d,output_pid; 
        // 误差 = 期望角速度  -  odom当前角速度
        error = sp - pv ;

        // 控制器各环节 输出 计算
        output_p += ( Kd * (error - error_last) );
        output_i += ( Kp * (error) );
        output_d += ( Kd2 * (error - 2*error_last + error_last_last) );

        // 更新偏差量
        error_last_last = error_last ;
        error_last = error ;
        // std::cout << "error" << error << "  error_last" << error_last << "  error_last_last" << error_last_last << std::endl;
        if(output_i>max_output_i)
        {
            output_i = max_output_i;
        }else if(output_i<min_output_i)
        {
            output_i = min_output_i;
        }        
        output_pid = output_p + output_i + output_d;
        if(output_pid>max_output_pid)
        {
            output_pid = max_output_pid;
        }else if(output_pid<min_output_pid)
        {
            output_pid = min_output_pid;
        }
        return output_pid; 
    }

    // Evaluate a polynomial.
    double SfyForkliftLocalPlanner::polyeval(Eigen::VectorXd coeffs, double x) {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }
    
    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd SfyForkliftLocalPlanner::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }



    int SfyForkliftLocalPlanner::findTargetIndexPP(double  l_d) {
        double min = abs(sqrt(pow(forklift_odom_.x - global_plan_[0].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[0].pose.position.y, 2)));
        int index = 0;
        for (int i = 0; i < global_plan_.size(); i++)
        {
            double d = abs(sqrt(pow(forklift_odom_.x - global_plan_[i].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[i].pose.position.y, 2)));
            if (d < min) 
            {
                min = d;
                index = i;
            }
        }
        
        double delta_l =  abs(sqrt(pow(forklift_odom_.x - global_plan_[index].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[index].pose.position.y, 2)));

        while (l_d > delta_l && index < global_plan_.size()-1){
            delta_l = abs(sqrt(pow(forklift_odom_.x - global_plan_[index+1].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[index+1].pose.position.y, 2)));
            index+=1;
        }
        return index;
    }

    int SfyForkliftLocalPlanner::findTargetIndexGlobalPlan()
    {
        double min = abs(sqrt(pow(forklift_odom_.x - global_plan_[0].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[0].pose.position.y, 2)));
        int index = 0;
        for (int i = 0; i < global_plan_.size(); i++)
        {
            double d = abs(sqrt(pow(forklift_odom_.x - global_plan_[i].pose.position.x, 2) + pow(forklift_odom_.y - global_plan_[i].pose.position.y, 2)));
            if (d < min)
            {
                min = d;
                index = i;
            }
        }

        //索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
        if ((index + 1) < global_plan_.size())
        {
            double current_x = global_plan_[index].pose.position.x; double current_y = global_plan_[index].pose.position.y;
            double next_x = global_plan_[index + 1].pose.position.x; double next_y = global_plan_[index + 1].pose.position.y;
            double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
            double L_1 = abs(sqrt(pow(forklift_odom_.x - next_x, 2) + pow(forklift_odom_.y - next_y, 2)));
            if (L_1 < L_)
            {
                index += 1;
            }
        }
        return index;
    }

    void SfyForkliftLocalPlanner::localPPControl(double rho, double alpha, double &v, double &omega , double &expected_steer){
        // v = cfg_.optimization.k_rho * rho; 
        v = cfg_.agv.expected_vel_x;

        int find_pp_next = findTargetIndexGlobalPlan();
        double pp_next_x = global_plan_[find_pp_next].pose.position.x;
        double pp_next_y = global_plan_[find_pp_next].pose.position.y;
        double pp_next_yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[find_pp_next].pose.orientation));
        alpha = mytools_->normalizeAngle(forklift_odom_.yaw - pp_next_yaw);

        if(is_reverse_){
            v = v * (-1.0);
        }
        // if ((fabs(alpha) > M_PI * 0.5 + 0.1) || ((fabs(alpha) > M_PI * 0.5 - 0.1) && last_back_))
        // {
        //     v = v * (-1.0);
        //     alpha = angDiff(alpha, M_PI);
        //     last_back_ = true;
        // }
        // else
        //     last_back_ = false;

        double cos_yaw = std::cos(forklift_odom_.yaw);
        double sin_yaw = std::sin(forklift_odom_.yaw);
        std::vector<double> vec_target_2_rear = {pp_next_x - forklift_odom_.x, pp_next_y - forklift_odom_.y };
        double e_lat = -vec_target_2_rear[0] *sin_yaw + vec_target_2_rear[1] * cos_yaw;
        double e_yaw = alpha;
        double k_fuzzy_distance , l_d;
        if(cfg_.forklift_chose.fuzzy_ld_model){
            k_fuzzy_distance = fuzzyController_.calculateK(abs(e_lat), abs(forklift_odom_.v));   
            l_d = k_fuzzy_distance + cfg_.agv.pp_view_distance;    
        }else{
            k_fuzzy_distance = cfg_.agv.pp_view_distance_ratio;
            l_d = cfg_.agv.pp_view_distance;
        }
       
        int find_pp_index = findTargetIndexPP(l_d);
        double pp_need_x = global_plan_[find_pp_index].pose.position.x;
        double pp_need_y = global_plan_[find_pp_index].pose.position.y;
        double pp_need_yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[find_pp_index].pose.orientation));

        double pp_need_dx = pp_need_x - forklift_odom_.x;
        double pp_need_dy = pp_need_y - forklift_odom_.y;
        // double pp_d = std::hypot(pp_dx, pp_dy);

        double alpha_pp , delta_pp;
        if (fabs(pp_need_dx) <  1e-3 && fabs(pp_need_dy) <  1e-3){
            alpha_pp = 0;
            delta_pp = 0;
        }else{
            alpha_pp = atan2(pp_need_dy , pp_need_dx) - forklift_odom_.yaw;
            if(fabs(l_d) <  1e-3){
                delta_pp = 0;
            }else{
                delta_pp = atan2(2 * cfg_.agv.forklift_wheelbase * sin(alpha_pp) , l_d); 
            }
            delta_pp = mytools_->normalizeAngle(delta_pp);
        }
        
        // cout << "pp_need_x：" <<  pp_need_x << ",  pp_need_y"  << pp_need_y << endl;
        // const double expect_steer_vel = v * tan(delta_pp) / cfg_.agv.forklift_wheelbase;
        double expect_steer_vel = delta_pp;
        if (cfg_.forklift_chose.steer_vel_delay_pid_model){
            double pp_steer_vel;
            if(is_reverse_){
                pid_controller_.setK(cfg_.optimization.pp_rear_k_p , cfg_.optimization.pp_rear_k_i , cfg_.optimization.pp_rear_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }else{
                // pp_steer_vel = z_vel_pid_controller(forklift_odom_.kesi , expect_steer_vel);
                pid_controller_.setK(cfg_.optimization.pp_front_k_p , cfg_.optimization.pp_front_k_i , cfg_.optimization.pp_front_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }               
            std::cout << "odom角速度: " << forklift_odom_.kesi << ", 期望角速度: " << expect_steer_vel << ", Pid后角速度:" << pp_steer_vel << std::endl;

        }else{ 
            omega = expect_steer_vel;
        } 

        expected_steer = delta_pp;

        // 收集数据 发布话题
        collect_pp_control_.e_lat = e_lat;
        collect_pp_control_.e_yaw = e_yaw;
        collect_pp_control_.k_fuzzy_distance = k_fuzzy_distance;
        collect_pp_control_.l_d = l_d;
        collect_pp_control_.pp_need_x = pp_need_x;
        collect_pp_control_.pp_need_y = pp_need_y;
        collect_pp_control_.pp_next_x = pp_next_x;
        collect_pp_control_.pp_next_y = pp_next_y;
        collect_pp_control_.alpha_pp = alpha_pp;
        collect_pp_control_.delta_pp = delta_pp;
        collect_pp_control_.expect_steer_vel = expect_steer_vel;
        collect_pp_control_pub_.publish(collect_pp_control_);
    }

    void SfyForkliftLocalPlanner::localPidControl(double rho, double alpha, double &v, double &omega , double &expected_steer){
        pid_controller_.setTarget(0);
        pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
        pid_controller_.setK(cfg_.optimization.pid_front_k_p , cfg_.optimization.pid_front_k_i , cfg_.optimization.pid_front_k_d);

        // v = cfg_.optimization.k_rho * rho; 
        v = cfg_.agv.expected_vel_x;

        if ((fabs(alpha) > M_PI * 0.5 + 0.1) || ((fabs(alpha) > M_PI * 0.5 - 0.1) && last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
            pid_controller_.setK(cfg_.optimization.pid_rear_k_p , cfg_.optimization.pid_rear_k_i , cfg_.optimization.pid_rear_k_d);
            
        }
        else
            last_back_ = false;
        
        // int global_index = findTargetIndexGlobalPlan();
        // double pid_need_x = global_plan_[global_index].pose.position.x;
        // double pid_need_y = global_plan_[global_index].pose.position.y;
        double pid_need_x = via_points_[0][0];
        double pid_need_y = via_points_[0][1];
        double alpha_pid = atan2(pid_need_y- forklift_odom_.y , pid_need_x - forklift_odom_.x);
        double l_d = sqrt(pow(pid_need_y-forklift_odom_.y,2) + pow(pid_need_x - forklift_odom_.x,2));
        double theta_e = alpha_pid - forklift_odom_.yaw;
        double e_y = -l_d * sin(theta_e);
        // e_y = -l_d * np.sign(math.sin(theta_e))  # 第二种误差表示
        // e_y = robot_state[1]-refer_path[ind, 1]  # 第三种误差表示
        double delta_f = pid_controller_.calOutput(e_y);
        delta_f = mytools_->normalizeAngle(delta_f);
        const double expect_steer_vel = v * tan(delta_f) / cfg_.agv.forklift_wheelbase;
        omega = expect_steer_vel;
        // if (cfg_.forklift_chose.steer_vel_delay_pid_model){
        //     double pp_steer_vel;
        //     if(last_back_){
        //         pid_controller_.setK(cfg_.optimization.pp_rear_k_p , cfg_.optimization.pp_rear_k_i , cfg_.optimization.pp_rear_k_d);
        //         pid_controller_.setTarget(0);
        //         pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
        //         double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
        //         pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
        //         omega = pp_steer_vel;
        //     }else{
        //         pp_steer_vel = z_vel_pid_controller(forklift_odom_.kesi , expect_steer_vel);
        //         omega = pp_steer_vel;
        //     }               
        //     std::cout << "odom角速度: " << forklift_odom_.kesi << ", 期望角速度: " << expect_steer_vel << ", Pid后角速度:" << pp_steer_vel << std::endl;

        // }else{ 
        //     omega = expect_steer_vel;
        // } 
         expected_steer = delta_f;
    }

    void SfyForkliftLocalPlanner::localMpcControl(double rho, double alpha, double &v, double &omega , double &expected_steer){
        int mpc_step = 5;
        mpc_params_["DT"] = control_duration_;
        mpc_params_["STEPS"]    = mpc_step;
        mpc_params_["REF_CTE"]  = 0.01;
        mpc_params_["REF_ETHETA"] = 0.01;
        mpc_params_["REF_V"]    = 0.5;
        mpc_params_["W_CTE"]    = 100;
        mpc_params_["W_EPSI"]   = 100;
        mpc_params_["W_V"]      = 10;        // 代价系数  加速度平方      100
        mpc_params_["W_ANGVEL"]  = 100;       //  代价系数  角度平方      100
        mpc_params_["W_A"]       =  30;       //  代价系数  加速度平方    50.0
        mpc_params_["W_DANGVEL"] =  50;       //  代价系数  角度差平方    3.0
        mpc_params_["W_DA"]      =  0.01;       //  代价系数  加速度差平方  10.0
        mpc_params_["ANGVEL"]   = 1;        // 前轮倾角上下线限制
        mpc_params_["MAXTHR"]   = 0.0;          // 加速度上下限制
        mpc_params_["BOUND"]    = 1.0e6;      // 其他输入参数的上下限
        // mpc_controller_.LoadParams(mpc_params_);
        mpc_base_controller_.LoadParams(mpc_params_);

        v = cfg_.agv.expected_vel_x; 
        // if ((fabs(alpha) > M_PI * 0.5 + 0.1) || ((fabs(alpha) > M_PI * 0.5 - 0.1) && last_back_))
        // {
        //     v = v * (-1.0);
        //     alpha = angDiff(alpha, M_PI);
        //     last_back_ = true;
        // }
        // else
        //     last_back_ = false;

        // int global_index_n = global_plan_.size();
        // int global_index = findTargetIndexGlobalPlan();

        // const double costheta = cos(forklift_odom_.yaw);
        // const double sintheta = sin(forklift_odom_.yaw);
        // int local_index_n = local_plan_.size();
        // VectorXd x_veh(mpc_step);
        // VectorXd y_veh(mpc_step);
        // double gx = 0;
        // double gy = 0;
        // for(int i = 0; i < mpc_step; i++) {
        //     int current_index = i+1;
        //     if (current_index < local_index_n) {
        //         const double dx = abs(local_plan_[current_index].pose.position.x - forklift_odom_.x);
        //         const double dy = abs(local_plan_[current_index].pose.position.y - forklift_odom_.y);
        //         x_veh[i] = dx * costheta + dy * sintheta;
        //         y_veh[i] = dy * costheta - dx * sintheta;
        //         gx += local_plan_[current_index].pose.position.x - local_plan_[current_index-1].pose.position.x;
        //         gy += local_plan_[current_index].pose.position.y - local_plan_[current_index-1].pose.position.y;
        //     }else{
        //         const double dx = abs(local_plan_[local_index_n-1].pose.position.x - forklift_odom_.x);
        //         const double dy = abs(local_plan_[local_index_n-1].pose.position.y - forklift_odom_.y);
        //         x_veh[i] = dx * costheta + dy * sintheta;
        //         y_veh[i] = dy * costheta - dx * sintheta;
        //         gx += local_plan_[local_index_n-1].pose.position.x - local_plan_[local_index_n-2].pose.position.x;
        //         gy += local_plan_[local_index_n-1].pose.position.y - local_plan_[local_index_n-2].pose.position.y;
        //     }
        // }

        // auto coeffs = polyfit(x_veh, y_veh, 3); 
        // cout << "coeffs : " << coeffs << endl;
        // const double cte  = polyeval(coeffs, 0.0);
        // double etheta = atan(coeffs[1]);

        // double pred_px = 0.0 + v * control_duration_; // Since psi is zero, cos(0) = 1, can leave out
        // const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
        // double pred_psi = 0.0 + forklift_odom_.kesi *control_duration_;
        // double pred_v = v + 0 * control_duration_;
        // double pred_cte = cte + v * sin(etheta) * control_duration_;
        // double pred_epsi = etheta - forklift_odom_.kesi *control_duration_;
        // // v * forklift_odom_.kesi / cfg_.agv.forklift_wheelbase * control_duration_;

        // VectorXd state(6);
        // // state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;
        // state << 0, 0, 0, forklift_odom_.v, cte, etheta;
        // vector<double> mpc_results = mpc_controller_.Solve(state, coeffs);

        // cout << "=" <<endl;
        MatrixXd coeffs(4, mpc_step);

        int local_index_n = local_plan_.size();
        cout<< "local_index_n:" << local_index_n << endl;
        for (int i = 0; i < mpc_step; i++) {
            // 计算当前索引
            int current_index = i+1;
            // 检查是否越界
            if (current_index < local_index_n) {
                coeffs(0, i) = local_plan_[current_index].pose.position.x;
                coeffs(1, i) = local_plan_[current_index].pose.position.y;
                if(last_back_){
                    coeffs(2, i) = mytools_->normalizeAngle(tf::getYaw(local_plan_[current_index].pose.orientation) - M_PI) ;
                }else{
                    coeffs(2, i) = mytools_->normalizeAngle(tf::getYaw(local_plan_[current_index].pose.orientation));
                }
                
                coeffs(3, i) = v;
            } else {
                // 如果越界，使用最后一个有效的索引
                coeffs(0, i) = local_plan_[local_index_n - 1].pose.position.x;
                coeffs(1, i) = local_plan_[local_index_n - 1].pose.position.y;
                 if(last_back_){
                    coeffs(2, i) = mytools_->normalizeAngle(tf::getYaw(local_plan_[local_index_n - 1].pose.orientation) - M_PI) ;
                }else{
                    coeffs(2, i) = mytools_->normalizeAngle(tf::getYaw(local_plan_[local_index_n - 1].pose.orientation));
                }
                coeffs(3, i) = v;
            }
        }
        VectorXd state(4);
        state << forklift_odom_.x, forklift_odom_.y, mytools_->normalizeAngle(forklift_odom_.yaw) , forklift_odom_.v;
        vector<double> mpc_results = mpc_base_controller_.Solve(state, coeffs);

        omega = mpc_results[0];
        cout << "mpc规划结果：" << omega << endl;
        omega = v * tan(mpc_results[0]) / cfg_.agv.forklift_wheelbase; 

        expected_steer = mpc_results[0];
        const double expect_steer_vel = omega;

        if (cfg_.forklift_chose.steer_vel_delay_pid_model){
            double pp_steer_vel;
            if(last_back_){
                pid_controller_.setK(cfg_.optimization.pid_front_k_p , cfg_.optimization.pid_front_k_i , cfg_.optimization.pid_front_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }else{
                pid_controller_.setK(cfg_.optimization.pid_front_k_p , cfg_.optimization.pid_front_k_i , cfg_.optimization.pid_front_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }               
            std::cout << "odom角速度: " << forklift_odom_.kesi << ", 期望角速度: " << expect_steer_vel << ", Pid后角速度:" << pp_steer_vel << std::endl;

        }else{ 
            omega = expect_steer_vel;
        } 
        

    }

    void SfyForkliftLocalPlanner::localLqrDynamicControl(double rho, double alpha, double &v, double &omega , double &expected_steer){
        v = cfg_.agv.expected_vel_x; 

        int find_pp_next = findTargetIndexGlobalPlan();
        double pp_next_x = global_plan_[find_pp_next].pose.position.x;
        double pp_next_y = global_plan_[find_pp_next].pose.position.y;
        double pp_next_yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[find_pp_next].pose.orientation));
        alpha = mytools_->normalizeAngle(forklift_odom_.yaw - pp_next_yaw);
        if ((fabs(alpha) > M_PI * 0.5 + 0.1) || ((fabs(alpha) > M_PI * 0.5 - 0.1) && last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;

        double cos_yaw = std::cos(forklift_odom_.yaw);
        double sin_yaw = std::sin(forklift_odom_.yaw);
        std::vector<double> vec_target_2_rear = {forklift_odom_.x - pp_next_x, forklift_odom_.y - pp_next_y};
        double e_lat = abs(vec_target_2_rear[0] *cos_yaw + vec_target_2_rear[1] * sin_yaw);
        double e_yaw = abs(alpha);
        double k_fuzzy_distance , l_d;
        if(cfg_.forklift_chose.fuzzy_ld_model){
            k_fuzzy_distance = fuzzyController_.calculateK(e_lat*2, abs(forklift_odom_.v));   
            l_d = k_fuzzy_distance + cfg_.agv.pp_view_distance;    
        }else{
            k_fuzzy_distance = cfg_.agv.pp_view_distance_ratio;
            l_d = cfg_.agv.pp_view_distance;
        }
        
        int find_pp_index = findTargetIndexPP(l_d);
        double pp_need_x = global_plan_[find_pp_index].pose.position.x;
        double pp_need_y = global_plan_[find_pp_index].pose.position.y;

        double alpha_pp = atan2(pp_need_y- forklift_odom_.y , pp_need_x-forklift_odom_.x) - forklift_odom_.yaw;
        double delta_pp = atan2(2 * cfg_.agv.forklift_wheelbase * sin(alpha_pp) , l_d); 
        delta_pp = mytools_->normalizeAngle(delta_pp);
        
        waypoint Point , Point_Next;
        U U_current , U_result;

        int target_index = find_pp_next;

        if(target_index < global_plan_.size() - 1){
            Point.x = global_plan_[target_index].pose.position.x;
            Point.y = global_plan_[target_index].pose.position.y;
            Point.yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[target_index].pose.orientation));

            Point_Next.x = global_plan_[target_index + 1].pose.position.x;
            Point_Next.y = global_plan_[target_index + 1].pose.position.y;
            Point_Next.yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[target_index + 1].pose.orientation));
        }else{
            Point.x = global_plan_[target_index].pose.position.x;
            Point.y = global_plan_[target_index].pose.position.y;
            Point.yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[target_index].pose.orientation));

            Point_Next.x = global_plan_[target_index].pose.position.x;
            Point_Next.y = global_plan_[target_index].pose.position.y;
            Point_Next.yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[target_index].pose.orientation));
        }
       
        if(last_back_){
            Point.yaw = angDiff(Point.yaw , M_PI);
            Point_Next.yaw = angDiff(Point_Next.yaw , M_PI);
        }
        double K = mytools_ ->cal_k_poseStamped(global_plan_, target_index); //计算曲率
       
        //减速判断
        const double kesi = atan2(cfg_.agv.forklift_wheelbase * K, 1);
        
        U_current.v = v;
        U_current.kesi = delta_pp;

        vehicle_dynamic_param_.L = cfg_.agv.forklift_wheelbase;
        vehicle_dynamic_param_.lr = cfg_.agv.forklift_wheelbase * (1-cfg_.lqr.lf);
        vehicle_dynamic_param_.lf = cfg_.agv.forklift_wheelbase * cfg_.lqr.lf;
        vehicle_dynamic_param_.cf = cfg_.lqr.cf;
        vehicle_dynamic_param_.cr = cfg_.lqr.cr;    
        vehicle_dynamic_param_.mass = cfg_.lqr.mass;
        vehicle_dynamic_param_.mass_front = cfg_.lqr.mass * cfg_.lqr.lf;
        vehicle_dynamic_param_.mass_rear = cfg_.lqr.mass * (1-cfg_.lqr.lf);

        vehicle_dynamic_param_.iz = 
            vehicle_dynamic_param_.lf *vehicle_dynamic_param_.lf * vehicle_dynamic_param_.mass_front + 
            vehicle_dynamic_param_.lr * vehicle_dynamic_param_.lr * vehicle_dynamic_param_.mass_rear;

        vehicle_dynamic_param_.delta_t = control_duration_;
        vehicle_dynamic_param_.horizon = cfg_.lqr.horizon;
        vehicle_dynamic_param_.current_v = forklift_odom_.v;
        vehicle_dynamic_param_.current_w = forklift_odom_.kesi;
        vehicle_dynamic_param_.target_index_K = K;
        
        double q1 = cfg_.lqr.lqr_dynamic_q1;
        double q2 = cfg_.lqr.lqr_dynamic_q2;
        double q3 = cfg_.lqr.lqr_dynamic_q3;
        double q4 = cfg_.lqr.lqr_dynamic_q4;
        double r1 = cfg_.lqr.lqr_dynamic_r1;
        vector<double> Q_set = {q1, q2, q3 ,q4};
        vector<double> R_set = {r1};

        double Q[4];
	    double R[1];
        for(int q =0;q < Q_set.size(); q++) Q[q] = Q_set[q];
        for(int r =0;r< R_set.size();r++) R[r] = R_set[r];
        if(Q[0]>=R[0])
            ROS_WARN("Q >= R, the calculation result may be abnormal,please set R > Q ");
        
        lqr_dynamic_controller_.initial(vehicle_dynamic_param_, 
            control_duration_, forklift_odom_, Point, Point_Next , U_current, Q, R);
        U_result = lqr_dynamic_controller_.cal_vel();  
        vector<double> get_all_error = lqr_dynamic_controller_.get_all_error();

        omega = U_result.kesi;
        expected_steer = U_result.kesi;
        cout << "lqr omega" << omega << endl;
        const double expect_steer_vel = v * tan(omega) / cfg_.agv.forklift_wheelbase;
        if (cfg_.forklift_chose.steer_vel_delay_pid_model){
            double pp_steer_vel;
            if(last_back_){
                pid_controller_.setK(cfg_.optimization.pp_rear_k_p , cfg_.optimization.pp_rear_k_i , cfg_.optimization.pp_rear_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }else{
                pp_steer_vel = z_vel_pid_controller(forklift_odom_.kesi , expect_steer_vel);
                omega = pp_steer_vel;
            }               
            std::cout << "odom角速度: " << forklift_odom_.kesi << ", 期望角速度: " << expect_steer_vel << ", Pid后角速度:" << pp_steer_vel << std::endl;

        }else{ 
            omega = expect_steer_vel;
        } 
    }

    void SfyForkliftLocalPlanner::localLqrKinematicControl(double rho, double alpha, double &v, double &omega , double &expected_steer){
        v = cfg_.agv.expected_vel_x; 
        
        int find_pp_next = findTargetIndexGlobalPlan();
        double pp_next_x = global_plan_[find_pp_next].pose.position.x;
        double pp_next_y = global_plan_[find_pp_next].pose.position.y;
        double pp_next_yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[find_pp_next].pose.orientation));
        alpha = mytools_->normalizeAngle(forklift_odom_.yaw - pp_next_yaw);
        if ((fabs(alpha) > M_PI * 0.5 + 0.1) || ((fabs(alpha) > M_PI * 0.5 - 0.1) && last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;

        double cos_yaw = std::cos(forklift_odom_.yaw);
        double sin_yaw = std::sin(forklift_odom_.yaw);
        std::vector<double> vec_target_2_rear = {forklift_odom_.x - pp_next_x, forklift_odom_.y - pp_next_y};
        double e_lat = abs(vec_target_2_rear[0] *cos_yaw + vec_target_2_rear[1] * sin_yaw);
        double e_yaw = abs(alpha);
        double k_fuzzy_distance , l_d;
        if(cfg_.forklift_chose.fuzzy_ld_model){
            k_fuzzy_distance = fuzzyController_.calculateK(e_lat*2, abs(forklift_odom_.v));   
            l_d = k_fuzzy_distance + cfg_.agv.pp_view_distance;    
        }else{
            k_fuzzy_distance = cfg_.agv.pp_view_distance_ratio;
            l_d = cfg_.agv.pp_view_distance;
        }
        
        int find_pp_index = findTargetIndexPP(l_d);
        double pp_need_x = global_plan_[find_pp_index].pose.position.x;
        double pp_need_y = global_plan_[find_pp_index].pose.position.y;

        double alpha_pp = atan2(pp_need_y- forklift_odom_.y , pp_need_x-forklift_odom_.x) - forklift_odom_.yaw;
        double delta_pp = atan2(2 * cfg_.agv.forklift_wheelbase * sin(alpha_pp) , l_d); 
        delta_pp = mytools_->normalizeAngle(delta_pp);

        waypoint Point;
        U U_current , U_result;
        int target_index = find_pp_index;
        
        Point.x = global_plan_[target_index].pose.position.x;
        Point.y = global_plan_[target_index].pose.position.y;
        Point.yaw = mytools_->normalizeAngle(tf::getYaw(global_plan_[target_index].pose.orientation));
        
        if(last_back_){
            Point.yaw = angDiff(Point.yaw , M_PI);
        }

        cout << Point.x << "  " << Point.y << "  " << Point.yaw << endl;
        double K = mytools_->cal_k_poseStamped(global_plan_, target_index); //计算曲率

        //减速判断
        const double kesi = atan2(cfg_.agv.forklift_wheelbase * K, 1);
    
        U_current.v = v;
        U_current.kesi = delta_pp;

        double q1 = cfg_.lqr.lqr_kinematic_q1;
        double q2 = cfg_.lqr.lqr_kinematic_q2;
        double q3 = cfg_.lqr.lqr_kinematic_q3;
        double r1 = cfg_.lqr.lqr_kinematic_r1;
        double r2 = cfg_.lqr.lqr_kinematic_r2;

        vector<double> Q_set = {q1, q2, q3};
        vector<double> R_set = {r1 , r2};

        double Q[3];
	    double R[2];
        for(int q =0;q < Q_set.size(); q++) Q[q] = Q_set[q];
        for(int r =0;r< R_set.size();r++) R[r] = R_set[r];
        cout <<  " U_current.v " << U_current.v << endl;
        cout <<  " U_current.kesi " << U_current.kesi << endl;
        
        lqr_kinematic_controller_.initial(cfg_.agv.forklift_wheelbase, control_duration_, forklift_odom_, Point, U_current, Q, R);
        U_result = lqr_kinematic_controller_.cal_vel();  
        
        omega = U_result.kesi;
        expected_steer = U_result.kesi;
        
        const double expect_steer_vel = v * tan(omega) / cfg_.agv.forklift_wheelbase;
        // const double expect_steer_vel = omega;

        if (cfg_.forklift_chose.steer_vel_delay_pid_model){
            double pp_steer_vel;
            if(last_back_){
                pid_controller_.setK(cfg_.optimization.pp_rear_k_p , cfg_.optimization.pp_rear_k_i , cfg_.optimization.pp_rear_k_d);
                pid_controller_.setTarget(0);
                pid_controller_.setBound(cfg_.agv.max_vel_theta , cfg_.agv.max_vel_theta * (-1.0));
                double pid_steer_vel_error = expect_steer_vel - forklift_odom_.kesi;
                pp_steer_vel = pid_controller_.calOutput(pid_steer_vel_error);
                omega = pp_steer_vel;
            }else{
                pp_steer_vel = z_vel_pid_controller(forklift_odom_.kesi , expect_steer_vel);
                omega = pp_steer_vel;
            }               
            std::cout << "odom角速度: " << forklift_odom_.kesi << ", 期望角速度: " << expect_steer_vel << ", Pid后角速度:" << pp_steer_vel << std::endl;

        }else{ 
            omega = expect_steer_vel;
        } 
        
    }

    void SfyForkliftLocalPlanner::forkliftInfoCB(const sfy_forklift_msgs::ForkliftInfo::ConstPtr&  Msg)
    {
        forklift_info_ = *Msg;
    }

    void SfyForkliftLocalPlanner::locationEkfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Msg)
    {
        forklift_location_ekf_ = *Msg;
    }

    void SfyForkliftLocalPlanner::calTimer(const ros::TimerEvent&)
    {
        // // cout << "calTimer sfy!!!!!!!!!!!!!1" << endl;
        // checkSubscriberStatus();

        // local_plan_goal_reached_.flag = goal_reached_;
        // local_plan_goal_reached_pub_.publish(local_plan_goal_reached_);

        // if(is_forklift_path_planning_result_sub_ && forklift_path_planning_result_.is_start_control){
        //     forklift_path_tracking_status_.current_mission_completed = false;
        //     forklift_path_tracking_status_.current_goal_reached = goal_reached_;
        // }else{
        //     // goal_reached_ = false;
        //     forklift_path_tracking_status_.current_mission_completed = false;
        //     forklift_path_tracking_status_.current_goal_reached = false;
        // }
        
        // // cout << "goal_reached_:" << goal_reached_ << endl;
        // forklift_path_tracking_status_pub_.publish(forklift_path_tracking_status_);
    }

    void SfyForkliftLocalPlanner::forkliftPathPlanningResultCB(const sfy_forklift_msgs::ForkliftPathPlanningResult::ConstPtr& Msg)
    {
        forklift_path_planning_result_ = *Msg;
        is_reverse_ = forklift_path_planning_result_.is_reverse;
    } 

    void SfyForkliftLocalPlanner::checkSubscriberStatus() {
    // 1.获取当前所有活跃话题
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    // 2. 将活跃话题名称存入哈希集合（快速查找）
    std::unordered_set<std::string> active_topics;
    for (const auto& topic_info : topic_infos) {
        active_topics.insert(topic_info.name);
        // ROS_DEBUG("Active Topic: %s (Type: %s)", topic_info.name.c_str(), topic_info.datatype.c_str());
    }
    if (active_topics.count("/forklift_path_planning_result") > 0) {

        is_forklift_path_planning_result_sub_ = true;
    } else {

         is_forklift_path_planning_result_sub_ = false;
    }
}
    

}