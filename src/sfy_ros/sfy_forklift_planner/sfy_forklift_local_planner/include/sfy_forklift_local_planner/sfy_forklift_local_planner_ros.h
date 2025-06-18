/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-07 15:33:36
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-05-26 17:07:40
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_local_planner/include/sfy_forklift_local_planner/sfy_forklift_local_planner_ros.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#ifndef SFY_FORKLIFT_LOCAL_PLANNER_ROS_H_
#define SFY_FORKLIFT_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <algorithm>

#include <sfy_forklift_local_planner/misc.h>
#include <sfy_forklift_local_planner/sfy_forklift_local_config.h>
#include <sfy_forklift_local_planner/visualization.h>
#include <sfy_forklift_local_planner/one_euro_filter.h>
#include <sfy_forklift_local_planner/mytools.h>
#include "sfy_controller_algorithm/pid_controller.h"
// #include "sfy_controller_algorithm/mpc_waypoint_tracking_controller.h"
#include "sfy_controller_algorithm/mpc_base_controller.h"
#include "sfy_controller_algorithm/lqr_dynamic_control.h"
#include "sfy_controller_algorithm/lqr_kinematic_control.h"
#include "sfy_controller_algorithm/fuzzy_control.h"

#include "sfy_forklift_msgs/ForkliftLocation.h"
#include "sfy_forklift_msgs/ForkliftFlag.h"
#include "sfy_forklift_msgs/ForkliftPurePursuit.h"
#include "sfy_forklift_msgs/ForkliftInfo.h"
#include "sfy_forklift_msgs/ForkliftCmdVel.h"
#include "sfy_forklift_msgs/ForkliftPathPlanningResult.h"
#include "sfy_forklift_msgs/ForkliftPathTrackingStatus.h"
namespace sfy_forklift_local_planner
{
    class SfyForkliftLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        SfyForkliftLocalPlanner();

        ~SfyForkliftLocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();
         
    protected:
        template <typename P1, typename P2>

        inline double distancePoints2d(const P1 &point1, const P2 &point2)
        {
            return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
        };

        // 导入需要调参的变量cfg
        void reconfigureCB(SfyForkliftLocalPlannerReconfigureConfig &config, uint32_t level);

        // 2d路径平滑
        void smoothPlan2d(std::vector<geometry_msgs::PoseStamped> &global_plan);

        // 简化全局路径规划  2d路径平滑调用函数
        void simplifyGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan, double simplify_separation);

        // 裁剪全局路径，即去除全局路径中那些在机器人之前的位置
        bool pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose,
                             std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot = 1);

        // 将给定的全局路径转换为局部路径                
        bool transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                 const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap,
                                 const std::string &global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped> &transformed_plan,
                                 int *current_goal_idx = NULL, geometry_msgs::TransformStamped *tf_plan_to_global = NULL) const;

        void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation_via, double min_separation_goal);

        // 更新障碍物的局部代价地图
        void updateObstacleContainerWithCostmap();

        // 检查给定路径上是否存在潜在的碰撞，并找到距离AGV最近的碰撞距离
        double checkCollision(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double check_dist);
        
        // 将旋转矩阵转换为y-p-r
        Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);

        void poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi);

        // 修正角度 [-pi, pi]
        double angDiff(double alpha, double beta);

        // 计算点到原点的dist 和 斜率
        void cart2Pol(double x, double y, double &deg, double &dist);
        
        // 控制value范围 [lower, upper]
        double clip(double value, double lower, double upper); 

        // 角速度pid控制器
		float z_vel_pid_controller(float pv,float sp);

        double polyeval(Eigen::VectorXd coeffs, double x);

        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        // PP算法找到距离AGV前视距离为l_d最近的点
        int findTargetIndexPP(double l_d);

        // 全局路径找到离AGV最近的点
        int findTargetIndexGlobalPlan();

        // PP控制算法
        void localPPControl(double rho, double alpha, double &v, double &omega , double &expected_steer);

        // Pid控制算法
        void localPidControl(double rho, double alpha, double &v, double &omega , double &expected_steer);

        // MPC控制算法
        void localMpcControl(double rho, double alpha, double &v, double &omega , double &expected_steer);

        // LQR Dynamic控制算法
        void localLqrDynamicControl(double rho, double alpha, double &v, double &omega , double &expected_steer);

        // LQR Kinematic控制算法
        void localLqrKinematicControl(double rho, double alpha, double &v, double &omega , double &expected_steer);

        // 回调函数  叉车信息
        void forkliftInfoCB(const sfy_forklift_msgs::ForkliftInfo::ConstPtr&  Msg);

        // 回调函数
        void locationEkfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Msg);

        // 独立于movebase的定时器
        void calTimer(const ros::TimerEvent&);

        // 检查话题订阅状态
        void checkSubscriberStatus();
        
    private:
        // 构造函数
        ros::NodeHandle nt_; // 节点句柄
        ros::Timer myTimer_; // 定时器
            
        // 构造函数的参数
        bool initialized_;    // 本节点是否被初始化
        bool goal_reached_;   // 是否到达终点
        base_local_planner::WorldModel *world_model_;
        boost::shared_ptr<dynamic_reconfigure::Server<SfyForkliftLocalPlannerReconfigureConfig>> dynamic_recfg_; // 动态参数的共享指针
        tf2_ros::Buffer *tf_;
        
        // initialize初始化参数
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        std::string global_frame_;                              // 全局坐标系名称
        std::string robot_base_frame_;                          // 机器人base坐标系名称
        SfyForkliftConfig cfg_;                                 // 动态配置参数cfg文件
        ObstContainer obstacles_;                               // 障碍物
        SfyForkliftVisualizationPtr visualization_;             // 可视化工具指针  需要cfg的nh 和 global_frame_
        base_local_planner::OdometryHelperRos odom_helper_;     // 获取里程计
        double control_duration_;                               // 控制频率
        ros::Subscriber forklift_info_sub_;                     // 叉车发布的信息
        sfy_forklift_msgs::ForkliftInfo forklift_info_;
        ros::Subscriber forklift_location_ekf_sub_;             // 融合定位信息
        geometry_msgs::PoseWithCovarianceStamped forklift_location_ekf_;
                               
        
        // setPlan() 设置参数
        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
        double xy_reached_ = false;
        bool last_back_ = false;      
        bool is_reverse_ = false;                              // AGV是否是倒车
        bool is_goal_slow_ = false;                             // 到达终点前是否开始减速
        OneEuroFilter omega_filter_ = OneEuroFilter(ros::Time::now(), 0.0, 0.0, 1.0, 0.3, 1.0);
        geometry_msgs::Twist last_cmd_;                             // 上次的cmd_vel
        double slow_mini_goal_dist_;

        
        ros::Publisher collect_pp_control_pub_;                     // 发布话题 收集AGV在PP算法下收集的数据
        sfy_forklift_msgs::ForkliftPurePursuit collect_pp_control_; // AGV在PP算法下收集的数据
        ros::Publisher forklift_cmd_vel_pub_;                       // 发布自定义cmd_vel话题
        sfy_forklift_msgs::ForkliftCmdVel forklift_cmd_vel_;
         
        // computeVelocityCommands() 设置参数
        geometry_msgs::Twist robot_vel_; 
        geometry_msgs::PoseStamped robot_pose_;
        SfyMotionController::MyTools *mytools_;                      // 阿源的米奇妙妙工具箱
        vehicleState forklift_odom_;                                                // AGV状态
        double lethal_distance_;                                                    // 致命距离
        double dec_ratio_;                                                          // 减速比例
        sfy_forklift_msgs::ForkliftLocation forklift_location_odom_; // AGV的odom位置信息
        
        // updateViaPointsContainer() 设置参数
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> via_points_;
        std::vector<geometry_msgs::PoseStamped> local_plan_;

        // checkCollision（）设置参数
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> collision_points_;

        // 控制算法
        SfyMotionController::PID_Controller pid_controller_;    // PID控制器类
        // MPC mpc_controller_;                                 // MPC控制器类
        MpcBase mpc_base_controller_;
        map<string, double> mpc_params_;
        SfyMotionController::LQR_Dynamic_Control lqr_dynamic_controller_; // LQR Dynamic控制器类
        vehicleDynamicParam  vehicle_dynamic_param_;
        SfyMotionController::LQR_Kinematic_Control lqr_kinematic_controller_; // LQR kinematic 控制器
        SfyMotionController::FuzzyController fuzzyController_; // 模糊控制器


        //话题发布
        ros::Publisher local_plan_goal_reached_pub_;                // 发布话题 判断局部路径规划是否到达终点
        sfy_forklift_msgs::ForkliftFlag local_plan_goal_reached_;   // bool 判断局部路径规划是否到达终点

        ros::Publisher forklift_path_tracking_status_pub_; 
        sfy_forklift_msgs::ForkliftPathTrackingStatus forklift_path_tracking_status_; // 叉车路径跟踪状态


        // 话题订阅
        ros::Subscriber forklift_path_planning_result_sub_; 
        sfy_forklift_msgs::ForkliftPathPlanningResult forklift_path_planning_result_;
        bool is_forklift_path_planning_result_sub_ = false; // 是否订阅了叉车路径规划结果话
        void forkliftPathPlanningResultCB(const sfy_forklift_msgs::ForkliftPathPlanningResult::ConstPtr& Msg);


        
    };
};

#endif