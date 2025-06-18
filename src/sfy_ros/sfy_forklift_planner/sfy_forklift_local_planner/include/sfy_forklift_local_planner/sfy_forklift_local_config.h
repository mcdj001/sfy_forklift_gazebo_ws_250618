/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-07 13:47:30
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-03-04 14:01:29
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_local_planner/include/sfy_forklift_local_planner/sfy_forklift_local_config.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#ifndef SFY_FORKLIFT_LOCAL_CONFIG_H_
#define SFY_FORKLIFT_LOCAL_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <sfy_forklift_local_planner/SfyForkliftLocalPlannerReconfigureConfig.h>

namespace sfy_forklift_local_planner
{   
    class SfyForkliftConfig
    {
    public:
        std::string odom_topic;

        //! agv related parameters
        struct AGV
        {
            double expected_vel_x;      //!< Expected translational velocity of the agv
            double max_vel_x;           //!< Maximum translational velocity of the agv
            double max_vel_x_backwards; //!< Maximum translational velocity of the agv for driving backwards
            double max_vel_theta;       //!< Maximum angular velocity of the agv
            double acc_lim_x;           //!< Maximum translational acceleration of the agv
            double acc_lim_theta;       //!< Maximum angular acceleration of the agv
            double min_turn_radius;     //!< Minimum turning radius of the agv
            bool turn_around_priority;  //!< If true, the agv preferentially adjusts the orientation to fit the direction of the path
            double stop_dist;           //!< When the Euclidean distance between the nearst lethal point on planned path and the agv frame origin is less than this distance, the agv stops
            double dec_dist;            //!< When the Euclidean distance between the nearst lethal point on planned path and the agv frame origin is less than this distance, the agv slows down
            double forklift_wheelbase;  //!< The distance between the front and rear wheels of the forklift
            double pp_view_distance_ratio ;  //!< pp algorithm past life distance ratio
            double pp_view_distance ;        //!< pp algorithm past life distance
            double slow_level1_distance;     //!< slow level1 distance
            double slow_level1_vel;          //!< slow level1 velocity
        } agv;

        //! Goal tolerance related parameters
        struct GoalTolerance
        {
            double yaw_goal_tolerance; //!< Allowed final orientation error
            double xy_goal_tolerance;  //!< Allowed final euclidean distance to the goal position
        } goal_tolerance;              //!< Goal tolerance related parameters

        //! Trajectory related parameters
        struct Trajectory
        {
            double max_global_plan_lookahead_dist; //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the length is also bounded by the local costmap size!]
            double global_plan_viapoint_sep;       //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
            double global_plan_goal_sep;           //!< Min. separation between the last via-point and goal pose
            double global_plan_prune_distance;     //!< Distance between agv and via_points of global plan which is used for pruning
        } trajectory;

        //! Optimization related parameters
        struct Optimization
        {
            double k_rho;   //!< Proportional parameter for linear velocity adjustment based on the Euclidean distance of the agv position to the current target
            double k_alpha; //!< Proportional parameter for angular velocity adjustment based on the tangential angle of the target position in the agv's frame of reference
            double k_phi;   //!< Proportional parameter for angular velocity adjustment based on the difference between the agv's orientation(yaw) and the current target orientation(yaw) 
            double pp_front_k_p;
            double pp_front_k_i;
            double pp_front_k_d;
            double pp_rear_k_p;
            double pp_rear_k_i;
            double pp_rear_k_d;
            double pid_front_k_p;
            double pid_front_k_i;
            double pid_front_k_d;
            double pid_rear_k_p;
            double pid_rear_k_i;
            double pid_rear_k_d;
        } optimization;

        struct ForkliftChose
        {
            std::string control_model;       //!< The model of the forklift control
            bool steer_vel_delay_pid_model;  // 是否开角速度延迟PID模型
            bool debug_model;                    // 是否开启调试模型
            bool fuzzy_ld_model;                // 是否开启模糊控制模型调整前视距离
            bool use_forklift_odom_combined;        // 是否使用/odom_combined 话题的数据
        } forklift_chose;


        struct LQR
        {
            double lqr_dynamic_q1;
            double lqr_dynamic_q2;
            double lqr_dynamic_q3;
            double lqr_dynamic_q4;
            double lqr_dynamic_r1;
            double lqr_kinematic_q1;
            double lqr_kinematic_q2;
            double lqr_kinematic_q3;
            double lqr_kinematic_r1;
            double lqr_kinematic_r2;
            double lf;
            double cf;
            double cr;
            double mass;
            double horizon;
        } lqr;

        SfyForkliftConfig()
        {
            odom_topic = "odom";

            // AGV
            agv.expected_vel_x = 0.5;
            agv.max_vel_x = 1.0;
            agv.max_vel_x_backwards = 0.2;
            agv.max_vel_theta = 0.5;
            agv.acc_lim_x = 0.2;
            agv.acc_lim_theta = 0.2;
            agv.min_turn_radius = 0.0;
            agv.turn_around_priority = false;
            agv.stop_dist = 0.5;
            agv.dec_dist = 1.0;
            agv.forklift_wheelbase = 1.462;
            agv.pp_view_distance_ratio = 0.1;
            agv.pp_view_distance = 0.5;
            agv.slow_level1_distance = 2;
            agv.slow_level1_vel = 0.5;

            // GoalTolerance
            goal_tolerance.xy_goal_tolerance = 0.2;
            goal_tolerance.yaw_goal_tolerance = 0.2;

            // Trajectory
            trajectory.max_global_plan_lookahead_dist = 3.0;
            trajectory.global_plan_viapoint_sep = 0.5;
            trajectory.global_plan_goal_sep = 0.8;
            trajectory.global_plan_prune_distance = 0.2;

            // Optimization
            optimization.k_rho = 1.0;
            optimization.k_alpha = -3.0;
            optimization.k_phi = -1.0;
            optimization.pp_front_k_p = 0.1;
            optimization.pp_front_k_i = 0.0;
            optimization.pp_front_k_d = 0.0;
            optimization.pp_rear_k_p = 3;
            optimization.pp_rear_k_i = 0.05;
            optimization.pp_rear_k_d = 50;
            optimization.pid_front_k_p = 3;
            optimization.pid_front_k_i = 0.05;
            optimization.pid_front_k_d = 50;
            optimization.pid_rear_k_p = 3;
            optimization.pid_rear_k_i = 0.05;
            optimization.pid_rear_k_d = 50;

            //ForliftChose
            forklift_chose.control_model = "pp";
            forklift_chose.steer_vel_delay_pid_model = true;
            forklift_chose.debug_model = false;
            forklift_chose.fuzzy_ld_model = true;
            forklift_chose.use_forklift_odom_combined = false;
            
            // LQR
            lqr.lqr_dynamic_q1 = 1.0;
            lqr.lqr_dynamic_q2 = 1.0;
            lqr.lqr_dynamic_q3 = 1.0;
            lqr.lqr_dynamic_q4 = 1.0;
            lqr.lqr_dynamic_r1 = 2.0;
            lqr.lqr_kinematic_q1 = 1.0;
            lqr.lqr_kinematic_q2 = 1.0;
            lqr.lqr_kinematic_q3 = 1.0;
            lqr.lqr_kinematic_r1 = 2.0;
            lqr.lqr_kinematic_r2 = 2.0;
            lqr.lf = 0.5;
            lqr.cf = 155494.663;
            lqr.cr = 155494.663;
            lqr.mass = 2080;
            lqr.horizon = 100;
        }
        
        void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
        void reconfigure(SfyForkliftLocalPlannerReconfigureConfig &cfg);

        /**
         * @brief Return the internal config mutex
         */
        boost::mutex &configMutex() { return config_mutex_; }

    private:
        boost::mutex config_mutex_; //!< Mutex for config accesses and changes
    };
};

#endif