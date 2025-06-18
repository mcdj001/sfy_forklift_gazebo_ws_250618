/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-07 13:45:57
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-03-04 14:00:48
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_local_planner/src/sfy_forklift_local_config.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#include <sfy_forklift_local_planner/sfy_forklift_local_config.h>

namespace sfy_forklift_local_planner
{
    void SfyForkliftConfig::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
    {
        nh.param("odom_topic", odom_topic, odom_topic);

        // AGV
        nh.param("expected_vel_x", agv.expected_vel_x, agv.expected_vel_x);
        nh.param("max_vel_x", agv.max_vel_x, agv.max_vel_x);
        nh.param("max_vel_x_backwards", agv.max_vel_x_backwards, agv.max_vel_x_backwards);
        nh.param("max_vel_theta", agv.max_vel_theta, agv.max_vel_theta);
        nh.param("acc_lim_x", agv.acc_lim_x, agv.acc_lim_x);
        nh.param("acc_lim_theta", agv.acc_lim_theta, agv.acc_lim_theta);
        nh.param("min_turn_radius", agv.min_turn_radius, agv.min_turn_radius);
        nh.param("turn_around_priority", agv.turn_around_priority, agv.turn_around_priority);
        nh.param("stop_dist", agv.stop_dist, agv.stop_dist);
        nh.param("dec_dist", agv.dec_dist, agv.dec_dist);
        nh.param("forklift_wheelbase", agv.forklift_wheelbase, agv.forklift_wheelbase);
        nh.param("pp_view_distance", agv.pp_view_distance, agv.pp_view_distance);
        nh.param("pp_view_distance_ratio", agv.pp_view_distance_ratio, agv.pp_view_distance_ratio);
        nh.param("slow_level1_distance", agv.slow_level1_distance, agv.slow_level1_distance);
        nh.param("slow_level1_vel", agv.slow_level1_vel, agv.slow_level1_vel);
        
        // GoalTolerance
        nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
        nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);

        // Trajectory
        nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
        nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep);
        nh.param("global_plan_goal_sep", trajectory.global_plan_goal_sep, trajectory.global_plan_goal_sep);
        nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);

        // Optimization
        nh.param("k_rho", optimization.k_rho, optimization.k_rho);
        nh.param("k_alpha", optimization.k_alpha, optimization.k_alpha);
        nh.param("k_phi", optimization.k_phi, optimization.k_phi);
        nh.param("pp_front_k_p", optimization.pp_front_k_p, optimization.pp_front_k_p);
        nh.param("pp_front_k_i", optimization.pp_front_k_i, optimization.pp_front_k_i);
        nh.param("pp_front_k_d", optimization.pp_front_k_d, optimization.pp_front_k_d);
        nh.param("pp_rear_k_p", optimization.pp_rear_k_p, optimization.pp_rear_k_p);
        nh.param("pp_rear_k_i", optimization.pp_rear_k_i, optimization.pp_rear_k_i);
        nh.param("pp_rear_k_d", optimization.pp_rear_k_d, optimization.pp_rear_k_d);
        nh.param("pid_front_k_p", optimization.pid_front_k_p, optimization.pid_front_k_p);
        nh.param("pid_front_k_i", optimization.pid_front_k_i, optimization.pid_front_k_i);
        nh.param("pid_front_k_d", optimization.pid_front_k_d, optimization.pid_front_k_d);
        nh.param("pid_rear_k_p", optimization.pid_rear_k_p, optimization.pid_rear_k_p);
        nh.param("pid_rear_k_i", optimization.pid_rear_k_i, optimization.pid_rear_k_i);
        nh.param("pid_rear_k_d", optimization.pid_rear_k_d, optimization.pid_rear_k_d);
        
        //ForkliftChose
        nh.param("control_model", forklift_chose.control_model, forklift_chose.control_model);
        nh.param("steer_vel_delay_pid_model", forklift_chose.steer_vel_delay_pid_model, forklift_chose.steer_vel_delay_pid_model);
        nh.param("debug_model", forklift_chose.debug_model, forklift_chose.debug_model);
        nh.param("fuzzy_ld_model", forklift_chose.fuzzy_ld_model, forklift_chose.fuzzy_ld_model);
        nh.param("use_forklift_odom_combined", forklift_chose.use_forklift_odom_combined, forklift_chose.use_forklift_odom_combined);
        
        // LQR
        nh.param("lqr_dynamic_q1", lqr.lqr_dynamic_q1, lqr.lqr_dynamic_q1);
        nh.param("lqr_dynamic_q2", lqr.lqr_dynamic_q2, lqr.lqr_dynamic_q2);
        nh.param("lqr_dynamic_q3", lqr.lqr_dynamic_q3, lqr.lqr_dynamic_q3);
        nh.param("lqr_dynamic_q4", lqr.lqr_dynamic_q4, lqr.lqr_dynamic_q4);
        nh.param("lqr_dynamic_r1", lqr.lqr_dynamic_r1, lqr.lqr_dynamic_r1);
        nh.param("lqr_kinematic_q1", lqr.lqr_kinematic_q1, lqr.lqr_kinematic_q1);
        nh.param("lqr_kinematic_q2", lqr.lqr_kinematic_q2, lqr.lqr_kinematic_q2);
        nh.param("lqr_kinematic_q3", lqr.lqr_kinematic_q3, lqr.lqr_kinematic_q3);
        nh.param("lqr_kinematic_r1", lqr.lqr_kinematic_r1, lqr.lqr_kinematic_r1);
        nh.param("lqr_kinematic_r2", lqr.lqr_kinematic_r2, lqr.lqr_kinematic_r2);
        nh.param("lf", lqr.lf, lqr.lf);
        nh.param("cf", lqr.cf, lqr.cf);
        nh.param("cr", lqr.cr, lqr.cr);
        nh.param("mass", lqr.mass, lqr.mass);
        nh.param("horizon", lqr.horizon, lqr.horizon);
    }

    void SfyForkliftConfig::reconfigure(SfyForkliftLocalPlannerReconfigureConfig &cfg)
    {
        boost::mutex::scoped_lock l(config_mutex_);
        // AGV
        agv.expected_vel_x = cfg.expected_vel_x;
        agv.max_vel_x = cfg.max_vel_x;
        agv.max_vel_x_backwards = cfg.max_vel_x_backwards;
        agv.max_vel_theta = cfg.max_vel_theta;
        agv.acc_lim_x = cfg.acc_lim_x;
        agv.acc_lim_theta = cfg.acc_lim_theta;
        agv.min_turn_radius = cfg.min_turn_radius;
        agv.turn_around_priority = cfg.turn_around_priority;
        agv.stop_dist = cfg.stop_dist;
        agv.dec_dist = cfg.dec_dist;
        agv.forklift_wheelbase = cfg.forklift_wheelbase;
        agv.pp_view_distance = cfg.pp_view_distance;
        agv.pp_view_distance_ratio = cfg.pp_view_distance_ratio;
        agv.slow_level1_distance = cfg.slow_level1_distance;
        agv.slow_level1_vel = cfg.slow_level1_vel;

        // GoalTolerance
        goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
        goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;

        // Trajectory
        trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
        trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
        trajectory.global_plan_goal_sep = cfg.global_plan_goal_sep;
        trajectory.global_plan_prune_distance = cfg.global_plan_prune_distance;

        // Optimization
        optimization.k_rho = cfg.k_rho;
        optimization.k_alpha = cfg.k_alpha;
        optimization.k_phi = cfg.k_phi;
        optimization.pp_front_k_p = cfg.pp_front_k_p;
        optimization.pp_front_k_i = cfg.pp_front_k_i;
        optimization.pp_front_k_d = cfg.pp_front_k_d;
        optimization.pp_rear_k_p = cfg.pp_rear_k_p;
        optimization.pp_rear_k_i = cfg.pp_rear_k_i;
        optimization.pp_rear_k_d = cfg.pp_rear_k_d;
        optimization.pid_front_k_p = cfg.pid_front_k_p;
        optimization.pid_front_k_i = cfg.pid_front_k_i;
        optimization.pid_front_k_d = cfg.pid_front_k_d;
        optimization.pid_rear_k_p = cfg.pid_rear_k_p;
        optimization.pid_rear_k_i = cfg.pid_rear_k_i;
        optimization.pid_rear_k_d = cfg.pid_rear_k_d;       

        // ForkliftChose
        forklift_chose.control_model = cfg.control_model; 
        forklift_chose.steer_vel_delay_pid_model = cfg.steer_vel_delay_pid_model;
        forklift_chose.debug_model = cfg.debug_model;
        forklift_chose.fuzzy_ld_model = cfg.fuzzy_ld_model;
        forklift_chose.use_forklift_odom_combined = cfg.use_forklift_odom_combined;

        //LQR
        lqr.lqr_dynamic_q1 = cfg.lqr_dynamic_q1;
        lqr.lqr_dynamic_q2 = cfg.lqr_dynamic_q2;
        lqr.lqr_dynamic_q3 = cfg.lqr_dynamic_q3;
        lqr.lqr_dynamic_q4 = cfg.lqr_dynamic_q4;
        lqr.lqr_dynamic_r1 = cfg.lqr_dynamic_r1;
        lqr.lqr_kinematic_q1 = cfg.lqr_kinematic_q1;
        lqr.lqr_kinematic_q2 = cfg.lqr_kinematic_q2;
        lqr.lqr_kinematic_q3 = cfg.lqr_kinematic_q3;
        lqr.lqr_kinematic_r1 = cfg.lqr_kinematic_r1;
        lqr.lqr_kinematic_r2 = cfg.lqr_kinematic_r2;
        lqr.lf = cfg.lf;
        lqr.cf = cfg.cf;
        lqr.cr = cfg.cr;
        lqr.mass = cfg.mass;
        lqr.horizon = cfg.horizon;
        
    }
}