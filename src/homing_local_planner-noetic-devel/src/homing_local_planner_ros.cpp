/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-05 13:22:58
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-07-07 16:12:39
 */
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Lei ZENG
 *********************************************************************/

#include <pluginlib/class_list_macros.h>
#include "homing_local_planner/homing_local_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(homing_local_planner::HomingLocalPlanner, nav_core::BaseLocalPlanner)

namespace homing_local_planner
{

    HomingLocalPlanner::HomingLocalPlanner() : tf_(NULL), dynamic_recfg_(NULL),
                                               goal_reached_(false), world_model_(NULL), initialized_(false)
    {
    }

    HomingLocalPlanner::~HomingLocalPlanner() {}

    void HomingLocalPlanner::reconfigureCB(HomingLocalPlannerReconfigureConfig &config, uint32_t level)
    {
        cfg_.reconfigure(config);
    }

    void HomingLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
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

            visualization_ = HomingVisualizationPtr(new HomingVisualization(nh, global_frame_));
            dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<HomingLocalPlannerReconfigureConfig>>(nh);
            dynamic_reconfigure::Server<HomingLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&HomingLocalPlanner::reconfigureCB, this, _1, _2);
            dynamic_recfg_->setCallback(cb);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
            odom_helper_.setOdomTopic(cfg_.odom_topic);

            ros::NodeHandle nh_move_base("~");
            double controller_frequency = 5;
            nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
            control_duration_ = 1.0 / controller_frequency;

            initialized_ = true;
        }
    }

    bool HomingLocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("homing_local_planner has not been initialized");
            return false;
        }
        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        goal_reached_ = false;
        xy_reached_ = false;
        last_back_ = false;
        omega_filter_.filterCall(ros::Time::now(), last_cmd_.angular.z);
        smoothPlan2d(global_plan_);
        visualization_->publishGlobalPlan(global_plan_);
        return true;
    }

    bool HomingLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("homing_local_planner has not been initialized");
            return false;
        }

        // Get robot velocity
        geometry_msgs::PoseStamped robot_vel_tf;
        odom_helper_.getRobotVel(robot_vel_tf);
        robot_vel_.linear.x = robot_vel_tf.pose.position.x;
        robot_vel_.linear.y = robot_vel_tf.pose.position.y;
        robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

        goal_reached_ = false;
        costmap_ros_->getRobotPose(robot_pose_);

        // prune global plan to cut off parts of the past (spatially before the robot)
        pruneGlobalPlan(*tf_, robot_pose_, global_plan_, cfg_.trajectory.global_plan_prune_distance);

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

        const geometry_msgs::PoseStamped &goal_point = transformed_plan.back();

        // we assume the global goal is the last point in the global plan
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;
        const double goal_th = tf2::getYaw(goal_point.pose.orientation);
        double dx = goal_x - robot_pose_.pose.position.x;
        double dy = goal_y - robot_pose_.pose.position.y;
        double dyaw = fmod(goal_th - tf2::getYaw(robot_pose_.pose.orientation), 2 * M_PI);

        if ((fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance) && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance))
            goal_reached_ = true;
        else if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance)
            xy_reached_ = true;
        else if (xy_reached_ && (fabs(dyaw) < cfg_.goal_tolerance.yaw_goal_tolerance))
            goal_reached_ = true;

        updateViaPointsContainer(transformed_plan,
                                 cfg_.trajectory.global_plan_viapoint_sep, cfg_.trajectory.global_plan_goal_sep);
        via_points_.push_back(Eigen::Vector3d(goal_x, goal_y, goal_th));
        local_plan_.push_back(transformed_plan.back());

        obstacles_.clear();
        updateObstacleContainerWithCostmap();
        double lethal_distance = checkCollision(transformed_plan, cfg_.trajectory.max_global_plan_lookahead_dist);
        dec_ratio_ = std::min(lethal_distance / cfg_.robot.dec_dist, 1.0);

        Eigen::Quaterniond quat_world_robot(robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
                                            robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);
        Eigen::Matrix3d rot_mat_world_robot = quat_world_robot.toRotationMatrix();
        Eigen::Translation3d trans_world_robot(robot_pose_.pose.position.x, robot_pose_.pose.position.y, 0.0);
        Eigen::Affine3d T_world_robot = trans_world_robot * rot_mat_world_robot;

        Eigen::Vector3d euler_world_target(via_points_[0][2], 0, 0);
        Eigen::Matrix3d rot_mat_world_target;
        rot_mat_world_target = Eigen::AngleAxisd(euler_world_target(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(euler_world_target(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler_world_target(2), Eigen::Vector3d::UnitX());
        Eigen::Translation3d trans_world_target(via_points_[0][0], via_points_[0][1], 0.0);
        Eigen::Affine3d T_world_target = trans_world_target * rot_mat_world_target;

        Eigen::Affine3d T_robot_target = T_world_robot.inverse() * T_world_target;
        Eigen::Matrix3d rot_mat_robot_target = T_robot_target.rotation();
        Eigen::Vector3d euler_robot_target = R2ypr(rot_mat_robot_target);
        double dx1 = T_robot_target.matrix()(0, 3);
        double dy1 = T_robot_target.matrix()(1, 3);
        double dyaw1 = euler_robot_target(0);

        double rho, alpha, phi, v, omega;
        poseError(dx1, dy1, dyaw1, rho, alpha, phi);
        homingControl(rho, alpha, phi, v, omega);

        if (cfg_.robot.min_turn_radius > 0)
        {
            double omega_max = fabs(v / cfg_.robot.min_turn_radius);
            omega = clip(omega, omega_max * (-1.0), omega_max);
        }
        else
        {
            double d_atan = std::atan2(dy1, dx1);
            double d_atan_phi = fabs(d_atan - phi);
            if ((xy_reached_) or
                (cfg_.robot.turn_around_priority and fabs(phi) > 0.75 and
                 (d_atan_phi < 0.5 or fmod(d_atan_phi, M_PI) < 0.5 or fmod(d_atan_phi, M_PI) > 2.6)))
            {
                v = 0;
                omega = clip(phi, cfg_.robot.max_vel_theta * (-1.0), cfg_.robot.max_vel_theta);
            }
        }

        if (lethal_distance < cfg_.robot.stop_dist)
        {
            v = 0;
            omega = 0;
        }

        const double min_feasible_angular_speed = robot_vel_.angular.z - cfg_.robot.acc_lim_theta * control_duration_;
        const double max_feasible_angular_speed = robot_vel_.angular.z + cfg_.robot.acc_lim_theta * control_duration_;
        omega = clip(omega, min_feasible_angular_speed, max_feasible_angular_speed);

        const double min_feasible_linear_speed = robot_vel_.linear.x - cfg_.robot.acc_lim_x * control_duration_;
        const double max_feasible_linear_speed = robot_vel_.linear.x + cfg_.robot.acc_lim_x * control_duration_;
        v = clip(v, min_feasible_linear_speed, max_feasible_linear_speed);

        cmd_vel.linear.x = v;
        cmd_vel.angular.z = omega;

        last_cmd_ = cmd_vel;
        visualization_->publishViaPoints(via_points_);
        visualization_->publishViaPoints(collision_points_, "CollsionPoints", visualization_->toColorMsg(1.0, 1.0, 0.65, 0.0));
        visualization_->publishLocalPlan(local_plan_);
        visualization_->publishObstacles(obstacles_, costmap_->getResolution());

        return true;
    }

    bool HomingLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (goal_reached_)
        {
            ROS_INFO("GOAL Reached!");
            // planner_->clearPlanner();
            return true;
        }
        return false;
    }

    bool HomingLocalPlanner::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
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

    void HomingLocalPlanner::smoothPlan2d(std::vector<geometry_msgs::PoseStamped> &global_plan)
    {
        if (global_plan.empty())
            return;
        simplifyGlobalPlan(global_plan, 0.03);
        std::vector<geometry_msgs::PoseStamped> smoothed_global_plan(global_plan);
        double weight_data = 0.5;
        double weight_smooth = 1.0 - weight_data;
        double tolerance = 0.007;
        double change = tolerance;
        double aux1, aux2;
        while (change >= tolerance)
        {
            change = 0;
            for (std::size_t i = 1; i < global_plan.size() - 1; ++i)
            {
                aux1 = smoothed_global_plan[i].pose.position.x;
                smoothed_global_plan[i].pose.position.x += weight_data * (global_plan[i].pose.position.x - smoothed_global_plan[i].pose.position.x) + weight_smooth * (smoothed_global_plan[i - 1].pose.position.x + smoothed_global_plan[i + 1].pose.position.x - 2 * smoothed_global_plan[i].pose.position.x);

                aux2 = smoothed_global_plan[i].pose.position.y;
                smoothed_global_plan[i].pose.position.y += weight_data * (global_plan[i].pose.position.y - smoothed_global_plan[i].pose.position.y) + weight_smooth * (smoothed_global_plan[i - 1].pose.position.y + smoothed_global_plan[i + 1].pose.position.y - 2 * smoothed_global_plan[i].pose.position.y);
                change += (fabs(aux1 - smoothed_global_plan[i].pose.position.x) + fabs(aux2 - smoothed_global_plan[i].pose.position.y));
            }
        }
        global_plan = smoothed_global_plan;
    }

    void HomingLocalPlanner::simplifyGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan, double simplify_separation)
    {
        if (plan.empty())
            return;
        std::vector<geometry_msgs::PoseStamped> simple_plan;
        simple_plan.push_back(plan[0]);
        std::size_t prev_idx = 0;
        for (std::size_t i = 1; i < plan.size(); ++i)
        {
            if ((distancePoints2d(plan[prev_idx].pose.position, plan[i].pose.position) < simplify_separation) ||
                (distancePoints2d(plan.back().pose.position, plan[i].pose.position) < simplify_separation))
                continue;
            simple_plan.push_back(plan[i]);
            prev_idx = i;
        }

        simple_plan.push_back(plan.back());
        plan = simple_plan;
    }

    void HomingLocalPlanner::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation_via, double min_separation_goal)
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

    double HomingLocalPlanner::checkCollision(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double check_dist)
    {

        collision_points_.clear();
        double path_point_yaw;
        double plan_length = 0;
        double lethal_point_distance = 999;
        std::size_t i = 1;
        while (i < transformed_plan.size() && plan_length < check_dist)
        {
            path_point_yaw = std::atan2(transformed_plan[i].pose.position.y - transformed_plan[i - 1].pose.position.y,
                                        transformed_plan[i].pose.position.x - transformed_plan[i - 1].pose.position.x);
            double footprint_cost = world_model_->footprintCost(transformed_plan[i].pose.position.x,
                                                                transformed_plan[i].pose.position.y, path_point_yaw, costmap_ros_->getRobotFootprint());

            plan_length += distance_points2d(transformed_plan[i].pose.position, transformed_plan[i - 1].pose.position);
            if (footprint_cost == -1)
            {
                collision_points_.push_back(Eigen::Vector3d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y, path_point_yaw));

                if (plan_length < lethal_point_distance)
                    lethal_point_distance = plan_length;
            }

            i++;
        }

        return lethal_point_distance;
    }

    double HomingLocalPlanner::clip(double value, double lower, double upper)
    {
        if (value < lower)
            return lower;
        else if (value > upper)
            return upper;
        else
            return value;
    }

    void HomingLocalPlanner::cart2Pol(double x, double y, double &deg, double &dist)
    {
        dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        deg = std::atan2(y, x);
    }

    double HomingLocalPlanner::angDiff(double alpha, double beta)
    {
        double delta;
        delta = fmod((alpha - beta), 2.0 * M_PI);
        if (delta > M_PI)
            delta = delta - 2 * M_PI;
        else if (delta < M_PI * (-1.0))
            delta = delta + 2 * M_PI;
        return delta;
    }

    void HomingLocalPlanner::poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi)
    {
        cart2Pol(dx, dy, alpha, rho);
        alpha = angDiff(0, alpha);
        phi = dyaw;
    }

    void HomingLocalPlanner::homingControl(double rho, double alpha, double phi, double &v, double &omega)
    {
        v = cfg_.optimization.k_rho * rho;
        if ((fabs(alpha) > M_PI * 0.5 + 0.1) or ((fabs(alpha) > M_PI * 0.5 - 0.1) and last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;
        v = clip(v, cfg_.robot.max_vel_x * (-1.0) * dec_ratio_, cfg_.robot.max_vel_x * dec_ratio_);

        omega = cfg_.optimization.k_alpha * alpha + cfg_.optimization.k_phi * phi;
        omega = clip(omega, cfg_.robot.max_vel_theta * (-1.0) * dec_ratio_, cfg_.robot.max_vel_theta * dec_ratio_);
    }

    Eigen::Vector3d HomingLocalPlanner::R2ypr(const Eigen::Matrix3d &R)
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

    void HomingLocalPlanner::updateObstacleContainerWithCostmap()
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

    bool HomingLocalPlanner::transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
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
            dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap

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
}