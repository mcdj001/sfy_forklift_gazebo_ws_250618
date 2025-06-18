/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-18 16:18:55
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-20 14:32:40
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_local_planner/include/sfy_controller_algorithm/mpc_base_controller.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef MPC_BASE_CONTROLLER_H
#define MPC_BASE_CONTROLLER_H

#include <vector>
#include <map>
#include <Eigen/Core>

using namespace std;

class MpcBase {
    public:
        MpcBase();

        vector<double> Solve(Eigen::VectorXd state, Eigen::MatrixXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        void LoadParams(const std::map<string, double> &params);

    private:
    
        double _max_angvel, _max_throttle, _bound_value;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
        std::map<string, double> _params;

        unsigned int dis_cnt;
};

#endif