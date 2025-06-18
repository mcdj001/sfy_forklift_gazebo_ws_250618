/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-13 13:56:01
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-13 14:00:37
 */

#ifndef  SFY_MOTION_CONTROLLER_TRACK_MPC_H
#define SFY_MOTION_CONTROLLER_TRACK_MPC_H

#include <vector>
#include <map>
#include <Eigen/Core>

using namespace std;

class MPC
{
    public:
        MPC();
    
        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        double _mpc_totalcost;
        double _mpc_ctecost;
        double _mpc_ethetacost;
        double _mpc_velcost;


        void LoadParams(const std::map<string, double> &params);
    
    private:
        // Parameters for mpc solver
        double _max_angvel, _max_throttle, _bound_value;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
        std::map<string, double> _params;

        unsigned int dis_cnt;
};

#endif
