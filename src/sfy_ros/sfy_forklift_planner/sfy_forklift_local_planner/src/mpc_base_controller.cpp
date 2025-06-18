/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-18 16:16:44
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-22 15:55:31
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_local_planner/src/mpc_base_controller.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 * Authors : Geonhee Lee, Balamurugan Kandan
 */

#include "sfy_controller_algorithm/mpc_base_controller.h"
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes

using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval 
{
    public:
        // Fitted polynomial coefficients
        Eigen::MatrixXd coeffs;

        double _dt, _ref_cte, _ref_etheta, _ref_vel; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;

        AD<double> cost_cte, cost_etheta, cost_vel;
        // Constructor
        FG_eval(Eigen::MatrixXd coeffs) 
        { 
            this->coeffs = coeffs; 

            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_etheta  = 0;
            _ref_vel   = 0.5; // m/s
            _w_cte     = 100;
            _w_etheta    = 100;
            _w_vel     = 1;
            _w_angvel   = 100;
            _w_accel   = 50;
            _w_angvel_d = 0;
            _w_accel_d = 0;

            _mpc_steps   = 5;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _angvel_start = _v_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_etheta  = params.find("W_EPSI") != params.end()  ? params.at("W_EPSI") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_accel = params.find("W_A") != params.end()     ? params.at("W_A") : _w_accel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_accel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_accel_d;

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _angvel_start = _v_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
            
            //cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl; 
        }

        AD<double> custom_fmod(const AD<double>& value, const AD<double>& divisor) {
            // 确保除数不为零
            if (divisor == 0) {
                throw std::invalid_argument("Divisor cannot be zero.");
            }
            
            AD<double> remainder = value;
            
            // 处理负值
            if (value >= 0) {
                while (remainder >= divisor) {
                    remainder -= divisor;
                }
            } else {
                while (remainder < 0) {
                    remainder += divisor;
                }
            }
            
            return remainder;
        }

        AD<double> normal_angel(const AD<double>& angle) {
            const AD<double> two_pi = 2.0 * M_PI;
            const AD<double> pi = M_PI;

            AD<double> delta;
            delta = custom_fmod(angle, two_pi);
            if (delta > M_PI)
                delta = delta - 2 * M_PI;
            else if (delta < M_PI * (-1.0))
                delta = delta + 2 * M_PI;
            return delta;
            // AD<double> normalized_angle = custom_fmod(angle + pi, two_pi) - pi;
            // return normalized_angle;
        }

        // AD<double> normal_angel(AD<double> temp1){
        //     AD<double> temp = temp1;
        //     while(temp > M_PI){
        //         temp -= 2.0*M_PI;
        //     }
        //     while(temp < - M_PI){
        //         temp += 2.0*M_PI;
        //     }
        //     return temp;
        // }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        // fg: function that evaluates the objective and constraints using the syntax       
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            // fg[0] for cost function
            fg[0] = 0;
            cost_cte =  0;
            cost_etheta = 0;
            cost_vel = 0;

            // for (int i = 0; i < _mpc_steps; i++) 
            // {
            //     cout << i << endl;
            //     cout << "_x_start: " << vars[_x_start + i] <<endl;
            //     cout << "_y_start: " << vars[_y_start + i] <<endl;
            //     cout << "_theta_start: " << vars[_theta_start + i] <<endl;
            //     // cout << "_v_start: " << vars[_v_start + i] <<endl;
            // }
            // cout << endl;
            // for (int i = 0; i < _mpc_steps - 1; i++) 
            // {
            //     cout << i << endl;
            //     cout << "_angvel_start: " << vars[_angvel_start + i] <<endl;
            //     cout << "_a_start: " << vars[_a_start + i] <<endl;
            // }
            // cout << endl;


            // for (int i = 0; i < _mpc_steps; i++) 
            // {
            //   fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);          // cross deviation error
            //   fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2); // heading error
            //   fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2);            // speed error

            //   cost_cte +=  _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);
            //   cost_etheta +=  (_w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2)); 
            //   cost_vel +=  (_w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2)); 
            // }
            // cout << "-----------------------------------------------" <<endl;
            // cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta  << ", " << cost_vel << endl;
            // cout << "coeffs:" << coeffs << endl;
            // for (int i = 0; i < _mpc_steps; i++){
            //     fg[0] += CppAD::pow(coeffs(0, i) - vars[_x_start + i], 2);
            //     fg[0] += CppAD::pow(coeffs(1, i) - vars[_y_start + i], 2);
            //     fg[0] += 0.5 * CppAD::pow(coeffs(2, i) - vars[_theta_start+i], 2);
            //     fg[0] += 0.5 * CppAD::pow(coeffs(3, i) - vars[_v_start +i ], 2);
            // }

            // Minimize the use of actuators.
            for (int i = 0; i < _mpc_steps - 1; i++) {
                fg[0] += 0.01 * CppAD::pow(vars[_angvel_start + i], 2);
                fg[0] += 0.01 * CppAD::pow(vars[_a_start + i], 2);
            }
            // cout << "cost of actuators: " << fg[0] << endl; 

            // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++) {
                fg[0] += 1 * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
                fg[0] += 0.01 * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
            }
            cout << "cost of gap: " << fg[0] << endl; 
            
            // fg[0] += CppAD::pow(coeffs(0, 0) - vars[_x_start], 2);
            // fg[0] += CppAD::pow(coeffs(1, 0) - vars[_y_start], 2);
            // fg[0] += 0.5 * CppAD::pow(coeffs(2, 0) - vars[_theta_start], 2);
            // fg[0] += 0.5 * CppAD::pow(coeffs(3, 0) - vars[_angvel_start], 2);
            
            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _theta_start] = vars[_theta_start];
            fg[1 + _v_start] = vars[_v_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];
                AD<double> theta1 = normal_angel(vars[_theta_start + i + 1]);
                AD<double> v1 = vars[_v_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];
                AD<double> theta0 = normal_angel(vars[_theta_start + i]);
                AD<double> v0 = vars[_v_start + i];

                // Only consider the actuation at time t.
                AD<double> w0 = normal_angel( vars[_angvel_start + i]);
                AD<double> a0 = vars[_a_start + i];

                // cout << "theta1: " << theta1 << endl;
                // cout << "theta0: " << theta0 << endl;
                cout << "w0: " << w0 << endl;

                AD<double> L = 1.432;
                // theta0 + v0 * CppAD::tan(w0) / 1.432 * _dt

                AD<double> temp = normal_angel(v0 / L * CppAD::tan(w0)  *_dt); 
                AD<double> my_temp_theta = theta0 + temp;      
                // if(theta0 <= M_PI && theta0 >= 0){
                //     my_temp_theta = theta0 + temp;
                // }else{
                //     my_temp_theta = theta0 - temp;
                // }
               
                cout << "theta1: " << theta1 << endl;
                cout << "theta0: " << theta0 << endl;
                cout << "my_temp_theta: " << my_temp_theta << endl;

                fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
                fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
                fg[2 + _theta_start + i] = normal_angel(theta1 - my_temp_theta) ;
                fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);

                AD<double> coeffs0 = coeffs(0 , i+1);
                AD<double> coeffs1 = coeffs(1 , i+1);
                AD<double> coeffs2 = normal_angel(coeffs(2 , i+1));
                AD<double> coeffs3 = coeffs(3 , i+1);
                fg[0] += 1*CppAD::pow(coeffs0 - (x0 + v0 * CppAD::cos(theta0) * _dt), 2);
                fg[0] += 1*CppAD::pow(coeffs1 -  (y0 + v0 * CppAD::sin(theta0) * _dt), 2);
                fg[0] += 0.5 * CppAD::pow(normal_angel(coeffs2 -  my_temp_theta)  , 2);
                fg[0] += 0.5 * CppAD::pow(coeffs3 - (v0 + a0 * _dt), 2);

                cout << "coeffs(2 , i+1): " << coeffs(2 , i+1) - my_temp_theta << endl;
                cout << "coeffs(2 , i+1): " << normal_angel(coeffs(2 , i+1) - my_temp_theta) << endl;
                cout << endl;
            }

            cout << endl;


        }


};

// ====================================
// MPC class definition implementation.
// ====================================
MpcBase::MpcBase() 
{
}

void MpcBase::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _v_start     = _theta_start + _mpc_steps;
    _angvel_start = _v_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}


vector<double> MpcBase::Solve(Eigen::VectorXd state, Eigen::MatrixXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];

    cout << "state" << state << endl;

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * 4 + (_mpc_steps - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 4;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < n_vars; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _angvel_start + _mpc_steps - 1; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < _a_start + _mpc_steps - 1; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;

    // object that computes objective and constraints
    cout << "coeffs" << coeffs << endl;
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";
    options += "Integer max_iter        100\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);


    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // cout << "ok " << ok << endl;
    // Cost
    auto cost = solution.obj_value;
    // std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    // cout << "max_angvel:" << _max_angvel <<endl;
    // cout << "max_throttle:" << _max_throttle <<endl;
    // cout << "-----------------------------------------------" <<endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}
