/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-09-23 16:08:22
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-09-23 16:51:19
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_local_planner/include/sfy_controller_algorithm/pid_controller.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H
#include <iostream>

using namespace std;

/**
 * 位置式PID实现
 */

namespace  SfyMotionController {
class PID_Controller {
private:

    double kp, ki, kd, target, upper, lower;
    double error = 0, pre_error = 0, sum_error = 0;

public:
    PID_Controller();
    PID_Controller(double kp, double ki, double kd, double target, double upper, double lower);

    void setTarget(double target);

    void setK(double kp, double ki, double kd);

    void setBound(double upper,double lower);

    double calOutput(double state);

    void reset();

    void setSumError(double sum_error);
};

}

#endif 