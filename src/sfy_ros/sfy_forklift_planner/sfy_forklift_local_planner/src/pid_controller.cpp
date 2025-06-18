/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-09-23 16:09:01
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-09-23 16:51:06
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_local_planner/src/pid_controller.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include "sfy_controller_algorithm/pid_controller.h"
using namespace SfyMotionController;

PID_Controller::PID_Controller() {}

/**
 * @author: Fengyuan Shen
 * @brief   PID控制算法的构造函数
 * @param {double} kp
 * @param {double} ki
 * @param {double} kd
 * @param {double} target
 * @param {double} upper
 * @param {double} lower
 * @return {*}
 */
PID_Controller::PID_Controller(double kp, double ki, double kd, double target, double upper, double lower) : kp(kp),
                                                                                                             ki(ki),
                                                                                                             kd(kd),
                                                                                                             target(target),
                                                                                                             upper(upper),
                                                                                                             lower(lower) {}

/**
 * @author: Fengyuan Shen
 * @brief  设置PID目标
 * @param {double} target
 * @return {*}
 */
void PID_Controller::setTarget(double target) {
    this->target = target;
}

/**
 * @author: Fengyuan Shen
 * @brief  设置参数 
 * @param {double} kp
 * @param {double} ki
 * @param {double} kd
 * @return {*}
 */
void PID_Controller::setK(double kp, double ki, double kd) {
    this->kp=kp;
    this->ki=ki;
    this->kd=kd;
}

/**
 * @author: Fengyuan Shen
 * @brief 设置控制量边界
 * @param {double} upper
 * @param {double} lower
 * @return {*}
 */
void PID_Controller::setBound(double upper, double lower) {
    this->upper=upper;
    this->lower=lower;
}

/**
 * @author: Fengyuan Shen
 * @brief 计算控制输出
 * @param {double} state 当前状态量
 * @return {*} 
 */
double PID_Controller::calOutput(double state) {
    this->error = this->target - state;
    double u = this->error*this->kp + this->sum_error*this->ki + (this->error-this->pre_error)*this->kd;
    if(u<this->lower)u=this->lower;
    else if(u>this->upper)u=this->upper;
    this->pre_error = this->error;
    this->sum_error = this->sum_error + this->error;
    return u;
}


/**
 * @author: Fengyuan Shen
 * @brief  重置误差 
 * @return {*}
 */
void PID_Controller::reset() {
    error=0.0, pre_error=0.0, sum_error=0.0;
}


/**
 * @author: Fengyuan Shen
 * @brief 设置累计误差
 * @param {double} sum_error
 * @return {*}
 */
void PID_Controller::setSumError(double sum_error) {
    this->sum_error = sum_error;
}