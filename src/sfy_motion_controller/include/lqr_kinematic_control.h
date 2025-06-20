/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-04 16:59:48
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-24 12:05:04
 * @FilePath: /sfy_forklift_gazebo_ws/sfy_motion_controller/include/lqr_kinematic_control.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef  SFY_MOTION_CONTROLLER_LQR_KINEMATIC_CONTROL_H
#define SFY_MOTION_CONTROLLER_LQR_KINEMATIC_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include "mytools.h"
using namespace std;
 
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 2, 1> Matrix2x1;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;
 
//状态方程变量: X = [x_e  y_e  yaw_e]^T
//状态方程控制输入: U = [v_e  kesi_e]^T
namespace SfyMotionController  {
class LQR_Kinematic_Control
{
private:
	Matrix3x3 A_d;
	Matrix3x2 B_d;
	Matrix3x3 Q;
	Matrix2x2 R;
	Matrix3x1 X_e;
	Matrix2x1 U_e;
	
	double L;//车辆轴距
	double T;//采样间隔
	double x_car, y_car, yaw_car, x_d, y_d, yaw_d;//车辆位姿和目标点位姿
	double v_d, kesi_d;//期望速度和前轮偏角
	double Q3[3];  //Q权重，三项
	double R2[2];  //R权重，两项
	int temp = 0;

public:
	void initial(double L_, double T_, vehicleState car, waypoint waypoint, U U_r, double* Q_, double* R_);//初始化
	void param_struct();//构造状态方程参数
	Matrix2x3 cal_Riccati();//黎卡提方程求解
	U cal_vel();//LQR控制器计算速度
	void test();
};
}


 #endif   // SFY_MOTION_CONTROLLER_LQR_KINEMATIC_CONTROL_H