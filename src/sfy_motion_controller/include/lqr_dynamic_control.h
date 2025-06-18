/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date:2024-03-26 11:52:21
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-23 13:09:54
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/include/lqr_dynamic_control.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef  SFY_MOTION_CONTROLLER_LQR_DYNAMIC_CONTROL_H
#define SFY_MOTION_CONTROLLER_LQR_DYNAMIC_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "mytools.h"
using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 1> Matrix2x1;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;
typedef Eigen::Matrix<double, 4, 4> Matrix4x4;
typedef Eigen::Matrix<double, 4, 1> Matrix4x1;
typedef Eigen::Matrix<double, 1, 4> Matrix1x4;
typedef Eigen::Matrix<double, 1, 1> Matrix1x1;

//状态方程变量: X = [x_e  y_e  yaw_e]^T
//状态方程控制输入: U = [v_e  kesi_e]^T
namespace SfyMotionController {
class LQR_Dynamic_Control
{
private:
	Matrix4x4 A_d;
	Matrix4x4 A_coeff;
	Matrix4x4 A_d_by_v;
	Matrix4x1 B_d;
	Matrix4x4 Q;
    Matrix4x4 Q0; // 状态误差终端矩阵
	Matrix4x4 I_d; 
	Matrix1x1 R;
	Matrix4x1 X_e;
	
	double L; //车辆轴距
	double T; //采样间隔
	vehicleDynamicParam vehicle_param;
	double x_car, y_car, yaw_car,  kesi_car , v_car , w_car; // 当前车辆参数
	double x_d, y_d, yaw_d,x_d_next,y_d_next ,yaw_d_next; //目标点位姿
	double w_d; // 当前角速度
	double v_d, kesi_d;//期望速度和前轮偏角
	double Q3[4];  //Q权重，
	double R2[1];  //R权重，
	int temp = 0;
	// 每次进来迭代，需要返回的数据
	double lateral_error , lateral_error_rate , heading_error , heading_error_rate;
	MyTools* mytools;

public:
	void initial(vehicleDynamicParam param, double T_, vehicleState car, waypoint waypoint_d, waypoint waypoint_next, U U_r, double* Q_, double* R_);//初始化
	void param_struct();		//构造状态方程参数
	Matrix1x4 cal_Riccati();	//黎卡提方程求解
	U cal_vel();				//LQR控制器计算速度
	void change_A_by_v();
	Matrix3x1 calc_proj_pose(const Matrix3x1 p0, const Matrix3x1 p1, const Matrix3x1 p2);
	vector<double> get_all_error();
};

}
 
 
#endif // SFY_MOTION_CONTROLLER_LQR_DYNAMIC_CONTROL_H