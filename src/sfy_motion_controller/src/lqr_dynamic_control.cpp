/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-03-29 09:18:06
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-23 12:59:49
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/src/lqr_dynamic_control.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include "lqr_dynamic_control.h"
using namespace  SfyMotionController;

void LQR_Dynamic_Control::initial(vehicleDynamicParam param, double T_, vehicleState car, waypoint waypoint_d, waypoint waypoint_next , U U_r, double *Q_, double *R_) { 
	vehicle_param = param;
	T = T_;
	x_car = car.x; y_car = car.y; yaw_car = car.yaw;
	v_car = car.v;
	// kesi_car = car.kesi; 
	// w_car = v_car * tan(kesi_car)/vehicle_param.L;
	// 注意  传进来的是角速度   实物仿真的时候变了
	w_car = car.kesi;
	kesi_car = atan(w_car * vehicle_param.L / v_car); 
	

	x_d = waypoint_d.x; y_d = waypoint_d.y; yaw_d = waypoint_d.yaw;
    x_d_next = waypoint_next.x;  y_d_next = waypoint_next.y;  yaw_d_next = waypoint_next.yaw;
	v_d = U_r.v;   kesi_d = U_r.kesi;
	w_d= v_d * tan(kesi_d)/vehicle_param.L;
 
	for (int i = 0; i < 4; i++) {
		Q3[i] = Q_[i];
	}
	for (int j = 0; j < 1; j++) {
		R2[j] = R_[j];
	}

	if(isnan(vehicle_param.target_index_K))  vehicle_param.target_index_K = 0;
	
	// param_struct();
}

Matrix3x1 LQR_Dynamic_Control::calc_proj_pose(const Matrix3x1 p0, const Matrix3x1 p1, const Matrix3x1 p2)
{
    const double tol = 1e-4;
    Matrix3x1 proj_pose;

    if (std::abs(p2[0] - p1[0]) < tol)
    {
        // p1 and p2 have infinite slope
        double x = p1[0]; // x-coordinate of the projection point is the x-coordinate of p1
        double y = p0[1]; // y-coordinate of the projection point is the y-coordinate of p0
        proj_pose << x, y, 0;
    }
    else if (std::abs(p2[1] - p1[1]) < tol)
    {
        // p1 and p2 have infinite slope
        double x = p0[0]; // x-coordinate of the projection point is the x-coordinate of p1
        double y = p1[1]; // y-coordinate of the projection point is the y-coordinate of p0
        proj_pose << x, y, 0;
    }
    else
    {
        double k1 = (p2[1] - p1[1]) / (p2[0] - p1[0]); // slope of the line between p1 and p2
        double k2 = -1 / k1;                           // slope of the line between p0 and the projection point (two perpendicular lines' slope multiplication is -1)
        double x = (p0[1] - p1[1] + k1 * p1[0] - k2 * p0[0]) / (k1 - k2);
        double y = p0[1] + k2 * (x - p0[0]);
        proj_pose << x, y, 0;
    }

    double dist = (p2.head(2) - p1.head(2)).norm();         // distance from p1 to p2  点p1到点p2的距离
    double dist2 = (p2.head(2) - proj_pose.head(2)).norm(); // distance from the projection point to p2  投影点到点p2的距离
	double ratio;
	if(dist == 0){
		ratio = 0;
	}else {
		ratio = dist2 / dist;
	}
    
    double theta = ratio * p1[2] + (1 - ratio) * p2[2];
    proj_pose[2] = theta;

    return proj_pose;
}

void LQR_Dynamic_Control::param_struct() {
	// cout << "vehicle_param.cf:" << vehicle_param.cf << endl;
	// cout << "vehicle_param.cf:" << vehicle_param.cr << endl;
	// cout << "vehicle_param.cf:" << vehicle_param.mass << endl;
	Q << Q3[0], 0.0, 0.0, 0.0,
		    0.0, Q3[1], 0.0, 0.0,
			0.0, 0.0, Q3[2],0.0, 
        	0.0, 0.0, 0.0, Q3[3];
	// cout << "Q矩阵为：\n" << Q << endl;
	R << R2[0];
	// cout << "R矩阵为：\n" << R << endl;
	A_d << 0, 1.0, 0, 0,
        0, 0, (vehicle_param.cf + vehicle_param.cr) / vehicle_param.mass, 0,
        0, 0, 0, 1.0,
        0, 0, (vehicle_param.lf * vehicle_param.cf - vehicle_param.lr * vehicle_param.cr) / vehicle_param.iz, 0;
	// cout << "A_d矩阵为：\n" << A_d << endl;
	A_coeff << 0, 0, 0, 0,
			0, -(vehicle_param.cf + vehicle_param.cr) / vehicle_param.mass, 0, -1.0 *(vehicle_param.lr * vehicle_param.cr + vehicle_param.lf * vehicle_param.cf) / vehicle_param.mass,
			0, 0, 0, 1.0,
			0, (vehicle_param.lr *vehicle_param.cr - vehicle_param.lf * vehicle_param.cf) / vehicle_param.iz, 0, -1.0 * (vehicle_param.lf * vehicle_param.lf * vehicle_param.cf + vehicle_param.lr * vehicle_param.lr * vehicle_param.cr) / vehicle_param.iz;
	// cout << "A_coeff矩阵为：\n" << A_coeff << endl;
    A_d(1, 1) = A_coeff(1, 1);
    A_d(1, 3) = A_coeff(1, 3);
    A_d(3, 1) = A_coeff(3, 1);
    A_d(3, 3) = A_coeff(3, 3);

	B_d << 0, vehicle_param.cf / vehicle_param.mass, 0, vehicle_param.lf * vehicle_param.cf / vehicle_param.iz;
	I_d = Eigen::MatrixXd::Identity(4, 4);
	Q0 = Eigen::MatrixXd::Identity(4, 4);

	Matrix3x1 veh_current_pose ,  trajref_ind_pose , trajref_ind_next_pose;
	veh_current_pose << x_car , y_car , yaw_car; 
	trajref_ind_pose << x_d ,  y_d , yaw_d;
	trajref_ind_next_pose  << x_d_next ,  y_d_next , yaw_d_next;
	Matrix3x1  ref_pose = calc_proj_pose(veh_current_pose , trajref_ind_pose ,  trajref_ind_next_pose);
	cout << "veh_current_pose:   " << veh_current_pose << endl;
	cout << "ref_pose:  " << ref_pose<<endl;
	// 位姿偏差
	Matrix3x1 delta_x = veh_current_pose - ref_pose;
	delta_x[2] = mytools->normalizeAngle(delta_x[2]);
	double dx = delta_x[0];
    double dy = delta_x[1];
    double dtheta = delta_x[2];
	cout << "dtheta = " << dtheta << endl;
	double theta_des = yaw_d;
	double one_min_k = 1 - vehicle_param.target_index_K;
    if (one_min_k <= 0) {
        one_min_k = 0.01;
    }
	// 当前转向角速度
    double angular_v = w_car;     // current angular velocity
    // 期望角速度
    double angular_v_des = v_car * vehicle_param.target_index_K;  //  角速度 = 当前速度 / 曲率半径
    if (angular_v_des < 0.01) {
        angular_v_des = 0;
    }
	lateral_error = dy * cos(theta_des) - dx * sin(theta_des);
	X_e(0) = lateral_error;
    // lateral error rate
	lateral_error_rate = v_car * sin(dtheta);
	// cout << "v_car: " << v_car << endl;
    X_e(1) = lateral_error_rate;
    // heading error
	heading_error = mytools->normalizeAngle(dtheta);
    X_e(2) = heading_error;
    // heading error rate
	heading_error_rate = angular_v - angular_v_des;
    X_e(3) = heading_error_rate;
	cout << "lateral_error_rate:" << lateral_error_rate << "   heading_error: "<<heading_error<< "heading_error_rate:"<< heading_error_rate << endl;
}

void LQR_Dynamic_Control::change_A_by_v(){
	if(v_car == 0){
		A_d_by_v = A_d;
	}else{
		A_d_by_v = A_d;
		A_d_by_v(1, 1) = A_d(1, 1) / v_car;
        A_d_by_v(1, 3) = A_d(1, 3) / v_car;
		A_d_by_v(3, 1) = A_d(3, 1) / v_car;
		A_d_by_v(3, 3) = A_d(3, 3) / v_car;
	}
};

Matrix1x4 LQR_Dynamic_Control::cal_Riccati() {
	int N = vehicle_param.horizon;  //迭代终止次数
	double err = 100; //误差值
	double err_tolerance = 0.01;  //误差收敛阈值
	Matrix4x4 Qf = Q;
	Matrix4x4 P = Qf; //迭代初始值
	//cout << "P初始矩阵为\n" << P << endl;
	Matrix4x4 Pn;//计算的最新P矩阵
	for (int iter_num = 0; iter_num < N; iter_num++) {
		Pn = Q + A_d_by_v.transpose() * P * A_d_by_v - A_d_by_v.transpose() * P * B_d * (R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d_by_v;//迭代公式
		//cout << "收敛误差为" << (Pn - P).array().abs().maxCoeff() << endl;
		//err = (Pn - P).array().abs().maxCoeff();//
		err = (Pn - P).lpNorm<Eigen::Infinity>();
		if(err < err_tolerance)//
		{
			P = Pn;
			cout << "求解黎卡提方程迭代次数" << iter_num << endl;
			break;
		}
		P = Pn;
	}
	// cout << "P矩阵为\n" << P << endl;
	//P = Q;
	cout << "求解方程迭代最终误差是："<<  err << endl;
	Matrix1x4 K = -(R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d_by_v;  //反馈率K
	return K;
}
 
U LQR_Dynamic_Control::cal_vel() {
	U output;
	param_struct();
	change_A_by_v();
	A_d_by_v = (I_d +  T*0.5*A_d_by_v) * (I_d -  T*0.5*A_d_by_v).inverse();
	// cout << "A_d_by_v_T矩阵为：\n" << A_d_by_v << endl;
	B_d = T * B_d ;
	// cout << "B_d_T矩阵为：\n" << B_d << endl;
	Matrix1x4 K = cal_Riccati();
	Matrix1x1 U = K * X_e;
	cout << "反馈增益K为：\n" << K << endl;
	cout << "误差X_e为：\n" << X_e << endl;
	cout << "控制输入U为: " << U << endl;
	// output.v = U[0] + v_d;

	// 前馈角度更新
	double kv =vehicle_param.lr * vehicle_param.mass/2/vehicle_param.cf/vehicle_param.L - vehicle_param.lf * vehicle_param.mass/2/vehicle_param.cr/vehicle_param.L;
	
    // 前轮补偿角度
	double steer_feedforward = atan(vehicle_param.L * vehicle_param.target_index_K + kv *v_car*v_car*vehicle_param.target_index_K);
    steer_feedforward = mytools->normalizeAngle(steer_feedforward);
	cout << "前轮补偿角度steer_feedforward: " << steer_feedforward << endl;
	// cout << "kesi_d: " << kesi_d << endl;
	
	// output.kesi = U[0] + kesi_d;
	output.kesi = U[0]  + steer_feedforward;
	cout << "最终控制量：" <<  output.kesi << endl;

	return output;
}

vector<double>  LQR_Dynamic_Control::get_all_error(){
	vector<double> error_lqr;
	error_lqr.push_back(lateral_error);
	error_lqr.push_back(lateral_error_rate);
	error_lqr.push_back(heading_error);
	error_lqr.push_back(heading_error_rate);
	return error_lqr;
}
 