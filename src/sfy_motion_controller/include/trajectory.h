/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 16:58:41
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-11 15:13:24
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/include/trajectory.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef  SFY_MOTION_TRAJECTORY_H
#define SFY_MOTION_TRAJECTORY_H 
#include <iostream>
#include <vector>
#include "mytools.h"
#include "cubic_spline.hpp"
#include <string>

using namespace std;
using namespace SfyMotionController;
MyTools* mytools;

class trajectory {
private:
	vector<waypoint> waypoints;  // 生成的最终路径
	vector<double> wx , wy;
	string trajectory_type;  // 路径形状 或  仿真模式
	double R;

	void mypath();
	void mypath_line();
	void mypath_bezier();
	void circle();
	void epitrochoid();
	void infinite();
	bool is_turn(double x1, double y1, double x2, double y2, double x3, double y3);
	void mypath_bezier_plan_a();
	void mypath_bezier_plan_b();

public:
	trajectory(string type_, vector<double> wx ,vector<double> wy , double R):
		trajectory_type(type_), wx(wx),wy(wy),R(R){};
	//set reference trajectory
	void refer_path();   // 生成路径点
	vector<waypoint> get_path();
};

#endif