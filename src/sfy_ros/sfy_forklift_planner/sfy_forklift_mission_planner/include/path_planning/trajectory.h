/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 16:58:41
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-05-30 10:23:29
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/include/path_planning/trajectory.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef  __SFY_MISSION_TRAJECTORY_H
#define  __SFY_MISSION_TRAJECTORY_H 
#include <iostream>
#include <vector>
#include "path_planning/mytools.h"
#include "path_planning/cubic_spline.hpp"
#include <string>

using namespace std;

class trajectory {
private:
	SfyMissionPlanner::MyTools* mytools;
	vector<waypoint> waypoints;  // 生成的最终路径
	vector<double> wx , wy;
	string trajectory_type;  // 路径形状 或 仿真模式
	double R;
	double line_distance_;  // 直线路径的步长
	vector<double> genera_line_x_, genera_line_y_;  // 根据步长生成的直线路径
	int curve_num_;  // 贝塞尔曲线的段数

	void mypath();
	void mypath_line();
	void mypath_bezier();
	void circle();
	void epitrochoid();
	void infinite();
	bool is_turn(double x1, double y1, double x2, double y2, double x3, double y3);
	void mypath_bezier_plan_a();
	void mypath_bezier_plan_b();

	void generateLinePath(double x1, double y1, double x2, double y2);

public:
	
	trajectory(string type_, vector<double> wx ,vector<double> wy , double R , double line_distance , int curve_num):
		trajectory_type(type_), wx(wx),wy(wy),R(R),line_distance_(line_distance),curve_num_(curve_num) {};
	// trajectory(
	// 	const string& type_, 
	// 	const std::vector<double>& wx,  // 通过 const 引用传递
	// 	const std::vector<double>& wy, 
	// 	double R, 
	// 	double line_distance, 
	// 	int curve_num
	//  ): trajectory_type(type_), wx(wx),wy(wy),R(R),line_distance_(line_distance),curve_num_(curve_num) {};
	void refer_path();   // 生成路径点
	vector<waypoint> get_path();
};

#endif