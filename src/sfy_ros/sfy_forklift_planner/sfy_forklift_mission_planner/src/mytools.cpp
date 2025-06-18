/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 13:56:34
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-16 16:19:22
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_mission_planner/src/mytools.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#include "path_planning/mytools.h"
using namespace SfyMissionPlanner;

double MyTools::cal_K(vector<waypoint> waypoints, int index){
	double res;
	//差分法求一阶导和二阶导
	double dx, dy, ddx, ddy;
	if (index == 0) {
		dx = waypoints[1].x - waypoints[0].x;
		dy = waypoints[1].y - waypoints[0].y;
		ddx = waypoints[2].x + waypoints[0].x - 2 * waypoints[1].x;
		ddy = waypoints[2].y + waypoints[0].y - 2 * waypoints[1].y;
	}
	else if (index == (waypoints.size() - 1)) {
		dx = waypoints[index].x - waypoints[index - 1].x;
		dy = waypoints[index].y - waypoints[index - 1].y;
		ddx = waypoints[index].x + waypoints[index - 2].x - 2 * waypoints[index].x;
		ddy = waypoints[index].y + waypoints[index - 2].y - 2 * waypoints[index].y;
	}
	else {
		dx = waypoints[index + 1].x - waypoints[index].x;
		dy = waypoints[index + 1].y - waypoints[index].y;
		ddx = waypoints[index + 1].x + waypoints[index - 1].x - 2 * waypoints[index].x;
		ddy = waypoints[index + 1].y + waypoints[index - 1].y - 2 * waypoints[index].y;
	}
	//res.yaw = atan2(dy, dx);//yaw
	//计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
	res = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2.0) + pow(dy, 2.0)), 3.0)));
	if(dx == 0 &&  dy == 0)  res = 0;
	return res;
}

double MyTools::normalizeAngle(double angle) {
	angle = fmod(angle, 2.0 * M_PI);
    while(angle >= M_PI){
        angle -= 2.0*M_PI;
    }
    while(angle <= - M_PI){
        angle += 2.0*M_PI;
    }
    return angle;
}

double MyTools::factorial(int n) {
    if(n<=1)return 1;
    return factorial(n-1)*n;
}

Vector2d MyTools::bezierCommon(vector<Vector2d> Ps, double t) {

    if(Ps.size()==1)return Ps[0];

    Vector2d p_t(0.,0.);
    int n = Ps.size()-1;
    for(int i=0;i<Ps.size();i++){
        double C_n_i = factorial(n)/ (factorial(i)* factorial(n-i));
        p_t +=  C_n_i * pow((1.0-t),(n-i)) * pow(t,i)*Ps[i];
        //cout<<t<<","<<1-t<<","<<n-i<<","<<pow((1-t),(n-i))<<endl;
    }
    return p_t;
}

void MyTools::generateBezierPath(const vector<Vector2d>& Ps, double line_distance, vector<Vector2d>& waypoints) {
	double t = 0.0;
	Vector2d lastPoint = bezierCommon(Ps, t);
	waypoints.push_back(lastPoint);

	while (t < 1.0) {
		double nextT = t + line_distance / (sqrt(pow(Ps[1][0] - Ps[0][0], 2) + pow(Ps[1][1] - Ps[0][1], 2))); // 计算下一点t的值

		if (nextT > 1.0) {
			nextT = 1.0; // 确保不超过 1.0
		}

		Vector2d newPoint = bezierCommon(Ps, nextT);
		double distance = (newPoint - lastPoint).norm();

		// 只有当新点与最后一个点的距离大于 line_distance 时才添加
		if (distance >= line_distance) {
			waypoints.push_back(newPoint);
			lastPoint = newPoint;
			t = nextT; // 更新 t
		} else {
			t = nextT; // 继续更新 t
		}
	}
}
