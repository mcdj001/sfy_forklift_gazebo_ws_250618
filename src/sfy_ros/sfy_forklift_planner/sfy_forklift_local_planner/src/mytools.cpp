/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-09-09 13:16:42
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-23 14:16:49
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_local_planner/src/mytools.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#include <sfy_forklift_local_planner/mytools.h>
using namespace SfyMotionController;

double MyTools::cal_k_poseStamped(vector<geometry_msgs::PoseStamped> waypoints, int index){
    double res;
    //差分法求一阶导和二阶导
    double dx, dy, ddx, ddy;
    if (index == 0) {
        dx = waypoints[1].pose.position.x - waypoints[0].pose.position.x;
        dy = waypoints[1].pose.position.y - waypoints[0].pose.position.y;
        ddx = waypoints[2].pose.position.x + waypoints[0].pose.position.x - 2 * waypoints[1].pose.position.x;
        ddy = waypoints[2].pose.position.y + waypoints[0].pose.position.y - 2 * waypoints[1].pose.position.y;
    }
    else if (index == (waypoints.size() - 1)) {
        dx = waypoints[index].pose.position.x - waypoints[index - 1].pose.position.x;
        dy = waypoints[index].pose.position.y - waypoints[index - 1].pose.position.y;
        ddx = waypoints[index].pose.position.x + waypoints[index - 2].pose.position.x - 2 * waypoints[index].pose.position.x;
        ddy = waypoints[index].pose.position.y + waypoints[index - 2].pose.position.y - 2 * waypoints[index].pose.position.y;
    }
    else {
        dx = waypoints[index + 1].pose.position.x - waypoints[index].pose.position.x;
        dy = waypoints[index + 1].pose.position.y - waypoints[index].pose.position.y;
        ddx = waypoints[index + 1].pose.position.x + waypoints[index - 1].pose.position.x - 2 * waypoints[index].pose.position.x;
        ddy = waypoints[index + 1].pose.position.y + waypoints[index - 1].pose.position.y - 2 * waypoints[index].pose.position.y;
    }
    //res.yaw = atan2(dy, dx);//yaw
    //计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
    res = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2.0) + pow(dy, 2.0)), 3.0)));
    if(dx == 0 &&  dy == 0)  res = 0;
    return res;
}

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
    // while(angle > M_PI){
    //     angle -= 2.0*M_PI;
    // }
    // while(angle < - M_PI){
    //     angle += 2.0*M_PI;
    // }
    // return angle;
	double delta;
	delta = fmod(angle, 2.0 * M_PI);
	if (delta > M_PI)
		delta = delta - 2 * M_PI;
	else if (delta < M_PI * (-1.0))
		delta = delta + 2 * M_PI;
	return delta;
	// return fmod(angle + M_PI, 2.0*M_PI) - M_PI;
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
