/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-03-26 11:52:21
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-11 15:25:18
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/src/trajectory.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include <iostream>
#include <vector>
#include "trajectory.h"
#include <math.h>

using namespace std;

double dt = 0.01;  //轨迹计算频率

bool trajectory::is_turn(double x1, double y1, double x2, double y2, double x3, double y3) {
	double EPSILON = 1e-6;
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;
    return (std::abs(dx1 * dy2 - dy1 * dx2) > 0);
}

void trajectory::mypath_bezier_plan_b(){
	vector<double>  new_x , new_y;
	vector<int> new_turn;
	vector<int> turn(wx.size(), 0);

	for (int i = 1; i < wx.size() - 1; ++i) {
        if (is_turn(wx[i - 1], wy[i - 1], wx[i], wy[i], wx[i + 1], wy[i + 1])) {
            turn[i] = 1;
        }
    }
	// cout << "执行到这了1" <<endl;
	int n = wx.size();
    for (int i = 0; i < n; ++i) {
        if (turn[i] == 0) {
            new_x.push_back(wx[i]);
            new_y.push_back(wy[i]);
            new_turn.push_back(0);	
        } 

        if (i > 0 && i < n - 1 && turn[i] == 1) {
            double prev_x = wx[i - 1];
            double prev_y = wy[i - 1];
            double next_x = wx[i + 1];
            double next_y = wy[i + 1];

            double dx1 = wx[i] - prev_x;
            double dy1 = wy[i] - prev_y;
            double length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            dx1 /= length1;
            dy1 /= length1;

            double dx2 = next_x - wx[i];
            double dy2 = next_y - wy[i];
            double length2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            dx2 /= length2;
            dy2 /= length2;

			double prev_result_x = wx[i] - dx1 * 2 * R;
            double prev_result_y = wy[i] - dy1 * 2 * R;

			new_x.push_back(prev_result_x);
            new_y.push_back(prev_result_y);
            new_turn.push_back(0);

            prev_result_x = wx[i] - dx1 * R;
            prev_result_y = wy[i] - dy1 * R;

			new_x.push_back(prev_result_x);
            new_y.push_back(prev_result_y);
            new_turn.push_back(0);
            

            new_x.push_back(wx[i]);
            new_y.push_back(wy[i]);
            new_turn.push_back(1);

			double next_result_x = wx[i] + dx2 * R ;
            double next_result_y = wy[i] + dy2 * R ;

            new_x.push_back(next_result_x);
            new_y.push_back(next_result_y);
            new_turn.push_back(0);

			next_result_x = wx[i] + dx2 * 2 *R ;
            next_result_y = wy[i] + dy2 * 2 *R ;

            new_x.push_back(next_result_x);
            new_y.push_back(next_result_y);
            new_turn.push_back(0);

            // ++i;  
        }
    }

	waypoint PP;
	int k = 0;
	for(int i = 0 ; i < new_x.size() - 1 ; i++){
		if(new_turn[i] == 0 && new_turn[i+2] == 1){  // 贝塞尔曲线优化
			vector<Vector2d>Ps { Vector2d (new_x[i],new_y[i]),Vector2d(new_x[i+1],new_y[i+1]),Vector2d(new_x[i+3],new_y[i+3]),Vector2d(new_x[i+4],new_y[i+4])};
			// 50个点
			 for(int t = 1; t < 15; t++){
        		Vector2d pos = mytools->bezierCommon(Ps,(double)t/15);
				PP.ID = k++;
				PP.x = pos[0];
				PP.y = pos[1];
				waypoints.push_back(PP);
			}
			i = i + 3;
			continue;			
		}		
		if(new_turn[i] == 0 && new_turn[i+1] == 0){   // 生成直线
			vector<double>  line_x , line_y;
			line_x.push_back(new_x[i]);
			line_x.push_back(new_x[i+1]);
			line_y.push_back(new_y[i]);
			line_y.push_back(new_y[i+1]);
			Spline2D csp_obj_line(line_x, line_y);
			for(double j=0; j <= csp_obj_line.s.back(); j += 1.0){
				vector<double> point_= csp_obj_line.calc_postion(j);
				PP.ID = k++;
				PP.x = point_[0];
				PP.y = point_[1];
				waypoints.push_back(PP);
				// std::cout << "Pointss " << i << ": (" << point_[0] << ", " << point_[1] << "), Turn: " << new_turn[i] << std::endl;
			}
			// std::cout << "Point " << i << ": (" << new_x[i+1] << ", " << new_y[i+1] << "), Turn: " << new_turn[i+1] << std::endl;	
		}
		
	}//  for  生成直线和曲线
	//  for (int i = 0; i < new_x.size(); ++i) {
    //     std::cout << "Point " << i << ": (" << new_x[i] << ", " << new_y[i] << "), Turn: " << new_turn[i] << std::endl;
    // }
	// cout <<    "======================="<<endl;
	double last_k = 0 , sum_k = 0;
	for(int i = 0 ; i < waypoints.size() ; i++){
		double k = mytools->cal_K(waypoints , i);
		last_k = k;
		double exit_k = k - last_k;
		sum_k += exit_k;	
		cout << k << endl;		
	}
	double p_max =   tan(38.f / 180.f * M_PI) / 1.19;
	cout <<  "平均曲率为：" << sum_k / waypoints.size() - 1 << "  p_max:  " << p_max<< endl;	
}

void trajectory::mypath_bezier_plan_a(){
	vector<double>  new_x , new_y;
	vector<int> new_turn;
	vector<int> turn(wx.size(), 0);

	for (int i = 1; i < wx.size() - 1; ++i) {
        if (is_turn(wx[i - 1], wy[i - 1], wx[i], wy[i], wx[i + 1], wy[i + 1])) {
            turn[i] = 1;
        }
    }
	// cout << "执行到这了1" <<endl;
	int n = wx.size();
    for (int i = 0; i < n; ++i) {
        if (turn[i] == 0) {
            new_x.push_back(wx[i]);
            new_y.push_back(wy[i]);
            new_turn.push_back(0);	
        } 

        if (i > 0 && i < n - 1 && turn[i] == 1) {
            double prev_x = wx[i - 1];
            double prev_y = wy[i - 1];
            double next_x = wx[i + 1];
            double next_y = wy[i + 1];

            double dx1 = wx[i] - prev_x;
            double dy1 = wy[i] - prev_y;
            double length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            dx1 /= length1;
            dy1 /= length1;

            double dx2 = next_x - wx[i];
            double dy2 = next_y - wy[i];
            double length2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            dx2 /= length2;
            dy2 /= length2;

            double prev_result_x = wx[i] - dx1 * R;
            double prev_result_y = wy[i] - dy1 * R;

			new_x.push_back(prev_result_x);
            new_y.push_back(prev_result_y);
            new_turn.push_back(0);
            

            new_x.push_back(wx[i]);
            new_y.push_back(wy[i]);
            new_turn.push_back(1);

			double next_result_x = wx[i] + dx2 * R ;
            double next_result_y = wy[i] + dy2 * R ;

            new_x.push_back(next_result_x);
            new_y.push_back(next_result_y);
            new_turn.push_back(0);

            // ++i;  
        }
    }

	waypoint PP;
	int k = 0;
	for(int i = 0 ; i < new_x.size() - 1 ; i++){
		// cout << "当前i为" << i << endl;		
		if(new_turn[i] == 0 && new_turn[i+1] == 0){   // 生成直线
			vector<double>  line_x , line_y;
			line_x.push_back(new_x[i]);
			line_x.push_back(new_x[i+1]);
			line_y.push_back(new_y[i]);
			line_y.push_back(new_y[i+1]);
			Spline2D csp_obj_line(line_x, line_y);
			for(double j=0; j <= csp_obj_line.s.back(); j += 1.0){
				vector<double> point_= csp_obj_line.calc_postion(j);
				PP.ID = k++;
				PP.x = point_[0];
				PP.y = point_[1];
				waypoints.push_back(PP);
				// std::cout << "Pointss " << i << ": (" << point_[0] << ", " << point_[1] << "), Turn: " << new_turn[i] << std::endl;
			}
			// std::cout << "Point " << i << ": (" << new_x[i+1] << ", " << new_y[i+1] << "), Turn: " << new_turn[i+1] << std::endl;	
		}
		if(new_turn[i] == 0 && new_turn[i+1] == 1){  // 贝塞尔曲线优化
			vector<Vector2d>Ps { Vector2d (new_x[i],new_y[i]),Vector2d(new_x[i+1],new_y[i+1]),Vector2d(new_x[i+2],new_y[i+2])};
			// 50个点
			 for(int t = 1; t < 15; t++){
        		Vector2d pos = mytools->bezierCommon(Ps,(double)t/15);
				PP.ID = k++;
				PP.x = pos[0];
				PP.y = pos[1];
				waypoints.push_back(PP);
			}
			i++;			
		}
	}//  for  生成直线和曲线
	//  for (int i = 0; i < new_x.size(); ++i) {
    //     std::cout << "Point " << i << ": (" << new_x[i] << ", " << new_y[i] << "), Turn: " << new_turn[i] << std::endl;
    // }
	// cout <<    "======================="<<endl;
	double last_k = 0 , sum_k = 0;
	for(int i = 0 ; i < waypoints.size() ; i++){
		double k = mytools->cal_K(waypoints , i);
		last_k = k;
		double exit_k = k - last_k;
		sum_k += exit_k;	
		cout << k << endl;		
	}
	double p_max =   tan(38.f / 180.f * M_PI) / 1.19;
	cout <<  "平均曲率为：" << sum_k / waypoints.size() - 1 << "  p_max:   " << p_max << endl;	
}

void trajectory::mypath_bezier(){
	// mypath_bezier_plan_a();
	mypath_bezier_plan_b();

}

void trajectory::mypath_line(){
	waypoint PP;
	for(int i = 0 ; i < wx.size() - 1 ; i++){
		vector<double>  new_x , new_y;
		new_x.push_back(wx[i]);
		new_x.push_back(wx[i+1]);
		new_y.push_back(wy[i]);
		new_y.push_back(wy[i+1]);
		Spline2D csp_obj(new_x, new_y);
		for(double i=0; i <= csp_obj.s.back(); i += 1.0){
			vector<double> point_= csp_obj.calc_postion(i);
			PP.ID = i;
			PP.x = point_[0];
			PP.y = point_[1];
			waypoints.push_back(PP);
		}	
	}
}

void trajectory::mypath(){
	waypoint PP;
	Spline2D csp_obj(wx, wy);
	for(double i=0; i < csp_obj.s.back(); i += 1.0){
		vector<double> point_= csp_obj.calc_postion(i);
		PP.ID = i;
		PP.x = point_[0];
		PP.y = point_[1];
		waypoints.push_back(PP);
	}	
};

void trajectory::infinite(){
	waypoint PP;
	int iter = 1000;
	double period = 1000;
	double scale_factor = 1;
    for (int t = 0; t < iter; t++) {
		PP.ID = t;
		PP.x = 10 * cos(2 * M_PI* t / period) / (pow(sin(2 * M_PI * (t) / period), 2) + 1); 
        PP.y = 10 * sin(2 * M_PI* t / period) * cos(2 * M_PI* t / period) / (pow(sin(2 * M_PI * (t) / period), 2) + 1); 
		waypoints.push_back(PP);
	}
};

void trajectory::epitrochoid(){
	waypoint PP;
	int iter = 1000;
	double R = 5;
	double r = 1;
	double d = 3;
	double period = 1000;
	double scale_factor = 1;
    for (int t = 0; t < iter; t++) {
		PP.ID = t;
		PP.x = scale_factor * ((R + r) * cos(2 * M_PI * t/ period) - d * cos(((R + r) / r) * 2 * M_PI * t / period)); 
        PP.y = scale_factor * ((R + r) * sin(2 * M_PI * t/ period) - d * sin(((R + r) / r) * 2 * M_PI * t / period)); 
		waypoints.push_back(PP);
	}
};

void trajectory::circle(){
	waypoint PP;
	int iter = 1000;
	double  radius = 5;
    double  period = 1000;
    for (int t = 0; t < iter; t++) {
		PP.ID = t;
		PP.x = radius * sin(2 * M_PI * t / period); 
        PP.y = -radius * cos(2 * M_PI * t / period); 
		waypoints.push_back(PP);
	}
};

void trajectory::refer_path() {
	if(trajectory_type == "mypath") mypath();
	else if(trajectory_type == "mypath_line") mypath_line();
	else if(trajectory_type == "mypath_bezier") mypath_bezier();
	else if(trajectory_type == "circle")  circle();
	else if(trajectory_type == "epitrochoid")  epitrochoid();
	else if(trajectory_type == "infinite")  infinite();
	

	//计算切线方向并储存
	for (int j=0; j<waypoints.size();j++){
		double dx, dy, yaw;
		if (j == 0) {
			dx = waypoints[1].x - waypoints[0].x;
			dy = waypoints[1].y - waypoints[0].y;
		}
		else if (j == (waypoints.size() - 1)) {
			dx = waypoints[j].x - waypoints[j - 1].x;
			dy = waypoints[j].y - waypoints[j - 1].y;
		}
		else {
			dx = waypoints[j + 1].x - waypoints[j].x;
			dy = waypoints[j + 1].y - waypoints[j].y;
		}
		yaw = atan2(dy, dx);//yaw
		waypoints[j].yaw = mytools->normalizeAngle(yaw);
	}
}

vector<waypoint> trajectory::get_path() {
	return waypoints;
}
