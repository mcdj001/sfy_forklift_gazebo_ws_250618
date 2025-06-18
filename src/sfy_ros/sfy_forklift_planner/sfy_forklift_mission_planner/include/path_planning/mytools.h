/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 13:59:26
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-16 15:58:56
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_planner/sfy_forklift_mission_planner/include/path_planning/mytools.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#ifndef SFY_MISSION_PLANNER_TOOL_H
#define SFY_MISSION_PLANNER_TOOL_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//定义路径点
typedef struct waypoint {
    int ID;
    double x, y, yaw;  //x , y
}waypoint;

//定义小车状态
typedef struct vehicleState {
    double x, y, yaw, v, kesi; // x,y,yaw,前轮偏角kesi
}vehicleState;

//定义控制量
typedef struct U {
    double v;
    double kesi;  //速度v,前轮偏角kesi
}U;

typedef struct vehicleDynamicParam{
    double L;           // 轴距
    double lr;          // 后轮距离重心轴距
    double lf;          // 前轮距离重心轴距
    double cf;          // 前轮侧偏刚度
    double cr;          // 后轮侧偏刚度
    double iz;          // z轴的转动惯量
    double mass;        // 整车质量
    double mass_front;
    double mass_rear;
    double delta_t;     // LQR的时间步长
    int ref_index;      // 跟踪参考点
    int horizon;        // 求解黎卡提方程的迭代次数
    double current_v;
    double current_w;
    double target_index_K; //目标点曲率
}vehicleDynamicParam;

namespace  SfyMissionPlanner {

    class MyTools{
        public:
            double cal_K(vector<waypoint> waypoints, int index);  //计算曲率K
            double normalizeAngle(double angle);  // 角度限制在[-pi  , pi]
            double factorial(int n);  // 阶乘
            Vector2d bezierCommon(vector<Vector2d> Ps, double t); // 贝塞尔曲线最简单形式
            void generateBezierPath(const vector<Vector2d>& Ps, double line_distance, vector<Vector2d>& waypoints); // 等距离生成贝塞尔曲线
        private:

    };
}


#endif    // SFY_MISSION_PLANNER_TOOL_H