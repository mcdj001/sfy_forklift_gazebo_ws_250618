/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 15:00:19
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-26 16:23:15
 */
#include "mypath.h"
using namespace  SfyMotionController;

//添加新的路径点
void MyPath::Add_new_point(waypoint& p)
{
    path.push_back(p);
}

// 更新路径 
void MyPath::Add_new_point(vector<waypoint>& p) 
{
    path = p;
}

//路径点个数
unsigned int MyPath::Size()
{
    return path.size();
}

//获取路径点
waypoint MyPath::Get_waypoint(int index)
{
    waypoint p;
    p.ID = path[index].ID;
    p.x = path[index].x;
    p.y = path[index].y;
    p.yaw = path[index].yaw;
    return p;
}

vector<waypoint> MyPath::Get_waypoints(){
    return path;
}

// 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
int MyPath::Find_target_index(vehicleState state)
{
    double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
    int index = 0;
    for (int i = 0; i < path.size(); i++)
    {
        double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
        if (d < min)
        {
            min = d;
            index = i;
        }
    }

    //索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
    if ((index + 1) < path.size())
    {
        double current_x = path[index].x; double current_y = path[index].y;
        double next_x = path[index + 1].x; double next_y = path[index + 1].y;
        double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
        double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
        //ROS_INFO("L is %f,Lf is %f",L,Lf);
        if (L_1 < L_)
        {
            index += 1;
        }
    }
    return index;
}

int MyPath::Find_target_index_PP(vehicleState state , double  l_d  , int  pre_index)
{
    double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
    int index = 0;
    for (int i = 0; i < path.size(); i++)
    {
        double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
        if (d < min) 
        {
            min = d;
            index = i;
        }
    }
    
    double delta_l =  abs(sqrt(pow(state.x - path[index].x, 2) + pow(state.y - path[index].y, 2)));

    while (l_d > delta_l && index < path.size()-1){
        delta_l = abs(sqrt(pow(state.x - path[index+1].x, 2) + pow(state.y - path[index+1].y, 2)));
        index+=1;
    }
    return index;
}