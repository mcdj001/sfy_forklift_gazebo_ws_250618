/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 14:59:42
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-25 21:28:34
 */

#ifndef  SFY_MOTION_CONTROLLER_MY_PATH_H
#define SFY_MOTION_CONTROLLER_MY_PATH_H

#include "mytools.h"
using namespace std;

namespace  SfyMotionController {
    
    class MyPath {
        private:
	        vector<waypoint> path;
        public:
            //添加新的路径点
            void Add_new_point(waypoint& p);
            // 更新路径 
	        void Add_new_point(vector<waypoint>& p) ;
            //路径点个数
	        unsigned int Size();
            //获取路径点
	        waypoint Get_waypoint(int index);
            vector<waypoint> Get_waypoints();
            // 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
	        int Find_target_index(vehicleState state);
            // PP算法独有前视搜索
            int Find_target_index_PP(vehicleState state , double  l_d , int  pre_index);
    };
}

#endif
