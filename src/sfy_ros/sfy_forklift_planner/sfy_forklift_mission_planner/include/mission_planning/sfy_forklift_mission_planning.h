/*
 * @Author: clint-sfy 2786435349@qq.com
 * @Date: 2025-05-07 15:07:14
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 19:48:47
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/include/mission_planning/sfy_forklift_mission_planning.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __FORKLIFT_MISSION_PLANNING_CPP__
#define __FORKLIFT_MISSION_PLANNING_CPP__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <math.h>
#include <tf/tf.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#include <limits>

#include "common_utils/common_enum.h"

#include "sfy_forklift_msgs/ForkliftMissionTargetPoint.h"
#include "sfy_forklift_msgs/ForkliftStatus.h"
#include "sfy_forklift_msgs/ForkliftPathPlanningStatus.h"
#include "sfy_forklift_msgs/ForkliftPathTrackingStatus.h"
using namespace std;

typedef enum { 
    FORKLIFT_NO_TASK,         // 待机任务状态
    FORKLIFT_PICKUP_TASK,     // 取货任务状态
    FORKLIFT_DROPOFF_TASK,    // 放货任务状态
    FORKLIFT_PARKING_TASK,    // 停车任务状态
} ForkliftTaskType;

typedef
class SfyForkliftMissionPlanning{
    public:
        SfyForkliftMissionPlanning();
        ~SfyForkliftMissionPlanning();
        void control();

    private:
        //  接收的数据
        double frequency_;
        bool is_start_mission_;
        int  forklift_id_;
        double forklift_target_point_;
        int forklift_target_yaw_;
        string  mission_start_time;
        string  mission_end_time;

        ForkliftTaskType current_forklift_task_type_; // 叉车任务类型
        AGVForkliftStatus current_forklift_status_; // 叉车当前状态
        bool is_start_pickup_task_; // 是否开始取货任务
        bool is_start_dropoff_task_; // 是否开始放货任务
        bool is_start_parking_task_; // 是否开始停车任务

        void forkliftTask();
         
        // 发布话题相关函数
        ros::Publisher mission_target_pub_;     // 任务点发布 
        sfy_forklift_msgs::ForkliftMissionTargetPoint forklift_mission_target_point_;     // 任务点话题内容
        void pubForkliftMissionTargetPoint();

        ros::Publisher forklift_status_pub_;                   
        sfy_forklift_msgs::ForkliftStatus   forklift_status_; 

        // 订阅话题相关函数
        ros::Subscriber forklift_path_planning_status_sub_; 
        sfy_forklift_msgs::ForkliftPathPlanningStatus forklift_path_planning_status_; 
        PathPlanningStatus current_forklift_path_planning_status_;
        PathTrackingStatus current_forklift_path_tracking_status_;
        void forkliftPathPlanningStatusCallback(const sfy_forklift_msgs::ForkliftPathPlanningStatus::ConstPtr& Msg);

        ros::Subscriber forklift_path_tracking_status_sub_;    // 路径跟踪状态
        sfy_forklift_msgs::ForkliftPathTrackingStatus forklift_path_tracking_status_; // 叉车路径跟踪状态
        void pathTrackingStatusCallback(const sfy_forklift_msgs::ForkliftPathTrackingStatus::ConstPtr& Msg);
};

#endif