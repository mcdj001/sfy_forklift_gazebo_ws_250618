/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-11 15:56:43
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 19:35:43
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/include/path_planning/sfy_forklift_path_planning.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef __FORKLIFT_PATH_PLANNING_CPP__
#define __FORKLIFT_PATH_PLANNING_CPP__

#include <ros/ros.h>
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <ros/master.h>
#include <math.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

#include "path_planning/trajectory.h"
#include "path_planning/mytools.h"
#include "common_utils/common_enum.h"

#include "sfy_forklift_msgs/ForkliftMissionTargetPoint.h"
#include "sfy_forklift_msgs/ForkliftStatus.h"
#include "sfy_forklift_msgs/ForkliftPathPlanningStatus.h"
#include "sfy_forklift_msgs/ForkliftPathPlanningResult.h"
#include "sfy_forklift_msgs/ForkliftPathTrackingStatus.h"

#include <actionlib/client/simple_action_client.h>
#include <sfy_forklift_mission_planner/SfyForkliftMoveBasePluginAction.h>
using namespace std;

// 顶点结构体
typedef struct Vertex {
    int id;
    double x, y, weight;
    std::string name;
}Vertex;

// 图结构体
typedef struct Graph {
    bool useUndigraph;
    int vertexCount;
    int edgeCount;
    std::vector<Vertex> vertices;
    std::vector<std::pair<int, int>> edges;
    std::unordered_map<int, std::vector<std::pair<int, double>>> adjList;
}Graph;

typedef enum {
    ARRAY_GENERATION,      // 提供数组生成路径模式（开发调试用）
    ALGORITHM_TESTING,     // 测试路径规划算法模式（算法验证）
    DEPLOYMENT             // 实机部署模式（实际运行）
} PathPlanningMode;

struct State {
    int vertex;
    Yaw yaw;
    double totalWeight;
    int turns;
    vector<int> path;
    bool operator<(const State& other) const {
        if (totalWeight != other.totalWeight)
            return totalWeight > other.totalWeight;
        return turns > other.turns;
    }
};

typedef struct findClosestEdgeResult {
    int x1_id;
    int x2_id;
    Yaw yaw;
    bool success;
}findClosestEdgeResult;


class SfyForkliftPathPlanning{
public:
    SfyForkliftPathPlanning();
    ~SfyForkliftPathPlanning();
    void control();

private:

    vector<double> wx_ , wy_;
    string trajectory_type_;
    string map_file_path_;
    string desired_path_frame_id_;
    double R_;
    double frequency_;
    double line_distance_;
    int curve_num_;
    PathPlanningMode path_planning_mode_;
    int int_path_planning_mode_;
    Graph graph_;
    ros::Publisher path_pub_;
    PathPlanningStatus path_planning_status_;   

    nav_msgs::Path produceRosPath();
    nav_msgs::Path produceRosPathByVector(const vector<double> wx_vector, const vector<double> wy_vector);
    
    void chooseModel();
    void algorithmTesting();
    void deployment();
    void checkSubscriberStatus();

    double distance(const Vertex& a, const Vertex& b);
    Graph loadStaticPointMap(string map_file_path);
    vector<int> dijkstra(const Graph& graph, int start, int end);

    int findPreviousPoint(Graph& graph, int targetId, double orientation);   // 寻找转弯点
    findClosestEdgeResult findClosestEdge(const Graph &graph, double forklift_x, double forklift_y, double forklift_yaw, double R);
    double computeYaw(const Vertex& v1, const Vertex& v2);
    Vertex getVertexById(const Graph &graph, int id);

    Yaw calculateYaw(const Vertex& u, const Vertex& v);
    vector<Yaw> getPerpendicular(Yaw yaw);
    pair<vector<int>, Yaw> findShortestPath(const Graph& graph, int startId, Yaw startYaw, int endId, Yaw targetYaw);
    Yaw doubleToYaw(double yaw);
    string yawToString(Yaw yaw);
    double yawToDouble(Yaw yaw);


    // 订阅话题相关函数
    ros::Subscriber odom_sub_;                                                  // 订阅里程计话题
    nav_msgs::Odometry odom_;
    bool is_odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

    ros::Subscriber mission_target_sub_;    // 任务点
    sfy_forklift_msgs::ForkliftMissionTargetPoint forklift_mission_target_point_;     // 任务点话题内容
    bool is_mission_target_sub_;
    void missionTargetPointCallback(const sfy_forklift_msgs::ForkliftMissionTargetPoint::ConstPtr& Msg);

    ros::Subscriber forklift_status_sub_;    // 当前叉车的状态
    sfy_forklift_msgs::ForkliftStatus forklift_status_;     // 任务点话题内容
    AGVForkliftStatus current_forklift_status_;
    bool is_current_forklift_status_sub_;
    void forkliftStatusCallback(const sfy_forklift_msgs::ForkliftStatus::ConstPtr& Msg);

    ros::Subscriber forklift_path_tracking_status_sub_;    // 路径跟踪状态
    sfy_forklift_msgs::ForkliftPathTrackingStatus forklift_path_tracking_status_; // 叉车路径跟踪状态
    bool is_path_tracking_status_sub_;
    void pathTrackingStatusCallback(const sfy_forklift_msgs::ForkliftPathTrackingStatus::ConstPtr& Msg);

    // 发布话题相关函数
    ros::Publisher forklift_path_planning_status_pub_;     // 发布路径规划状态             
    sfy_forklift_msgs::ForkliftPathPlanningStatus   forklift_path_planning_status_;  // 发布路径规划状态
    void pubPathPlanningStatus();

    ros::Publisher forklift_path_planning_result_pub_;     // 发布路径规划结果
    sfy_forklift_msgs::ForkliftPathPlanningResult forklift_path_planning_result_;  // 发布路径规划结果
    
    // 实机部署参数
    vector<double> positive_wx_, positive_wy_;
    vector<double> reverse_wx_, reverse_wy_;
    int current_path_index_ , totoal_path_index_;  // 当前路径数和总路径数
    PathTrackingStatus path_tracking_status_;      // 路径规划状态
    nav_msgs::Path deployment_path_;

    actionlib::SimpleActionClient<sfy_forklift_mission_planner::SfyForkliftMoveBasePluginAction> move_base_plugin_ac_;


    

};    


#endif