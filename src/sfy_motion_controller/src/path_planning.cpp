/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-03-26 11:52:21
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-15 14:48:42
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_motion_controller/src/path_planning.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
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
#include <limits>

#include "trajectory.h"
#include "mytools.h"
using namespace std;

vector<double> wx , wy;
string trajectory_type;
double R;

// 顶点结构体
struct Vertex {
    int id;
    double x, y, weight;
    std::string name;
};

// 图结构体
struct Graph {
    bool useUndigraph;
    int vertexCount;
    std::vector<Vertex> vertices;
    std::vector<std::pair<int, int>> edges;
    std::unordered_map<int, std::vector<std::pair<int, double>>> adjList;
};

// Dijkstra算法函数
std::vector<int> dijkstra(const Graph& graph, int start, int end) {
    std::vector<double> distances(graph.vertexCount, std::numeric_limits<double>::max());
    std::vector<int> previous(graph.vertexCount, -1);
    typedef std::pair<double, int> pii;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq;
    
    distances[start] = 0.0;
    pq.push(std::make_pair(0.0, start));

    while (!pq.empty()) {
        pii top = pq.top();
        pq.pop();
        double dist = top.first;
        int u = top.second;

        if (u == end) break;

        for (const auto& neighbor : graph.adjList.at(u)) {
            int v = neighbor.first;
            double weight = neighbor.second;
            double alt = dist + weight;
            if (alt < distances[v]) {
                distances[v] = alt;
                previous[v] = u;
                pq.push(std::make_pair(alt, v));
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

nav_msgs::Path produce_path(){
    trajectory* trajec  = new trajectory(trajectory_type, wx, wy, R);//实例化trajectory类；
    vector<waypoint> waypoint_vec;//定义vector类用于接收由trajec生成的路径,得到若干组[ID,x,y]
    nav_msgs::Path waypoints;//定义nav_msgs::Path类用于构造ROS的Path消息

    //获取路径
    trajec->refer_path();
    waypoint_vec = trajec->get_path();
 
    //构造适用于ROS的Path消息
    for(int i =0; i < waypoint_vec.size(); i++){
        waypoints.header.stamp = ros::Time::now();
        waypoints.header.frame_id = "odom";
        waypoints.header.seq = i;
        
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.seq = i;
        //ROS_INFO("path_id is %d", this_pose.header.seq);
        this_pose.header.frame_id = "odom";
        this_pose.pose.position.x = waypoint_vec[i].x;
        this_pose.pose.position.y = waypoint_vec[i].y;
        this_pose.pose.position.z = 0;
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(waypoint_vec[i].yaw);
        //ROS_INFO("the yaw is %f",waypoint_vec[i].yaw);
        this_pose.pose.orientation.x = goal_quat.x;
        this_pose.pose.orientation.y = goal_quat.y;
        this_pose.pose.orientation.z = goal_quat.z;
        this_pose.pose.orientation.w = goal_quat.w;

        waypoints.poses.push_back(this_pose);
    }
    return waypoints;//返回适用于ROS的Path消息
}

int main(int argc,char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "sfy_path_planning");
    ros::NodeHandle n;
    ros::NodeHandle n_prv("~");
    n_prv.param<string>("trajectory_type",trajectory_type,"mypath");
    n_prv.param("wx", wx, wx);
	n_prv.param("wy", wy , wy);
    n_prv.param<double>("R", R , 1);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("desired_path", 500 );

//     string filename = "/home/ubuntu20_sfy/sfy_ws/sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/map/test.map";
//     std::ifstream file(filename);
//     Graph graph;
//     std::string line;
    
//     // 读取是否是无向图
//     std::getline(file, line);
//     graph.useUndigraph = (line == "use_undigraph");

//     // 读取顶点数量
//     std::getline(file, line);
//     graph.vertexCount = std::stoi(line);
    
//     // 读取顶点信息
//     for (int i = 0; i < graph.vertexCount; ++i) {
//         std::getline(file, line);
//         std::istringstream vertexStream(line);
//         Vertex vertex;
//         vertexStream >> vertex.x >> vertex.y >> vertex.weight;
//         std::getline(file, vertex.name);
//         vertex.id = i;
//         graph.vertices.push_back(vertex);
//     }

//     // 读取边的数量
//     std::getline(file, line);
//     int edgeCount = std::stoi(line);

//    // 读取边信息
//     for (int i = 0; i < edgeCount; ++i) {
//         std::getline(file, line);
//         std::istringstream edgeStream(line);
//         int u, v;
//         edgeStream >> u >> v;
//         double weight = 1.0; // 假设边的权重为1，可以根据需要调整
//         graph.edges.push_back({u, v});
//         graph.adjList[u].push_back({v, weight});
//         if (graph.useUndigraph) {
//             graph.adjList[v].push_back({u, weight});
//         }
//     }

//     std::cout << "Graph is " << (graph.useUndigraph ? "undirected" : "directed") << std::endl;
//     std::cout << "Vertices:" << std::endl;
//     for (const auto& vertex : graph.vertices) {
//         std::cout << "ID: " << vertex.id << ", Name: " << vertex.name 
//                   << ", Position: (" << vertex.x << ", " << vertex.y << "), Weight: " << vertex.weight << std::endl;
//     }

//     std::cout << "Edges:" << std::endl;
//     for (const auto& edge : graph.edges) {
//         std::cout << "From " << edge.first << " to " << edge.second << std::endl;
//     }

//     int start = 0; // V0
//     int end = 8; // V4

//     std::vector<int> path = dijkstra(graph, start, end);

//     std::cout << "Path: ";
//     for (int vertex : path) {
//         std::cout << graph.vertices[vertex].name << " ";
//     }
//     std::cout << std::endl;

//     // 输出坐标
//     std::cout << "Coordinates: ";
//     for (int vertex : path) {
//         std::cout << "(" << graph.vertices[vertex].x << ", " << graph.vertices[vertex].y << ") ";
//     }
//     std::cout << std::endl;

    ros::Rate loop_rate(1);
    while(ros::ok()){
        nav_msgs::Path path = produce_path();
        // ROS_INFO("the trajectory size is: %ld", path.poses.size() );
        path_pub.publish(path);
        loop_rate.sleep();
    }
    return 0;
}