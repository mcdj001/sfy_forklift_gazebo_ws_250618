/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-10-11 15:05:39
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 19:57:53
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/src/sfy_forklift_path_planning.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#include"path_planning/sfy_forklift_path_planning.h"

void SfyForkliftPathPlanning::pathTrackingStatusCallback(const sfy_forklift_msgs::ForkliftPathTrackingStatus::ConstPtr& Msg){
    forklift_path_tracking_status_ = *Msg;
}


void SfyForkliftPathPlanning::pubPathPlanningStatus(){
    // forklift_path_planning_status_.forklift_id = forklift_status_.forklift_id;
    // forklift_path_planning_status_.forklift_path_planning_status = path_planning_status_;
    // forklift_path_planning_status_pub_.publish(forklift_path_planning_status_);
}

void SfyForkliftPathPlanning::forkliftStatusCallback(const sfy_forklift_msgs::ForkliftStatus::ConstPtr& Msg){
    forklift_status_ =  *Msg;
}

void SfyForkliftPathPlanning::missionTargetPointCallback(const sfy_forklift_msgs::ForkliftMissionTargetPoint::ConstPtr& Msg)
{
    forklift_mission_target_point_ =  *Msg;
}

void SfyForkliftPathPlanning::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom_ = *odomMsg;        
}

Vertex SfyForkliftPathPlanning::getVertexById(const Graph &graph, int id) {
    for (const auto &v : graph.vertices) {
        if (v.id == id) {
            return v;
        }
    }
    throw runtime_error("Vertex not found");
}

double SfyForkliftPathPlanning::computeYaw(const Vertex& v1, const Vertex& v2) {
    const double eps = 1e-6;
    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    if (abs(dy) <= eps) {
        if (dx > eps) {
            return 0.0;
        } else if (dx < -eps) {
            return M_PI;
        } else {
            return 0.0;
        }
    } else if (abs(dx) <= eps) {
        if (dy > eps) {
            return M_PI / 2;
        } else {
            return -M_PI / 2;
        }
    } else {
        throw runtime_error("Edge direction not allowed");
    }
}

findClosestEdgeResult SfyForkliftPathPlanning::findClosestEdge(const Graph &graph, double forklift_x, double forklift_y, double forklift_yaw, double R) {
    if (graph.vertices.empty()) {
        return { -1, -1, EAST, false };
    }
    // Find closest vertex x1
    Vertex x1 = graph.vertices[0];
    double min_dist = numeric_limits<double>::max();
    for (const auto &v : graph.vertices) {
        double dx = v.x - forklift_x;
        double dy = v.y - forklift_y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < min_dist) {
            min_dist = dist;
            x1 = v;
        }
    }
    // Check adjacency list of x1
    if (graph.adjList.find(x1.id) == graph.adjList.end()) {
        return { -1, -1, EAST, false };
    }
    const auto &neighbors = graph.adjList.at(x1.id);
    for (const auto &neighbor : neighbors) {
        int x2_id = neighbor.first;
        Vertex x2;
        try {
            x2 = getVertexById(graph, x2_id);
        } catch (const runtime_error &) {
            continue;
        }
        // Compute yaw1 between x1 and x2
        double yaw1;
        try {
            yaw1 = computeYaw(x1, x2);
        } catch (const runtime_error &) {
            continue;
        }
        // Compute delta yaw
        double delta = yaw1 - forklift_yaw;
        delta = fmod(delta + M_PI, 2*M_PI) - M_PI;
        if (abs(delta) >= 0.2) {
            continue;
        }
        // Check distance to x2
        double dx_to_x2 = forklift_x - x2.x;
        double dy_to_x2 = forklift_y - x2.y;
        double dist_to_x2 = sqrt(dx_to_x2*dx_to_x2 + dy_to_x2*dy_to_x2);
        if (dist_to_x2 >= R) {
            return { x1.id, x2.id, doubleToYaw(yaw1), true };
        } else {
            // Traverse along yaw1 direction
            unordered_set<int> visited;
            visited.insert(x1.id);
            visited.insert(x2.id);
            Vertex current = x2;
            bool found = false;
            while (true) {
                if (graph.adjList.find(current.id) == graph.adjList.end()) {
                    break;
                }
                const auto &current_neighbors = graph.adjList.at(current.id);
                int next_id = -1;
                Vertex next_v;
                for (const auto &n : current_neighbors) {
                    int candidate_id = n.first;
                    if (visited.count(candidate_id)) {
                        continue;
                    }
                    Vertex candidate;
                    try {
                        candidate = getVertexById(graph, candidate_id);
                    } catch (const runtime_error &) {
                        continue;
                    }
                    double current_yaw;
                    try {
                        current_yaw = computeYaw(current, candidate);
                    } catch (const runtime_error &) {
                        continue;
                    }
                    if (abs(current_yaw - yaw1) < 1e-6) {
                        next_id = candidate_id;
                        next_v = candidate;
                        break;
                    }
                }
                if (next_id == -1) {
                    break;
                }
                if (visited.count(next_id)) {
                    break;
                }
                visited.insert(next_id);
                double dx_next = forklift_x - next_v.x;
                double dy_next = forklift_y - next_v.y;
                double dist_next = sqrt(dx_next*dx_next + dy_next*dy_next);
                if (dist_next >= R) {
                    return { x1.id, next_v.id, doubleToYaw(yaw1), true };
                } else {
                    current = next_v;
                }
            }
        }
    }
    return { -1, -1, EAST, false };
}

Yaw SfyForkliftPathPlanning::calculateYaw(const Vertex& u, const Vertex& v) {
    double dx = v.x - u.x;
    double dy = v.y - u.y;
    const double epsilon = 1e-6;
    if (dx > epsilon) return EAST;
    if (dx < -epsilon) return WEST;
    if (dy > epsilon) return NORTH;
    return SOUTH;
}

vector<Yaw> SfyForkliftPathPlanning::getPerpendicular(Yaw yaw) {
    switch(yaw) {
        case EAST: return {NORTH, SOUTH};
        case NORTH: return {EAST, WEST};
        case WEST: return {NORTH, SOUTH};
        case SOUTH: return {EAST, WEST};
        default: return {};
    }
}

pair<vector<int>, Yaw> SfyForkliftPathPlanning::findShortestPath(const Graph& graph, int startId, Yaw startYaw, int endId, Yaw targetYaw) {
    unordered_map<int, Vertex> vertexMap;
    for (const auto& v : graph.vertices) {
        vertexMap[v.id] = v;
    }
    vector<Yaw> targetYaws = getPerpendicular(targetYaw);
    priority_queue<State> pq;
    pq.push({startId, startYaw, 0.0, 0, {startId}});
    unordered_map<int, unordered_map<Yaw, pair<double, int>>> visited;
    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();
        if (visited[current.vertex].count(current.yaw)) {
            auto& record = visited[current.vertex][current.yaw];
            if (record.first < current.totalWeight || (record.first == current.totalWeight && record.second <= current.turns)) {
                continue;
            }
        }
        visited[current.vertex][current.yaw] = {current.totalWeight, current.turns};
        if (current.vertex == endId) {
            if (find(targetYaws.begin(), targetYaws.end(), current.yaw) != targetYaws.end()) {
                return {current.path, current.yaw};
            }
        }
        Vertex u = vertexMap[current.vertex];
        if (graph.adjList.find(current.vertex) == graph.adjList.end()) continue;
        for (const auto& edge : graph.adjList.at(current.vertex)) {
            int vId = edge.first;
            if (vertexMap.find(vId) == vertexMap.end()) continue;
            Vertex v = vertexMap[vId];
            double edgeWeight = edge.second;
            Yaw newYaw = calculateYaw(u, v);
            int newTurns = current.turns + (newYaw != current.yaw ? 1 : 0);
            double newTotalWeight = current.totalWeight + edgeWeight;
            vector<int> newPath = current.path;
            newPath.push_back(vId);
            State nextState = {vId, newYaw, newTotalWeight, newTurns, newPath};
            if (visited[vId].count(newYaw)) {
                auto& existing = visited[vId][newYaw];
                if (existing.first < newTotalWeight || (existing.first == newTotalWeight && existing.second <= newTurns)) {
                    continue;
                }
            }
            pq.push(nextState);
        }
    }
    return {{}, EAST};
}

Yaw SfyForkliftPathPlanning::doubleToYaw(double yaw) {
    const double eps = 1e-6;
    if (abs(yaw - 0.0) < eps) return EAST;
    if (abs(yaw - M_PI/2) < eps) return NORTH;
    if (abs(yaw - M_PI) < eps) return WEST;
    if (abs(yaw + M_PI/2) < eps) return SOUTH;
    return EAST;
}

string SfyForkliftPathPlanning::yawToString(Yaw yaw) {
    switch(yaw) {
        case EAST: return "0";
        case NORTH: return "pi/2";
        case WEST: return "pi";
        case SOUTH: return "-pi/2";
        default: return "unknown";
    }
}

double SfyForkliftPathPlanning::yawToDouble(Yaw yaw) {
    switch(yaw) {
        case EAST: return 0.0;
        case NORTH: return M_PI/2;
        case WEST: return M_PI;
        case SOUTH: return -M_PI/2;
        default: return 0.0;  // default to EAST direction
    }
}

int SfyForkliftPathPlanning::findPreviousPoint(Graph& graph, int targetId, double orientation) {
    // 查找目标顶点
    Vertex target;
    bool found = false;
    int notPoint = -1;
    for (const auto& v : graph.vertices) {
        if (v.id == targetId) {
            target = v;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cout << "没有找到转弯点" << std::endl;
        return -1;
    }
    double x0 = target.x;
    double y0 = target.y;
    std::vector<Vertex> candidates;
    // 检查邻接点是否存在
    if (graph.adjList.find(targetId) == graph.adjList.end()) {
        std::cout << "没有找到转弯点" << std::endl;
        return -1;
    }
    // 遍历邻接点
    for (const auto& neighbor : graph.adjList[targetId]) {
        int neighborId = neighbor.first;
        Vertex neighborVertex;
        bool neighborFound = false;
        for (const auto& v : graph.vertices) {
            if (v.id == neighborId) {
                neighborVertex = v;
                neighborFound = true;
                break;
            }
        }
        if (!neighborFound) continue;
        // 根据姿态筛选候选点
        if (orientation == M_PI) { // 西，同一y，x更小
            if (neighborVertex.y == y0 && neighborVertex.x < x0) {
                candidates.push_back(neighborVertex);
            }
        } else if (orientation == 0) { // 东，同一y，x更大
            if (neighborVertex.y == y0 && neighborVertex.x > x0) {
                candidates.push_back(neighborVertex);
            }
        } else if (orientation == M_PI/2) { // 北，同一x，y更大
            if (neighborVertex.x == x0 && neighborVertex.y > y0) {
                candidates.push_back(neighborVertex);
            }
        } else if (orientation == -M_PI/2) { // 南，同一x，y更小
            if (neighborVertex.x == x0 && neighborVertex.y < y0) {
                candidates.push_back(neighborVertex);
            }
        }
    }
    if (candidates.empty()) {
        std::cout << "没有找到转弯点" << std::endl;
        return -1;
    }
    // 确定最近的点
    Vertex prev;
    bool hasPrev = false;
    if (orientation == M_PI) { // 西，x最大的
        double maxX = -INFINITY;
        for (const auto& v : candidates) {
            if (v.x > maxX) {
                maxX = v.x;
                prev = v;
                hasPrev = true;
            }
        }
    } else if (orientation == 0) { // 东，x最小的
        double minX = INFINITY;
        for (const auto& v : candidates) {
            if (v.x < minX) {
                minX = v.x;
                prev = v;
                hasPrev = true;
            }
        }
    } else if (orientation == M_PI/2) { // 北，y最小的
        double minY = INFINITY;
        for (const auto& v : candidates) {
            if (v.y < minY) {
                minY = v.y;
                prev = v;
                hasPrev = true;
            }
        }
    } else if (orientation == -M_PI/2) { // 南，y最大的
        double maxY = -INFINITY;
        for (const auto& v : candidates) {
            if (v.y > maxY) {
                maxY = v.y;
                prev = v;
                hasPrev = true;
            }
        }
    }
    if (hasPrev) {
        // std::cout << prev.id << std::endl;
        return prev.id;
    } else {
        std::cout << "没有找到转弯点" << std::endl;
        return -1;
    }
}

double SfyForkliftPathPlanning::distance(const Vertex& a, const Vertex& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

vector<int> SfyForkliftPathPlanning::dijkstra(const Graph& graph, int start, int end) {
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

Graph SfyForkliftPathPlanning::loadStaticPointMap(string map_file_path){
    string filename = map_file_path;
    std::ifstream file(filename);
    Graph graph;
    std::string line;
    
    // 读取是否是无向图
    std::getline(file, line);
    graph.useUndigraph = (line == "use_undigraph");
    // 读取顶点部分
    std::getline(file, line); // 读取"point"行
    if (line != "point") {
        // 处理格式错误，例如抛出异常或返回
        std::cout << "Invalid file format: 'point' marker missing." << std::endl;
    }
    // 初始化顶点计数
    graph.vertexCount = 0;
    // 读取顶点信息，直到遇到"edge"行
    while (true) {
        // 先读取一行，判断是否是"edge"
        if (!std::getline(file, line)) {
            // 文件意外结束
            std::cout << "Unexpected end of file while reading vertices." << std::endl;
            break;
        }
        if (line == "edge") {
            break;
        }
        // 解析顶点坐标和权重
        std::istringstream vertexStream(line);
        Vertex vertex;
        if (!(vertexStream >> vertex.x >> vertex.y >> vertex.weight)) {
                std::cout << "Error parsing vertex coordinates: " << line << std::endl;
                continue;
            }
            // 读取顶点名称行
            if (!std::getline(file, vertex.name)) {
                std::cout << "Unexpected end of file while reading vertex name." << std::endl;
                break;
            }
            // 分配ID并添加到图中
            vertex.id = graph.vertices.size();
            graph.vertices.push_back(vertex);
            graph.vertexCount++;
    }

    // 读取边部分
    graph.edgeCount = 0;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue; // 跳过空行
        }
        std::istringstream edgeStream(line);
        int u, v;
        if (!(edgeStream >> u >> v)) {
            std::cout << "Error parsing edge: " << line << std::endl;
            continue;
        }
        // 添加边到结构体
        double weight = 1.0; // 默认权重
        graph.edges.push_back({u, v});
        graph.adjList[u].push_back({v, weight});
        if (graph.useUndigraph) {
            graph.adjList[v].push_back({u, weight});
        }
        graph.edgeCount++;
    }
    return graph; 
}

nav_msgs::Path SfyForkliftPathPlanning::produceRosPathByVector(const vector<double> wx_vector, const vector<double> wy_vector){
    trajectory* trajec_vector  = new trajectory(trajectory_type_, wx_vector, wy_vector, R_ , line_distance_ , curve_num_);
    vector<waypoint> waypoint_vec; // 定义vector类用于接收由trajec生成的路径,得到若干组[ID,x,y]
    nav_msgs::Path waypoints;      

    //获取路径
    trajec_vector->refer_path();
    waypoint_vec = trajec_vector->get_path();
 
    //构造适用于ROS的Path消息
    for(int i = 0; i < waypoint_vec.size(); i++){
        waypoints.header.stamp = ros::Time::now();
        waypoints.header.frame_id = desired_path_frame_id_;
        waypoints.header.seq = i;
        
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.seq = i;
        //ROS_INFO("path_id is %d", this_pose.header.seq);
        this_pose.header.frame_id = desired_path_frame_id_;
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
    return waypoints;
}

nav_msgs::Path SfyForkliftPathPlanning::produceRosPath(){
    trajectory* trajec  = new trajectory(trajectory_type_, wx_, wy_, R_ , line_distance_ , curve_num_);
    vector<waypoint> waypoint_vec; // 定义vector类用于接收由trajec生成的路径,得到若干组[ID,x,y]
    nav_msgs::Path waypoints;      

    //获取路径
    trajec->refer_path();
    waypoint_vec = trajec->get_path();
 
    //构造适用于ROS的Path消息
    for(int i = 0; i < waypoint_vec.size(); i++){
        waypoints.header.stamp = ros::Time::now();
        waypoints.header.frame_id = desired_path_frame_id_;
        waypoints.header.seq = i;
        
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.seq = i;
        //ROS_INFO("path_id is %d", this_pose.header.seq);
        this_pose.header.frame_id = desired_path_frame_id_;
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
    return waypoints;
}

void SfyForkliftPathPlanning::algorithmTesting(){
    std::cout << "Graph is " << (graph_.useUndigraph ? "undirected" : "directed") << std::endl;
    std::cout << "Vertices:" << std::endl;
    for (const auto& vertex : graph_.vertices) {
        std::cout << "ID: " << vertex.id << ", Name: " << vertex.name 
                  << ", Position: (" << vertex.x << ", " << vertex.y << "), Weight: " << vertex.weight << std::endl;
    }

    std::cout << "Edges:" << std::endl;
    for (const auto& edge : graph_.edges) {
        std::cout << "From " << edge.first << " to " << edge.second << std::endl;
    }

    // int start = 0;
    // int end = 8; 

    // std::vector<int> path = dijkstra(graph, start, end);

    // std::cout << "Path: ";
    // for (int vertex : path) {
    //     std::cout << graph.vertices[vertex].name << " ";
    // }
    // std::cout << std::endl;

    // // 输出坐标
    // std::cout << "Coordinates: ";
    // for (int vertex : path) {
    //     std::cout << "(" << graph.vertices[vertex].x << ", " << graph.vertices[vertex].y << ") ";
    // }
    // std::cout << std::endl;

    double forklift_x = 5.1;
    double forklift_y = 5.0;
    double forklift_yaw = 0.1;
    double forklift_r = 1.6;

    // 找到离叉车最近的边
    findClosestEdgeResult find_edge = findClosestEdge(graph_,forklift_x, forklift_y, forklift_yaw, forklift_r );
    cout << "x1_id:" << find_edge.x1_id  << "  x2_id:" << find_edge.x2_id << "  yaw_str:" << yawToString(find_edge.yaw).c_str() << endl;

    int startId = find_edge.x2_id;
    // double startYawDouble = 0.0;
    // Yaw startYaw = doubleToYaw(startYawDouble);
    Yaw startYaw = find_edge.yaw;
    int endId = 24;
    double targetYawDouble = -M_PI/2;
    Yaw targetYaw = doubleToYaw(targetYawDouble);
   
    // 找到转弯点
    int turnId = findPreviousPoint(graph_, endId, targetYawDouble);
    cout << "转弯点的id为：" << turnId << endl;

    if(turnId != -1){
        // 生成最短路径
        auto result = findShortestPath(graph_, startId, startYaw, turnId, targetYaw);
        vector<int> path_positive = result.first;
        if (path_positive.empty()) {
            std::cout << "生成最短路径失败" << std::endl;
        } else {
            path_positive.insert(path_positive.begin(), find_edge.x1_id);
        }
        Yaw turnYaw = result.second;
        printf("最短路径Id号: ");
        for (int id : path_positive) {
            printf("%d ", id);
        }
        printf("\n转弯点方向: %s\n", yawToString(turnYaw).c_str());

        double turn_yaw = yawToDouble(turnYaw);
        Vertex turn_vertex = getVertexById(graph_, turnId);
        Vertex end_vertex = getVertexById(graph_, endId);

        double extra_x = turn_vertex.x + forklift_r * cos(turn_yaw);
        double extra_y = turn_vertex.y + forklift_r * sin(turn_yaw);

        vector<double> positive_wx, positive_wy;
        for (int vertexId : path_positive) {
            Vertex v = getVertexById(graph_, vertexId);
            positive_wx.push_back(v.x);
            positive_wy.push_back(v.y);
        }
        positive_wx.push_back(extra_x);
        positive_wy.push_back(extra_y);

        vector<double> reverse_wx = {extra_x , turn_vertex.x , end_vertex.x};
        vector<double> reverse_wy = {extra_y , turn_vertex.y , end_vertex.y};

        cout << "正向行驶路径点" << endl;
        for (size_t i = 0; i < positive_wx.size(); ++i) {
            std::cout << "(" << positive_wx[i] << ", " << positive_wy[i] << ")" << std::endl;
        }
        cout << "逆向行驶路径点" << endl;
        for (size_t i = 0; i < reverse_wx.size(); ++i) {
            std::cout << "(" << reverse_wx[i] << ", " << reverse_wy[i] << ")" << std::endl;
        }
    }
}

void SfyForkliftPathPlanning::deployment(){
    // 任务点话题和叉车状态话题是否发布   没有发布不能路径规划
    // cout << "is_current_forklift_status_sub_:" << is_current_forklift_status_sub_ << endl;
    // cout << "is_mission_target_sub_:" << is_mission_target_sub_ << endl;
    if(is_current_forklift_status_sub_ && is_mission_target_sub_ ){
        // 叉车在路径规划状态中
        current_forklift_status_ = static_cast<AGVForkliftStatus>(forklift_status_.forklift_status);
        cout << "当前叉车状态: " << current_forklift_status_ << endl;
        if(current_forklift_status_ == AGV_PICKUP_PATH_PLANNING || current_forklift_status_ == AGV_DROPOFF_PATH_PLANNING
                || current_forklift_status_ == AGV_PARKING_PATH_PLANNING){
            // 任务规划点允许路径规划
            if(forklift_mission_target_point_.is_start_mission && path_planning_status_ != PATH_PLANNING_COMPLETE ){
                path_planning_status_ = PATH_PLANNING;
                
                double forklift_x , forklift_y, forklift_yaw;
                double forklift_r = 1.6;
                ROS_INFO("开始路径规划");

                // forklift_x = 5.1;
                // forklift_y = 5.0;
                // forklift_yaw = 0.1;
                forklift_x = odom_.pose.pose.position.x;
                forklift_y = odom_.pose.pose.position.y;
                forklift_yaw = tf::getYaw(odom_.pose.pose.orientation);
                cout << "叉车位置: (" << forklift_x << ", " << forklift_y << "), 方向: " << forklift_yaw << endl;
                // 找到离叉车最近的边
                findClosestEdgeResult find_edge = findClosestEdge(graph_,forklift_x, forklift_y, forklift_yaw, forklift_r);
                cout << "x1_id:" << find_edge.x1_id  << "  x2_id:" << find_edge.x2_id << "  yaw_str:" << yawToString(find_edge.yaw).c_str() << endl;

                int startId = find_edge.x2_id;
                Yaw startYaw = find_edge.yaw;
                int endId = forklift_mission_target_point_.forklift_target_point;
                
                Yaw targetYaw = static_cast<Yaw>(forklift_mission_target_point_.forklift_target_yaw);
                double targetYawDouble = yawToDouble(targetYaw);
            
                // 找到转弯点
                int turnId = findPreviousPoint(graph_, endId, targetYawDouble);
                cout << "转弯点的id为：" << turnId << endl;

                if(turnId != -1){
                    // 生成最短路径
                    auto result = findShortestPath(graph_, startId, startYaw, turnId, targetYaw);
                    vector<int> path_positive = result.first;
                    if (path_positive.empty()) {
                        std::cout << "生成最短路径失败" << std::endl;
                        path_planning_status_ = PATH_PLANNING_FAILED;
                    } else {
                        path_positive.insert(path_positive.begin(), find_edge.x1_id);
                        Yaw turnYaw = result.second;
                        // printf("最短路径Id号: ");
                        // for (int id : path_positive) {
                        //     printf("%d ", id);
                        // }
                        // printf("\n转弯点方向: %s\n", yawToString(turnYaw).c_str());

                        double turn_yaw = yawToDouble(turnYaw);
                        Vertex turn_vertex = getVertexById(graph_, turnId);
                            Vertex end_vertex = getVertexById(graph_, endId);

                        double extra_x = turn_vertex.x + forklift_r * cos(turn_yaw);
                        double extra_y = turn_vertex.y + forklift_r * sin(turn_yaw);

                        positive_wx_.clear();
                        positive_wy_.clear();
                        for (int vertexId : path_positive) {
                            Vertex v = getVertexById(graph_, vertexId);
                            positive_wx_.push_back(v.x);
                            positive_wy_.push_back(v.y);
                        }
                        positive_wx_.push_back(extra_x);
                        positive_wy_.push_back(extra_y);
                        
                        reverse_wx_.clear();
                        reverse_wy_.clear();
                        reverse_wx_ = {extra_x , turn_vertex.x , end_vertex.x};
                        reverse_wy_ = {extra_y , turn_vertex.y , end_vertex.y};
                        
                        current_path_index_ = 1;
                        totoal_path_index_ = 2;
                        path_tracking_status_ = PATH_TRACKING_FORWARD_PUBLISHING;

                        path_planning_status_ = PATH_PLANNING_COMPLETE;
                    }
                }else{
                    path_planning_status_ = PATH_PLANNING_FAILED;
                }
            }
           

        }else if(current_forklift_status_ == AGV_MOVING_TO_PICKUP  || current_forklift_status_ == AGV_MOVING_TO_DROPOFF
                            || current_forklift_status_ ==  AGV_MOVING_TO_PARKING){
            path_planning_status_ = PATH_PLANNING_PUB;
            // cout << "正向行驶路径点" << endl;
            // for (size_t i = 0; i < positive_wx_.size(); ++i) {
            //     std::cout << "(" << positive_wx_[i] << ", " << positive_wy_[i] << ") ";
            // }               
            // cout << endl;                
            // cout << "逆向行驶路径点" << endl;
            // for (size_t i = 0; i < reverse_wx_.size(); ++i) {
            //     std::cout << "(" << reverse_wx_[i] << ", " << reverse_wy_[i] << ")" ;
            // }
            // cout << endl;
                                
            forklift_path_planning_result_.map_model = DEPLOYMENT;
            nav_msgs::Path path_1 = produceRosPathByVector(positive_wx_ , positive_wy_);
            nav_msgs::Path path_2 = produceRosPathByVector(reverse_wx_, reverse_wy_);

            if(path_tracking_status_ == PATH_TRACKING_FORWARD_PUBLISHING){
                if(current_path_index_ == 1){
                            
                    forklift_path_planning_result_.is_reverse = false;
                    forklift_path_planning_result_.is_start_control = true;
                    forklift_path_planning_result_.current_path_index = 1;
                    forklift_path_planning_result_.total_path_index = 2;

                    path_tracking_status_ = PATH_TRACKING_FORWARD_EXECUTING;
                    ros::Duration(2).sleep(); 
                }
            }else if(path_tracking_status_ == PATH_TRACKING_FORWARD_EXECUTING){
                if (move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
                    move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::PENDING)
                {
                    ROS_WARN("Server is currently processing a task. Please wait for it to complete.");
                }
                else
                {
                    // 发送任务
                    sfy_forklift_mission_planner::SfyForkliftMoveBasePluginGoal goal;
                    goal.target_pose.pose.position.x = 5.0;
                    goal.target_pose.pose.position.y = 5.0;
                    goal.target_pose.pose.orientation.w = 1.0;

                    move_base_plugin_ac_.sendGoal(goal);

                    // 等待任务完成
                    bool finished_before_timeout = move_base_plugin_ac_.waitForResult();

                    if(move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        path_tracking_status_ = PATH_TRACKING_REVERSE_PUBLISHING;
                        current_path_index_ = 2;
                        forklift_path_planning_result_.is_start_control = false;
                        ros::Duration(2).sleep(); 
                    }

                }
            }else if(path_tracking_status_ == PATH_TRACKING_REVERSE_PUBLISHING){
                if(current_path_index_ == 2){
                    forklift_path_planning_result_.is_reverse = true;
                    forklift_path_planning_result_.is_start_control = true;
                    forklift_path_planning_result_.current_path_index = 2;
                    forklift_path_planning_result_.total_path_index = 2;

                    path_tracking_status_ = PATH_TRACKING_REVERSE_EXECUTING;
                    ros::Duration(2).sleep(); 
                }
            }
            else if(path_tracking_status_ == PATH_TRACKING_REVERSE_EXECUTING){
                if (move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
                    move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::PENDING)
                {
                    ROS_WARN("Server is currently processing a task. Please wait for it to complete.");
                }
                else
                {
                    // 发送任务
                    sfy_forklift_mission_planner::SfyForkliftMoveBasePluginGoal goal;
                    goal.target_pose.pose.position.x = 5.0;
                    goal.target_pose.pose.position.y = 5.0;
                    goal.target_pose.pose.orientation.w = 1.0;

                    move_base_plugin_ac_.sendGoal(goal);

                    // 等待任务完成
                    bool finished_before_timeout = move_base_plugin_ac_.waitForResult();

                    if(move_base_plugin_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        path_tracking_status_ = PATH_TRACKING_ALL_COMPLETE;
                        forklift_path_planning_result_.is_start_control = false;
                        ros::Duration(2).sleep(); 
                    }

                }
            }
            cout << "path_tracking_status_ : " << path_tracking_status_ << endl;
           
            if(current_path_index_ == 1){
                path_pub_.publish(path_1);
            }else{
                path_pub_.publish(path_2);
            }
            
            forklift_path_planning_result_pub_.publish(forklift_path_planning_result_);
        }else{
            path_planning_status_ = PATH_PLANNING_WAITING;
        }
    }else{
        path_planning_status_ = PATH_PLANNING_TOPIC_CLOSED;
    }
    forklift_path_planning_status_.forklift_id = forklift_status_.forklift_id;
    forklift_path_planning_status_.forklift_path_planning_status = path_planning_status_;
    forklift_path_planning_status_.forklift_path_tracking_status = path_tracking_status_;
    forklift_path_planning_status_pub_.publish(forklift_path_planning_status_);
}

void SfyForkliftPathPlanning::chooseModel(){
    // cout << "path_planning_mode_:" << path_planning_mode_ << endl;
    if(path_planning_mode_ == ARRAY_GENERATION){  // 提供数组生成路径模式（开发调试用）
        nav_msgs::Path path = produceRosPath();
        // ROS_INFO("the trajectory size is: %ld", path.poses.size() );
        path_pub_.publish(path);
    }else if(path_planning_mode_ == ALGORITHM_TESTING){  // 测试路径规划算法模式（算法验证）
        algorithmTesting();
    }else if(path_planning_mode_ == DEPLOYMENT){  // 实机部署模式（实际运行）
        deployment();
    }
}

void SfyForkliftPathPlanning::checkSubscriberStatus(){
    // 1.获取当前所有活跃话题
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    // 2. 将活跃话题名称存入哈希集合（快速查找）
    std::unordered_set<std::string> active_topics;
    for (const auto& topic_info : topic_infos) {
        active_topics.insert(topic_info.name);
        // ROS_DEBUG("Active Topic: %s (Type: %s)", topic_info.name.c_str(), topic_info.datatype.c_str());
    }
    if (active_topics.count("/forklift_target_point") > 0) {
        // ROS_INFO("[✓] %s (已发布)", "/forklift_target_point");
        is_mission_target_sub_ = true;
    } else {
        // ROS_WARN("[✗] %s (未发布)", "/forklift_target_point");
         is_mission_target_sub_ = false;
    }
    if (active_topics.count("/forklift_status") > 0) {
        is_current_forklift_status_sub_ = true;
    } else {
         is_current_forklift_status_sub_ = false;
    }
    if (active_topics.count("/odom") > 0) {
        is_odom_sub_ = true;
    } else {
        is_odom_sub_ = false;
    }
}

void SfyForkliftPathPlanning::control(){
    ros::Rate loop_rate(frequency_);
    // 等待插件启动
    move_base_plugin_ac_.waitForServer();
    while(ros::ok()){
        checkSubscriberStatus();
        chooseModel();
        pubPathPlanningStatus();   // 发布路径规划状态
        ros::spinOnce();
        loop_rate.sleep();
    }
}

SfyForkliftPathPlanning::~SfyForkliftPathPlanning(){

}

SfyForkliftPathPlanning::SfyForkliftPathPlanning()
        : move_base_plugin_ac_("sfy_forklift_move_base_plugin_action", true)
{
    ros::NodeHandle n;
    ros::NodeHandle n_prv("~");
    n_prv.param<string>("trajectory_type",trajectory_type_,"mypath");
    n_prv.param<string>("map_file_path",map_file_path_,"none");
    n_prv.param<string>("desired_path_frame_id",desired_path_frame_id_,"odom");
    n_prv.param("wx", wx_, wx_);
	n_prv.param("wy", wy_ , wy_);
    n_prv.param<double>("R", R_ , 1);
    n_prv.param<double>("frequency", frequency_ , 10);
    n_prv.param<double>("line_distance", line_distance_ , 0.01);
    n_prv.param<int>("curve_num", curve_num_ , 20);
    n_prv.param<int>("int_path_planning_mode", int_path_planning_mode_ , 0);
    path_planning_mode_ = static_cast<PathPlanningMode>(int_path_planning_mode_);

    path_pub_ = n.advertise<nav_msgs::Path>("desired_path", 500);
    forklift_path_planning_status_pub_ = n.advertise<sfy_forklift_msgs::ForkliftPathPlanningStatus>("forklift_path_planning_status", 1);
    forklift_path_planning_result_pub_ = n.advertise<sfy_forklift_msgs::ForkliftPathPlanningResult>("forklift_path_planning_result", 1);

    odom_sub_ = n.subscribe("odom", 5, 
            &SfyForkliftPathPlanning::odomCallback, this, ros::TransportHints().tcpNoDelay(true));
    forklift_status_sub_ = n.subscribe("forklift_status", 5, 
            &SfyForkliftPathPlanning::forkliftStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
    mission_target_sub_ = n.subscribe("forklift_target_point", 5, 
            &SfyForkliftPathPlanning::missionTargetPointCallback, this, ros::TransportHints().tcpNoDelay(true));
    forklift_path_tracking_status_sub_ = n.subscribe("forklift_path_tracking_status", 1, 
            &SfyForkliftPathPlanning::pathTrackingStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
    

    graph_ = loadStaticPointMap(map_file_path_);

    
}

int main(int argc, char** argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "sfy_forklift_path_planner");
    SfyForkliftPathPlanning sfy_forklift_path_planner;
    sfy_forklift_path_planner.control();
    return 0;
}
