/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-05 14:10:14
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-23 12:53:36
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_motion_controller/src/lqr_dynamic_node.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */

#include "lqr_dynamic_node.h"

vehicleState LQR_Dynamic_Node::update_state(U control, vehicleState forklift) {
	forklift.v = control.v;
	forklift.kesi = control.kesi;
	forklift.x += forklift.v * cos(forklift.yaw) * T;
	forklift.y += forklift.v * sin(forklift.yaw) * T;
	forklift.yaw += forklift.v / vehicle_dynamic_param.L * tan(forklift.kesi) * T;
	forklift.yaw = mytools ->normalizeAngle(forklift.yaw);
	return forklift;
}

vehicleState LQR_Dynamic_Node::delay_odom_postion(){
    vehicleState forklift_delay;
    const vehicleState forklift_odom_const = forklift_odom;
    if(delay_model){
        forklift_delay.x = forklift_odom_const.v * cos(forklift_odom_const.yaw) * T;
        forklift_delay.y = forklift_odom_const.v * sin(forklift_odom_const.yaw) * T;
        forklift_delay.yaw = forklift_odom_const.yaw +  forklift_odom_const.kesi * T;
        forklift_delay.yaw = mytools->normalizeAngle(forklift_odom_const.yaw);
        forklift_delay.v = forklift_odom_const.v;
        // 注意 这里是角速度   不是rad
        forklift_delay.kesi = forklift_odom_const.kesi;
    }else{
        forklift_delay.x = forklift_odom_const.x ;
        forklift_delay.y = forklift_odom_const.y ;
        forklift_delay.yaw = mytools->normalizeAngle(forklift_odom_const.yaw);
        forklift_delay.v = forklift_odom_const.v;
        // 注意 这里是角速度   不是rad
        forklift_delay.kesi = forklift_odom_const.kesi;
    }
    return forklift_delay;
}

U LQR_Dynamic_Node::v_and_kesi_limit(U control_value){
    if(control_value.v >= v_max)//速度限幅
    {
        control_value.v = v_max;
        ROS_WARN("The calculated value may be inaccurate ");
    }
    else if(control_value.v<=-v_max){
        control_value.v = -v_max;
        ROS_WARN("The calculated value may be inaccurate ");
    }
        
    if(control_value.kesi>= (max_steer_state * M_PI / 180) )  //前轮转角限幅
    {
        control_value.kesi = max_steer_state * M_PI / 180;
        ROS_WARN("The calculated value may be inaccurate ");
    }
    else if(control_value.kesi<=  (- max_steer_state * M_PI / 180)){
        control_value.kesi = - max_steer_state * M_PI / 180 ;
        ROS_WARN("The calculated value may be inaccurate ");
    }
    return control_value;
}

void LQR_Dynamic_Node::Marker_set() {
    //设置消息类型参数
    visual_state_trajectory.header.frame_id = "odom";
    visual_state_trajectory.header.stamp = ros::Time::now();
    visual_state_trajectory.action = visualization_msgs::Marker::ADD;
    visual_state_trajectory.ns = "LQR";
    //设置点的属性
    visual_state_trajectory.id = 0;
    visual_state_trajectory.type = visualization_msgs::Marker::POINTS;
    visual_state_trajectory.scale.x = 0.02;
    visual_state_trajectory.scale.y = 0.02;
    visual_state_trajectory.color.r = 1.0;
    visual_state_trajectory.color.a = 1.0;
}

void LQR_Dynamic_Node::shutdown_controller(){
    if(action == "the car has reached the goal!"){
        temp+=1;
        if(temp ==50){
            ROS_WARN("shutdown the LQR Dynamic controller!");
            temp = 0;
            ros::shutdown();
        }
    }
}

void LQR_Dynamic_Node::PUB(){
    visual_state_pose.x = forklift.x; visual_state_pose.y = forklift.y;
    actual_pose.x = forklift.x; actual_pose.y = forklift.y; actual_pose.theta = forklift.yaw;
    vel_msg.linear.x = control.v; 
    vel_msg.angular.z = control.v*tan(control.kesi)/vehicle_dynamic_param.L; //横摆角速度为w = v*tan(kesi)/L
    visual_state_trajectory.points.push_back(visual_state_pose);//visualization_msgs::Marker为一个容器，所以现需要向里面push_back结构体，再发布
    visual_state_pub.publish(visual_state_trajectory);//发布虚拟轨迹
    vel_pub.publish(vel_msg);//发布速度
    actual_state_pub.publish(actual_pose);//发布位姿
}

float LQR_Dynamic_Node::z_vel_pid_controller(float pv,  float sp)
{
    float Kp = steer_vel_delay_pid_kp , Ki = steer_vel_delay_pid_ki ,Kd = steer_vel_delay_pid_kd; 
    // 角速度最大输出   rad
    float max_output_pid = 10 , min_output_pid = -10 ;
    // 误差最大不超过
    float max_output_i = 5,min_output_i = -5 ;
    static float error = 0,error_last=0,error_last_last=0;
    static float output_p,output_i,output_d,output_pid; 
    // 误差 = 期望角速度  -  odom当前角速度
    error = sp - pv ;

    // 控制器各环节 输出 计算
    output_p += ( Kp * (error - error_last) );
    output_i += ( Ki * (error) );
    output_d += ( Kd * (error - 2*error_last + error_last_last) );

    // 更新偏差量
    error_last_last = error_last ;
    error_last = error ;
    if(output_i>max_output_i)
    {
        output_i = max_output_i;
    }else if(output_i<min_output_i)
    {
        output_i = min_output_i;
    }        
    output_pid = output_p + output_i + output_d;
    if(output_pid>max_output_pid)
    {
        output_pid = max_output_pid;
    }else if(output_pid<min_output_pid)
    {
        output_pid = min_output_pid;
    }
    return output_pid; 
}

//控制启停函数
void LQR_Dynamic_Node::node_control() {

    ros::Rate loop_rate(freq);

    Marker_set();//设置Marker属性
    //设置tf坐标转换
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    start_time = std::chrono::steady_clock::now();

    while (ros::ok()) {
        // transform.setOrigin(tf::Vector3(forklift.x, forklift.y, 0));
        // q.setRPY(0, 0, forklift.yaw);
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "forklift"));

        ros::spinOnce();
        LQR_track();
        loop_rate.sleep();
    }
}

//控制器流程
void LQR_Dynamic_Node::LQR_track() {
    U U_r;
    waypoint Point , Point_Next;
    //ROS_INFO("path size is: %d",path->Size());
    if(path->Size() !=0 ){
        // 是否开启延迟模式
        const vehicleState forklift_const = delay_odom_postion();
        //搜索路径点
        int target_index = path->Find_target_index(forklift_const);
        ROS_INFO("期望路径点下标为 : %d", target_index);

        //获取路径点信息，构造期望控制量
        Point = path->Get_waypoint(target_index); //获取x,y
        Point_Next = path->Get_waypoint(target_index + 1);

        ROS_INFO("叉车当前期望路径点为x:%f,y:%f,yaw:%f", Point.x, Point.y, Point.yaw);
        double K = mytools ->cal_K(path->Get_waypoints() , target_index); //计算曲率
        cout << "期望目标路径点曲率为  "  << K << endl;
        // if(isnan(K))  K = 0;
        vehicle_dynamic_param.target_index_K = K;

        //减速判断
        const double kesi = atan2(vehicle_dynamic_param.L * K, 1);
        const double v_distance = abs(sqrt(pow(forklift_const.x - lastPoint.x, 2) + pow(forklift_const.y - lastPoint.y, 2)));
        //ROS_INFO("the distance is %f\n", v_distance);
        U_r.v = slow_judge(v_distance , target_index , path->Size());  U_r.kesi = kesi;
        ROS_INFO("%s", action.c_str());  //机器人动作
        // ROS_INFO("the desired v is: %f,the desired kesi is: %f", U_r.v,U_r.kesi);

        //设置权重矩阵
        for(int q =0;q < Q_set.size(); q++) Q[q] = Q_set[q];
        for(int r =0;r< R_set.size();r++) R[r] = R_set[r];
        if(Q[0]>=R[0])
            ROS_WARN("Q >= R, the calculation result may be abnormal,please set R < Q ");

        //使用LQR控制器  进行横向控制
        controller->initial(vehicle_dynamic_param, T, forklift_const, Point, Point_Next , U_r, Q, R);//初始化控制器
        control = controller->cal_vel();  //计算控制量为前轮转角[kesi]

        // 动力学方程先只控制前轮转角   速度恒定
        control.v =  slow_judge(v_distance , target_index , path->Size()); 
        if(U_r.v==0)  control.v = 0; //判断，期望速度为0，则机器人停下
        if(limit_v_and_kesi)   control = v_and_kesi_limit(control); //速度和前轮转角限幅
        ROS_INFO("控制速度为: %f,   控制前轮转角为: %f rad", control.v, control.kesi);
        ROS_INFO("叉车当前位置x: %f, y: %f,yaw:%f", forklift_const.x, forklift_const.y,forklift_const.yaw);

        double pid_steer_vel;
        if(steer_vel_delay_pid_model){
            const double expect_steer_vel =  control.v*tan(control.kesi)/vehicle_dynamic_param.L;
            pid_steer_vel = z_vel_pid_controller(forklift_const.kesi , expect_steer_vel);
            ROS_INFO(" odom角速度: %f, 期望角速度: %f, Pid后角速度:%f", forklift_const.kesi, expect_steer_vel, pid_steer_vel);
        }else{
            pid_steer_vel =  control.v*tan(control.kesi)/vehicle_dynamic_param.L;
        }

        //话题发布
        // PUB();
        visual_state_pose.x = forklift_const.x; visual_state_pose.y = forklift_const.y;
        actual_pose.x = forklift_const.x; actual_pose.y = forklift_const.y; actual_pose.theta = forklift_const.yaw;
        vel_msg.linear.x = control.v; 
        vel_msg.angular.z = pid_steer_vel; //横摆角速度为w = v*tan(kesi)/L
        visual_state_trajectory.points.push_back(visual_state_pose);//visualization_msgs::Marker为一个容器，所以现需要向里面push_back结构体，再发布
        visual_state_pub.publish(visual_state_trajectory);  //发布虚拟轨迹
        vel_pub.publish(vel_msg);//发布速度
        actual_state_pub.publish(actual_pose);//发布位姿


        //  数据收集处理  
        double ditance_error = sqrt(pow(forklift_const.x - Point.x, 2) + pow(forklift_const.y - Point.y, 2));
        current_time = std::chrono::steady_clock::now();
        std::chrono::milliseconds use_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        double x_error = forklift_const.x - Point.x;
        double y_error = forklift_const.y - Point.y;
        vector<double> get_all_error = controller->get_all_error();

        // file << use_time.count() / 1000.0 << "," << forklift.x << "," << forklift.y<< "," <<  Point.x << "," 
        //                     << Point.y << "," <<  vel_msg.linear.x  <<  "," << vel_msg.angular.z <<  ","
        //                     << control.kesi  * 180 / M_PI << "," <<  ditance_error << ","<< x_error << ","<< y_error << ","
        //                     << get_all_error[0]  << "," <<   get_all_error[1] << "," <<  get_all_error[2] << "," <<  get_all_error[3] <<"\n" ;
        
        //小车位姿状态更新
        // forklift = update_state(control, forklift);

        //控制器关闭判断
        shutdown_controller();

        ROS_INFO("-------------------------------------------");
    }
}

double LQR_Dynamic_Node::slow_judge(double distance , int cur_index , int all_index) {
    if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE  && cur_index  > (all_index / 2)) {
        return slow_LEVE1_V;
    }
    else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE  && cur_index  > (all_index / 2)) {
        return slow_LEVE2_V;
    }
    else if (distance < goal_tolerance_DISTANCE && cur_index  > (all_index / 2)) {
        action = "the car has reached the goal!";
        return 0.0;
    }
    else
    {
        return v_expect;
    }
}

void LQR_Dynamic_Node::addpointcallback(const nav_msgs::Path::ConstPtr& msg){
    vector<waypoint> waypoints;
    for(int i=0;i<msg->poses.size();i++){
        waypoint waypoint;
        //ROS_INFO("THE PATH[%d]'s ID is %d",i,msg->poses[i].header.seq);
        waypoint.ID = msg->poses[i].header.seq;
        waypoint.x = msg->poses[i].pose.position.x;
        waypoint.y = msg->poses[i].pose.position.y;
        // 获取角度
        double roll,pitch,yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        waypoint.yaw = yaw;
        waypoints.push_back(waypoint);
    }
    path->Add_new_point(waypoints);//路径点vector数组传到path类中
    lastIndex = path->Size() - 1;
    lastPoint = path->Get_waypoint(lastIndex);
}

void LQR_Dynamic_Node::odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    forklift_odom.x = odom -> pose.pose.position.x;
    forklift_odom.y = odom -> pose.pose.position.y;
     double roll,pitch,yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom -> pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    forklift_odom.yaw = mytools->normalizeAngle(yaw);
    forklift_odom.v = odom ->twist.twist.linear.x;
    forklift_odom.kesi = odom ->twist.twist.angular.z;
}

LQR_Dynamic_Node::~LQR_Dynamic_Node() {
            delete(controller);
            delete(path);
}

LQR_Dynamic_Node::LQR_Dynamic_Node(ros::NodeHandle& nh)//初始化中添加轨迹、小车初始位姿
{
    controller = new LQR_Dynamic_Control();
    path = new MyPath();
    
    //ROS:
    path_sub = nh.subscribe("desired_path", 10, &LQR_Dynamic_Node::addpointcallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose",10);
    actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("LQR_pose",10);

    odomSub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &LQR_Dynamic_Node::odomCallback, this);

    //robot state initialize:
    action = "the car is tracking!!";
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
	ros::init(argc, argv, "LQR_Dynamic_Node");
	ros::NodeHandle n;
	ros::NodeHandle n_prv("~");

	n_prv.param<double>("freq",freq,20);
	n_prv.param<double>("L", vehicle_dynamic_param.L, 1.19);
    n_prv.param<double>("max_steer_state",max_steer_state , 30);
    n_prv.param<double>("mass", vehicle_dynamic_param.mass, 2080);
    n_prv.param<double>("mass_front", vehicle_dynamic_param.mass_front, 1040);
    n_prv.param<double>("mass_rear", vehicle_dynamic_param.mass_rear, 1040);
    n_prv.param<double>("cf", vehicle_dynamic_param.cf, 155494.663);
    n_prv.param<double>("cr", vehicle_dynamic_param.cr, 155494.663);
    n_prv.param<int>("horizon", vehicle_dynamic_param.horizon, 100);
	n_prv.param<double>("v_expect",v_expect,0.5);
	n_prv.param<double>("v_max",v_max,5.0);
	n_prv.param<double>("slow_LEVE1_DISTANCE",slow_LEVE1_DISTANCE,5.0);
	n_prv.param<double>("slow_LEVE2_DISTANCE",slow_LEVE2_DISTANCE,2.0);
	n_prv.param<double>("goal_tolerance_DISTANCE",goal_tolerance_DISTANCE,0.1);
	n_prv.param<double>("slow_LEVE1_V",slow_LEVE1_V,0.35);
	n_prv.param<double>("slow_LEVE2_V",slow_LEVE2_V,0.15);
	n_prv.param<bool>("limit_v_and_kesi",limit_v_and_kesi,true);
    n_prv.param<bool>("delay_model",delay_model, false);
    n_prv.param<bool>("steer_vel_delay_pid_model", steer_vel_delay_pid_model , true);
    n_prv.param<double>("steer_vel_delay_pid_kp",steer_vel_delay_pid_kp, 6);
    n_prv.param<double>("steer_vel_delay_pid_ki",steer_vel_delay_pid_ki, 0.01);
    n_prv.param<double>("steer_vel_delay_pid_kd",steer_vel_delay_pid_kd, 3);
	n_prv.param("Q_set",Q_set, Q_set);
	n_prv.param("R_set", R_set, R_set);
    vehicle_dynamic_param.lf = vehicle_dynamic_param.L * (1.0 - vehicle_dynamic_param.mass_front / vehicle_dynamic_param.mass);
	vehicle_dynamic_param.lr = vehicle_dynamic_param.L * (1.0 - vehicle_dynamic_param.mass_rear / vehicle_dynamic_param.mass);
	vehicle_dynamic_param.iz = vehicle_dynamic_param.lf *vehicle_dynamic_param.lf * vehicle_dynamic_param.mass_front + vehicle_dynamic_param.lr * vehicle_dynamic_param.lr * vehicle_dynamic_param.mass_rear; 

	// file.open("/home/jetsonbysfy/ros_code/sfy_forklift_robot/src/path_track/assets/lqr_dynamic.csv" , std::ios::trunc);
	// file << "time" << "," << "forklift_x" << "," << "forklift_y"<<"," << "path_x" << "," << "path_y" << "," 
	// 										<< "to32_x_v" << "," << "to32_z_v" << ","
	// 										 <<"kesi" << ","<< "ditance_error" << ","<< "x_error"<< ","<<  "y_error" << ","
	// 										 << "lateral_error" << "," << "lateral_error_rate" << "," << "heading_error" << "," <<  "heading_error_rate" << "\n" ;


	LQR_Dynamic_Node* node = new LQR_Dynamic_Node(n);
	node->node_control();
	// file.close();
	return (0);
}
