#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
 
uint32_t count = 0;
uint8_t set_once = 0;
int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "goal_publisher");
 
    // 创建节点句柄
    ros::NodeHandle nh;
 
    // 创建一个Publisher，用于发布目标点消息到/move_base_simple/goal话题
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    geometry_msgs::PoseStamped goal;
    goal.header.seq = 0;
    goal.header.stamp = ros::Time();  // 设置当前时间为时间戳
	goal.header.frame_id = "robot_foot_init";
	goal.pose.position.x = 0;
	goal.pose.position.y = 0;
	goal.pose.position.z = 0;
	goal.pose.orientation.x = 0;
	goal.pose.orientation.y = 0;
	goal.pose.orientation.z = 0;
	goal.pose.orientation.w = 0;
    goal_pub.publish(goal);
    ros::Duration(0.1).sleep();//bug 初始化时发布一次延时一会后 后面发布的才能正常
    
    // ros::Rate rate(1);  // 设置为1Hz，即每秒发布一次目标点
    while (ros::ok()) {
        // 创建一个PoseStamped消息
        
        // goal.header.stamp = ros::Time::now();  // 设置时间戳为当前时间
        if(set_once == 0)
        {
            // 设置目标点的位置和姿态
            goal.header.frame_id = "robot_foot_init";  // 设置坐标系为map
            goal.header.stamp = ros::Time::now();  // 设置时间戳为当前时间
            goal.pose.position.x = 1.0;
            goal.pose.position.y = 3.0;
            goal.pose.position.z = 0.0;  // 设置为地面
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.021647965630357723;
            goal.pose.orientation.w = 0.9997656553333221;
 
            // 发布目标点消息
            goal_pub.publish(goal);
            set_once = 1;
        }
        // rate.sleep();
        ros::spinOnce();
    }
    return 0;
}