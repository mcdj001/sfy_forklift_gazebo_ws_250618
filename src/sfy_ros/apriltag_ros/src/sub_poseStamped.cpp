#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
using namespace std; 
 
void pathCallback(const nav_msgs::Odometry::ConstPtr& odom_3d)
{
    float getX = odom_3d->pose.pose.position.x;
    float gety = odom_3d->pose.pose.position.z;
        // 提取四元数信息
    double qx =  odom_3d->pose.pose.orientation.x;
    double qy =  odom_3d->pose.pose.orientation.y;
    double qz =  odom_3d->pose.pose.orientation.z;
    double qw =  odom_3d->pose.pose.orientation.w;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_3d->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  
    cout<<"Point position:"<< endl;
    cout<<"相机的x坐标: "<<getX<<"   y坐标："<< gety<<endl;    
    cout << "相机的欧拉角roll："<< roll / M_PI * 180.0 <<"   pitch: "<<pitch / M_PI * 180.0 <<"   yaw:"<<yaw / M_PI * 180.0<<endl;
}
 
int main (int argc, char **argv)
{
    ros::init (argc, argv, "sub_poseStamped");
    ros::NodeHandle ph;
 
    ros::Subscriber odomSub = ph.subscribe<nav_msgs::Odometry>("tag_Odometry", 10, pathCallback);  
    
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
