/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-03-10 10:31:53
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-10-08 18:35:02
 * @FilePath: /sfy_forklift_robot/src/apriltag_ros/src/sub_AprilTagDetection.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf/transform_datatypes.h"
#include "iostream"
#include <Eigen/Dense>

using namespace std;

ros::Subscriber ar_sub_;

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Localizer::number_callback, this);
  }

  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
      if(msg->detections.size()>0){
          cout << "======================================================" << endl;
            float getX = msg->detections[0].pose.pose.pose.position.x;
            float gety = msg->detections[0].pose.pose.pose.position.y;
            float getz = msg->detections[0].pose.pose.pose.position.z;
             // 提取四元数信息
            double qx =  msg->detections[0].pose.pose.pose.orientation.x;
            double qy =  msg->detections[0].pose.pose.pose.orientation.y;
            double qz =  msg->detections[0].pose.pose.pose.orientation.z;
            double qw =  msg->detections[0].pose.pose.pose.orientation.w;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg->detections[0].pose.pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            short negativeNumber = 0;
            negativeNumber = getX*100;
            uint8_t byte1 = (negativeNumber ); // 获取低8位
            uint8_t byte2 = ((negativeNumber >> 8) ); // 获取高8位 
            int16_t reconstructedNumber = (int16_t)((byte2 << 8) | byte1);
            // cout<<"Point position:"<<endl;
            // cout << "size:   " << msg->detections.size() << endl;
            cout<<"相机的x坐标: "<<getX<<"   相机的y坐标："<< gety <<"    相机的z坐标："<< getz<<endl;    
            std::cout << "相机的x坐标放大100倍为： " << static_cast<int>(reconstructedNumber) << "cm" << std::endl;
            // cout << "Apri的x欧拉角roll："<< roll / M_PI * 180.0 <<"   pitch: "<<pitch / M_PI * 180.0 <<"   yaw:"<<yaw / M_PI * 180.0<<endl;

            // 假设已知的相机坐标系在世界坐标系下的四元数和xyz坐标
            // tf::Quaternion q_camera_world(0.5, 0.5, 0.5, 0.5); // 示例值，请替换为实际的四元数
            Eigen::Vector3d camera_position(getX, gety, getz);    // 示例值，请替换为实际的相机坐标

            // 将四元数转换为旋转矩阵
            tf::Matrix3x3 rot_matrix(quat);
            Eigen::Matrix3d R_camera_world;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    R_camera_world(i, j) = rot_matrix[i][j];
                }
            }

            // 找到旋转矩阵R_camera_world的列向量x和y
            Eigen::Vector3d x_axis = R_camera_world.col(0);
            Eigen::Vector3d y_axis = R_camera_world.col(1);

            // 计算投影到x0y平面上的法向量
            Eigen::Vector3d n_projected = (x_axis.cross(y_axis)).normalized();

            // 计算在x0y平面上的角度
            double angle = atan2(n_projected.y(), n_projected.x()); // 使用反正切函数计算角度

            // 将弧度转换为角度
            double angle_deg = angle * 180.0 / M_PI;

            double rolls,pitchs,yaws;
            tf::Quaternion quats;
            tf::quaternionMsgToTF(msg->detections[0].pose.pose.pose.orientation, quats);
            tf::Matrix3x3(quats).getRPY(rolls, pitchs, yaws);
			
           
            cout << "xoy平面上弧度："<< angle << endl;     
            cout << "xoy平面上角度：" << angle_deg << endl;             
            
            // cout << "其他计算方法弧度：" << yaws << endl;
            // cout << "其他计算方法角度：" << yaws * 180.0 / M_PI<< endl;
                         
        }
        else {
            // cout << "size:   " << msg->detections.size() << endl;
            cout << "没有数据"<< endl;
        }
  	 }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"sub_AprilTagDetection");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);    
    ROS_INFO("节点开始");
    ros::spin();
    return 0;
}
