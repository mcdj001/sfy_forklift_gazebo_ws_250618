/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-15 10:41:56
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-03-04 11:20:07
 * @FilePath: /sfy_forklift_agv_ws/src/turn_on_sfy_robot/include/sfy_robot.h
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#ifndef __SFY_ROBOT_H_
#define __SFY_ROBOT_H_

#include "ros/ros.h"
#include <string.h> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h> 
#include <termios.h> 
#include <stdio.h>  
#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

#include "apriltag_ros/AprilTagDetectionArray.h"
#include <Eigen/Dense>

#include <serial/serial.h>
#include "sfy_forklift_msgs/ForkliftInfo.h"
#include "sfy_forklift_msgs/ForkliftInitInfo.h"
#include "sfy_forklift_msgs/ForkliftInitImu.h"
#include "sfy_forklift_msgs/ForkliftCmdVel.h"
#include "sfy_forklift_msgs/ForkliftApriltagInfo.h"


// #include <boost/asio.hpp>
using namespace std;

#define SEND_DATA_CHECK   1           //发送数据校验标志位
#define READ_DATA_CHECK   0          //接收数据校验标志位
#define FRAME_HEADER      0x7B       // Frame head //帧头
#define FRAME_TAIL        0x7D             // Frame tail //帧尾
#define IMU_FRAME_HEADER 	0x55  // IMU帧头
#define  IMU_FRAME_TAIL        0x7D
#define RECEIVE_DATA_SIZE 20         	// 下位机发送过来的数据的长度
#define SEND_DATA_SIZE    17          	 // ROS向下位机发送的数据的长度
#define PI 				3.1415926f  			 	//圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	


//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//串口相关对象
// boost::system::error_code err;
// boost::asio::io_service iosev;
// boost::asio::serial_port sp(iosev);

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __JY901S_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 
	short mag_x_data;
	short mag_y_data;
	short mag_z_data;
	short orientation_x_data;
	short orientation_y_data;
	short orientation_z_data;
}JY901S_DATAD;

//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_position;	       
	float Y_position;           
	float Z_position;         
	unsigned char Frame_Tail;
}SEND_DATA;

//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;  
	float Y_speed;  
	float Z_speed;  
	float Power_Voltage;	
	unsigned char Frame_Tail;
}RECEIVE_DATA;

// 叉车的模式选择  与单片机有关
typedef enum {
	AGV_USE_ROS_MODEL = 1,
	AGV_ELECTROMAGNETIC_MODEL,
	AGV_PARK_MODEL,
    AGV_STOP_MODEL,
}AGV_MODEL;

// 自定义定时器模式选择
typedef enum {
	FIRST_USE_APRILTAG=1,				  //  只在第一次使用二维码定位
	ALWAYS_USE_APRILTAG,				 //  只要识别到二维码就替换掉里程计定位
	TIMER_LINE_TEST,					     	//  定时任务   定时X秒后车辆停止  走直线
}DEBUG_MODEL;

typedef struct _FORKLIFT_FLAGS_{
	bool  mission_start_location;
	bool current_has_apriltag;
	bool only_set_time;
	bool only_time_over;
}FORKLIFT_FLAGS;

typedef struct _LAST_TIME_RECORD_{
	ros::Time	last_apriltag_time;				// 上一次apriltag采集到的时间
	ros::Time	init_location_time;				//  初始化定位的时间
	ros::Time	last_only_set_time;			 //  设置5s的定时
}LAST_TIME_RECORD;

class turn_on_sfy_robot{
    public:
        turn_on_sfy_robot();
        ~turn_on_sfy_robot();
        void Control();
		serial::Serial Stm32_Serial;  //Declare a serial object //声明串口对象
		
    private:
		ros::NodeHandle n;           //创建ROS节点句柄
		ros::Time _Now, _Last_Time;  
		float Sampling_Time;         
		ros::Time _Apriltag_Now, _Apriltag_Last_Time;  		//时间相关，用于积分求位移(里程)
		float Apriltag_Sampling_Time;         								//采样时间，用于积分求位移(里程)
		ros::Time _My_Timer_Now, _My_Timer_Last_Time;  
		float My_Timer_Sampling_Time;        							 //采样时间，用于积分求位移(里程)
		double frequency_; 																 //频率
		ros::Timer my_timer_;														 // 阿源的自定义定时器  20HZ frequency_
		void My_Timer(const ros::TimerEvent&);
		tf::TransformBroadcaster odom_broadcaster;      // 发布odom到basefoot的TF坐标变换
		int  debug_model_;															 //  选择定时器的debug模式
		bool debug_apriltag_model_; 									//是否调试AprilTag二维码
		bool use_apriltag_change_imu_yaw_;        			  // 是否使用二维码修正Imu
		bool use_imu_yaw_to_odom_;									// 使用Imu的yaw去odom

        ros::Subscriber Apriltag_Positon_Sub;   //初始化话题订阅者
		void Apriltag_Positon_Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr&  msg); //坐标话题订阅回调函数

		ros::Subscriber Cmd_Vel_Sub; //Initialize the topic subscriber //初始化话题订阅者
		void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);   //速度话题订阅回调函数
		bool is_receive_cmd_vel_  = false;
		ros::Time last_cmd_vel_time_ = ros::Time::now();  // 上一次接收到 cmd_vel 的时间
		geometry_msgs::Twist cmd_vel_;

	    ros::Publisher odom_publisher;  //  voltage_publisher
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题

		ros::Publisher forklift_init_info_publisher;
		void Publish_Forklift_Init_Info();    //发布AGV叉车初始化定位信息
		sfy_forklift_msgs::ForkliftInitInfo forklift_init_info_;

		ros::Publisher forklift_apriltag_info_publisher;
		void Publish_Forklift_Apriltag_Info();      // 发布换算后的Apriltag信息
		sfy_forklift_msgs::ForkliftApriltagInfo forklift_apriltag_info_;

		ros::Publisher forklift_info_publisher;
		void Publish_Forklift_Info();    //发布AGV叉车信息话题
		sfy_forklift_msgs::ForkliftInfo forklift_info_;
		int forklift_id_;

		ros::Subscriber  forklift_init_imu_sub_;    // 接收imu初始化成功和Imu的姿态
		void forkliftInitImuCB(const sfy_forklift_msgs::ForkliftInitImu::ConstPtr&  Msg);      // imu_inti话题的回调函数
		sfy_forklift_msgs::ForkliftInitImu forklift_init_imu_;

		ros::Subscriber  forklift_cmd_vel_sub_; 
		void forkliftCmdVelCB(const sfy_forklift_msgs::ForkliftCmdVel::ConstPtr&  Msg); 
		sfy_forklift_msgs::ForkliftCmdVel forklift_cmd_vel_;

		unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
		bool Get_Sensor_Data_New();

		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

		string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables //定义相关变量
		int serial_baud_rate , imu_serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

		Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
		Vel_Pos_Data Current_Apriltag_Pos;
		Vel_Pos_Data Init_Robot_Pos;      // 初始化定位时叉车的位置
		
        float Power_voltage;       //Power supply voltage //电源电压
		uint16_t forklift_fcode;
        float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative;    //里程计修正参数
		float camera_to_base_footprint_x , camera_to_base_footprint_y; // 相机

		bool isSendToSTM32 = false;  //Whether to send data to the STM32 //是否发送数据到STM32
		ros::Subscriber move_base_goal_sub_;
    	bool move_base_simple_goal_bool_ = false;            // 订阅 move_base_simple/goal 
		void moveBaseGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
		void SendToSTM32(int model);

		FORKLIFT_FLAGS forklift_flags;
		LAST_TIME_RECORD last_time_records;
		void Init_Last_Time_Record();      //初始化时间

};

#endif