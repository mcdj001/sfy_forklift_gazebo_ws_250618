/*
 * @Description: 
 * @Version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-07-15 10:41:56
 * @LastEditors: clint-sfy 2786435349@qq.com
 * @LastEditTime: 2025-04-15 12:44:25
 * @FilePath: /sfy_forklift_agv_ws/src/turn_on_sfy_robot/src/sfy_robot.cpp
 * Copyright (c) 2024, GPL-3.0 license, Fengyuan Shen, All Rights Reserved.
 */
#include "sfy_robot.h"

void turn_on_sfy_robot::My_Timer(const ros::TimerEvent&)
{
   _My_Timer_Now = ros::Time::now();
  if(_My_Timer_Last_Time.toSec()==0 ) _My_Timer_Last_Time=_My_Timer_Now;
  My_Timer_Sampling_Time = (_My_Timer_Now - _My_Timer_Last_Time).toSec();

  if(last_time_records.init_location_time.toSec()==0)  last_time_records.init_location_time = ros::Time::now();

  // 里程计误差修正
  Robot_Vel.X = Robot_Vel.X * odom_x_scale;
  Robot_Vel.Y = Robot_Vel.Y * odom_y_scale;
  if(Robot_Vel.Z >= 0)
    Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_positive;
  else
    Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_negative;
  // cout << "当前debug模式：" << debug_model_ << endl;
  // cout << "robot_pose_x: " << Robot_Pos.X << endl;

  // DEBUG模式1    在导航过程一直使用二维码    只要出现二维码就替换掉里程计的定位
  if(debug_model_ == ALWAYS_USE_APRILTAG){
    // 初始定位不成功   叉车速度为0   当前相机识别到二维码且二维码位姿稳定 且要识别二维码4秒
    if(forklift_flags.mission_start_location == false){
      if(Robot_Vel.X == 0  &&  forklift_flags.current_has_apriltag && ((ros::Time::now() - last_time_records.init_location_time).toSec() > 4)){
        Init_Robot_Pos.X  = Current_Apriltag_Pos.X + camera_to_base_footprint_x * cos( Current_Apriltag_Pos.Z) - camera_to_base_footprint_y * sin( Current_Apriltag_Pos.Z);
        Init_Robot_Pos.Y  = Current_Apriltag_Pos.Y + camera_to_base_footprint_x * sin( Current_Apriltag_Pos.Z) +  camera_to_base_footprint_y * cos( Current_Apriltag_Pos.Z);
        Init_Robot_Pos.Z  = Current_Apriltag_Pos.Z;
        Robot_Pos = Init_Robot_Pos;

        // 定位成功
        last_time_records.init_location_time = ros::Time::now();
        forklift_flags.mission_start_location = true;
      }else{
        SendToSTM32(0);
      }
    }

    //初始化定位成功
    if(forklift_flags.mission_start_location  && forklift_init_imu_.init_imu_yaw_flag){
      if(forklift_flags.current_has_apriltag){
        Robot_Pos.X  =  Current_Apriltag_Pos.X + camera_to_base_footprint_x * cos( Current_Apriltag_Pos.Z) - camera_to_base_footprint_y * sin( Current_Apriltag_Pos.Z);
        Robot_Pos.Y   =  Current_Apriltag_Pos.Y + camera_to_base_footprint_x * sin( Current_Apriltag_Pos.Z) +  camera_to_base_footprint_y * cos( Current_Apriltag_Pos.Z);
        Robot_Pos.Z  =  Current_Apriltag_Pos.Z;
      }else{
        Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算X方向的位移，单位：m
        Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算Y方向的位移，单位：m
        if(use_imu_yaw_to_odom_){
          Robot_Pos.Z = forklift_init_imu_.imu_actual_yaw;
        }else{
          Robot_Pos.Z += Robot_Vel.Z * My_Timer_Sampling_Time;   //   绕Z轴的角位移，单位：rad
        }       
      } // 是否有二维码

      cout << "robot_x: " << Robot_Pos.X << "       robot_y: " << Robot_Pos.Y  << "     robot_z:  " << Robot_Pos.Z << endl;
      if ((ros::Time::now() - last_cmd_vel_time_).toSec() > 0.3)
      {
        SendToSTM32(0);
      }
      else
      {
        SendToSTM32(1);
      }
      
    }else{
      SendToSTM32(0);
    }

  }  // DEBUG模式
  else if(debug_model_ == FIRST_USE_APRILTAG){
    // 初始定位不成功   叉车速度为0   当前相机识别到二维码且二维码位姿稳定 且要识别二维码三秒
    if(forklift_flags.mission_start_location == false){
      if(Robot_Vel.X == 0  &&  forklift_flags.current_has_apriltag && ((ros::Time::now() - last_time_records.init_location_time).toSec() > 4)){
        Init_Robot_Pos.X  = Current_Apriltag_Pos.X + camera_to_base_footprint_x * cos( Current_Apriltag_Pos.Z) - camera_to_base_footprint_y * sin( Current_Apriltag_Pos.Z);
        Init_Robot_Pos.Y  = Current_Apriltag_Pos.Y + camera_to_base_footprint_x * sin( Current_Apriltag_Pos.Z) +  camera_to_base_footprint_y * cos( Current_Apriltag_Pos.Z);
        Init_Robot_Pos.Z  = Current_Apriltag_Pos.Z;
        Robot_Pos = Init_Robot_Pos;
        // 定位成功
        last_time_records.init_location_time = ros::Time::now();
        forklift_flags.mission_start_location = true;
      }else{
        SendToSTM32(0);
      }
    }
    // 上次识别到了二维码  且  过去时间大于6s 且当前相机有二维码
    // if(forklift_flags.mission_start_location && 
    //     ((ros::Time::now() - last_time_records.init_location_time).toSec() > 6)  &&   forklift_flags.current_has_apriltag){
    //     forklift_flags.mission_start_location = false;
    // }
    //初始化定位成功
    if(forklift_flags.mission_start_location && forklift_init_imu_.init_imu_yaw_flag){
      // 里程计数据更新
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算X方向的位移，单位：m
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算Y方向的位移，单位：m
      Robot_Pos.Z += Robot_Vel.Z * My_Timer_Sampling_Time;   //   绕Z轴的角位移，单位：rad

      if ((ros::Time::now() - last_cmd_vel_time_).toSec() > 0.3)
      {
        SendToSTM32(0);
      }
      else
      {
        SendToSTM32(1);
      }
    }else{
      SendToSTM32(0);
    }
  } // DEBUG模式    TIMER_LINE_TEST
  else if(debug_model_ == TIMER_LINE_TEST){
    Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算X方向的位移，单位：m
    Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * My_Timer_Sampling_Time; //计算Y方向的位移，单位：m
    Robot_Pos.Z += Robot_Vel.Z * My_Timer_Sampling_Time;   //   绕Z轴的角位移，单位：rad

    if ((ros::Time::now() - last_time_records.last_only_set_time).toSec() > 5  && forklift_flags.only_set_time == true){
      forklift_flags.only_time_over = true;
    }
  
    // 当cmd_vel话题超时 发送默认数据  否则根据话题发送数据
    if ((ros::Time::now() - last_cmd_vel_time_).toSec() > 0.3 || forklift_flags.only_time_over == true)
    {
      SendToSTM32(0);
      forklift_flags.only_set_time = false;
      last_time_records.last_only_set_time = ros::Time::now();
    }
    else
    {
      SendToSTM32(1);
      forklift_flags.only_set_time = true;
    }
  }  // DEBUG模式    TIMER_LINE_TEST


  _My_Timer_Last_Time = _My_Timer_Now;
}

void turn_on_sfy_robot::forkliftCmdVelCB(const sfy_forklift_msgs::ForkliftCmdVel::ConstPtr&  Msg)
{
  forklift_cmd_vel_ = *Msg;
}


void turn_on_sfy_robot::forkliftInitImuCB(const sfy_forklift_msgs::ForkliftInitImu::ConstPtr&  Msg)
{
  forklift_init_imu_ = *Msg;
}
    
void turn_on_sfy_robot::moveBaseGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
  move_base_simple_goal_bool_ = true;
}

/**************************************
Date: July 16, 2024
Function: Data conversion function
功能: 数据转换函数
***************************************/
float turn_on_sfy_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

/**************************************
Date: January 4, 2025
Function: Publish the  Forkift Init Location info
功能: 发布叉车在初始化定位的信息
***************************************/
void turn_on_sfy_robot::Publish_Forklift_Init_Info()
{
  std_msgs::Time temp_time;
  temp_time.data = last_time_records.init_location_time;
  
  forklift_init_info_.init_loation_flag = forklift_flags.mission_start_location;
  forklift_init_info_.init_location_time = temp_time;
  forklift_init_info_.x = Init_Robot_Pos.X;
  forklift_init_info_.y = Init_Robot_Pos.Y;
  forklift_init_info_.yaw = Init_Robot_Pos.Z;

  forklift_init_info_publisher.publish(forklift_init_info_);
}

void turn_on_sfy_robot::Publish_Forklift_Apriltag_Info()
{
  forklift_apriltag_info_.forklift_id = forklift_id_ ;
  forklift_apriltag_info_.current_has_apriltag= forklift_flags.current_has_apriltag;
  forklift_apriltag_info_.use_apriltag_change_imu_yaw = use_apriltag_change_imu_yaw_;
  forklift_apriltag_info_.apriltag_x = Current_Apriltag_Pos.X;
  forklift_apriltag_info_.apriltag_y = Current_Apriltag_Pos.Y;
  forklift_apriltag_info_.apriltag_yaw = Current_Apriltag_Pos.Z;
  forklift_apriltag_info_.forklift_x = Robot_Pos.X;
  forklift_apriltag_info_.forklift_y = Robot_Pos.Y;

  forklift_apriltag_info_publisher.publish(forklift_apriltag_info_);
}


/**************************************
Date: July 16, 2024
Function: Publish the  STM32 info
功能: 发布叉车STM32的信息  包括FCODE  电池电压  叉车当前前轮倾角
***************************************/
void turn_on_sfy_robot::Publish_Forklift_Info()
{
  forklift_info_.forklift_id = forklift_id_;
  forklift_info_.forklift_fcode = forklift_fcode;
  forklift_info_.forklift_voltage = 100;
  forklift_info_.forklift_steer = 0;

  forklift_info_publisher.publish(forklift_info_);
}

/**************************************
Date: July 16, 2024
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_sfy_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

    nav_msgs::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //发布坐标变换  odom->base_footprint
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now(); 
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id  = robot_frame_id;
    odom_trans.transform.translation.x = Robot_Pos.X;
    odom_trans.transform.translation.y = Robot_Pos.Y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
            
    odom_broadcaster.sendTransform(odom_trans);

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X == 0 && Robot_Vel.Y ==0 && Robot_Vel.Z ==0){
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    }else{
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));      
    }
    odom_publisher.publish(odom); //Pub odometer topic //发布里程计话题
}

void turn_on_sfy_robot::SendToSTM32(int model)
{
    short transition;  //intermediate variable //中间变量
    float temp_rad;
    if(model == 0){
      Send_Data.tx[0]=FRAME_HEADER;
      Send_Data.tx[1] = 0;  
      Send_Data.tx[2] = 0; 
      Send_Data.tx[3] = AGV_USE_ROS_MODEL;  
      Send_Data.tx[4] = 0;     
      
      Send_Data.tx[6] = 0;
      Send_Data.tx[5] = 0;  

      Send_Data.tx[8] = 0;  
      Send_Data.tx[7] = 0;    

      Send_Data.tx[10] = 0;  
      Send_Data.tx[9] = 0; 

      Send_Data.tx[12] = 0; 
      Send_Data.tx[11] = 0; 

      Send_Data.tx[13] = 0; 
      Send_Data.tx[14] = 0; 
      Send_Data.tx[15]=Check_Sum(15,SEND_DATA_CHECK); 
      Send_Data.tx[16]=FRAME_TAIL; 
    }else if(model == 1){
      Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
      Send_Data.tx[1] = 0; //set aside //预留位
      Send_Data.tx[2] = 0; //set aside //预留位

      Send_Data.tx[3] = AGV_USE_ROS_MODEL; // ROS巡航模式 
      
      Send_Data.tx[4] = 0x00; //

      //The target velocity of the X-axis of the robot
      //机器人x轴的目标线速度
      transition=0;
      transition = cmd_vel_.linear.x*1000; //将浮点数放大一千倍，简化传输
      Send_Data.tx[6] = transition;     //取数据的低8位
      Send_Data.tx[5] = transition>>8;  //取数据的高8位

      //The target velocity of the Y-axis of the robot
      //机器人y轴的目标线速度
      transition=0;
      transition = cmd_vel_.linear.y*1000;
      Send_Data.tx[8] = transition;
      Send_Data.tx[7] = transition>>8;

      //The target angular velocity of the robot's Z axis
      //机器人z轴的目标角速度
      transition=0;
      // if(cmd_vel_.linear.x != 0){
      //   temp_rad = atan(cmd_vel_.angular.z * 1.462 /  cmd_vel_.linear.x);
      // }else{
      //   temp_rad = 0;
      // }
      temp_rad = forklift_cmd_vel_.expected_steer;
      transition = temp_rad*1000;
      // transition = cmd_vel_.angular.z*1000;
      Send_Data.tx[10] = transition;
      Send_Data.tx[9] = transition>>8;

      // 叉子离地面高度
      transition = 0;
      Send_Data.tx[12] = transition;
      Send_Data.tx[11] = transition>>8;

      // 错误码
      transition = 0;
      transition = 404;
      Send_Data.tx[14] = transition;
      Send_Data.tx[13]  = transition>>8;

      Send_Data.tx[15]=Check_Sum(15,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
      Send_Data.tx[16]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    }
    // cout << "================================" << endl;
    // ROS_INFO("Send_Data.tx[3] = %02x", Send_Data.tx[3]);
    // ROS_INFO("Send_Data.tx[4] = %02x", Send_Data.tx[4]);
    // ROS_INFO("Send_Data.tx[5] = %02x", Send_Data.tx[5]);
    // ROS_INFO("Send_Data.tx[6] = %02x", Send_Data.tx[6]);
    // ROS_INFO("Send_Data.tx[7] = %02x", Send_Data.tx[7]);
    // ROS_INFO("Send_Data.tx[8] = %02x", Send_Data.tx[8]);
    // ROS_INFO("Send_Data.tx[9] = %02x", Send_Data.tx[9]);
    // ROS_INFO("Send_Data.tx[10] = %02x", Send_Data.tx[10]);
    // ROS_INFO("Send_Data.tx[11] = %02x", Send_Data.tx[11]);
    // ROS_INFO("Send_Data.tx[12] = %02x", Send_Data.tx[12]);
    // ROS_INFO("Send_Data.tx[13] = %02x", Send_Data.tx[13]);
    // ROS_INFO("Send_Data.tx[14] = %02x", Send_Data.tx[14]);
    try
    {
      Stm32_Serial.write(Send_Data.tx , sizeof(Send_Data.tx)); 
    } 
    catch (serial::IOException& e)   
    {
      ROS_ERROR_STREAM("Unable to send data through stm32 serial port"); 
    }
  
}

/**************************************
Date: June 17, 2024
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_sfy_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  is_receive_cmd_vel_ = true;
  cmd_vel_ = twist_aux;
}

/**************************************
Date: January 5, 2024
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_sfy_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
    }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
    }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 5, 2024
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 位置话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_sfy_robot::Apriltag_Positon_Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr&  msg)
{
    _Apriltag_Now = ros::Time::now();
    if(_Apriltag_Last_Time.toSec()==0 ) _Apriltag_Last_Time=_Apriltag_Now;
    Apriltag_Sampling_Time = (_Apriltag_Now - _Apriltag_Last_Time).toSec();
    
    if(debug_apriltag_model_){
      cout << "============Apriltag_Positon_Callback  Start==============" << endl;
      cout << "Apriltag_Sampling_Time: " << Apriltag_Sampling_Time << endl;
    }

    const float getX = msg->detections[0].pose.pose.pose.position.x;
    const float getY = msg->detections[0].pose.pose.pose.position.y;
    const float getZ = msg->detections[0].pose.pose.pose.position.z;

    if(msg->detections.size()>0 && (getX >=0 &&  getY>=0)  ){         
     
        // tf::Quaternion quat;
        // tf::quaternionMsgToTF(msg->detections[0].pose.pose.pose.orientation, quat);
        double x =  msg->detections[0].pose.pose.pose.orientation.x;
        double y = msg->detections[0].pose.pose.pose.orientation.y;
        double z =  msg->detections[0].pose.pose.pose.orientation.z;
        double w =  msg->detections[0].pose.pose.pose.orientation.w;
        tf::Quaternion q(x, y, z, w);

        // 将四元数转换为旋转矩阵
        tf::Matrix3x3 rot_matrix(q);
        Eigen::Matrix3d R_camera_world;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_camera_world(i, j) = rot_matrix[i][j];
            }
        }

        // 找到旋转矩阵R_camera_world的列向量x和y
        Eigen::Vector3d x_axis = R_camera_world.col(0);
        Eigen::Vector3d y_axis = R_camera_world.col(1);

        // 计算投影到xoy平面上的法向量
        Eigen::Vector3d n_projected = (x_axis.cross(y_axis)).normalized();

        // 计算在xoy平面上的rad
        const float angle = atan2(n_projected.y(), n_projected.x()); // 使用反正切函数计算角度
        // 将rad转换为角度
        const float angle_deg = angle * 180.0 / M_PI;
        
        Current_Apriltag_Pos.X = getX;
        Current_Apriltag_Pos.Y  = getY;
        Current_Apriltag_Pos.Z = angle;

        if(debug_apriltag_model_){
          cout<< "识别到了二维码坐标" << endl;
          cout<<"相机在XOY投影的x坐标: "<< getX <<" 相机的y坐标："<< getY <<" 相机的z坐标："<< getZ <<endl; 
          cout << "xoy平面上角度：" << angle_deg << " 度"<< endl;
          cout << "xoy平面上弧度："<< angle << " rad" << endl;
          cout << "camera_to_base_footprint_y" << camera_to_base_footprint_y  << endl;
          cout<<"叉车base_printf的x坐标: "<< Robot_Pos.X <<" y坐标："<< Robot_Pos.Y <<" yaw："<< Robot_Pos.Z <<endl; 
          cout << "============Apriltag_Positon_Callback  End==============" << endl;
        }

        forklift_flags.current_has_apriltag = true;    // 当前有二维码
        last_time_records.last_apriltag_time = _Apriltag_Now;   
    }else{
      forklift_flags.current_has_apriltag = false;                  // 当前没有二维码
      // Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Apriltag_Sampling_Time; //计算X方向的位移，单位：m
      // Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Apriltag_Sampling_Time; //计算Y方向的位移，单位：m
      // Robot_Pos.Z += Robot_Vel.Z * Apriltag_Sampling_Time;   //   绕Z轴的角位移，单位：rad 
      
      if(debug_apriltag_model_){
        cout << "没有识别到二维码坐标" << endl;
        cout << "============Apriltag_Positon_Callback  End==============" << endl;
      }
    }
    _Apriltag_Last_Time = _Apriltag_Now;    
}

/**************************************
Date: January 5, 2024
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_sfy_robot::Get_Sensor_Data_New()
{
  // cout << "============Get_Sensor_Data_New  Start==============" << endl;
  short transition_16=0; //Intermediate variable //中间变量
  uint8_t i = 0, check = 0, error = 1, Receive_Data_Pr[1]; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  static int count; //Static variable for counting //静态变量，用于计数

  try
  {
    Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); 
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to receive data through serial port");
  }
  
  // cout << "当前count为：" << count << endl;
  // ROS_INFO("当前字节原始数据为：%02x", Receive_Data_Pr[0]);

  Receive_Data.rx[count] = Receive_Data_Pr[0]; //Fill the array with serial data //串口数据填入数组
  Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[19];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
    count++;
  else 
  	count=0;
  if(count == 20) //Verify the length of the packet //验证数据包的长度
  {
    count=0;  //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
    if(Receive_Data.Frame_Tail == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
    {
      check=Check_Sum(18,READ_DATA_CHECK);  //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错

      if(check == Receive_Data.rx[18])  
      {
        error=0;  //XOR bit check successful //异或位校验成功
      }
      if(error == 0)
      {
        //Check receive_data.rx for debugging use //查看Receive_Data.rx，调试使用 
        // cout << "================================" << endl;
        // ROS_INFO("Receive_Data.rx[1] = %02x", Receive_Data.rx[1]);
        // ROS_INFO("Receive_Data.rx[2] = %02x", Receive_Data.rx[2]);
        // ROS_INFO("Receive_Data.rx[3] = %02x", Receive_Data.rx[3]);
        // ROS_INFO("Receive_Data.rx[4] = %02x", Receive_Data.rx[4]);
        // ROS_INFO("Receive_Data.rx[5] = %02x", Receive_Data.rx[5]);
        // ROS_INFO("Receive_Data.rx[6] = %02x", Receive_Data.rx[6]);
        // ROS_INFO("Receive_Data.rx[7] = %02x", Receive_Data.rx[7]);
        // ROS_INFO("Receive_Data.rx[8] = %02x", Receive_Data.rx[8]);
        // ROS_INFO("Receive_Data.rx[9] = %02x", Receive_Data.rx[9]);
        // ROS_INFO("Receive_Data.rx[10] = %02x", Receive_Data.rx[10]);
        // ROS_INFO("Receive_Data.rx[11] = %02x", Receive_Data.rx[11]);
        // ROS_INFO("Receive_Data.rx[12] = %02x", Receive_Data.rx[12]);
        // ROS_INFO("Receive_Data.rx[13] = %02x", Receive_Data.rx[13]);

        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);  //获取运动底盘X方向速度
          
        // Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);  //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Y = 0;  // 先这样        
                                                                         
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //获取运动底盘Z方向速度   

        // ROS_INFO("Robot_Vel.X:%f,Robot_Vel.Y:%f,Robot_Vel.Z:%f", Robot_Vel.X, Robot_Vel.Y, Robot_Vel.Z);

        //获取电池电压
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[8]<<8;
        transition_16 |=  Receive_Data.rx[9];  
        Power_voltage = transition_16/1000 + (transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)

        // fcode
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[12]<<8;
        transition_16 |=  Receive_Data.rx[13];  
        forklift_fcode = transition_16;
        // cout <<"forklift_fcode: " << forklift_fcode << endl;
        return true;
      }
    }
  }
  return false;
}

/**************************************
Date: January 5, 2024
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_sfy_robot::Control()
{
  // ros::Rate loop_rate(frequency_);
  while(ros::ok())
  {  
    if (true == Get_Sensor_Data_New() ) 
    {
      _Now = ros::Time::now();
      if(_Last_Time.toSec()==0) _Last_Time = _Now; 
      Sampling_Time = (_Now - _Last_Time).toSec(); 

      Publish_Odom();
      Publish_Forklift_Info();
      Publish_Forklift_Init_Info();
      Publish_Forklift_Apriltag_Info();
     
      if(is_receive_cmd_vel_){
          is_receive_cmd_vel_ = false;
          last_cmd_vel_time_ = ros::Time::now(); 
      }

       _Last_Time = _Now;
      // cout << "Sampling_Time: " << Sampling_Time << endl; 

    }
    ros::spinOnce();  
    // loop_rate.sleep();  // 控制循环频率
  }
}

/**************************************
Date: January 5, 2024
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_sfy_robot::turn_on_sfy_robot():Sampling_Time(0),Power_voltage(0),Apriltag_Sampling_Time(0)
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));

  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyUSB0"); 
  private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200); 
  
  private_nh.param<std::string>("odom_frame_id",    odom_frame_id,    "odom");      //The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  private_nh.param<std::string>("robot_frame_id",   robot_frame_id,   "base_footprint"); //The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  private_nh.param<std::string>("gyro_frame_id",    gyro_frame_id,    "gyro_link"); //IMU topics correspond to TF coordinates //IMU话题对应TF坐标

  //Odometer correction parameters
  //里程计误差修正参数
  private_nh.param<float>("odom_x_scale",    odom_x_scale,    1.0); 
  private_nh.param<float>("odom_y_scale",    odom_y_scale,    1.0); 
  private_nh.param<float>("odom_z_scale_positive",    odom_z_scale_positive,    1.0); 
  private_nh.param<float>("odom_z_scale_negative",    odom_z_scale_negative,    1.0); 
  private_nh.param<float>("camera_to_base_footprint_x",    camera_to_base_footprint_x,    1.0); 
  private_nh.param<float>("camera_to_base_footprint_y",    camera_to_base_footprint_y,    0.0); 

  private_nh.param<double >("frequency",  frequency_,    20.0); 
  private_nh.param<int >("forklift_id",  forklift_id_,    1); 

  // debug模式
  private_nh.param<bool>("use_apriltag_change_imu_yaw", use_apriltag_change_imu_yaw_,  true); 
  private_nh.param<bool>("use_imu_yaw_to_odom", use_imu_yaw_to_odom_,  true); 
  private_nh.param<bool>("debug_apriltag_model", debug_apriltag_model_,  false); 
  private_nh.param<int>("debug_model", debug_model_,  1); 

  forklift_info_publisher = n.advertise<sfy_forklift_msgs::ForkliftInfo>("forklift_info", 100);
  forklift_init_info_publisher = n.advertise<sfy_forklift_msgs::ForkliftInitInfo>("forklift_init_info", 1);
  forklift_apriltag_info_publisher = n.advertise<sfy_forklift_msgs::ForkliftApriltagInfo>("forklift_apriltag_info", 1);
  odom_publisher    = n.advertise<nav_msgs::Odometry>("odom", 100);
  
  Cmd_Vel_Sub = n.subscribe("cmd_vel", 1, &turn_on_sfy_robot::Cmd_Vel_Callback, this , ros::TransportHints().tcpNoDelay(true)); 
  Apriltag_Positon_Sub = n.subscribe("/tag_detections", 100,  &turn_on_sfy_robot::Apriltag_Positon_Callback, this, ros::TransportHints().tcpNoDelay(true));
  move_base_goal_sub_   = n.subscribe( "/move_base_simple/goal", 1, 
        &turn_on_sfy_robot::moveBaseGoalCB, this);
  forklift_init_imu_sub_ = n.subscribe("sfy/imu_init", 1,  &turn_on_sfy_robot::forkliftInitImuCB, this, 
      ros::TransportHints().tcpNoDelay(true));
  forklift_cmd_vel_sub_ =  n.subscribe("forklift_cmd_vel", 1,  &turn_on_sfy_robot::forkliftCmdVelCB, this, 
      ros::TransportHints().tcpNoDelay(true));

        
  my_timer_ = n.createTimer(ros::Duration((1.0)/frequency_), &turn_on_sfy_robot::My_Timer, this);
  ROS_INFO_STREAM("Data ready"); 
  try
  { 
    serial::parity_t pt = serial::parity_t::parity_none;//创建奇偶校验位为0位
    serial::bytesize_t bt = serial::bytesize_t::eightbits;//创建发送字节数为8位
    serial::flowcontrol_t ft = serial::flowcontrol_t::flowcontrol_none;//创建数据流控制，不使用
    serial::stopbits_t st = serial::stopbits_t::stopbits_one;//创建终止位为1位

    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    Stm32_Serial.setParity(pt);//设置校验位
    Stm32_Serial.setBytesize(bt);//设置发送字节数
    Stm32_Serial.setFlowcontrol(ft);//设置数据流控制
    Stm32_Serial.setStopbits(st);//设置终止位

    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); 
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); 
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("sfy_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("sfy_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}


/**************************************
Date: January 5, 2024
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_sfy_robot::~turn_on_sfy_robot()
{
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 
  Send_Data.tx[3] = AGV_STOP_MODEL;  
  Send_Data.tx[4] = 0;     
  
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    

  Send_Data.tx[10] = 0;  
  Send_Data.tx[9] = 0; 

  Send_Data.tx[12] = 0; 
  Send_Data.tx[11] = 0; 

  Send_Data.tx[13] = 0; 
  Send_Data.tx[14] = 0; 

  Send_Data.tx[15]=Check_Sum(15,SEND_DATA_CHECK); 
  Send_Data.tx[16]=FRAME_TAIL; 

  try
  {
    Stm32_Serial.write(Send_Data.tx , sizeof(Send_Data.tx)); 
  } 
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to Ros send data through stm32 serial port"); 
  }
   
}

/**************************************
Date: January 5, 2024
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "sfy_robot"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  turn_on_sfy_robot Robot_Control; //Instantiate an object //实例化一个对象
  Robot_Control.Control(); //Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  return 0;  
} 


