/*
 * @Author: clint-sfy 2786435349@qq.com
 * @Date: 2025-05-07 15:12:38
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 19:53:49
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/src/sfy_forklift_mission_planning.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include"mission_planning/sfy_forklift_mission_planning.h"

void SfyForkliftMissionPlanning::pathTrackingStatusCallback(const sfy_forklift_msgs::ForkliftPathTrackingStatus::ConstPtr& Msg)
{
    forklift_path_tracking_status_ = *Msg;
}

void SfyForkliftMissionPlanning::forkliftPathPlanningStatusCallback(const sfy_forklift_msgs::ForkliftPathPlanningStatus::ConstPtr& Msg)
{
    forklift_path_planning_status_ =  *Msg;
    current_forklift_path_planning_status_ = static_cast<PathPlanningStatus>(forklift_path_planning_status_.forklift_path_planning_status);
    current_forklift_path_tracking_status_ = static_cast<PathTrackingStatus>(forklift_path_planning_status_.forklift_path_tracking_status);
}

 
void SfyForkliftMissionPlanning::pubForkliftMissionTargetPoint(){

    forklift_mission_target_point_.forklift_id = forklift_id_;
    forklift_mission_target_point_.is_start_mission = is_start_mission_;
    forklift_mission_target_point_.forklift_target_point = forklift_target_point_;
    forklift_mission_target_point_.forklift_target_yaw = forklift_target_yaw_;
    
    mission_target_pub_.publish(forklift_mission_target_point_);
}

void SfyForkliftMissionPlanning::forkliftTask(){
      if(current_forklift_task_type_ == FORKLIFT_NO_TASK){
        is_start_mission_ = false;  // 路径规划模块不允许开始
        current_forklift_status_ == AGV_STOPPED;

        // 允许开始取货任务   还有一件事  程序启动两秒之内都不允许操作这个
        if(is_start_pickup_task_){
            current_forklift_task_type_ = FORKLIFT_PICKUP_TASK;
            // forklift_target_point_ = 
            // forklift_target_yaw_ = 
        // 允许开始停车任务
        }else if(is_start_parking_task_){
            current_forklift_task_type_ = FORKLIFT_PARKING_TASK;
            // forklift_target_point_ = 
            // forklift_target_yaw_ = 
        }
    }else if(current_forklift_task_type_ == FORKLIFT_PICKUP_TASK){

        if(current_forklift_status_ == AGV_STOPPED){               //叉车状态  停车
            current_forklift_status_ = AGV_PICKUP_PATH_PLANNING;            

        }else if(current_forklift_status_ == AGV_PICKUP_PATH_PLANNING){  // 叉车状态  取货路径规划中
            cout << "====current_forklift_status_ : AGV_PICKUP_PATH_PLANNING  取货路径规划中===="  << endl;
            is_start_mission_ = true;
 
            // 查看路径规划的状态更新AGV叉车的状态
            if(current_forklift_path_planning_status_ == PATH_PLANNING_TOPIC_CLOSED){
                cout << "路径规划话题关闭" << endl;
            }else if(current_forklift_path_planning_status_ == PATH_PLANNING){
                cout << "路径规划中  mision" << endl;
            }else if(current_forklift_path_planning_status_ == PATH_PLANNING_FAILED){
                current_forklift_status_ = AGV_PATH_PLANNING_FAILED;
                current_forklift_task_type_ = FORKLIFT_NO_TASK;      // 本次规划失败 需要重新点击确认
                cout << "路径规划失败" << endl;
            }else if(current_forklift_path_planning_status_ == PATH_PLANNING_COMPLETE){    // 路径规划完成
                current_forklift_status_ = AGV_MOVING_TO_PICKUP;
                cout << "路径规划完成" << endl;
            }else if(current_forklift_path_planning_status_ == PATH_PLANNING_PUB){    // 路径规划完毕  开始控制
                is_start_mission_ = false; 
            }else if(current_forklift_path_planning_status_ == PATH_PLANNING_WAITING){
                cout << "路径规划等待中" << endl;
            }
        }else if(current_forklift_status_ == AGV_MOVING_TO_PICKUP){       // 叉车状态 前往点取货中
            cout << "====current_forklift_status_ : AGV_MOVING_TO_PICKUP  前往点取货中 ===="  << endl;
            is_start_mission_ = false;  // 路径规划关闭

            if(current_forklift_path_tracking_status_ == PATH_TRACKING_FORWARD_PUBLISHING){
                cout << "路径跟踪发布中(正向) " << endl;
            }else if(current_forklift_path_tracking_status_ == PATH_TRACKING_FORWARD_EXECUTING){
                cout << "路径跟踪执行中(正向) " << endl;
            }else if(current_forklift_path_tracking_status_ == PATH_TRACKING_REVERSE_PUBLISHING){
                cout << "路径跟踪发布中(逆向) " << endl;
            }else if(current_forklift_path_tracking_status_ == PATH_TRACKING_REVERSE_EXECUTING){
                cout << "路径跟踪执行中(逆向) " << endl;
            }else if(current_forklift_path_tracking_status_ == PATH_TRACKING_FAILED){
                cout << "路径跟踪失败" << endl;
            }else if(current_forklift_path_tracking_status_ == PATH_TRACKING_ALL_COMPLETE){
                cout << "所有路径跟踪完成" << endl;
                current_forklift_status_ = AGV_ARRIVED_AT_PICKUP;  // 到达取货点
            }
            
        }else if(current_forklift_status_ == AGV_ARRIVED_AT_PICKUP){      // 叉车状态 到达取货点
            cout << "====current_forklift_status_ : AGV_ARRIVED_AT_PICKUP  到达取货点===="  << endl;


        }else if(current_forklift_status_ == AGV_ALIGNING_WITH_PALLET){   // 叉车状态 托盘对准中

        }else if(current_forklift_status_ == AGV_ALIGNMENT_COMPLETE){     // 叉车状态 托盘对准完成
            
        }else if(current_forklift_status_ == AGV_PICKING_UP){             // 叉车状态  取货中
            
        }else if(current_forklift_status_ == AGV_PICKUP_COMPLETE){        // 叉车状态  取货完成
            current_forklift_task_type_ = FORKLIFT_DROPOFF_TASK;          // 取货完成后 必须进行放货任务
        }   


    }else if(current_forklift_task_type_ == FORKLIFT_DROPOFF_TASK){

        
    }else if(current_forklift_task_type_ == FORKLIFT_PARKING_TASK){

    }

    forklift_status_.forklift_id = forklift_id_;
    forklift_status_.forklift_status = current_forklift_status_;
    forklift_status_pub_.publish(forklift_status_);
}

void SfyForkliftMissionPlanning::control(){
    ros::Rate loop_rate(frequency_);
    while(ros::ok()){
        pubForkliftMissionTargetPoint();
        forkliftTask();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

SfyForkliftMissionPlanning::~SfyForkliftMissionPlanning(){

  

}

SfyForkliftMissionPlanning::SfyForkliftMissionPlanning(){
    ros::NodeHandle n;
    ros::NodeHandle n_prv("~");

    n_prv.param<double>("frequency", frequency_ , 20);
    // n_prv.param<bool>("is_start_mission", is_start_mission_ , false);
    n_prv.param<int >("forklift_id", forklift_id_ , 1);
    n_prv.param<double>("forklift_target_point", forklift_target_point_ , 0);
    n_prv.param<int >("forklift_target_yaw", forklift_target_yaw_ , 1);

    forklift_path_planning_status_sub_ = n.subscribe("forklift_path_planning_status", 1, 
            &SfyForkliftMissionPlanning::forkliftPathPlanningStatusCallback, this, ros::TransportHints().tcpNoDelay(true));

    forklift_path_tracking_status_sub_ = n.subscribe("forklift_path_tracking_status", 1,
            &SfyForkliftMissionPlanning::pathTrackingStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
    mission_target_pub_ = n.advertise<sfy_forklift_msgs::ForkliftMissionTargetPoint>("forklift_target_point", 10);
    forklift_status_pub_ = n.advertise<sfy_forklift_msgs::ForkliftStatus>("forklift_status", 10);
    current_forklift_status_ = AGV_STOPPED;
    current_forklift_task_type_ = FORKLIFT_NO_TASK;

    is_start_pickup_task_ = true;
    is_start_dropoff_task_= false;
    is_start_parking_task_ = false;
}

int main(int argc, char** argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "sfy_forklift_mission");
    SfyForkliftMissionPlanning sfy_forklift_mission_planner;
    sfy_forklift_mission_planner.control();
    return 0;
}