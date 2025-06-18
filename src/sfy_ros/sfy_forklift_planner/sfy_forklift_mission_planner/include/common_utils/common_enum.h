/*
 * @Author: clint-sfy 2786435349@qq.com
 * @Date: 2025-05-18 14:51:29
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2025-06-02 19:50:51
 * @FilePath: /sfy_forklift_gazebo_ws/src/sfy_ros/sfy_forklift_planner/sfy_forklift_mission_planner/include/common_utils/common_enum.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __FORKLIFT_COMMON_ENUMS_CPP__
#define __FORKLIFT_COMMON_ENUMS_CPP__

typedef enum { 
    EAST,              // 0
    NORTH,             // pi/2
    WEST,              // pi
    SOUTH              // -pi/2
} Yaw;

typedef enum {
   	AGV_STOPPED,                  // 停止待机
    AGV_FAULT,                    // 故障
    AGV_POWER_OFF,                // 关机
    AGV_PICKUP_PATH_PLANNING,     // 取货路径规划中
    AGV_MOVING_TO_PICKUP,         // 前往任务点取货中
    AGV_ARRIVED_AT_PICKUP,        // 到达取货点
    AGV_ALIGNING_WITH_PALLET,     // 托盘对准中
    AGV_ALIGNMENT_COMPLETE,       // 托盘对准完成
    AGV_PICKING_UP,               // 取货中
    AGV_PICKUP_COMPLETE,          // 取货完成
    AGV_DROPOFF_PATH_PLANNING,    // 放货路径规划中
    AGV_MOVING_TO_DROPOFF,        // 前往任务点放货中
    AGV_ARRIVED_AT_DROPOFF,       // 到达放货点
    AGV_DROPPING_OFF,             // 放货中
    AGV_DROPOFF_COMPLETE,         // 放货完成
    AGV_LEAVING_AFTER_DROPOFF,    // 放货后离开托盘中
    AGV_PARKING_PATH_PLANNING,    // 停车点路径规划中
    AGV_MOVING_TO_PARKING,        // 前往停车点中
    AGV_PATH_PLANNING_FAILED      // 路径规划失败
} AGVForkliftStatus;

typedef enum {
   	PATH_PLANNING_TOPIC_CLOSED,     // 路径规划话题关闭
    PATH_PLANNING_WAITING,          // 路径规划等待中
    PATH_PLANNING,            	    // 路径规划中
    PATH_PLANNING_FAILED,           // 路径规划失败
    PATH_PLANNING_COMPLETE,		    // 路径规划完成
    PATH_PLANNING_PUB,    		    // 路径发布中
} PathPlanningStatus ;


typedef enum {
    PATH_TRACKING_FORWARD_PUBLISHING,       // 路径跟踪发布中(正向)
    PATH_TRACKING_FORWARD_EXECUTING,        // 路径跟踪执行中(正向)
    PATH_TRACKING_REVERSE_PUBLISHING,       // 路径跟踪发布中(逆向)
    PATH_TRACKING_REVERSE_EXECUTING,        // 路径跟踪执行中(逆向)
    PATH_TRACKING_FAILED,           		// 路径跟踪失败
    PATH_TRACKING_ALL_COMPLETE      		// 所有路径跟踪完成
} PathTrackingStatus;


#endif