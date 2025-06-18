#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sfy_forklift_mission_planner/SfyForkliftMoveBasePluginAction.h>

using namespace std;
bool mission_success = true;


// 反馈回调函数
void feedbackCallback(const sfy_forklift_mission_planner::SfyForkliftMoveBasePluginFeedbackConstPtr& feedback)
{
    // ROS_INFO("Feedback: Mission progress: %.2f%%", feedback->progress);
}

void activeCallback(){
    ROS_INFO("服务已经被激活....");
}

// 结果回调函数
void resultCallback(const actionlib::SimpleClientGoalState& state,
                    const sfy_forklift_mission_planner::SfyForkliftMoveBasePluginResultConstPtr& result)
{
    ROS_INFO("Result: Mission finished with state: %s", state.toString().c_str());
    if (result->success)
    {
        ROS_INFO("Mission succeeded with message: %s", result->message.c_str());
        mission_success = true;  // Mission succeeded, so we allow sending a new goal
    }
    else
    {
        ROS_INFO("Mission failed with message: %s", result->message.c_str());
        mission_success = false;  // Mission failed, don't send new goal
    }
    cout << "mission_success callback: " << mission_success << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_action_client");

    actionlib::SimpleActionClient<sfy_forklift_mission_planner::SfyForkliftMoveBasePluginAction> ac("sfy_forklift_move_base_plugin_action", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();

    ros::Rate rate(1);  // 每秒检查一次
    while (ros::ok())
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
            ac.getState() == actionlib::SimpleClientGoalState::PENDING)
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
            goal.x1 = 5.1;
            goal.y1 = 5.1;
            goal.x2 = 10.0;
            goal.y2 = 5.1;

            ac.sendGoal(goal, &resultCallback, &activeCallback, &feedbackCallback);

            cout << "mission_success: " << mission_success << endl;
            // 等待任务完成
            bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

            if (finished_before_timeout)
            { 
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Mission finished with state: %s", state.toString().c_str());
            }
            else
            {
                ROS_INFO("Mission did not finish before the time out.");
            }

            break;  // 任务发送完成后退出循环
        }

        ros::spinOnce();
        rate.sleep();  // 保持循环频率
    }

    cout << "循环结束" << endl;

    return 0;
}
