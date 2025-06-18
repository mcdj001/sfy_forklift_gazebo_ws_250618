#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sfy_forklift_mission_planner/SfyForkliftMoveBasePluginAction.h>
#include <pluginlib/class_loader.h>  
#include <nav_core/base_local_planner.h> 
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>  
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>  
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
class SfyForkliftMoveBasePluginServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<sfy_forklift_mission_planner::SfyForkliftMoveBasePluginAction> as_;
    std::string action_name_;
    sfy_forklift_mission_planner::SfyForkliftMoveBasePluginFeedback feedback_;
    sfy_forklift_mission_planner::SfyForkliftMoveBasePluginResult result_;

    boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;  // Declare shared pointer for local planner
    boost::shared_ptr<nav_core::BaseGlobalPlanner> global_planner_;
    ros::Publisher cmd_vel_pub_;

    tf2_ros::Buffer tf_buffer_; // TF buffer
    tf2_ros::TransformListener tf_listener_; // TF listener
    costmap_2d::Costmap2DROS* costmap_ros_;  // Costmap ROS object

public:
    SfyForkliftMoveBasePluginServer(std::string name) : 
        as_(nh_, name, boost::bind(&SfyForkliftMoveBasePluginServer::executeCB, this, _1), false),
        action_name_(name),
        tf_listener_(tf_buffer_)
    {
        // Dynamically load the local planner plugin
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugin_loader("nav_core", "nav_core::BaseLocalPlanner");
        try
        {
            // The plugin name here should be the name of your local planner plugin as specified in the XML file
            local_planner_ = plugin_loader.createInstance("sfy_forklift_local_planner/SfyForkliftLocalPlanner");
            ROS_INFO("Successfully loaded local planner plugin");
        }
        catch (const pluginlib::LibraryLoadException& e)
        {
            ROS_ERROR("Failed to load local planner plugin: %s", e.what());
            ros::shutdown();
        }

        // Dynamically load the global planner plugin
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> global_plugin_loader("nav_core", "nav_core::BaseGlobalPlanner");
        try
        {
            // Load global planner plugin
            global_planner_ = global_plugin_loader.createInstance("sfy_forklift_global_planner/SfyForkliftGlobalPlanner");
            ROS_INFO("Successfully loaded global planner plugin");
        }
        catch (const pluginlib::LibraryLoadException& e)
        {
            ROS_ERROR("Failed to load global planner plugin: %s", e.what());
            ros::shutdown();
        }

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Initialize tf2 buffer and costmap_ros (these might need to be set up differently in your system)
        costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_buffer_);
        
        // Initialize the local planner with these components
        global_planner_->initialize("sfy_forklift_global_planner", costmap_ros_);
        local_planner_->initialize("sfy_forklift_local_planner", &tf_buffer_, costmap_ros_);
        
        // Start the action server
        as_.start();
    }

    void executeCB(const sfy_forklift_mission_planner::SfyForkliftMoveBasePluginGoalConstPtr &goal)
    {
        ROS_INFO("%s: Executing mission", action_name_.c_str());

        // Create a global plan for the mission (based on the goal pose)
        double x1 = goal->x1;
        double y1 = goal->y1;
        double x2 = goal->x2;
        double y2 = goal->y2;
        // vector<geometry_msgs::PoseStamped> global_plan = generateStraightLinePath(x1, y1, x2, y2, 0.5);
        geometry_msgs::PoseStamped start_pose, goal_pose;
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
            ROS_INFO("Successfully retrieved transform from map to odom");
            
            // Apply transform if needed, example here if transforming start_pose:
            tf2::doTransform(start_pose, start_pose, transformStamped);  // Applying the transform
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not get transform: %s", ex.what());
            result_.success = false;
            result_.message = "Failed to get TF transform";
            as_.setAborted(result_);
            return;
        }

        std::vector<geometry_msgs::PoseStamped> global_plan;
        


        // ROS_INFO("Global Plan: ");
        // for (size_t i = 0; i < global_plan.size(); ++i)
        // {
        //     ROS_INFO("Pose %zu: Position (x: %.2f, y: %.2f, z: %.2f), Orientation (x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
        //         i,
        //         global_plan[i].pose.position.x,
        //         global_plan[i].pose.position.y,
        //         global_plan[i].pose.position.z,
        //         global_plan[i].pose.orientation.x,
        //         global_plan[i].pose.orientation.y,
        //         global_plan[i].pose.orientation.z,
        //         global_plan[i].pose.orientation.w);
        // }

        bool success = global_planner_->makePlan(start_pose, goal_pose, global_plan);
        if (!success)
        {
            ROS_ERROR("Failed to generate a global plan");
            result_.success = false;
            result_.message = "Global plan generation failed";
            as_.setAborted(result_);
            return;
        }

        local_planner_->setPlan(global_plan);

        bool goal_reached = false;
        while (ros::ok())
        {
            

            goal_reached = local_planner_->isGoalReached();
            if (goal_reached)
            {
                // Clear cmd_vel once the goal is reached
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_.publish(cmd_vel);  // Publish the stop command

                // Provide feedback and set the result
                result_.success = true;
                result_.message = "Mission completed successfully";
                as_.setSucceeded(result_);

                // Optionally, print out the info
                ROS_INFO("Goal reached, stopping robot.");
                break;  
            }
            else
            {
                // If goal is not reached, compute the velocity command
                geometry_msgs::Twist cmd_vel;
                if (local_planner_->computeVelocityCommands(cmd_vel))
                {
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Publishing cmd_vel: linear.x = %f, angular.z = %f", cmd_vel.linear.x, cmd_vel.angular.z);

                    // Publish progress feedback
                    feedback_.progress = 100.0 * (local_planner_->isGoalReached() ? 1.0 : 0.0);
                    as_.publishFeedback(feedback_);
                }
            }
            ros::Duration(0.1).sleep(); // Adjust for real-time control
        }
    }

    std::vector<geometry_msgs::PoseStamped> generateStraightLinePath(double x1, double y1, double x2, double y2, double step_size)
    {
     std::vector<geometry_msgs::PoseStamped> path;

      // Calculate the distance between the two points
      double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

      // Calculate the number of steps required to cover the distance
      int num_steps = static_cast<int>(distance / step_size);
      if (num_steps == 0)
      {
          num_steps = 1;  // Ensure at least one step
      }

      // Calculate the increment in x and y
      double dx = (x2 - x1) / num_steps;
      double dy = (y2 - y1) / num_steps;

      // Generate the path
      for (int i = 0; i <= num_steps; ++i)
      {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = x1 + i * dx;
          pose.pose.position.y = y1 + i * dy;

          // You can add orientation here if needed. For now, we set it to 0
          pose.pose.orientation.w = 1.0;

          path.push_back(pose);
      }

      return path;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sfy_forklift_move_base_plugin");

    SfyForkliftMoveBasePluginServer mission_planner("sfy_forklift_move_base_plugin_action");
    ros::spin();

    return 0;
}
