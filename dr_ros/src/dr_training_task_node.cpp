#include <ros/ros.h>
#include <ros/time.h>
#include <tuple>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

#include <std_srvs/SetBool.h>
#include <dr_msgs/TrainingPoses.h>

#include <eigen_conversions/eigen_msg.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// DR appication
#include <dr/training_task.h>

// ROS messages

#include <abb_robot_msgs/TriggerWithResultCode.h>
//#include <abb_robot_driver_actions/EGMSettingsAction.h>
#include <abb_rapid_sm_addin_msgs/EGMSettings.h>
#include <abb_rapid_sm_addin_msgs/GetEGMSettings.h>
#include <abb_rapid_sm_addin_msgs/SetEGMSettings.h>
#include <controller_manager_msgs/SwitchController.h>

  /***********************************************************************************************************************
   * Class definitions: YumiDRTrainingTask
   **********************************************************************************************************************/
  class YumiDRTrainingTask {
    private:
      ros::Publisher drtask_pub;
      ros::ServiceServer drtask_srv;
      dr_msgs::TrainingPoses poses;

      dr::DRTrainingTask dr_task;
      dr::DRTrainingTask::TrainingTaskResult dr_task_result;
      
      std::string planning_group;

      bool result = false;

    public:
      /**************************************************************************
       * Constructor
       **************************************************************************/
      YumiDRTrainingTask (ros::NodeHandle *nh, std::string robot_model, std::string ee_name) {

        drtask_pub = nh->advertise<dr_msgs::TrainingPoses>("/result_dr_task", 10);
        drtask_srv = nh->advertiseService("/dr_task", &YumiDRTrainingTask::callback_drtask, this);

        dr_task_result.joint_values.clear();dr_task_result.joint_values.resize(7);
        poses.joints_pos.clear();poses.joints_pos.resize(7);

        //-----------------------
        // Robot configuration
        //-----------------------
        dr_task.robot_model = robot_model;// Robot description (used in ros_control)
        dr_task.current_ee_name = ee_name;// Link to which apply the calibration (e.g. last link, tool, etc)

        planning_group = "rob_r";
        dr_task.planning_group = planning_group;
      }

      /**************************************************************************
       * DRTask callback
       **************************************************************************/
      bool callback_drtask(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

        if (req.data)
        {      
          //-----------------------
          // DR training task
          //-----------------------
          result = dr_task.setRobot(dr_task.kinematic_model);

          // Capture error setting the robot
          if (!result)
          {
            poses.result = false;
            ROS_ERROR("Error setting robot");
          }

          // Planing group to perform the movement
          moveit::planning_interface::MoveGroupInterface move_group(planning_group);
      
          ROS_INFO("Planning group created: %s", planning_group.c_str());

          geometry_msgs::Pose target_pose;
          target_pose.position.x = 0.20445;
          target_pose.position.y = -0.32462;
          target_pose.position.z = 0.35699;
          target_pose.orientation.x = -0.08378;
          target_pose.orientation.y = 0.76356;
          target_pose.orientation.z = 0.092061;
          target_pose.orientation.w = 0.63363;

          //ROS_INFO("Target pose: x - %g y - %g z - %g", target_pose.position.x, target_pose.position.y, target_pose.position.z);

          result = dr_task.moveRobotPose(target_pose, move_group);

          // Capture error moving the robot
          /*if (!result)
          {
            poses.result = false;
            ROS_ERROR("Error moving robot");
          }*/

          ROS_INFO("Getting pose");
          dr_task_result = dr_task.getEEPoses(move_group,
                                              dr_task.kinematic_model);

          //-----------------------
          // Publish results
          //-----------------------
          poses.result = true;
          poses.joints_pos.clear();poses.joints_pos.resize(7);
          
          for (int i = 0; i < 7; i++){
            poses.joints_pos.at(i) = dr_task_result.joint_values.at(i);
          }
          poses.pose = dr_task_result.pose;

          drtask_pub.publish(poses);

          //-----------------------
          // Return HOME POSE
          //-----------------------

          std::vector<double> target_joint;
          target_joint.resize(7);
          target_joint.at(0) = 0.00;
          target_joint.at(1) = -130 * M_PI / 180;
          target_joint.at(2) = -135 * M_PI / 180;
          target_joint.at(3) = 30 * M_PI / 180;
          target_joint.at(4) = 0.00;
          target_joint.at(5) = 40 * M_PI / 180;
          target_joint.at(6) = 0.00;

          result = dr_task.moveRobotJoint(target_joint, move_group);
          
          // Capture error moving the robot
          /*if (!result)
          {
            poses.result = false;
            ROS_ERROR("Error moving robot");
          }*/

          res.success = true;
          return true;
        }
      }
  };// End YumiDRTrainingTask class

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "dr_training_task");
  
  ros::NodeHandle nh;

  std::string robot_model;
  std::string ee_name;

  if (argc < 3)
  {
      ROS_ERROR("Error calling node. Usage: dr_pretraining_task 'robot_model' 'endeffector_name'");
  }
  else
  {
    robot_model = argv[1];
    ee_name = argv[2];
  }

  YumiDRTrainingTask yumi_dr_task = YumiDRTrainingTask(&nh, robot_model, ee_name);

  ros::MultiThreadedSpinner s(16);

  ros::spin(s);
  
}