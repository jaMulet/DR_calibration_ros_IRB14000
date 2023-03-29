#include <ros/ros.h>
#include <ros/time.h>
#include <tuple>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include <std_srvs/SetBool.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// DR appication
#include <dr/training_task.h>
#include <dr/target_manipulator.h>

// ROS messages
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <dr_msgs/TrainingPoses.h>

#include <eigen_conversions/eigen_msg.h>

#include <abb_robot_msgs/TriggerWithResultCode.h>
#include <abb_rapid_sm_addin_msgs/EGMSettings.h>
#include <abb_rapid_sm_addin_msgs/GetEGMSettings.h>
#include <abb_rapid_sm_addin_msgs/SetEGMSettings.h>
#include <controller_manager_msgs/SwitchController.h>

#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

  /***********************************************************************************************************************
   * Class definitions: YumiDRTrainingTask
   **********************************************************************************************************************/
  class DRTrainingTask {
    private:
      ros::Publisher drtask_pub;
      ros::ServiceServer drtask_srv;
      dr_msgs::TrainingPoses poses;

      geometry_msgs::Pose target_pose;

      dr::DRTrainingTask dr_task;
      dr::DRTrainingTask::TrainingTaskResult dr_task_result;
      dr::TargetManipulator target_task;

      std::string planning_group;

      int n_dof;

      bool result = false;

    public:
      /**************************************************************************
       * Constructor
       **************************************************************************/
      DRTrainingTask (ros::NodeHandle *nh, std::string task_target_pose, std::string robot_model, std::string planning_group_name, int robot_n_dof, std::string ee_name) {

        drtask_pub = nh->advertise<dr_msgs::TrainingPoses>("/result_dr_task", 10);
        drtask_srv = nh->advertiseService("/dr_task", &DRTrainingTask::callback_drtask, this);

        dr_task_result.joint_values.clear();dr_task_result.joint_values.resize(7);
        poses.joints_pos.clear();poses.joints_pos.resize(n_dof);

        //-----------------------
        // Robot configuration
        //-----------------------
        n_dof = robot_n_dof;

        dr_task.robot_model = robot_model;// Robot description (used in ros_control)
        dr_task.current_ee_name = ee_name;// Link to which apply the calibration (e.g. last link, tool, etc)

        planning_group = planning_group_name;
        dr_task.planning_group = planning_group;

        // Target pose
        ROS_INFO("Target pose: %s", task_target_pose.c_str());

        task_target_pose = target_task.removeBrackets(task_target_pose);

        std::vector<std::string> tokens;
        tokens = target_task.split(task_target_pose, ';');

        //ROS_ERROR("Target pose (final): x: %s y: %s z: %s qua_x: %s qua_y: %s qua_z: %s qua_w: %s", tokens.at(0).c_str(), tokens.at(1).c_str(), tokens.at(2).c_str(), tokens.at(3).c_str(), tokens.at(4).c_str(), tokens.at(5).c_str(), tokens.at(6).c_str());
        target_pose.position.x = std::stod(tokens.at(0));
        target_pose.position.y = std::stod(tokens.at(1));
        target_pose.position.z = std::stod(tokens.at(2));
        target_pose.orientation.x = std::stod(tokens.at(3));
        target_pose.orientation.y = std::stod(tokens.at(4));
        target_pose.orientation.z = std::stod(tokens.at(5));
        target_pose.orientation.w = std::stod(tokens.at(6));
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

          result = dr_task.moveRobotPose(target_pose, move_group);

          // Capture error moving the robot
          /*if (!result)
          {
            poses.result = false;
            ROS_ERROR("Error moving robot");
          }*/

          ROS_INFO("Getting pose");
          dr_task_result = dr_task.getEEPoses(move_group, dr_task.kinematic_model);

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
          target_joint.resize(n_dof);
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
  };// End DRTrainingTask class

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "dr_training_task");
  
  ros::NodeHandle nh;

  std::string target_pose;
  std::string robot_model;
  std::string planning_group;
  int robot_n_dof;
  std::string ee_name;

  if (argc != 6)
  {
      ROS_ERROR("Error calling node. Usage: dr_pretraining_task 'target_pose (cartesians and quaternions separated by semocolon)' 'robot_model' 'planning_group' 'robot_n_dof' 'endeffector_name'");
      return 1;
  }
  else
  {
    target_pose = argv[1];
    robot_model = argv[2];
    planning_group = argv[3];
    robot_n_dof = std::stoi(argv[4]);
    ee_name = argv[5];
  }
  DRTrainingTask yumi_dr_task = DRTrainingTask(&nh, target_pose, robot_model, planning_group, robot_n_dof, ee_name);

  ros::MultiThreadedSpinner s(16);

  ros::spin(s);
  
}