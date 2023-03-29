#include <ros/ros.h>
#include <ros/time.h>
#include <tuple>

// ROS messages
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <dr_msgs/PretrainingPoses.h>
#include <eigen_conversions/eigen_msg.h>

// String
#include <string>
#include <vector>

// DR application
#include <dr/pretraining_task.h>
#include <dr/target_manipulator.h>

  /***********************************************************************************************************************
   * Class definitions: YumiDRTask
   **********************************************************************************************************************/
  class DRTask {
    private:
      ros::Publisher drtask_pub;
      ros::ServiceServer drtask_srv;
      dr_msgs::PretrainingPoses poses;

      std::string planning_group;

      int n_dof;

      geometry_msgs::Pose target_pose;

      dr::DRPretrainingTask dr_task;
      dr::DRPretrainingTask::PretrainingTaskResult dr_task_result;
      dr::TargetManipulator target_task;

    public:

      /**************************************************************************
       * Constructor
       **************************************************************************/
      DRTask (ros::NodeHandle *nh, std::string task_target_pose, std::string ideal_robot_model, std::string random_robot_model, std::string planning_group_name, int robot_n_dof, std::string ee_name) {

        drtask_pub = nh->advertise<dr_msgs::PretrainingPoses>("/result_dr_task", 10);
        drtask_srv = nh->advertiseService("/dr_task", &DRTask::callback_drtask, this);

        // Robot configuration
        dr_task.ideal_robot_model = ideal_robot_model;// Ideal robot description (used in ros_control)
        dr_task.random_robot_model = random_robot_model;// Randomised robot description (used in the sim)
        dr_task.current_ee_name = ee_name;// Link to which apply the calibration (e.g. last link, tool, etc)

        planning_group = planning_group_name;

        // Robot number of DOFs
        n_dof = robot_n_dof;
        dr_task.n_dof = robot_n_dof;

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

        // Results
        dr_task_result.joint_values.clear();dr_task_result.joint_values.resize(n_dof);
        poses.joints_pos.clear();poses.joints_pos.resize(n_dof);
      }

      /**************************************************************************
       * DRTask callback
       **************************************************************************/
      bool callback_drtask(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

        //-----------------------
        // DR task execution
        //-----------------------
        dr_task_result = dr_task.task(planning_group, target_pose);
    
        //-----------------------
        // Evaluate and publish results
        //-----------------------
        if (dr_task_result.joint_values.at(0) == -1)
        {
          poses.result = false;
        }
        else
        {
          poses.result = true;
        }

        for (int i = 0; i < n_dof; i++)
        {
          poses.joints_pos.at(i) = dr_task_result.joint_values.at(i);
        }
        poses.ideal_pose = dr_task_result.ideal_pose;
        poses.random_pose = dr_task_result.random_pose;

        drtask_pub.publish(poses);
    
        res.success = true;
        return true;
      }
  };

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "dr_pretraining_task");
  
  ros::NodeHandle nh;

  std::string target_pose;
  std::string ideal_robot_model;
  std::string random_robot_model;
  std::string planning_group;
  int robot_n_dof;
  std::string ee_name;

  if (argc != 7)
  {
      ROS_ERROR("Error calling node. Usage: dr_pretraining_task 'target_pose (cartesians and quaternions separated by semocolon)' 'ideal_robot_model' 'random_robot_model' 'planning_group' 'robot_n_dof' 'endeffector_name'");
      return 1;
  }
  else
  {
    target_pose = argv[1];
    ideal_robot_model = argv[2];
    random_robot_model = argv[3];
    planning_group = argv[4];
    robot_n_dof = std::stoi(argv[5]);
    ee_name = argv[6];
  }

  DRTask yumi_dr_task = DRTask(&nh, target_pose, ideal_robot_model, random_robot_model, planning_group, robot_n_dof, ee_name);

  ros::MultiThreadedSpinner s(16);

  ros::spin(s);
  
}