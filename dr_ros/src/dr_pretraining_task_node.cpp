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

// DR application
#include <dr/pretraining_task.h>

  /***********************************************************************************************************************
   * Class definitions: YumiDRTask
   **********************************************************************************************************************/
  class YumiDRTask {
    private:
      ros::Publisher drtask_pub;
      ros::ServiceServer drtask_srv;
      dr_msgs::PretrainingPoses poses;

      dr::DRPretrainingTask dr_task;
      dr::DRPretrainingTask::PretrainingTaskResult dr_task_result;

      std::string planning_group;

    public:

      /**************************************************************************
       * Constructor
       **************************************************************************/
      YumiDRTask (ros::NodeHandle *nh, std::string ideal_robot_model, std::string random_robot_model, std::string ee_name) {

        drtask_pub = nh->advertise<dr_msgs::PretrainingPoses>("/result_dr_task", 10);
        drtask_srv = nh->advertiseService("/dr_task", &YumiDRTask::callback_drtask, this);

        dr_task_result.joint_values.clear();dr_task_result.joint_values.resize(7);
        poses.joints_pos.clear();poses.joints_pos.resize(7);

        //-----------------------
        // Robot configuration
        //-----------------------
        dr_task.ideal_robot_model = ideal_robot_model;// Ideal robot description (used in ros_control)
        dr_task.random_robot_model = random_robot_model;// Randomised robot description (used in the sim)
        dr_task.current_ee_name = ee_name;// Link to which apply the calibration (e.g. last link, tool, etc)

        planning_group = "rob_r";
      }

      /**************************************************************************
       * DRTask callback
       **************************************************************************/
      bool callback_drtask(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

        //-----------------------
        // Target pose (end-effector in robot frame)
        //-----------------------
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 0.20445;
        target_pose.position.y = -0.32462;
        target_pose.position.z = 0.35699;
        target_pose.orientation.x = -0.08378;
        target_pose.orientation.y = 0.76356;
        target_pose.orientation.z = 0.092061;
        target_pose.orientation.w = 0.63363;

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

        for (int i = 0; i < 7; i++){
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

  std::string ideal_robot_model;
  std::string random_robot_model;
  std::string ee_name;

  if (argc < 4)
  {
      ROS_ERROR("Error calling node. Usage: dr_pretraining_task 'ideal_robot_model' 'random_robot_model' 'endeffector_name'");
  }
  else
  {
    ideal_robot_model = argv[1];
    random_robot_model = argv[2];
    ee_name = argv[3];
  }

  YumiDRTask yumi_dr_task = YumiDRTask(&nh, ideal_robot_model, random_robot_model, ee_name);

  ros::MultiThreadedSpinner s(16);

  ros::spin(s);
  
}