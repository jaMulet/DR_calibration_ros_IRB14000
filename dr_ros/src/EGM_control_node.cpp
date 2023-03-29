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

#include <eigen_conversions/eigen_msg.h>

// Gazebo API
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/Empty.h>

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

// ROS messages

#include <abb_robot_msgs/TriggerWithResultCode.h>
#include <abb_rapid_sm_addin_msgs/EGMSettings.h>
#include <abb_rapid_sm_addin_msgs/GetEGMSettings.h>
#include <abb_rapid_sm_addin_msgs/SetEGMSettings.h>
#include <controller_manager_msgs/SwitchController.h>

  /***********************************************************************************************************************
   * Class definitions: YumiDRTrainingTask
   **********************************************************************************************************************/
  class EGMControl {
    private:
      ros::ServiceServer start_EGM_control_srv;
      ros::ServiceServer stop_EGM_control_srv;
      ros::ServiceClient start_RAPID_client;
      ros::ServiceClient stop_RAPID_client;
      ros::ServiceClient reset_pp_RAPID_client;
      ros::ServiceClient start_EGM_client;
      ros::ServiceClient stop_EGM_client;
      ros::ServiceClient start_controller_EGM_client;
      ros::ServiceClient get_EGM_settings_client;
      ros::ServiceClient set_EGM_settings_client;
      
      //bool result = false;
      std::string controller;
      std::string task;

      float t_short_sleep;
      float t_long_sleep;

    public:
      /**************************************************************************
       * Constructor
       **************************************************************************/
      EGMControl (ros::NodeHandle *nh, std::string controller_name, std::string robot_task) {
        start_EGM_control_srv = nh->advertiseService("/start_egm_control", &EGMControl::callback_start_RAPID_EGM_control, this);
        stop_EGM_control_srv = nh->advertiseService("/stop_egm_control", &EGMControl::callback_stop_EGM_RAPID_control, this);
        
        start_RAPID_client = nh->serviceClient<abb_robot_msgs::TriggerWithResultCode>("/rws/start_rapid");
        stop_RAPID_client = nh->serviceClient<abb_robot_msgs::TriggerWithResultCode>("/rws/stop_rapid");
        reset_pp_RAPID_client = nh->serviceClient<abb_robot_msgs::TriggerWithResultCode>("/rws/pp_to_main");
        start_EGM_client = nh->serviceClient<abb_robot_msgs::TriggerWithResultCode>("/rws/sm_addin/start_egm_joint");
        stop_EGM_client = nh->serviceClient<abb_robot_msgs::TriggerWithResultCode>("/rws/sm_addin/stop_egm");
        start_controller_EGM_client = nh->serviceClient<controller_manager_msgs::SwitchController>("/egm/controller_manager/switch_controller");
        get_EGM_settings_client = nh->serviceClient<abb_rapid_sm_addin_msgs::GetEGMSettings>("/rws/sm_addin/get_egm_settings");
        set_EGM_settings_client = nh->serviceClient<abb_rapid_sm_addin_msgs::SetEGMSettings>("/rws/sm_addin/set_egm_settings");

        controller = controller_name;
        task = robot_task;

        t_short_sleep = 1.0;
        t_long_sleep = 2.0;
      }
      /**************************************************************************
       * starts EGM and RAPID callback
       **************************************************************************/    
      bool callback_start_RAPID_EGM_control(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

        if (req.data)
        {
          //-----------------------
          // Reset RAPID pointers
          //-----------------------
          abb_robot_msgs::TriggerWithResultCode reset_pp_RAPID_srv;

          if (reset_pp_RAPID_client.call(reset_pp_RAPID_srv))
          {
            if (reset_pp_RAPID_srv.response.result_code == 1)
            {
              ROS_INFO("Reset RAPID pointers result: %i", reset_pp_RAPID_srv.response.result_code);
            }
            else
            {
                ROS_WARN("RAPID pointers cannot be resset. Result code: %i", reset_pp_RAPID_srv.response.result_code);
            }
          }
          else
          {
            ROS_ERROR("Failed to call service 'reset_pp_RAPID': %s", reset_pp_RAPID_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'reset_pp_to_main'. Error: " + reset_pp_RAPID_srv.response.message;
            return false;
          }
          
          //-----------------------
          // Start RAPID
          //-----------------------
          abb_robot_msgs::TriggerWithResultCode start_RAPID_srv;

          if (start_RAPID_client.call(start_RAPID_srv))
          {
            ROS_INFO("Start RAPID result: %i", start_RAPID_srv.response.result_code);
          }
          else
          {
            ROS_ERROR("Failed to call service 'start_RAPID': %s", start_RAPID_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'start_RAPID'. Error: " + start_RAPID_srv.response.message;
            return false;
          }
          //-----------------------
          ROS_WARN("Sleeping for %f sec...", t_short_sleep);
          ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to correctly start RAPID. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          //-----------------------
          // Start EGM
          //-----------------------
          abb_robot_msgs::TriggerWithResultCode start_EGM_srv;

          if (start_EGM_client.call(start_EGM_srv))
          {
            ROS_INFO("'Start EGM' result: %i", start_EGM_srv.response.result_code);
          }
          else
          {
            ROS_ERROR("Failed to call service 'start_EGM': %s", start_EGM_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'start_EGM'. Error: " + start_EGM_srv.response.message;
            return false;
          }

          //-----------------------
          ROS_WARN("Sleeping for 1 sec...");
          ros::Duration(1).sleep(); // sleep for 1s. Time required to correctly start EGM. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          //-----------------------
          // Change EGM settings
          //-----------------------
          abb_rapid_sm_addin_msgs::GetEGMSettings in_EGM_settings_srv;
          abb_rapid_sm_addin_msgs::SetEGMSettings out_EGM_settings_srv;

          in_EGM_settings_srv.request.task = task;

          if (get_EGM_settings_client.call(in_EGM_settings_srv))
          {        
            ROS_INFO("Get 'EGM settings' result: %i", in_EGM_settings_srv.response.result_code);

            if (in_EGM_settings_srv.response.result_code == 1)
            {
              ROS_INFO("EGM settings obtained");
              abb_rapid_sm_addin_msgs::EGMSettings settings = in_EGM_settings_srv.response.settings;

              // Apply changes to current settings
              settings.activate.max_speed_deviation = 400 * 180 / M_PI;// max_speed_deviation is in deg/s, we convert from rad/s

              // Send settings back to EGM controller
              out_EGM_settings_srv.request.task = task;
              out_EGM_settings_srv.request.settings = settings;

              if (set_EGM_settings_client.call(out_EGM_settings_srv))
              {
                ROS_INFO("Set 'EGM settings' result: %i", out_EGM_settings_srv.response.result_code);

                if (out_EGM_settings_srv.response.result_code == 1)
                {
                  ROS_INFO("EGM settings changed");
                }
                else
                {
                  ROS_WARN("EGM settings NOT set. Error: %s", out_EGM_settings_srv.response.message.c_str());
                }
              }
              else
              {
                ROS_INFO("Error calling 'EGM settings': %s", out_EGM_settings_srv.response.message.c_str());
              }
            }
            else
            {
              ROS_WARN("EGM settings NOT received. Error: %s", in_EGM_settings_srv.response.message.c_str());
            }
          }
          else
          {
            ROS_ERROR("Imposible to call 'EGM settings' service: %s", in_EGM_settings_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'EGM_settings'. Error: "+ in_EGM_settings_srv.response.message;
            return false;
          }

          //-----------------------
          ROS_WARN("Sleeping for %f sec...", t_short_sleep);
          ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to correctly set parameters. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          //-----------------------
          // Start controllers over EGM
          //-----------------------
          controller_manager_msgs::SwitchController start_controller_srv;

          start_controller_srv.request.start_controllers = {controller};
          start_controller_srv.request.stop_controllers = {""};
          start_controller_srv.request.strictness = 1;
          start_controller_srv.request.start_asap = true;
          start_controller_srv.request.timeout = 0.0;

          if (start_controller_EGM_client.call(start_controller_srv))
          {
            ROS_INFO("Switch controller result: %i", start_controller_srv.response.ok);
            if (start_controller_srv.response.ok != 1)
            {
              ROS_ERROR("Failed to switch controllers");
              res.success = false;
              res.message = "Failed to switch controllers";
              return false;
            }
          }
          else
          {
            ROS_ERROR("Failed to call service switch_controller");
            return false;
          }

          //-----------------------
          ROS_WARN("Sleeping for %f sec...", t_short_sleep);
          ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to correctly start controllers. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          res.success = true;
          return true;
        }
      }
      /**************************************************************************
       * stop EGM and RAPID callback
       **************************************************************************/    
      bool callback_stop_EGM_RAPID_control(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

        if (req.data)
        {

          //-----------------------
          // Stop EGM
          //-----------------------
          abb_robot_msgs::TriggerWithResultCode stop_EGM_srv;

          if (stop_EGM_client.call(stop_EGM_srv))
          {
            ROS_INFO("Stop EGM result: %i", stop_EGM_srv.response.result_code);
          }
          else
          {
            ROS_ERROR("Failed to call service stop_EGM: %s", stop_EGM_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'stop_EGM'. Error: " + stop_EGM_srv.response.message;
            return false;
          }
          
          //-----------------------
          ROS_WARN("Sleeping for %f sec...", t_long_sleep);
          ros::Duration(t_long_sleep).sleep(); // sleep for 1s. Time required to get correctly current state. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          //-----------------------
          // Stop RAPID
          //-----------------------
          abb_robot_msgs::TriggerWithResultCode stop_RAPID_srv;

          if (stop_RAPID_client.call(stop_RAPID_srv))
          {
            ROS_INFO("Stop RAPID result: %i", stop_RAPID_srv.response.result_code);
          }
          else
          {
            ROS_ERROR("Failed to call service 'stop_RAPID': %s", stop_RAPID_srv.response.message.c_str());
            res.success = false;
            res.message = "Failed to call service 'stop_RAPID'. Error: " + stop_RAPID_srv.response.message;
            return false;
          }

          //-----------------------
          ROS_WARN("Sleeping for %f sec...", t_long_sleep);
          ros::Duration(t_long_sleep).sleep(); // sleep for 1s. Time required to get correctly current state. CHECK IF THIS DELAY IS NEEDED!
          //-----------------------

          res.success = true;
          return true;
        }
      }
  };// End EGMControl class

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv)
{
  ros::init(argc, argv, "EGM_control");
  
  ros::NodeHandle nh;

  std::string controller_name;
  std::string robot_task;

  if (argc != 3)
  {
      ROS_ERROR("Error calling node. Usage: EGM_control 'controller_name' 'robot_task'");
      return 1;
  }
  else
  {
    controller_name = argv[1];
    robot_task = argv[2];
  }

  EGMControl yumi_EGM_control = EGMControl(&nh, controller_name, robot_task);

  ros::spin();
  
}