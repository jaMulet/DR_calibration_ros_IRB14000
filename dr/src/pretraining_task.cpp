#include "pretraining_task.h"

#include <ros/ros.h>
#include <ros/time.h>

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

#include <geometry_msgs/Pose.h>


namespace dr {

    /***********************************************************************************************************************
     * Class definitions: DRPretrainingTask
     */
    /***********************************************************************************************************************
     * Constructor
     **************************************************************************/
    DRPretrainingTask::DRPretrainingTask(){
        
        t_short_sleep = 1;
        t_timeout = 1.0;

    }

    /**************************************************************************
     * PretrainingTask method
     **************************************************************************/
    DRPretrainingTask::PretrainingTaskResult DRPretrainingTask::task(std::string planning_group, geometry_msgs::Pose target_pose){
    
        DRPretrainingTask::PretrainingTaskResult task_result;
        task_result.joint_values.clear();
        task_result.joint_values.resize(7);

        //-----------------------
        // Robot model loader
        //-----------------------
        // Load robot model: ideal and randomised.

        // Ideal model
        robot_model_loader::RobotModelLoader ideal_robot_model_loader(ideal_robot_model);
        robot_model::RobotModelPtr ideal_kinematic_model = ideal_robot_model_loader.getModel();

        if (ideal_kinematic_model->isEmpty()) {
            ROS_INFO("Error creating robot model (ideal)");
            
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        ROS_INFO("Model frame (ideal model): %s", ideal_kinematic_model->getModelFrame().c_str());

        // randomised model
        robot_model_loader::RobotModelLoader random_robot_model_loader(random_robot_model);
        robot_model::RobotModelPtr randomised_kinematic_model = random_robot_model_loader.getModel();

        if (randomised_kinematic_model->isEmpty()){
            ROS_INFO("Error creating robot model (randomised)");

            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        ROS_INFO("Model frame (random model): %s", randomised_kinematic_model->getModelFrame().c_str());

        //-----------------------
        // Planning group
        //-----------------------
        // Create planning group, which is equal for both models.
        moveit::planning_interface::MoveGroupInterface move_group(planning_group);
        
        ROS_INFO("Planning group created: %s", planning_group.c_str());

        //-----------------------
        // Robot state
        //-----------------------
        // Ideal model
        robot_state::RobotStatePtr ideal_kinematic_state(new robot_state::RobotState(ideal_kinematic_model));
        const robot_state::JointModelGroup* ideal_joint_model_group = ideal_kinematic_model->getJointModelGroup(planning_group);

        ROS_INFO("Robot model (ideal joint group) for group %s created.", planning_group.c_str());

        // randomised model
        robot_state::RobotStatePtr random_kinematic_state(new robot_state::RobotState(randomised_kinematic_model));
        const robot_state::JointModelGroup* random_joint_model_group = randomised_kinematic_model->getJointModelGroup(planning_group);
        
        ROS_INFO("Robot model (random joint group) for group %s created.", planning_group.c_str());

        // Create robot state (joint names)
        ideal_kinematic_state->setToDefaultValues();

        //const std::vector<std::string>& joint_names = ideal_joint_model_group->getVariableNames();
        
        // Retreive the current set of joint values
        /*std::vector<double> joint_values;
        ideal_kinematic_state->copyJointGroupPositions(ideal_joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }*/

        //===========================================================================================================
        // TASK BEGINS
        //===========================================================================================================
        //-----------------------
        // Plan the movement
        //-----------------------

        // Goal position
        move_group.setPoseTarget(target_pose);

        // Initial position
        move_group.setStartStateToCurrentState();
        ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to get correctly current state.

        // Call the planner whithin a timeout
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout (t_timeout);

        while (ros::Time::now() - start_time < timeout){
            plan_result = move_group.plan(my_plan);
        }

        // Plan error handling:
        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        //bool plan_success = (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //ROS_INFO("Visualizing plan (pose goal) %s", plan_success ? "" : "FAILED");

        //===========================================================================================================
        //-----------------------
        // Move the robot
        //-----------------------
        
        move_result = move_group.execute(my_plan);

        // Move error handling:
        if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
                
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        move_success = (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO("Executing movement %s", move_success ? "" : "FAILED");

        //===========================================================================================================
        //-----------------------
        // Get EE pose and print
        //-----------------------
        //    1. Ideal model
        //-----------------------

        // Get ideal joint values
        std::vector<double> ideal_joint_values = move_group.getCurrentJointValues();
        
        // Compute FK to find EE with ideal model
        ideal_kinematic_state->setJointGroupPositions(ideal_joint_model_group, ideal_joint_values); 
        const Eigen::Isometry3d& ideal_endeffector_state = ideal_kinematic_state->getGlobalLinkTransform(current_ee_name);

        // Convert Eigen::Isometry to Pose ideal message:
        geometry_msgs::Pose ideal_pose;
        tf::poseEigenToMsg(ideal_endeffector_state, ideal_pose);

        //-----------------------
        //    2. Randomised model
        //-----------------------

        // Compute FK to find EE with randomised model
        random_kinematic_state->setJointGroupPositions(random_joint_model_group, ideal_joint_values); 
        const Eigen::Isometry3d& random_endeffector_state = random_kinematic_state->getGlobalLinkTransform(current_ee_name);

        // Convert Eigen::Isometry to Pose random message:
        geometry_msgs::Pose random_pose;
        tf::poseEigenToMsg(random_endeffector_state, random_pose);

        //===========================================================================================================
        //-----------------------
        // Return results
        //-----------------------
        
        for (int i = 0; i < 7; i++){
            task_result.joint_values.at(i) = ideal_joint_values.at(i);
        }
        task_result.ideal_pose = ideal_pose;
        task_result.random_pose = random_pose;

        return task_result;
    }

}; // end dr namespace