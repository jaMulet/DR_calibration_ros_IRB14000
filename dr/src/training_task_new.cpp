#include "training_task.h"

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
     * Class definitions: DRTrainingTask
     */
    /***********************************************************************************************************************
     * Constructor
     **************************************************************************/
    DRTrainingTask::DRTrainingTask(){
        
        t_short_sleep = 1;
        t_long_sleep = 1;
        t_timeout = 1.0;

        ros::Duration timeout (t_timeout);

        //-----------------------
        // Robot model loader
        //-----------------------

        robot_model_loader::RobotModelLoader robot_model_loader(robot_model);
        kinematic_model = robot_model_loader.getModel();

        /*if (kinematic_model->isEmpty()) {
            ROS_INFO("Error creating robot model");
            
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }*/

        ROS_INFO("Model frame : %s", kinematic_model->getModelFrame().c_str());

        //-----------------------
        // Planning group
        //-----------------------
        // Create planning group, which is equal for both models.
        moveit::planning_interface::MoveGroupInterface move_group(planning_group);
        
        ROS_INFO("Planning group created: %s", planning_group.c_str());

        //-----------------------
        // Robot state
        //-----------------------
        kinematic_state(new robot_state::RobotState(kinematic_model));
        joint_model_group = kinematic_model->getJointModelGroup(planning_group);
        //const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);// **************

        ROS_INFO("Robot model for group %s created.", planning_group.c_str());

        // Create robot state (joint names)
        kinematic_state->setToDefaultValues();
        

    }

    /**************************************************************************
     * TrainingTask method
     **************************************************************************/
    DRTrainingTask::TrainingTaskResult DRTrainingTask::task(std::string planning_group, geometry_msgs::Pose target_pose, std::vector<double> target_joint){

        DRTrainingTask::TrainingTaskResult task_result;
        task_result.joint_values.clear();
        task_result.joint_values.resize(7);

        //-----------------------
        // Robot model loader
        //-----------------------

        /*robot_model_loader::RobotModelLoader robot_model_loader(robot_model);
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();*/

        if (kinematic_model->isEmpty()) {
            ROS_INFO("Error creating robot model");
            
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        /*ROS_INFO("Model frame : %s", kinematic_model->getModelFrame().c_str());

        //-----------------------
        // Planning group
        //-----------------------
        // Create planning group, which is equal for both models.
        moveit::planning_interface::MoveGroupInterface move_group(planning_group);
        
        ROS_INFO("Planning group created: %s", planning_group.c_str());

        //-----------------------
        // Robot state
        //-----------------------
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        //const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);// **************

        ROS_INFO("Robot model for group %s created.", planning_group.c_str());

        // Create robot state (joint names)
        kinematic_state->setToDefaultValues();*/

        //const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();*/

        //===========================================================================================================
        // TASK BEGINS
        //===========================================================================================================
        //-----------------------
        // Plan movement
        //-----------------------

        // Set goal pose
        move_group.setPoseTarget(target_pose);
        ROS_INFO("'target_pose' set");

        // Set initial position
        move_group.setStartStateToCurrentState();
        ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to get correctly current state.

        // Call the planner (within a timeout)
        ros::Time task_start_time = ros::Time::now();
        //ros::Duration timeout (t_timeout);// ******************

        while (ros::Time::now() - task_start_time < timeout){
            plan_result = move_group.plan(my_plan);
        }

        // Plan error handling
        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Error planning movement");
        }
        
        ROS_INFO("Plan set");

        //-----------------------
        // Move robot
        //-----------------------
        // NOTE: start_EGM_joint moves by joint trajectory, wich is not expected by MoveIt.
        //       Then it always throws a PATH_TOLERANCE_ERROR.

        ROS_INFO("Starting execution of movement");

        move_result = move_group.execute(my_plan);

        //-----------------------
        ROS_WARN("Sleeping for 5 sec to perform the cartesian move...");
        ros::Duration(t_long_sleep).sleep(); // sleep for 1s. Time required to get current state. CHECK IF THIS DELAY IS NEEDED!
        //-----------------------

        // Move error handling
        if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){     
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        move_success = (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);


        //-----------------------
        // Get joint pose
        //-----------------------
        ROS_INFO("Getting pose for joint group %s.", planning_group.c_str());
        //const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);// **************

        // EE name
        ROS_INFO("EE name: %s", current_ee_name.c_str());

        std::vector<double> joint_values = move_group.getCurrentJointValues();

        // Compute FK to find EE with robot model
        ROS_INFO("FK for EE model");
        //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values); 
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(current_ee_name);

        // Convert Eigen::Isometry to Pose message:
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);

        task_result.joint_values = joint_values;
        task_result.pose = pose;

        //===========================================================================================================
        // RETURN HOME POS
        //===========================================================================================================
        //-----------------------
        // Plan movement
        //-----------------------
        
        move_group.setJointValueTarget(target_joint);

        // Set initial position
        move_group.setStartStateToCurrentState();
        ros::Duration(t_short_sleep).sleep(); // sleep for 1s. Time required to get correctly current state.

        ros::Time homepos_start_time = ros::Time::now();
        //ros::Duration timeout (t_timeout);//**************************

        while (ros::Time::now() - homepos_start_time < timeout){
            plan_result = move_group.plan(my_plan);
        }

        // Plan error handling
        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;        }

        //-----------------------
        // Move robot
        //-----------------------
        // NOTE: start_EGM_joint moves by joint trajectory, wich is not expected by MoveIt.
        //       Then it always throws a PATH_TOLERANCE_ERROR.
            
        move_result = move_group.execute(my_plan);
        
        //-----------------------
        ROS_WARN("Sleeping for 5 sec to perform the joint move...");
        ros::Duration(t_long_sleep).sleep(); // sleep for 1s. Time required to get current state. CHECK IF THIS DELAY IS NEEDED!
        //-----------------------

        // Move error handling
        if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){     
            for (int i = 0; i < 7; i++){
                task_result.joint_values.at(i) = -1;
            }
            return task_result;
        }

        bool move_success = (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO("Executing movement %s", move_success ? "" : "FAILED");
    }


    /**************************************************************************
     * setRobot method
     **************************************************************************/
    /*bool DRTrainingTask::setRobot(robot_model::RobotModelPtr& kinematic_model){

        //-----------------------
        // Robot model loader
        //-----------------------

        robot_model_loader::RobotModelLoader robot_model_loader(robot_model);
        kinematic_model = robot_model_loader.getModel();

        if (kinematic_model->isEmpty()) {
            ROS_INFO("Error creating robot model");
        }

        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        //-----------------------
        // Robot state
        //-----------------------
        robot_state::RobotStatePtr new_kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state = new_kinematic_state;
        const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);

        ROS_INFO("Robot model (joint group) for group %s created.", planning_group.c_str());

        // Create robot state (joint names)
        kinematic_state->setToDefaultValues();

        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        
        // Retreive the current set of joint values (names)
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        // to prevent errors, evaluate planning group and robot state creation, then return TRUE
        return true;
    }*/

    /**************************************************************************
     * moveRobotPose
     **************************************************************************/
    /*bool DRTrainingTask::moveRobotPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface& move_group){

        DRTrainingTask::TrainingTaskResult result;
        
        //-----------------------
        // Plan movement
        //-----------------------

        // Set goal pose
        move_group.setPoseTarget(target_pose);
        ROS_INFO("'target_pose' set");

        // Set initial position
        move_group.setStartStateToCurrentState();
        ros::Duration(1).sleep(); // sleep for 1s. Time required to get correctly current state.

        // Call the planner (within a timeout)
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result;

        ros::Time start_time = ros::Time::now();
        ros::Duration timeout (1.0);

        while (ros::Time::now() - start_time < timeout){
            plan_result = move_group.plan(my_plan);
        }

        // Plan error handling
        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Error planning movement");
        }
        
        ROS_INFO("Plan set");

        //-----------------------
        // Move robot
        //-----------------------
        // NOTE: start_EGM_joint moves by joint trajectory, wich is not expected by MoveIt.
        //       Then it always throws a PATH_TOLERANCE_ERROR.

        ROS_INFO("Starting execution of movement");

        moveit::planning_interface::MoveItErrorCode move_result = move_group.execute(my_plan);

        //-----------------------
        ROS_WARN("Sleeping for 5 sec to perform the cartesian move...");
        ros::Duration(5).sleep(); // sleep for 1s. Time required to get current state. CHECK IF THIS DELAY IS NEEDED!
        //-----------------------

        // Move error handling
        if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){     
            return false;
        }

        bool move_success = (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO("Executing movement %s", move_success ? "" : "FAILED");

        if (move_success)
        {
            return true;
        }
        else
        {
            return false;
        }

        return true;
    }*/

    /**************************************************************************
     * moveRobotJoint
     **************************************************************************/
    /*bool DRTrainingTask::moveRobotJoint(std::vector<double> target_joint, moveit::planning_interface::MoveGroupInterface& move_group){

        //-----------------------
        // Plan movement
        //-----------------------
        
        move_group.setJointValueTarget(target_joint);

        // Set initial position
        move_group.setStartStateToCurrentState();
        ros::Duration(1).sleep(); // sleep for 1s. Time required to get correctly current state.

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        moveit::planning_interface::MoveItErrorCode plan_result;

        ros::Time start_time = ros::Time::now();
        ros::Duration timeout (1.0);

        while (ros::Time::now() - start_time < timeout){
            plan_result = move_group.plan(my_plan);
        }

        // Plan error handling
        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            return false;
        }

        //-----------------------
        // Move robot
        //-----------------------
        // NOTE: start_EGM_joint moves by joint trajectory, wich is not expected by MoveIt.
        //       Then it always throws a PATH_TOLERANCE_ERROR.
            
        moveit::planning_interface::MoveItErrorCode move_result = move_group.execute(my_plan);
        
        //-----------------------
        ROS_WARN("Sleeping for 5 sec to perform the joint move...");
        ros::Duration(5).sleep(); // sleep for 1s. Time required to get current state. CHECK IF THIS DELAY IS NEEDED!
        //-----------------------

        // Move error handling
        if (move_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){     
            return false;
        }

        bool move_success = (move_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO("Executing movement %s", move_success ? "" : "FAILED");

        if (move_success)
        {
            return true;
        }
        else
        {
            return false;
        }

    }*/

    /**************************************************************************
     * getEEPoses
     **************************************************************************/
    /*DRTrainingTask::TrainingTaskResult DRTrainingTask::getEEPoses(std::string ee_name, moveit::planning_interface::MoveGroupInterface& move_group, robot_model::RobotModelPtr& kinematic_model){

        DRTrainingTask::TrainingTaskResult result;
    
        ROS_INFO("Getting pose for joint group %s.", planning_group.c_str());
        const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);

        // EE name
        const std::string current_ee_name = "yumi_tool0_R";
        ROS_INFO("EE name: %s", current_ee_name.c_str());

        std::vector<double> joint_values = move_group.getCurrentJointValues();

        // Compute FK to find EE with robot model
        ROS_INFO("FK for EE model");
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values); 
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(current_ee_name);

        // Convert Eigen::Isometry to Pose message:
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);

        result.joint_values = joint_values;
        result.pose = pose;

        return result;
    }*/
}; // end dr namespace