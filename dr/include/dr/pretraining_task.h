#ifndef PRETRAINING_TASK_H_
#define PRETRAINING_TASK_H_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/Pose.h>

#include <ros/time.h>
#include <vector>

namespace dr {

class DRPretrainingTask{

    public:

        /**
         * @brief The PretrainingTaskResult struct collects all pretraining task results (robot joint values, ideal pose and randomised ones)
         *
         */
        struct PretrainingTaskResult{
            std::vector<double> joint_values;
            geometry_msgs::Pose ideal_pose;
            geometry_msgs::Pose random_pose;
        };
        
        std::string ideal_robot_model;
        std::string random_robot_model;
        std::string current_ee_name;

        /**
         * @brief The constructor
         * 
         */
        DRPretrainingTask();
        
        /**
         * @brief The DR task method
         * @param planning_group The planning group which executes the movement with
         * @param target_pose Cartesian (position and orientation) target
         */
        DRPretrainingTask::PretrainingTaskResult task(std::string planning_group, geometry_msgs::Pose target_pose);       

    private:
    
        int t_timeout;
        float t_short_sleep;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result;
        moveit::planning_interface::MoveItErrorCode move_result;
        bool move_success;

};// end DRPretrainingTask class

};//end namespace

#endif // PRETRAINING_TASK_H_