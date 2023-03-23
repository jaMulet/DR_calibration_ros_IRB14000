#ifndef TRAINING_TASK_H_
#define TRAINING_TASK_H_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/Pose.h>

#include <vector>

namespace dr {

class DRTrainingTask{

    public:

        /**
         * @brief The TrainingTaskResult struct collects all training task results (robot joint values and ee robot pose)
         *
         */
        struct TrainingTaskResult{
            std::vector<double> joint_values;
            geometry_msgs::Pose pose;
        };

        std::string planning_group;

        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        std::string robot_model;
        std::string current_ee_name;
        /**
         * Constructor
         */
        DRTrainingTask();
        
        /**
         * @brief Sets the robot
         * @param kinematic_model Kinematic model of the robot
         */
        bool setRobot(robot_model::RobotModelPtr& kinematic_model);

        /**
         * @brief Moves the robot to a cartesian pose
         * @param target_pose Cartesian pose to target
         * @param move_group Robot to move
         */
        bool moveRobotPose(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface& move_group);

       /**
         * @brief Moves the robot to a joint pose
         * @param target_joint Joint pose to target
         * @param move_group Robot to move
         */
        bool moveRobotJoint(std::vector<double> target_joint, moveit::planning_interface::MoveGroupInterface& move_group);

       /**
         * @brief Obtains the robot pose using the forward kinematic model
         * @param ee_name Last link to which obtain the pose
         * @param move_group Robot to move
         * @param kinematic_model Kinematic model of the robot
         */
        DRTrainingTask::TrainingTaskResult getEEPoses(moveit::planning_interface::MoveGroupInterface& move_group, robot_model::RobotModelPtr& kinematic_model);

    private:
        
        int t_timeout;
        float t_short_sleep;
        float t_long_sleep;

};// end DRTrainingTask class

};//end namespace

#endif // TRAINING_TASK_H_