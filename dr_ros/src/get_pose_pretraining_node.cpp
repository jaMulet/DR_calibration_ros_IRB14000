#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <string>
#include <vector>

#include <iomanip>
#include <ctime>
#include <sstream>

#include <std_msgs/UInt32.h>

#include <dr_msgs/PretrainingPoses.h>
#include <dr/XMLGenerator.h>
#include <dr/tinyxml2.h>

    /***********************************************************************************************************************
     * Class definitions: GetPosePretraining
     **********************************************************************************************************************/
    class GetPosePretraining {

        private:
            ros::Subscriber pose_subscriber;
            ros::Subscriber index_subscriber;

            int maxobjs = 1;
            
            // Declarations poses variables
            std::vector<double> pos_joints;
            std::vector<double> ideal_3dpose;
            std::vector<double> random_3dpose;

            // Declarations dataset path
            boost::filesystem::path path_out;
            
            // Declarations file name
            std::string str_date;
            std::string str_index = "0";

            dr::XMLGenerator xml_generator;

        public:
        /**************************************************************************
         * Constructor
        **************************************************************************/
        GetPosePretraining(ros::NodeHandle *nh, std::string path_to_dataset_gen) {
            pose_subscriber = nh->subscribe("/result_dr_task", 1000, &GetPosePretraining::callback_pose, this);
            index_subscriber = nh->subscribe("/iteration", 1000, &GetPosePretraining::callback_index, this);

            //ROS_INFO("Result's subscribers alive");

            // Current time for filename
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y_%m_%d_%H:%M_");
            str_date = oss.str();

            pos_joints.clear(); pos_joints.resize(7);
            ideal_3dpose.clear(); ideal_3dpose.resize(7);
            random_3dpose.clear(); random_3dpose.resize(7);

            // Path for dataset generation
            path_out = path_to_dataset_gen + "pretraining_dataset_gen/";

            xml_generator.init(path_out);
        }

        /**************************************************************************
         * task index callback
         **************************************************************************/
        void callback_index(const std_msgs::UInt32 msg) {
            //str_index = std::to_string(msg.data);
            str_index = std::to_string(msg.data);
        }

        /**************************************************************************
         * pose callback
         **************************************************************************/
        void callback_pose(const dr_msgs::PretrainingPoses msg) {

            // Complete filename info
            std::string filename = "test_";
            filename.append(str_date);
            filename.append(str_index);
            filename.append(".xml");

            // Obtain poses from message
            if (msg.result)
            {
                pos_joints.at(0) = msg.joints_pos[0];
                pos_joints.at(1) = msg.joints_pos[1];
                pos_joints.at(2) = msg.joints_pos[2];
                pos_joints.at(3) = msg.joints_pos[3];
                pos_joints.at(4) = msg.joints_pos[4];
                pos_joints.at(5) = msg.joints_pos[5];
                pos_joints.at(6) = msg.joints_pos[6];
                
                ideal_3dpose.at(0) = msg.ideal_pose.position.x;
                ideal_3dpose.at(1) = msg.ideal_pose.position.y;
                ideal_3dpose.at(2) = msg.ideal_pose.position.z;
                ideal_3dpose.at(3) = msg.ideal_pose.orientation.x;
                ideal_3dpose.at(4) = msg.ideal_pose.orientation.y;
                ideal_3dpose.at(5) = msg.ideal_pose.orientation.z;
                ideal_3dpose.at(6) = msg.ideal_pose.orientation.w;

                random_3dpose.at(0) = msg.random_pose.position.x;
                random_3dpose.at(1) = msg.random_pose.position.y;
                random_3dpose.at(2) = msg.random_pose.position.z;
                random_3dpose.at(3) = msg.random_pose.orientation.x;
                random_3dpose.at(4) = msg.random_pose.orientation.y;
                random_3dpose.at(5) = msg.random_pose.orientation.z;
                random_3dpose.at(6) = msg.random_pose.orientation.w;

                ROS_WARN("File created: %s", filename.c_str());
            }
            else
            {
                pos_joints.at(0) = -1;
                pos_joints.at(1) = -1;
                pos_joints.at(2) = -1;
                pos_joints.at(3) = -1;
                pos_joints.at(4) = -1;
                pos_joints.at(5) = -1;
                pos_joints.at(6) = -1;
                
                ideal_3dpose.at(0) = -1;
                ideal_3dpose.at(1) = -1;
                ideal_3dpose.at(2) = -1;
                ideal_3dpose.at(3) = -1;
                ideal_3dpose.at(4) = -1;
                ideal_3dpose.at(5) = -1;
                ideal_3dpose.at(6) = -1;

                random_3dpose.at(0) = -1;
                random_3dpose.at(1) = -1;
                random_3dpose.at(2) = -1;
                random_3dpose.at(3) = -1;
                random_3dpose.at(4) = -1;
                random_3dpose.at(5) = -1;
                random_3dpose.at(6) = -1;

                ROS_ERROR("File %s created with dr_task execution ERROR", filename.c_str());
            }
        
            // Write poses in XML file and saves it
            xml_generator.generate(filename, pos_joints, ideal_3dpose, random_3dpose);
        }
    };

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv)
{   
    ros::init(argc, argv, "get_pose");
    ros::NodeHandle nh;
    
    std::string path_to_dataset_gen;
    
    if (argc > 1)
    {
        path_to_dataset_gen = argv[1];
    }

    GetPosePretraining nc = GetPosePretraining(&nh, path_to_dataset_gen);
    
    ros::spin();
}