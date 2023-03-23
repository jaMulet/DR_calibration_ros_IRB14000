#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <string>
#include <vector>

#include <iomanip>
#include <ctime>
#include <sstream>

#include <std_msgs/UInt32.h>
#include <std_srvs/SetBool.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>

#include <dr_msgs/TrainingPoses.h>
#include <dr/XMLGenerator.h>
#include <dr/tinyxml2.h>


class GetPoseTraining {

    private:
        ros::Subscriber aruco_pose_subscriber;
        ros::Subscriber robot_pose_subscriber;
        ros::Subscriber index_subscriber;
        ros::ServiceServer get_pose_service;
        //-----------------------------------
        // Create serviceServer to be called for measuring with camera (like a trigger)
        //-----------------------------------

        int maxobjs = 1;
        
        // Declarations poses variables
        std::vector<double> pos_joints;
        //std::vector<double> 3dpose;
        std::vector<std::vector<double>> v_poses_3d;
        std::vector<double> v_pose_3d_aux;
        std::vector<double> pose_3d;

        // Image filtering
        int max_readings = 20;
        int it = 0;

        // Declarations dataset path
        boost::filesystem::path path_out;
        
        // Declarations filename
        std::string str_date;
        std::string str_index = "0";

        dr::XMLGenerator xml_generator;

    public:
    /**************************************************************************
     * Constructor
     **************************************************************************/
    GetPoseTraining(ros::NodeHandle *nh, std::string path_to_dataset_gen) {
        aruco_pose_subscriber = nh->subscribe("/vision_pose", 1000, &GetPoseTraining::callback_aruco_pose, this);
        robot_pose_subscriber = nh->subscribe("/result_dr_task", 1000, &GetPoseTraining::callback_robot_pose, this);
        index_subscriber = nh->subscribe("/iteration", 1000, &GetPoseTraining::callback_index, this);
        get_pose_service = nh->advertiseService("/get_pose", &GetPoseTraining::callback_pose, this);

        //ROS_INFO("Result's subscribers alive");

        // Current time
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y_%m_%d_%H:%M_");
        str_date = oss.str();

        pos_joints.clear(); pos_joints.resize(7);
        pose_3d.clear(); pose_3d.resize(7);
        v_poses_3d.clear(); v_poses_3d.resize(max_readings);
        v_pose_3d_aux.clear(); v_pose_3d_aux.resize(7);

        // Path for dataset generation
        path_out = path_to_dataset_gen + "training_dataset_gen/";

        xml_generator.init(path_out);

    }

    /**************************************************************************
     * index callback
     **************************************************************************/
    void callback_index(const std_msgs::UInt32 msg) {
        str_index = std::to_string(msg.data);
    }

    /**************************************************************************
     * robot pose callback
     **************************************************************************/
    void callback_robot_pose(const dr_msgs::TrainingPoses msg) {
        
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
            
            ROS_INFO("Robot poses saved");
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
            
            ROS_WARN("Robot poses saved with robot poses error");
        }
    }
            
    /**************************************************************************
     * aruco pose callback
     **************************************************************************/
    void callback_aruco_pose(const geometry_msgs::TransformStamped msg) {

        // *** TO DO ***
        // Transform from ArUco to robot EE 
        /*tf::StampedTransform Taruco;
        tf::transformStampedMsgToTF(msg, Taruco);
        tf::Transform TEE = Taruco * Taruco_EE;*/

        v_pose_3d_aux.at(0) = msg.transform.translation.x;
        v_pose_3d_aux.at(1) = msg.transform.translation.y;
        v_pose_3d_aux.at(2) = msg.transform.translation.z;
        v_pose_3d_aux.at(3) = msg.transform.rotation.x;
        v_pose_3d_aux.at(4) = msg.transform.rotation.y;
        v_pose_3d_aux.at(5) = msg.transform.rotation.z;
        v_pose_3d_aux.at(6) = msg.transform.rotation.w;
        
        v_poses_3d.at(it) = v_pose_3d_aux;
        
        it++;
        
        // Reset if max_reading reached and compute average
        if (it == max_readings)
        {
            for (int i = 0; i < max_readings; i++)
            {
                v_pose_3d_aux.clear();
                v_pose_3d_aux = v_poses_3d.at(i);                    
                
                pose_3d.at(0) += v_pose_3d_aux.at(0);
                pose_3d.at(1) += v_pose_3d_aux.at(1);
                pose_3d.at(2) += v_pose_3d_aux.at(2);
                pose_3d.at(3) += v_pose_3d_aux.at(3);
                pose_3d.at(4) += v_pose_3d_aux.at(4);
                pose_3d.at(5) += v_pose_3d_aux.at(5);
                pose_3d.at(6) += v_pose_3d_aux.at(6);
            }

            pose_3d.at(0) = pose_3d.at(0)/v_poses_3d.size();
            pose_3d.at(1) = pose_3d.at(1)/v_poses_3d.size();
            pose_3d.at(2) = pose_3d.at(2)/v_poses_3d.size();
            pose_3d.at(3) = pose_3d.at(3)/v_poses_3d.size();
            pose_3d.at(4) = pose_3d.at(4)/v_poses_3d.size();
            pose_3d.at(5) = pose_3d.at(5)/v_poses_3d.size();
            pose_3d.at(6) = pose_3d.at(6)/v_poses_3d.size();   

            it = 0;
            ROS_INFO("ArUco pose computed and saved");
        }
    }

    /**************************************************************************
     * DRTask callback
     **************************************************************************/
    bool callback_pose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
            
        if(req.data)
        {
            // Complete filename info
            std::string filename = "test_";
            filename.append(str_date);
            filename.append(str_index);
            filename.append(".xml");
        
            // Write poses in XML file and saves it
            // MODIFY TO ADMIT CHANGES IN XML FILEPATH
            bool success = xml_generator.generate(filename, pos_joints, pose_3d);
            if (success)
            {
                res.success = true;
                res.message = "XML file created";
                ROS_INFO("XML file created: %s", filename.c_str());
                return true;
            }
            else
            {
                res.success = false;
                res.message = "ERROR creating XML file";
                ROS_ERROR("Error creating XML file: %s", filename.c_str());
                return false;
            }
        }
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

    GetPoseTraining nc = GetPoseTraining(&nh, path_to_dataset_gen);
    
    ros::spin();
}