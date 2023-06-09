  
// Gazebo API
#include <gazebo_msgs/DeleteModel.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>

class SimGazeboControl {
  private:
    ros::ServiceServer simcontrol_srv;
    ros::ServiceClient deleteModelClient;
    //ros::ServiceClient pauseGazeboClient;
    ros::ServiceClient resetGazeboClient;

    std::string gazebo_model_name;

  public:
    /**************************************************************************
     * Constructor
     **************************************************************************/
    SimGazeboControl (ros::NodeHandle *nh, std::string model_name) {
      simcontrol_srv = nh->advertiseService("/sim_control", &SimGazeboControl::callback_simcontrol, this);
      deleteModelClient = nh->serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
      //pauseGazeboClient = nh->serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
      resetGazeboClient = nh->serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
      
      gazebo_model_name = model_name;

    }
    
    /**************************************************************************
     * simulation control callback
     **************************************************************************/
    bool callback_simcontrol(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        
      if (req.data)
      { 
        //-----------------------
        // Delete model through service
        //-----------------------

        // Checking service availability
        ros::service::waitForService("gazebo/delete_model");

        // Create client for "gazebo/delete_model" service
        gazebo_msgs::DeleteModel deleteModel;

        // Specify model name to delete
        deleteModel.request.model_name = gazebo_model_name;

        // Deleting model and final checking
        if (deleteModelClient.call(deleteModel))
        {
            ROS_INFO_STREAM("Model "<<deleteModel.request.model_name<<" deleted");
        }

        // Create client for "pause physics" service
        // Note: If physics are paused, delays 
        //std_srvs::Empty pauseGazebo;
        //pauseGazeboClient.call(pauseGazebo);

        // Create client for "pause physics" service
        std_srvs::Empty resetGazebo;
        resetGazeboClient.call(resetGazebo);

        return true;
      }
    }
};

/**************************************************************************
 * main function
 **************************************************************************/
int main (int argc, char **argv){
  ros::init(argc, argv, "sim_control");

  ros::NodeHandle nh;

  std::string model_name;

   if (argc != 2)
  {
      ROS_ERROR("Error calling node. Usage: sim_control 'model_name'");
      return 1;
  }
  else
  {
    model_name = argv[1];
  }

  SimGazeboControl sim_control = SimGazeboControl(&nh, model_name);

  ros::spin();

  }
