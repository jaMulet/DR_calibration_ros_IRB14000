#include "XMLGenerator.h"

#include <ros/ros.h>

#include <ros/package.h>
#include <boost/filesystem.hpp>

#include <stdio.h>
#include "tinyxml2.h"

#include <string>
#include <vector>
#include <map>

using namespace tinyxml2;

namespace dr {
   
    /***********************************************************************************************************************
     * Class definitions: XMLGenerator
     */
    /***********************************************************************************************************************
     * Constructor
     **************************************************************************/
    XMLGenerator::XMLGenerator(){}
    
    void XMLGenerator::init(boost::filesystem::path path_out) {
        
        // Path to XML template
        boost::filesystem::path path_to_package(ros::package::getPath("dr"));
        boost::filesystem::path scripts("template");
        boost::filesystem::path base_file_name("base_annotation.xml");

        boost::filesystem::path path_to_scripts = path_to_package / scripts;
        path_to_base_annotation = path_to_scripts / base_file_name;
        ROS_WARN("Path to base annotation file: %s", path_to_base_annotation.c_str());
        
        if (boost::filesystem::exists(path_to_base_annotation))
        {
            ROS_WARN("Base annotation file exists");
        }
        else
        {
            ROS_ERROR("Base annotation file NOT found");
        }

        // Path to output XML files
        full_path_out = path_out;

        if (boost::filesystem::exists(full_path_out))
        {
            boost::filesystem::remove_all(full_path_out);
            ROS_WARN("Deleted existing directory to store new files");

        }

        boost::filesystem::create_directory(full_path_out);
        ROS_WARN("Created new directory in: %s", full_path_out.c_str());

    }

    /**************************************************************************
     * generate method (pretraining phase)
     **************************************************************************/
    bool XMLGenerator::generate(std::string filename, std::vector<double> pos_joints, std::vector<double> ideal_pose_3d, std::vector<double> random_pose_3d)
    {

        if (boost::filesystem::exists(path_to_base_annotation))
        {
            FILE * XML_annotation_file;
            XML_annotation_file = fopen(path_to_base_annotation.c_str(), "rb");
            
            XMLDocument xmlDoc_base;
            XMLError eResult = xmlDoc_base.LoadFile(path_to_base_annotation.c_str());
            //ROS_WARN("Load file result: %i. NOTE: success is %i value", eResult, XML_SUCCESS);    

            if (eResult == XML_SUCCESS)
            {
                
                XMLDocument xmlDoc;
                xmlDoc_base.DeepCopy(&xmlDoc);

                // The 'root' node
                XMLElement * pRoot = xmlDoc.FirstChildElement("annotation");

                // Insert new 'robot' element
                XMLElement * pElement = xmlDoc.NewElement("robot");
                pRoot->InsertEndChild(pElement);

                // Insert new 'robot' element
                XMLElement * pElement1 = xmlDoc.NewElement("name");
                pElement1->SetText("robot_model");
                pElement->InsertEndChild(pElement1);
                
                //--- JOINT POS
                XMLElement * pElement2 = xmlDoc.NewElement("joints_pos");
                    
                XMLElement * pListElement2_1 = xmlDoc.NewElement("q1");
                pListElement2_1->SetText(pos_joints.at(0));
                pElement2->InsertEndChild(pListElement2_1);
                XMLElement * pListElement2_2 = xmlDoc.NewElement("q2");
                pListElement2_2->SetText(pos_joints.at(1));
                pElement2->InsertEndChild(pListElement2_2);
                XMLElement * pListElement2_3 = xmlDoc.NewElement("q3");
                pListElement2_3->SetText(pos_joints.at(2));
                pElement2->InsertEndChild(pListElement2_3);
                XMLElement * pListElement2_4 = xmlDoc.NewElement("q4");
                pListElement2_4->SetText(pos_joints.at(3));
                pElement2->InsertEndChild(pListElement2_4);
                XMLElement * pListElement2_5 = xmlDoc.NewElement("q5");
                pListElement2_5->SetText(pos_joints.at(4));
                pElement2->InsertEndChild(pListElement2_5);
                XMLElement * pListElement2_6 = xmlDoc.NewElement("q6");
                pListElement2_6->SetText(pos_joints.at(5));
                pElement2->InsertEndChild(pListElement2_6);
                XMLElement * pListElement2_7 = xmlDoc.NewElement("q7");
                pListElement2_7->SetText(pos_joints.at(6));
                pElement2->InsertEndChild(pListElement2_7);
                pElement->InsertEndChild(pElement2);
                
                //--- IDEAL POS3D
                XMLElement * pElement3 = xmlDoc.NewElement("ideal_pose3d");
                XMLElement * pListElement3_1 = xmlDoc.NewElement("x_com");
                pListElement3_1->SetText(ideal_pose_3d.at(0));
                pElement3->InsertEndChild(pListElement3_1);
                XMLElement * pListElement3_2 = xmlDoc.NewElement("y_com");
                pListElement3_2->SetText(ideal_pose_3d.at(1));
                pElement3->InsertEndChild(pListElement3_2);
                XMLElement * pListElement3_3 = xmlDoc.NewElement("z_com");
                pListElement3_3->SetText(ideal_pose_3d.at(2));
                pElement3->InsertEndChild(pListElement3_3);
                XMLElement * pListElement3_4 = xmlDoc.NewElement("quat_x");
                pListElement3_4->SetText(ideal_pose_3d.at(3));
                pElement3->InsertEndChild(pListElement3_4);
                XMLElement * pListElement3_5 = xmlDoc.NewElement("quat_y");
                pListElement3_5->SetText(ideal_pose_3d.at(4));
                pElement3->InsertEndChild(pListElement3_5);
                XMLElement * pListElement3_6 = xmlDoc.NewElement("quat_z");
                pListElement3_6->SetText(ideal_pose_3d.at(5));
                pElement3->InsertEndChild(pListElement3_6);
                XMLElement * pListElement3_7 = xmlDoc.NewElement("quat_w");
                pListElement3_7->SetText(ideal_pose_3d.at(6));
                pElement3->InsertEndChild(pListElement3_7);
                pElement->InsertEndChild(pElement3);
                
                //--- RANDOM POS3D
                XMLElement * pElement4 = xmlDoc.NewElement("random_pose3d");
                XMLElement * pListElement4_1 = xmlDoc.NewElement("x_com");
                pListElement4_1->SetText(random_pose_3d.at(0));
                pElement4->InsertEndChild(pListElement4_1);
                XMLElement * pListElement4_2 = xmlDoc.NewElement("y_com");
                pListElement4_2->SetText(random_pose_3d.at(1));
                pElement4->InsertEndChild(pListElement4_2);
                XMLElement * pListElement4_3 = xmlDoc.NewElement("z_com");
                pListElement4_3->SetText(random_pose_3d.at(2));
                pElement4->InsertEndChild(pListElement4_3);
                XMLElement * pListElement4_4 = xmlDoc.NewElement("quat_x");
                pListElement4_4->SetText(random_pose_3d.at(3));
                pElement4->InsertEndChild(pListElement4_4);
                XMLElement * pListElement4_5 = xmlDoc.NewElement("quat_y");
                pListElement4_5->SetText(random_pose_3d.at(4));
                pElement4->InsertEndChild(pListElement4_5);
                XMLElement * pListElement4_6 = xmlDoc.NewElement("quat_z");
                pListElement4_6->SetText(random_pose_3d.at(5));
                pElement4->InsertEndChild(pListElement4_6);
                XMLElement * pListElement4_7 = xmlDoc.NewElement("quat_w");
                pListElement4_7->SetText(random_pose_3d.at(6));
                pElement4->InsertEndChild(pListElement4_7);
                pElement->InsertEndChild(pElement4);             

                boost::filesystem::path file_path_out(full_path_out / filename);
                XMLError eResult = xmlDoc.SaveFile(file_path_out.c_str());

            }
            else
            {
                ROS_ERROR("Error creating XML file");
                return false;
            }

            fclose(XML_annotation_file);
        }
        
        ROS_WARN("Process finished");
        return true;
    }

    /**************************************************************************
     * generate method (training phase)
     **************************************************************************/
    bool XMLGenerator::generate(std::string filename, std::vector<double> pos_joints, std::vector<double> pose_3d)
    {

        if (boost::filesystem::exists(path_to_base_annotation))
        {
            FILE * XML_annotation_file;
            XML_annotation_file = fopen(path_to_base_annotation.c_str(), "rb");
            
            XMLDocument xmlDoc_base;
            XMLError eResult = xmlDoc_base.LoadFile(path_to_base_annotation.c_str());
            //ROS_WARN("Load file result: %i. NOTE: success is %i value", eResult, XML_SUCCESS);    

            if (eResult == XML_SUCCESS)
            {
                
                XMLDocument xmlDoc;
                xmlDoc_base.DeepCopy(&xmlDoc);

                XMLElement * pRoot = xmlDoc.NewElement("robot");
                xmlDoc.InsertEndChild(pRoot);
                
                XMLElement * pElement1 = xmlDoc.NewElement("name");
                pElement1->SetText("robot_model");
                pRoot->InsertEndChild(pElement1);
                
                //--- JOINT POS
                XMLElement * pElement2 = xmlDoc.NewElement("joint_pos");
                    
                XMLElement * pListElement1_1 = xmlDoc.NewElement("q1");
                pListElement1_1->SetText(pos_joints.at(0));
                pElement2->InsertEndChild(pListElement1_1);
                XMLElement * pListElement1_2 = xmlDoc.NewElement("q2");
                pListElement1_2->SetText(pos_joints.at(1));
                pElement2->InsertEndChild(pListElement1_2);
                XMLElement * pListElement1_3 = xmlDoc.NewElement("q3");
                pListElement1_3->SetText(pos_joints.at(2));
                pElement2->InsertEndChild(pListElement1_3);
                XMLElement * pListElement1_4 = xmlDoc.NewElement("q4");
                pListElement1_4->SetText(pos_joints.at(3));
                pElement2->InsertEndChild(pListElement1_4);
                XMLElement * pListElement1_5 = xmlDoc.NewElement("q5");
                pListElement1_5->SetText(pos_joints.at(4));
                pElement2->InsertEndChild(pListElement1_5);
                XMLElement * pListElement1_6 = xmlDoc.NewElement("q6");
                pListElement1_6->SetText(pos_joints.at(5));
                pElement2->InsertEndChild(pListElement1_6);
                XMLElement * pListElement1_7 = xmlDoc.NewElement("q7");
                pListElement1_7->SetText(pos_joints.at(6));
                pElement2->InsertEndChild(pListElement1_7);
                pRoot->InsertEndChild(pElement2);


                //--- POS3D
                XMLElement * pElement3 = xmlDoc.NewElement("pos3d");
                XMLElement * pListElement2_1 = xmlDoc.NewElement("x_com");
                pListElement2_1->SetText(pose_3d.at(0));
                pElement3->InsertEndChild(pListElement2_1);
                XMLElement * pListElement2_2 = xmlDoc.NewElement("y_com");
                pListElement2_2->SetText(pose_3d.at(1));
                pElement3->InsertEndChild(pListElement2_2);
                XMLElement * pListElement2_3 = xmlDoc.NewElement("z_com");
                pListElement2_3->SetText(pose_3d.at(2));
                pElement3->InsertEndChild(pListElement2_3);
                XMLElement * pListElement2_4 = xmlDoc.NewElement("quat_x");
                pListElement2_4->SetText(pose_3d.at(3));
                pElement3->InsertEndChild(pListElement2_4);
                XMLElement * pListElement2_5 = xmlDoc.NewElement("quat_y");
                pListElement2_5->SetText(pose_3d.at(4));
                pElement3->InsertEndChild(pListElement2_5);
                XMLElement * pListElement2_6 = xmlDoc.NewElement("quat_z");
                pListElement2_6->SetText(pose_3d.at(5));
                pElement3->InsertEndChild(pListElement2_6);
                XMLElement * pListElement2_7 = xmlDoc.NewElement("quat_w");
                pListElement2_7->SetText(pose_3d.at(6));
                pElement3->InsertEndChild(pListElement2_7);
                pRoot->InsertEndChild(pElement3);
                
                boost::filesystem::path file_path_out(full_path_out / filename);
                XMLError eResult = xmlDoc.SaveFile(file_path_out.c_str());

            }
            else
            {
                ROS_ERROR("Error creating XML file");
                return false;
            }

            fclose(XML_annotation_file);
        }
        
        ROS_WARN("Process finished");
        return true;
    }

}; // end dr namespace