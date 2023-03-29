#include <ros/ros.h>
#include <ros/package.h>

#include <dr/tinyxml2.h>
#include <dr/random_gen.h>

#include <boost/filesystem.hpp>

#include <std_srvs/SetBool.h>

#include <string>

class URDFRandomiser{
    private:

        // ROS stuff
        ros::ServiceServer random_service;
        
        // Declaration of randomiser variables
        //boost::filesystem::path path_to_pkg;
        //boost::filesystem::path full_template_file_path;
    
        // Robot DOFs
        int n_dof;

        // Randomization variables
        double min_error;
        double max_error;

    public:

        boost::filesystem::path full_template_file_path;

        /**************************************************************************
         * Constructor
         **************************************************************************/
        URDFRandomiser(ros::NodeHandle *nh, std::string random_urdf, int n_dof, std::string range){

            random_service = nh->advertiseService("/randomiser", &URDFRandomiser::callback_urdf_randomiser, this);

            boost::filesystem::path path_to_pkg(ros::package::getPath("dr_description"));
            boost::filesystem::path launch_path("/launch");
            boost::filesystem::path file_path(random_urdf);
            full_template_file_path = path_to_pkg / launch_path / file_path;

            if (boost::filesystem::exists(full_template_file_path))
            {
                ROS_WARN("Path to template file: %s", full_template_file_path.c_str());
            }
            else
            {
                ROS_ERROR("Template file NOT found");
            }

            n_dof = n_dof;

            min_error = - std::stod(range);
            max_error = std::stod(range);
        }

        /**************************************************************************
         * index callback
         **************************************************************************/
        bool callback_urdf_randomiser(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

            
            if (req.data)
            {
                //ROS_WARN("Path to template file (in callback): %s", full_template_file_path.c_str());

                //Read XML file
                FILE * XML_template_file;
                XML_template_file = fopen(full_template_file_path.c_str(), "rb");

                tinyxml2::XMLDocument xmlDoc_template;
                tinyxml2::XMLError eResult = xmlDoc_template.LoadFile(full_template_file_path.c_str());
                //ROS_WARN("Load file result: %i. NOTE: success is %i value", eResult, tinyxml2::XML_SUCCESS);    

                if (eResult == tinyxml2::XML_SUCCESS)
                {
                    dr::Randomiser randomiser(min_error, max_error);

                    for (int i = 1; i <= n_dof; i++)
                    {
                        // Randomise values
                        double val = randomiser();
                        //ROS_INFO("Random value: %f", val);

                        // Change values
                        std::string joint_name = "joint_" + std::to_string(i) + "R";
                        findXMLElement(xmlDoc_template, "arg", joint_name, val);

                    }
                    
                    // Save file
                    tinyxml2::XMLError eResult = xmlDoc_template.SaveFile(full_template_file_path.c_str());

                    if (fclose(XML_template_file) == 0 && eResult == tinyxml2::XML_SUCCESS)
                    {
                        res.success = true;
                        res.message ="XML file created";
                        return true;
                    }
                    else
                    {
                        ROS_ERROR("Error creating XML file");
                        res.message ="XML creation failed";
                        return false;
                    }
                }
                else
                {
                    res.message ="XML creation failed";
                    return false;
                }
            }
        }

        /**************************************************************************
         * find XML element function
         **************************************************************************/
        void findXMLElement(tinyxml2::XMLDocument& doc, std::string element_value, std::string attribute_name, double new_attribute_value){
            if (doc.ErrorID() != tinyxml2::XML_SUCCESS)
            {
                ROS_ERROR("XML file parsing failed");

            }
            
            tinyxml2::XMLNode * Elem = doc.FirstChild();
            while(Elem)
            {
                if (Elem->Value() && !std::string(Elem->Value()).compare(element_value))
                {
                    modifyXMLAttribute(Elem, attribute_name, new_attribute_value);
                    //ROS_WARN("Find one element with element value: %s in line %i", std::string(Elem->Value()).c_str(), Elem->GetLineNum());
                }

                if (Elem->FirstChildElement())// 1st level
                    Elem = Elem->FirstChildElement();
                else if (Elem->NextSiblingElement()) // 2nd level
                    Elem = Elem->NextSiblingElement();
                else
                {
                    while(Elem->Parent() && !Elem->Parent()->NextSiblingElement())// 3rd level
                        Elem = Elem->Parent();
                    if (Elem->Parent() && Elem->Parent()->NextSiblingElement())
                        Elem = Elem->Parent()->NextSiblingElement();
                    else
                        break;
                }
            }
        }

        /**************************************************************************
         * modify XML attribute function
         **************************************************************************/
        void modifyXMLAttribute(tinyxml2::XMLNode* elem, std::string attribute_name, double new_attribute_value){
            
            tinyxml2::XMLElement* ElemCopy = elem->ToElement();
            const tinyxml2::XMLAttribute* Attribute = ElemCopy->FindAttribute("default");

            //ROS_INFO("Element name: %s, with attribute name: %s , and attribute default value: %s", ElemCopy->Name(), ElemCopy->Attribute("name"), ElemCopy->Attribute("default"));
            
            if (ElemCopy->Attribute("name") == attribute_name)
            {
                ElemCopy->SetAttribute("default", new_attribute_value);
                //ROS_INFO("This element with attribute name: %s , changed attribute default value to: %s", ElemCopy->Attribute("name"), ElemCopy->Attribute("default"));
            }
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "randomiser");
    ros::NodeHandle nh;
    
    std::string random_urdf;
    int n_dof;
    std::string range_random;

    if (argc != 4)
    {
        ROS_ERROR("Error calling node. Usage: randomiser_URDF 'random_URDF_file' 'robot_num_dof' 'range_random'");
        return 1;
    }
    else
    {
        random_urdf = argv[1];
        n_dof = std::stoi(argv[2]);
        range_random = argv[3];
    }

    URDFRandomiser nc = URDFRandomiser(&nh, random_urdf, n_dof, range_random);

    ros::spin();
}
