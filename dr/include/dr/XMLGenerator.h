#ifndef XMLGENERATOR_H_
#define XMLGENERATOR_H_

#include <boost/filesystem.hpp>

#include "tinyxml2.h"

#include <string>
#include <vector>

namespace dr {

class XMLGenerator {
    
        boost::filesystem::path path_out;

    public:
        
        /**
         * @brief The constructor
         */
        XMLGenerator();

        /**
         * @brief Folder initialiser to XML generation
         * @param path_out Path where to create the dataset
         */
        void init(boost::filesystem::path path_out);

        /**
         * @brief The XML file generator
         * @param filename The name of the output file
         * @param pos_joints Robot joint position to save in the XML file
         * @param ideal_pose_3d Ideal robot ee position and orientation (quaternions) to save in the XML file
         * @param random_pose_3d Randomised robot ee position and orientation (quaternions) to save in the XML file
         */
        bool generate(std::string filename, std::vector<double> pos_joints, std::vector<double> ideal_pose_3d, std::vector<double> random_pose_3d);

        /**
         * @brief The XML file generator
         * @param filename The name of the output file
         * @param pos_joints Robot joint position to save in the XML file
         * @param pose_3d Robot ee position and orientation (quaternions) to save in the XML file
         */
        bool generate(std::string filename, std::vector<double> pos_joints, std::vector<double> pose_3d);

    private:

        boost::filesystem::path path_to_base_annotation;
        boost::filesystem::path full_path_out;
};

};//end namespace

#endif // XMLGENERATOR_H_