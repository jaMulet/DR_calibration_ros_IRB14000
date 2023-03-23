#!/usr/bin/env python

# We load Python stuff first because afterwards it will be removed to avoid error with openCV
import sys
sys.path.append('/usr/lib/python2.7/dist-packages')
import rospy
import rospkg


rospy.logdebug("Start Module Loading...Remove CV ROS-Melodic version due to incompatibilities")
import csv

rospy.logdebug(sys.path)
try:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
except ValueError:
    rospy.logdebug("Its already removed..../opt/ros/melodic/lib/python2.7/dist-packages")
rospy.logdebug(sys.path)

import cv2
import glob
import os
import xml.etree.ElementTree as ET
import shutil

SPLIT_RATIO = 0.8

AUGMENTATION = False
AUGMENTATION_DEBUG = False
AUGMENTATION_PER_IMAGE = 25

rospy.logdebug("Loaded Modules...")



def generate_database(phase, path_to_source_training_package, path_to_database_training_package, file_date):
    
    rospy.loginfo("Start main...")
    
    #---------------------------------
    # START of Directories Setup

    if phase=="pretraining":
        dataset_folder = os.path.join(path_to_source_training_package, "pretraining_dataset_gen/")
    elif phase=="training":
        dataset_folder = os.path.join(path_to_source_training_package, "training_dataset_gen/")
    
    csv_folder_path = os.path.join(path_to_database_training_package, "dataset_gen_csv")
    train_csv_output_file = os.path.join(csv_folder_path, "train.csv")
    validation_csv_output_file = os.path.join(csv_folder_path, "validation.csv")

    if not os.path.exists(dataset_folder):
        rospy.logfatal("Dataset not found: '"+str(dataset_folder)+"'. please launch 'roslaunch dr_launch yumi_dr_pretraining.launch' and/or 'roslaunch dr_launch yumi_dr_training_off.launch'")
        return False
    else:
        rospy.loginfo("Training/Pretraining material path found: " + str(dataset_folder))

    if os.path.exists(csv_folder_path):
        shutil.rmtree(csv_folder_path)
    os.makedirs(csv_folder_path)
    rospy.loginfo("Created folder: " + str(csv_folder_path))

    rospy.loginfo("END of Directories Setup")
    # END OF Directories Setup
    #---------------------------------

    class_names = {}
    k = 0
    output = []

    rospy.loginfo("Retrieving the xml files")
    xml_files = glob.glob("{}/*xml".format(dataset_folder))
    rospy.loginfo("END Retrieving the xml files")

    rospy.loginfo("Reading XML Anotation Files, this could take a while depending on the number of files, please be patient...")
    for i, xml_file in enumerate(xml_files):
        tree = ET.parse(xml_file)

        path = os.path.join(dataset_folder)

        rospy.logdebug("path from XML: "+str(path))

        if (tree.findtext("./robot/joints_pos/q1") == "error"):
            pass
        
        else:
            if phase=="pretraining":

                q1 = float(tree.findtext("./robot/joints_pos/q1"))
                q2 = float(tree.findtext("./robot/joints_pos/q2"))
                q3 = float(tree.findtext("./robot/joints_pos/q3"))
                q4 = float(tree.findtext("./robot/joints_pos/q4"))
                q5 = float(tree.findtext("./robot/joints_pos/q5"))
                q6 = float(tree.findtext("./robot/joints_pos/q6"))
                q7 = float(tree.findtext("./robot/joints_pos/q7"))

                ideal_x_com = float(tree.findtext("./robot/ideal_pose3d/x_com"))
                ideal_y_com = float(tree.findtext("./robot/ideal_pose3d/y_com"))
                ideal_z_com = float(tree.findtext("./robot/ideal_pose3d/z_com"))
                ideal_quat_x = float(tree.findtext("./robot/ideal_pose3d/quat_x"))
                ideal_quat_y = float(tree.findtext("./robot/ideal_pose3d/quat_y"))
                ideal_quat_z = float(tree.findtext("./robot/ideal_pose3d/quat_z"))
                ideal_quat_w = float(tree.findtext("./robot/ideal_pose3d/quat_w"))

                random_x_com = float(tree.findtext("./robot/random_pose3d/x_com"))
                random_y_com = float(tree.findtext("./robot/random_pose3d/y_com"))
                random_z_com = float(tree.findtext("./robot/random_pose3d/z_com"))
                random_quat_x = float(tree.findtext("./robot/random_pose3d/quat_x"))
                random_quat_y = float(tree.findtext("./robot/random_pose3d/quat_y"))
                random_quat_z = float(tree.findtext("./robot/random_pose3d/quat_z"))
                random_quat_w = float(tree.findtext("./robot/random_pose3d/quat_w"))

                class_name = tree.findtext("./robot/name")
                if class_name not in class_names:
                    class_names[class_name] = k
                    k += 1

                output.append((q1, q2, q3, q4, q5, q6, q7, ideal_x_com, ideal_y_com, ideal_z_com, ideal_quat_x, ideal_quat_y, ideal_quat_z, ideal_quat_w, random_x_com, random_y_com, random_z_com, random_quat_x, random_quat_y, random_quat_z, random_quat_w, class_name, class_names[class_name]))
            
            elif phase=="training":
                
                q1 = float(tree.findtext("./robot/joints_pos/q1"))
                q2 = float(tree.findtext("./robot/joints_pos/q2"))
                q3 = float(tree.findtext("./robot/joints_pos/q3"))
                q4 = float(tree.findtext("./robot/joints_pos/q4"))
                q5 = float(tree.findtext("./robot/joints_pos/q5"))
                q6 = float(tree.findtext("./robot/joints_pos/q6"))
                q7 = float(tree.findtext("./robot/joints_pos/q7"))

                x_com = float(tree.findtext("./robot/pose3d/x_com"))
                y_com = float(tree.findtext("./robot/pose3d/y_com"))
                z_com = float(tree.findtext("./robot/pose3d/z_com"))
                quat_x = float(tree.findtext("./robot/pose3d/quat_x"))
                quat_y = float(tree.findtext("./robot/pose3d/quat_y"))
                quat_z = float(tree.findtext("./robot/pose3d/quat_z"))
                quat_w = float(tree.findtext("./robot/pose3d/quat_w"))

                class_name = tree.findtext("./robot/name")
                if class_name not in class_names:
                    class_names[class_name] = k
                    k += 1

                output.append((q1, q2, q3, q4, q5, q6, q7, x_com, y_com, z_com, quat_x, quat_y, quat_z, quat_w, class_name, class_names[class_name]))
 
            #print("{}/{}".format(i, xml_file), end="\r")

    rospy.logdebug(str(output))

    # Get the Number of elements of the same class, in this case all, because we only train one item.
    lengths = []
    i = 0
    last = 0
    rospy.loginfo("Getting Object elements")
    for j, row in enumerate(output):
        if last == row[-1]:
            i += 1
        else:
            rospy.logdebug("class {}: {} images".format(output[j-1][-2], i))
            lengths.append(i)
            i = 1
            last += 1

    lengths.append(i)
    rospy.loginfo("Object elements: "+str(lengths))


    ## START CSV generation
    rospy.loginfo("Starting CSV database Generation...")
    with open(train_csv_output_file, "w") as train, open(validation_csv_output_file, "w") as validate:
        csv_train_writer = csv.writer(train, delimiter=",")
        csv_validate_writer = csv.writer(validate, delimiter=",")

        s = 0
        for c in lengths:
            for i in range(c):
                #print("{}/{}".format(s + 1, sum(lengths)), end="\r")

                if phase=="pretraining":

                    q1, q2, q3, q4, q5, q6, q7, ideal_x_com, ideal_y_com, ideal_z_com, ideal_quat_x, ideal_quat_y, ideal_quat_z, ideal_quat_w, random_x_com, random_y_com, random_z_com, random_quat_x, random_quat_y, random_quat_z, random_quat_w, class_name, class_id = output[s]
                    data_list = [q1, q2, q3, q4, q5, q6, q7, ideal_x_com, ideal_y_com, ideal_z_com, ideal_quat_x, ideal_quat_y, ideal_quat_z, ideal_quat_w, random_x_com, random_y_com, random_z_com, random_quat_x, random_quat_y, random_quat_z, random_quat_w, class_name, class_names[class_name]]

                elif phase=="training":

                    q1, q2, q3, q4, q5, q6, q7, x_com, y_com, z_com, quat_x, quat_y, quat_z, quat_w, class_name, class_id = output[s]
                    data_list = [q1, q2, q3, q4, q5, q6, q7, -1, -1, -1, -1, -1, -1, -1, x_com, y_com, z_com, quat_x, quat_y, quat_z, quat_w, class_name, class_names[class_name]]

                # Decide if it goes to train folder or to validate folder

                if i <= c * SPLIT_RATIO:
                    csv_train_writer.writerow(data_list)
                else:
                    csv_validate_writer.writerow(data_list)

                s += 1

    ## END CSV generation

    rospy.loginfo("\nDone!")

    return True

def main():
    
    rospy.init_node('generate_database_node', anonymous=True, log_level=rospy.INFO)
    rospy.logwarn("Generate Database...START")
    
    if len(sys.argv) < 5:
        rospy.logfatal("usage: 'generate_dataset.py path_to_source_training_material, path_to_output_training_material, phase, file_date'")
    else:
        path_to_source_training_package = sys.argv[1]
        path_to_database_training_package = sys.argv[2]
        phase = sys.argv[3]
        file_date = sys.argv[4]
        
        if path_to_database_training_package == "None":
            rospack = rospkg.RosPack()
            # get the file path for dcnn_training_pkg
            path_to_database_training_package = rospack.get_path('dr_dnn_training')
            rospy.logwarn("NOT Found path_to_database_training_package, getting default:"+str(path_to_database_training_package))
        else:
            rospy.logwarn("Found path_to_database_training_package:"+str(path_to_database_training_package))
        
        rospy.logwarn("Path to Training Original Material:"+str(path_to_source_training_package))
        rospy.logwarn("Path to Training Created Database:"+str(path_to_database_training_package))
        
        generate_database(phase,
                          path_to_source_training_package,
                          path_to_database_training_package,
                          file_date)
        
        rospy.logwarn("Generate Database...END")


if __name__ == "__main__":
    main()
