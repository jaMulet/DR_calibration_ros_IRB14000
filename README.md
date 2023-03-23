A ROS-based Domain Randomization (DR) calibration approach for the ABB IRB14000 (YuMi)
=========

# OVERVIEW

This repository contains the files and ROS wrappers of the initial approach to test the Domain Randomization (DR) concept for the calibration of a ABB IRB14000 (YuMi) robot.

The approach is divided on two stages to create the datasets:

1. Synthetic data generation. Creates a dataset using the Gazebo simulator and the randomised model.
2. Real data generation. Creates a dataset using the real robot. This phase controls the robot using the *Robot Web Services* (RWS) and *External Guided Motion* (EGM) interfaces of the [ABB robot driver][1] (refer to the specific repository for the installation instructions).

The repository consists following parts:
* DR data generation.
* DR training.
* Robot description.

# ROS API

## Launch files

### Synthetic data generation

Creates the synthetic data for the pretraining in XML files.

     roslaunch dr_launch yumi_dr_pretraining.launch number_of_elements:=xx path_to_dataset_gen:=/dataset_path

### Real data generation

Creates the real data for the pretraining in XML files.

     roslaunch dr_launch yumi_dr_training_off.launch number_of_elements:=xx path_to_dataset_gen:=/dataset_path vision_sys_topic:=/topic

### Training

Creation of the dataset in CSV files (80% training data, 20% test data).

     roslaunch dr_dnn_training generate_dataset.launch path_to_source_training_material:=/path_XML_files path_to_output_training_material:=/path_to_CSV_creation phase:=/pretraining_or_training

Training of the model.

     roslaunch dr_dnn_training train_model.launch path_to_database_training_package:= /path_dataset retraining_file_name:=/filename_to_retrain training_name:=/output_filename

# Ackownledgement

This work has been developed in the context of the H2020-MSCA-ITN [DiManD project][3] funded by the European Union’s Horizon
2020 research and innovation programme under grant agreement no.814078.

<img align="center" src="https://raw.githubusercontent.com/jaMulet/DR_calibration_ros_IRB14000/melodic/images/dimand_MSCA-ITN.png"/>


[1]: https://github.com/ros-industrial/abb_robot_driver "ROS-In ABB robot driver"

[3]: https://dimanditn.eu/es/the-project "DiManD project"