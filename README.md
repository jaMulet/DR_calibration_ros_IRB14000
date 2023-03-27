# A ROS-based Domain Randomization (DR) calibration approach for the ABB IRB14000 (YuMi)

## Overview

This repository contains the files and ROS wrappers of the initial approach to test the Domain Randomization (DR) concept for the calibration of a ABB IRB14000 (YuMi) robot.

The approach is divided on two stages to automatically create the datasets:

1. *Synthetic data generation*. Creates a dataset using the Gazebo simulator and the randomised model.
2. *Real data generation*. Creates a dataset using the real robot. This phase controls the robot using the *Robot Web Services* (RWS) and *External Guided Motion* (EGM) interfaces of the [ABB robot driver][1] (refer to the specific repository for the installation instructions). The position and orientation of the robot is provided using the [3d grid board Aruco][2] library and a vision system.

The repository consists following parts:
* *DR data generation*. Generates the synthetic and real data, and stores it as XML files. The `dr_launch` package contains the launch files for the synthetic and real data generation ('yumi_dr_pretraining.launch' and 'yumi_dr_training_off.launch', respectively. See below for more details). The URDF model is randomised following a normal distribution and saved in the 'dr_description' package.
* *Model training*. Pretrains/trains the [Gated Recurrent Unit (GRU)][3] model. The `dr_dnn_training` package contains the files required to generate the dataset (CSV files) and train the model.
* *Robot controller*. Controls the robot using MoveIt to perform the task movements. The `yumi_moveit_config` package utilizes the [Trac-IK kinematic solver][4] for MoveIt (refer to the [MoveIt tutorial][5] for its installation) rather than the default KDL kinematic solver (see the `/yumi_moveit_config/config` folder).

## Usage

### Synthetic data generation

Creates the synthetic data for the pretraining in XML files.

     roslaunch dr_launch yumi_dr_pretraining.launch number_of_elements:=xx path_to_dataset_gen:=/dataset_path

### Real data generation

Creates the real data for the pretraining in XML files.

     roslaunch dr_launch yumi_dr_training_off.launch number_of_elements:=xx path_to_dataset_gen:=/dataset_path vision_sys_topic:=/topic

### Training

Creation of the dataset in CSV files format (80% training data and 20% test data, by default).

     roslaunch dr_dnn_training generate_dataset.launch path_to_source_training_material:=/path_XML_files path_to_output_training_material:=/path_to_CSV_creation phase:=/pretraining_or_training

Training of the model.

     roslaunch dr_dnn_training train_model.launch path_to_database_training_package:=/path_dataset weights_file_name:=/weights_filename training_name:=/output_filename

## ROS API

### Topics

* /result_dr_task

Outputs the results of the main task during the synthetic data generation as:

      Type: dr_msgs/PretrainingPoses

Or during the data generation using the real robot:

      Type: dr_msgs::TrainingPoses

These results are stored in the XML files.

* /iteration

Shares the number of iteration during the dataset generation process.

      Type: std_msgs::UInt32

* /vision_pose

Inputs the robot cartesian position and orientation obtained by the vision system.

      Type: geometry_msgs::TransformStamped

### Messages

* dr_msgs/PretrainingPoses

      Header header
      bool result
      float64[] joints_pos
      geometry_msgs/Pose ideal_pose
      geometry_msgs/Pose random_pose

* dr_msgs/TrainingPoses

      Header header
      bool result
      float64[] joints_pos
      geometry_msgs/Pose pose

## Disclaimer

This repository contains packages under development to calibrate the robot kinematic model and control a robot arm. Any use of this repository in environments with a real robot must be performed under strict caution to avoid damages to the robot itself and the operators.

## Ackownledgement

This work has been developed in the context of the H2020-MSCA-ITN [DiManD project][6] funded by the European Union’s Horizon
2020 research and innovation programme under grant agreement no.814078.

<img align="center" src="https://raw.githubusercontent.com/jaMulet/DR_calibration_ros_IRB14000/melodic/images/dimand_MSCA-ITN.png"/>


[1]: https://github.com/ros-industrial/abb_robot_driver "ROS-In ABB robot driver"

[2]: https://github.com/jaMulet/3dgridboard_aruco_ros "3d Grid Board Aruco repository"

[3]: https://en.wikipedia.org/wiki/Gated_recurrent_unit "Wikipedia - Gated Recurrent Unit model"

[4]: https://traclabs.com/projects/trac-ik/ "Traclabs - Trac-IK"

[5]: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html "MoveIt tutorials - Trac-IK kinematic solver"

[6]: https://dimanditn.eu/es/ "DiManD project"