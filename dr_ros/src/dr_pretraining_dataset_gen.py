#!/usr/bin/env python

import roslaunch
import rospy
import rospkg
import os
import sys

from std_msgs.msg import UInt32

from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

#----------------------------
# Definition of steps
#----------------------------
# 1. Randomize model
# 2. Load necesary stuff
#   2.1. Load model in parameter server (using .launch).
#   2.2. Spawn model
#   2.3. Load controller
#   2.3. Spawn controller
#   2.4. Load move_group
# 3. Task: Move robot + get positions and save results
# 4. Reset simulation
# 5. Clean launch and verify result
# ITERANTE FROM STEP 2 TO 6
#----------------------------

class DataSetGen:
    def __init__(self):
        self._index = UInt32()
        self.index_publisher = rospy.Publisher("/iteration", UInt32, queue_size=1)

        self.urdf_random_call = rospy.ServiceProxy("/random_model", SetBool)
        self.urdf_random_request = SetBoolRequest()
        self.urdf_random_result = False

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.dr_task_call = rospy.ServiceProxy("/dr_task", SetBool)
        self.dr_task_request = SetBoolRequest()
        self.dr_task_result = False

        self.sim_control_call = rospy.ServiceProxy("/sim_control", SetBool)
        self.sim_control_request = SetBoolRequest()
        self.sim_control_result = False

        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path('dr_ros')
        
    def dataset_generator(self, number_of_dataset_elements, path_to_dataset_gen="./dataset_gen/"):

        for i in range(number_of_dataset_elements):
            self._index.data = i
            self.index_publisher.publish(self._index) #publish n. iteration (i)
            rospy.logwarn("Starting iteration n. %i", self._index.data)

            
            #--------------------------------------------------
            # 1. Randomise model
            #--------------------------------------------------
            rospy.loginfo("Waiting randomising model service")
            rospy.wait_for_service("random_model")

            rospy.loginfo("Randomising model")
            self.urdf_random_request = True

            try:
                self.urdf_random_result = self.urdf_random_call(self.urdf_random_request)
                rospy.logwarn("Model randomised")
            except rospy.ServiceException as e:
                rospy.logwarn("Randomiser service CALL failed: "+str(e))
            
            if not (self.urdf_random_result):
                rospy.logerr("Randomiser service failed")

            #--------------------------------------------------
            # 2. Load necesary stuff (using .launch)
            #--------------------------------------------------
            #   2.1. Load model in parameter server
            #   2.2. Spawn model
            #   2.3. Load controller
            #   2.3. Spawn controller
            #   2.4. Load move_group
            #   2.5. DR task service node
            #--------------------------------------------------

            rospy.loginfo("Loading stuff")

            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self._pkg_path + "/launch/yumi_dr_pretraining_task.launch"])
            launch.start()
            rospy.logwarn("Task set")
            
            rospy.logwarn("Sleep for 5 sec")
            rospy.sleep(5)
            rospy.logwarn("Returning from 5 sec sleep")

            #--------------------------------------------------
            # 3. Task: Move robot
            #--------------------------------------------------
            rospy.loginfo("Waiting dr_task service")
            rospy.wait_for_service('dr_task')

            rospy.loginfo("Performing dr task")
            self.dr_task_request = True

            try:
                self.dr_task_result = self.dr_task_call(self.dr_task_request)
                rospy.logwarn("DR task performed")
            except rospy.ServiceException as e:
                rospy.logwarn("DR task service CALL failed: "+str(e))
            
            if not (self.dr_task_result):
                rospy.logerr("DR task service failed")
          
            #--------------------------------------------------
            # 4. Reset simulation
            #--------------------------------------------------
            rospy.loginfo("Waiting sim control service")
            rospy.wait_for_service("sim_control")

            rospy.loginfo("Reseting simulation")
            self.sim_control_request = True

            try:
                self.sim_control_result = self.sim_control_call(self.sim_control_request)
                rospy.logwarn("Simulation reset")
            except rospy.ServiceException as e:
                rospy.logwarn("Sim control service CALL failed: "+str(e))
            
            if not (self.sim_control_result):
                rospy.logerr("Sim control service failed")                

            #--------------------------------------------------
            # 5. Finish (clean)
            #--------------------------------------------------
            # Clean roslaunch
            launch.shutdown()

            if (self.urdf_random_result and self.dr_task_result and self.sim_control_result):
                rospy.logwarn("Task finished with success")
            else:
                rospy.logerr("Task finished with ERROR")

            self.urdf_random_request = False
            self.dr_task_request = False
            self.sim_control_request = False

if __name__ == "__main__":
    rospy.init_node('create_training_material_node', anonymous=True, log_level=rospy.INFO)

    #rospack = rospkg.RosPack(),

    if len(sys.argv) < 3:
        print("Usage: 'number_of_elements_in_dataset' 'path_to_dataset_generation'")
    else:
        number_of_elements = int(sys.argv[1])
        path_to_dataset_gen = sys.argv[2]

    rospy.logwarn("DR program START")

    if path_to_dataset_gen == "None":
        path_to_dataset_gen = rospkg.get_path('dr')
        rospy.logwarn("NO path to dataset_gen, setting current package: "+str(path_to_dataset_gen))
    else:
        rospy.loginfo("Found path to dataset_gen: "+str(path_to_dataset_gen))

    full_path_to_dataset_gen = os.path.join(path_to_dataset_gen, "dataset_gen")

    dataset_gen = DataSetGen()
    dataset_gen.dataset_generator(number_of_dataset_elements=number_of_elements,
                                  path_to_dataset_gen=full_path_to_dataset_gen)


    rospy.logwarn("DR program ENDED")