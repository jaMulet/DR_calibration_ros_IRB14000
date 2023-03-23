#!/usr/bin/env python

import roslaunch
import rospy
import rospkg
import os
import sys

from std_msgs.msg import UInt32

from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest

#----------------------------
# Training (offline) dataset creation 
#----------------------------
# Definition of steps
#----------------------------
# 1. Load necesary stuff
#   1.1. Load model in parameter server.
#   1.2. Load controller
#   1.3. Spawn controller
#   1.4. Load move_group
# 2. Start EGM control and specific controllers
# 3. Perform task: Move robot and publish positions
# 4. Gather and save results
# 5. Finish (clean)
#----------------------------
# ITERANTE FROM STEP 3 TO 4
#----------------------------

class TrainingDataSetGen:
    def __init__(self):
        self._index = UInt32()
        self.index_publisher = rospy.Publisher("/iteration", UInt32, queue_size=1)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.egm_control_call = rospy.ServiceProxy("/start_egm_control", SetBool)
        self.egm_control_request = SetBoolRequest()
        self.egm_control_result = False
        
        self.dr_task_call = rospy.ServiceProxy("/dr_task", SetBool)
        self.dr_task_request = SetBoolRequest()
        self.dr_task_result = False

        self.get_pose_call = rospy.ServiceProxy("/get_pose", SetBool)
        self.get_pose_request = SetBoolRequest()
        self.get_pose_result = False

        self.stop_egm_control_call = rospy.ServiceProxy("/stop_egm_control", SetBool)
        self.stop_egm_control_request = SetBoolRequest()
        self.stop_egm_control_result = False

        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path('dr_ros')
        
    def dataset_generator(self, number_of_dataset_elements, path_to_dataset_gen="./dataset_gen/"):

        #------------------------------------------------------
        # 1. Load necesary stuff (using .launch)
        #------------------------------------------------------
        #   2.1. Load model in parameter server
        #   2.2. Spawn model
        #   2.3. Load controller
        #   2.3. Spawn controller
        #   2.4. Load move_group
        #   2.5. 'DR_training_task' service node
        #------------------------------------------------------

        rospy.loginfo("Loading stuff")

        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self._pkg_path + "/launch/yumi_dr_training_task.launch"])
        launch.start()
        rospy.logwarn("Task set")
        
        rospy.logwarn("Sleep for 3 sec")
        rospy.sleep(3)
        rospy.logwarn("Returning from 3 sec sleep")

        #------------------------------------------------------
        # 2. Start EGM controller
        #------------------------------------------------------
        rospy.loginfo("Waiting 'start_EGM_control' service")
        rospy.wait_for_service('/start_egm_control')

        self.egm_control_request = True

        try:
            rospy.logwarn("Calling 'start_EGM_control'...")
            self.egm_control_result = self.egm_control_call(self.egm_control_request)
        except rospy.ServiceException as e:
            rospy.logwarn("'start_EGM_control' service CALL failed: "+str(e))
        
        if not (self.egm_control_result):
            rospy.logerr("'start_EGM_control' service failed")
            return False
        else:
            rospy.logwarn("'start_EGM_control' performed")
            #self.dr_task_request = True
        

        #------------------------------------------------------
        # Iterative processes
        #------------------------------------------------------
        for i in range(number_of_dataset_elements):
            self._index.data = i
            self.index_publisher.publish(self._index) #publish n. iteration (i)
            rospy.logwarn("Starting iteration n. %i", self._index.data)
               
            #--------------------------------------------------
            # 3. Task: Move robot
            #--------------------------------------------------
            rospy.loginfo("Waiting 'dr_task' service")
            rospy.wait_for_service('/dr_task')

            rospy.loginfo("Performing 'dr_task'")
            self.dr_task_request = True

            try:
                rospy.logwarn("Calling 'DR_task'...")
                self.dr_task_result = self.dr_task_call(self.dr_task_request)
            except rospy.ServiceException as e:
                rospy.logwarn("'DR_task' service CALL failed: "+str(e))
            
            if not (self.dr_task_result):
                rospy.logerr("'DR_task' service failed")
            else: 
                rospy.logwarn("'DR_task' performedd")
         
            #--------------------------------------------------
            # 4. Get positions and save results
            #--------------------------------------------------
            rospy.loginfo("Waiting 'get_pose' service")
            rospy.wait_for_service('/get_pose')

            rospy.loginfo("Saving pose")
            self.get_pose_request = True

            try:
                self.get_pose_result = self.get_pose_call(self.get_pose_request)
                rospy.logwarn("Poses saved in XML file")
            except rospy.ServiceException as e:
                rospy.logwarn("'Get_pose' service CALL failed: "+str(e))
            
            if not (self.get_pose_result):
                rospy.logerr("'Get_pose' service failed")

            self.dr_task_request = False
            self.get_pose_request = False

        #------------------------------------------------------
        # 5. Finish (clean)
        #------------------------------------------------------

        # Stop EGM control
        rospy.loginfo("Waiting 'stop_EGM_control' service")
        rospy.wait_for_service('/yumi/stop_egm_control')

        rospy.loginfo("Starting 'stop_EGM_control'")
        self.stop_egm_control_request = True

        try:
            self.stop_egm_control_result = self.stop_egm_control_call(self.stop_egm_control_request)
            rospy.logwarn("'stop_EGM_control' performed")
        except rospy.ServiceException as e:
            rospy.logwarn("'stop_EGM_control' service CALL failed: "+str(e))
        
        if not (self.egm_control_result):
            rospy.logerr("'stop_EGM_control' service failed")
            return False
            
        # Clean roslaunch
        launch.shutdown()

        if (self.egm_control_result and self.dr_task_result and self.get_pose_result and self.stop_egm_control_result):
            rospy.logwarn("Task %i finished with success", self._index.data)
        else:
            rospy.logerr("Task %i finished with ERRORS", self._index.data)

        self.egm_control_request = False
        self.stop_egm_control_request = False


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


    dataset_gen = TrainingDataSetGen()
    dataset_gen.dataset_generator(number_of_dataset_elements=number_of_elements,
                                  path_to_dataset_gen=full_path_to_dataset_gen)

    rospy.logwarn("DR program ENDED")