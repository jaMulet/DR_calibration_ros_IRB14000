#!/usr/bin/env python3

# We load Python stuff first because afterwards it will be removed to avoid error with openCV
import sys
sys.path.append('/usr/lib/python3.7/dist-packages')
import rospy
import rospkg
import time
import csv
import math
print("Start Module Loading...Remove CV ROS-Melodic version due to incompatibilities")
#print(sys.path)
try:
    sys.path.remove('/opt/ros/melodic/lib/python3.7/dist-packages')
except ValueError:
    print ("Its already removed..../opt/ros/melodic/lib/python3.7/dist-packages")
#print(sys.path)
import numpy as np
import tensorflow as tf
from keras import Model
from keras.models import Sequential
from keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau, TensorBoard
from keras.layers import Dense, GRU, Conv1D
from keras.utils import Sequence, plot_model

from matplotlib import pyplot as plt

from keras.optimizers import Adam, SGD
import keras.backend as K

import os
import shutil

class DataSequence(Sequence):

    def __init__(self, csv_file, batch_size=32):
           
        self.paths = []
        self.batch_size = batch_size
        self._number_of_elements_to_be_input = 7
        self._number_of_elements_to_be_output = 7

        with open(csv_file, "r") as file:
            """
            Which element do we have interest in?
            [q1, q2, q3, q4, q5, q6, q7, ideal_x_com, ideal_y_com, ideal_z_com, ideal_quat_x, ideal_quat_y, ideal_quat_z, ideal_quat_w random_x_com, random_y_com, random_z_com, random_quat_x, random_quat_y, random_quat_z, random_quat_w]
            We only want the DNN to learn to give us the Q vector from the XYZ 3D position of the robot EE.
            q1
            q2
            q3
            q4
            q5
            q6
            q7
            """
            
            self._length = sum(1 for line in file)

            self.x = np.zeros((self._length, self._number_of_elements_to_be_input))
            self.y = np.zeros((self._length, self._number_of_elements_to_be_output))

            file.seek(0)

            reader = csv.reader(file, delimiter=",")
            for index, (q1, q2, q3, q4, q5, q6, q7, _, _, _, _, _, _, _, rand_x_com, rand_y_com, rand_z_com, rand_qx_com, rand_qy_com, rand_qz_com, rand_qw_com, _, _) in enumerate(reader):
                """
                number_of_elements_to_be_input == 7:
                    self.x[index][0] = rand_x_com
                    self.x[index][1] = rand_y_com
                    self.x[index][2] = rand_z_com
                    self.x[index][3] = rand_qx_com
                    self.x[index][4] = rand_qy_com
                    self.x[index][5] = rand_qz_com
                    self.x[index][6] = rand_qw_com
                number_of_elements_to_be_output == 7:
                    self.y[index][0] = q1
                    self.y[index][1] = q2
                    self.y[index][2] = q3
                    self.y[index][3] = q4
                    self.y[index][4] = q5
                    self.y[index][5] = q6
                    self.y[index][6] = q7
                """
                self.x[index][0] = rand_x_com
                self.x[index][1] = rand_y_com
                self.x[index][2] = rand_z_com
                self.x[index][3] = rand_qx_com
                self.x[index][4] = rand_qy_com
                self.x[index][5] = rand_qz_com
                self.x[index][6] = rand_qw_com

                self.y[index][0] = q1
                self.y[index][1] = q2
                self.y[index][2] = q3
                self.y[index][3] = q4
                self.y[index][4] = q5
                self.y[index][5] = q6
                self.y[index][6] = q7

            #print (str(self.x))
            #print (str(self.y))
            #print (str(self.paths))

        self.indexes = np.arange(self._length)

    def __len__(self):
        return math.ceil(len(self.y) / self.batch_size)

    def __getitem__(self, idx):
        
        batch_x = self.x[idx * self.batch_size:(idx + 1) * self.batch_size]
        batch_y = self.y[idx * self.batch_size:(idx + 1) * self.batch_size]

        return batch_x.reshape(-1, 1, 7), batch_y.reshape(-1, 1, 7)

def create_model():
    
    # The GRU architecture
    modelGRU = Sequential()

    # First GRU layer w/ input layer
    modelGRU.add(GRU(units=120, input_shape=(1,7), return_sequences=True, time_major=True, dropout=0.0))
    # Second GRU layer
    modelGRU.add(GRU(units=60, return_sequences=True, time_major=True))
    # Third covID layer
    modelGRU.add(Conv1D(filters=60, kernel_size=1, batch_input_shape=(None, 60,), kernel_initializer='glorot_uniform'))
    # The output layer
    modelGRU.add(Dense(7, activation='linear'))
      
    rospy.logwarn("Model created: XYZ 3d Input -> Q Output")

    return Model(inputs=modelGRU.input, outputs=modelGRU.output)


def iou(y_true, y_pred):
    # https://keras.io/backend/, $HOME/.keras/keras.json
    # /home/user/.keras/models/mobilenet_v2_weights_tf_dim_ordering_tf_kernels_1.0_224_no_top.h5
   
    d_q1 = y_true[...,0] - y_pred[...,0]
    d_q2 = y_true[...,1] - y_pred[...,1]
    d_q3 = y_true[...,2] - y_pred[...,2]
    d_q4 = y_true[...,3] - y_pred[...,3]
    d_q5 = y_true[...,4] - y_pred[...,4]
    d_q6 = y_true[...,5] - y_pred[...,5]
    d_q7 = y_true[...,6] - y_pred[...,6]

    # Euclidian distance between the True Point3D and the Predicted
    simple_magn = K.sqrt(K.square(d_q1) + K.square(d_q2) + K.square(d_q3) + K.square(d_q4) + K.square(d_q5) + K.square(d_q6) + K.square(d_q7)) 
    
    unitary_tensor = K.ones(K.shape(simple_magn))
    # We consider that 1 meter - MagnitudeBetWeenPoints is the metric. 0 means that the magnitude is
    # bigger than 1 and 1 is the best
    result = K.clip( unitary_tensor - simple_magn, 0, 1)
    return result


def train(model, epochs, batch_size, patience, threads, train_csv, validation_csv, models_weight_checkpoints_folder, logs_folder, model_unique_id, load_weight_starting_file=None, initial_learning_rate=0.0001, min_learning_rate = 1e-8):
    
    train_datagen = DataSequence(train_csv, batch_size)
    validation_datagen = DataSequence(validation_csv, batch_size)

    if load_weight_starting_file:
        rospy.logwarn("Preload Weights, to continue prior training...."+str(load_weight_starting_file))
        model.load_weights(load_weight_starting_file)
    else:
        rospy.logerr("Starting from empty weights.......")

    #model.compile(loss="mean_squared_error", optimizer="adam", metrics=[iou])
    #model.compile(loss="mean_squared_error", optimizer="adam", metrics=["accuracy"])
    adam_optim = Adam(learning_rate=initial_learning_rate, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
    model.compile(loss="mean_squared_error", optimizer=adam_optim, metrics=["accuracy"])

    full_model_file_name = "model-"+ model_unique_id + "-{val_loss:.8f}.h5"
    model_file_path = os.path.join(models_weight_checkpoints_folder, full_model_file_name)
    
    checkpoint = ModelCheckpoint(model_file_path, monitor='val_loss', verbose=1, save_best_only=True,
                                 save_weights_only=True, mode="auto", save_freq="epoch")
    
    stop = EarlyStopping(monitor="val_loss", patience=patience*5, mode="auto")
    
    # https://rdrr.io/cran/kerasR/man/ReduceLROnPlateau.html
    reduce_lr = ReduceLROnPlateau(monitor="val_loss", factor=0.2, patience=patience, min_lr=min_learning_rate, verbose=1, mode="auto")

    tensorboard_clb = TensorBoard(log_dir=logs_folder, histogram_freq=0,
                                write_graph=True, write_images=True)

    model.summary()
    plot_model(model, to_file='./model.png', show_shapes=True, show_layer_names=True)
    
    history = model.fit(train_datagen,
                        epochs=epochs,
                        validation_data=validation_datagen,
                        callbacks=[checkpoint, reduce_lr, stop, tensorboard_clb],
                        workers=threads,
                        use_multiprocessing=True,
                        shuffle=False,
                        verbose=1)

    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('model accuracy')
    plt.ylabel('accuracy')
    plt.xlabel('epoch')
    plt.legend(['train', 'val'], loc='upper left')
    plt.show()

    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('model loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(['train', 'val'], loc='upper left')
    plt.show()

def main():


    rospy.init_node('generate_database_node', anonymous=True, log_level=rospy.WARN)
    rospy.logwarn("STARTING training process")
    
    if len(sys.argv) < 10:
        rospy.logfatal("usage: train_model.py ALPHA EPOCHS BATCH_SIZE PATIENCE THREADS training_name weight_file_name initial_learning_rate min_learning_rate path_to_database_training_package")
    else:
        EPOCHS = int(sys.argv[1])
        BATCH_SIZE = int(sys.argv[2])
        PATIENCE = int(sys.argv[3])
        THREADS = int(sys.argv[4])
        training_name = sys.argv[5]
        weight_file_name = sys.argv[6]
        initial_learning_rate = float(sys.argv[7])
        min_learning_rate = float(sys.argv[8])
        path_to_database_training_package = sys.argv[9]
        
        rospy.logwarn("EPOCHS to training:"+str(EPOCHS))
        rospy.logwarn("BATCH_SIZE to training:"+str(BATCH_SIZE))
        rospy.logwarn("PATIENCE to training:"+str(PATIENCE))
        rospy.logwarn("THREADS to training:"+str(THREADS))
        rospy.logwarn("training_name to training:"+str(training_name))
        rospy.logwarn("weight_file_name to training:"+str(weight_file_name))
        rospy.logwarn("initial_learning_rate to training:"+str(initial_learning_rate))
        rospy.logwarn("min_learning_rate to training:"+str(min_learning_rate))
        rospy.logwarn("path_to_database_training_package to training:"+str(path_to_database_training_package))
        
        
        rospy.logwarn("Retrieving Paths...")
    
        if path_to_database_training_package == "None":
            rospack = rospkg.RosPack()
            # get the file path for dcnn_training_pkg
            path_to_database_training_package = rospack.get_path('my_dnn_training_pkg')
            rospy.logwarn("Training Databse Path NOT found, setting default:"+str(path_to_database_training_package))
        else:
            rospy.logwarn("Training Databse FOUND, setting default:"+str(path_to_database_training_package))
    
        csv_folder_path = os.path.join(path_to_database_training_package, "dataset_gen_csv")
        train_csv_output_file = os.path.join(csv_folder_path, "train.csv")
        validation_csv_output_file = os.path.join(csv_folder_path, "validation.csv")
        
        models_weight_checkpoints_folder = os.path.join(path_to_database_training_package, "model_weight_checkpoints_gen")
        logs_folder = os.path.join(path_to_database_training_package, "logs_gen")
        
        # We clean up the training folders
        if os.path.exists(models_weight_checkpoints_folder):
            shutil.rmtree(models_weight_checkpoints_folder)
        os.makedirs(models_weight_checkpoints_folder)
        print("Created folder=" + str(models_weight_checkpoints_folder))
    
        if os.path.exists(logs_folder):
            shutil.rmtree(logs_folder)
        os.makedirs(logs_folder)
        print("Created folder=" + str(logs_folder))
    
        print ("Start Create Model")
        modelGRU = create_model()      
        
        model_unique_id = training_name +"-"+ str(EPOCHS) +"-"+ str(BATCH_SIZE) + "-TIME-" + str(time.time())
        
        
        if weight_file_name != "None":
            rospack = rospkg.RosPack()
            # get the file path for rospy_tutorials
            path_to_package = rospack.get_path('my_dcnn_training_pkg')
            backup_models_weight_checkpoints_folder = os.path.join(path_to_package, "bk")
            load_weight_starting_file = os.path.join(backup_models_weight_checkpoints_folder, weight_file_name)
        else:
            rospy.logerr("No load_weight_starting_file...We will start from scratch!")
            load_weight_starting_file = None
        
        rospy.logwarn("Train Model...START")
        
        train(modelGRU,
              EPOCHS,
              BATCH_SIZE,
              PATIENCE,
              THREADS,
              train_csv_output_file,
              validation_csv_output_file,
              models_weight_checkpoints_folder,
              logs_folder,
              model_unique_id,
              load_weight_starting_file,
              initial_learning_rate,
              min_learning_rate)
        
        rospy.logwarn("Train Model...END")

if __name__ == "__main__":
    main()