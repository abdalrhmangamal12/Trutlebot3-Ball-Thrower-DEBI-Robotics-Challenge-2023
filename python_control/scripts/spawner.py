#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import random
import os



# Initialize the ROS node
rospy.init_node('spawn_sdf_model')

# Wait for the spawn_model service to become available
# rospy.wait_for_service('/gazebo/spawn_sdf_model')

# Create a service proxy for the spawn_model service
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

# Create a Pose object to specify the model's initial position and orientation

pose = Pose()
pose.position.x = random.uniform(0.3,1.3)
pose.position.y = random.uniform(-2,-0.3)
pose.position.z = 0.0275
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
pose.orientation.w = 1

# Load the SDF file into a string
username = os.getlogin()

i = random.randint(1,5)
ball = 'model_'+str(i)+'.sdf'
sdf=open('/home/'+username+'/model_editor_models/debi_generic_ball/'+ball, 'r').read()


model_name = 'ball_model'
reference_frame = 'map'
spawn_model(model_name, sdf, 'robot_description', pose, reference_frame)

# Wait for the model to spawn
rospy.sleep(1)

# Shutdown the ROS node
rospy.signal_shutdown('Model spawned')