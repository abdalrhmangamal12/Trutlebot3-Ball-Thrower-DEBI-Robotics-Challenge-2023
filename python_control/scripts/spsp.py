#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState
import random
import os



# Initialize the ROS node
rospy.init_node('spawn_sdf_model')

# Wait for the spawn_model service to become available
# rospy.wait_for_service('/gazebo/spawn_sdf_model')

# Create a service proxy for the spawn_model service
ball_location = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

loc = ball_location('ball_model','map')


print(loc.pose.position.x,loc.pose.position.y,loc.pose.position.z)