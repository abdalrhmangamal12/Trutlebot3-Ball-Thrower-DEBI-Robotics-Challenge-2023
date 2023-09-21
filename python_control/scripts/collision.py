#!/usr/bin/env python3

import rospy
import moveit_commander

import sys

import numpy as np
import random
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi




def collision():
	# Initialize a node
	rospy.init_node('add_collision_object_node', anonymous=True)

	# Create a planning scene interface instance
	scene = moveit_commander.PlanningSceneInterface()
	rate = rospy.Rate(10000)

	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = "map"   # Replace "base_link" with your robot's base frame ID
	box_pose.pose.position.x = 1 #random.uniform(0.1, 0.5)   # Replace with the x position of the box

	box_pose.pose.position.y = 0    # Replace with the y position of the box
	box_pose.pose.position.z =  0.5 #random.uniform(-0.2, 0.5)     # Replace with the z position of the box
	box_pose.pose.orientation.w = 1.0

	box_size = (0.05, 0.05, 1)   # Replace with the size of the box in meters

	# Add the box to the planning scene
	scene.add_box("box1", box_pose, box_size)



if __name__ == '__main__':
    try:
        collision()
    except rospy.ROSInterruptException:
        pass