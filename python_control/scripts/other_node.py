#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose

from gazebo_msgs.srv import GetModelState

from python_control.srv import JointAngle, JointAngleRequest, JointAngleResponse
from python_control.srv import LocationOfBall, LocationOfBallRequest, LocationOfBallResponse
import subprocess
import random
import cv2
from cv_bridge import CvBridge
import numpy as np
joint_angles = False

def detect_ball(img):
    circle_det_center=[]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur to reduce noise
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
    # Detect edges using Canny edge detection
    edges = cv2.Canny(gray_blur, 9, 25)
          # Find contours of potential circles
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    balls=[]
    # Iterate through contours and find circles using their area and roundness
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        roundness = 4 * np.pi * area / ((perimeter ** 2)+np.exp(-7))
        
        # Check if contour is a circle based on its roundness and area
      
        if roundness > 0.5 and area > 80 and area < 10000:
            # Get circle parameters and draw it on the original image
            (x,y), radius = cv2.minEnclosingCircle(contour)
            center = [int(x),int(y)]
            balls.append(center)
            radius = int(radius)
            cv2.circle(img,center,radius,(0,255,0),2)



  #  img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)                 


    cv2.imshow("sdf",img)
    cv2.waitKey(1)
    return balls


def service1_callback(req):
    global joint_angles
    response = JointAngleResponse()
    response.joint_angles = copy.deepcopy(joint_angles)
    joint_angles = False
    print(response)
    return response




def service2_callback(req):

    response = LocationOfBallResponse()

    # Subscribe for ball Location >>> RGB / DEPTH
    # Needs Hardware specs, such as intrinsics and extrinsics (camera_info)

    # for Simulation purposes, we would use Service in gazebo between
    # link_5 frame and ball_location frame, then transform this location
    # to the required frame, according to the objective of the code at this moment

    # model location in xyz --> will be analogied in real with depth
    # and RGB camera model to get point in xyz,
    # using Intel RealSense D435i RGB-D camera on the robot, in /HARDWARE PHASE/

    ball_location = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    random.seed(10)

    loc = ball_location('ball_model','link5')
    response.point.x = loc.pose.position.x
    response.point.y = loc.pose.position.y
    response.point.z = loc.pose.position.z
    print("XYZ: ",response.point.x,response.point.y,response.point.z)



    return response


def subscriber_callback(data):
    global joint_angles
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.resize(img, (640, 480))
    ball=detect_ball(img)



    if ball is not False:
        if joint_angles is False:
            joint_angles = copy.deepcopy(arm.get_current_joint_values())
            rospy.sleep(0.1)

            joint_angles = copy.deepcopy(arm.get_current_joint_values())







if __name__== '__main__':

    try:

        rospy.init_node('vision_module', anonymous=True)
        arm = moveit_commander.MoveGroupCommander("arm")
        # Create the first service
        rospy.Service('service_1', JointAngle, service1_callback)

        # Create the second service
        rospy.Service('service_2', LocationOfBall, service2_callback)    
            
        # Subscribe for ball detected >>> RGB
        rospy.Subscriber('/rs_plugin/color/image_raw', Image, subscriber_callback)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass

