#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np 
import pyrealsense2 as rs
# Initialize the CvBridge class

import tf2_ros
import tf
# Define a callback function to handle incoming image messages






def pointconvert(balls):
        

        
        depth_min = 0.1 #meter
        depth_max = 10 #meter

#        depth_intrin = np.array([695.9951171875, 0.0, 640.0, 0.0, 695.9951171875, 360.0, 0.0, 0.0, 1.0])
   #     depth_intrin = np.reshape(depth_intrin ,(3,3))

#        tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
#        tf_listener = tf2_ros.TransformListener(tf_buffer)


        
#        depth_to_color_extrin =tf_buffer.lookup_transform('rs_camera_color_optical_frame', 'rs_camera_depth_optical_frame', rospy.Time(1))
#        color_to_depth_extrin = tf_buffer.lookup_transform('rs_camera_depth_optical_frame','rs_camera_color_optical_frame', rospy.Time(1)) 

        #Use pixel value of  depth-aligned color image to get 3D axes
        balls_with_depth_point=[]
        for i  in balls:
            color_point = [i[0],i[1]]
        
        
#        depth_point_ = rs.rs2_project_color_pixel_to_depth_pixel(
#              depth_frame, depth_scale,
#              depth_min, depth_max,
 #             depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, color_point)
           # print(depth_point_) 
            global depth_point
            depth_point = color_point  #in simulation 
            get_depth()
            
            if depth_point[0]<0 or depth_point[1]<0: 
                 print('camera is so near to object and depth cannot determine') 
            else:
               #  depth = depth_frame.get_distance(round(depth_point_[0]) , round(depth_point_ [1]))
                 print(depth ,"depth")
                 dx,dy, dz = rs.rs2_deproject_pixel_to_point(intrinsics, depth_point, ((depth+27.5)/1000)) 

                #  print("depth from point to camera ", depth)
                #  print(f'x is {dx} and y is {dy} and z is {dz}')
                 balls_with_depth_point.append([dx,dy,dz])

        
        return balls_with_depth_point


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
      
        if roundness > 0.6 and area > 80 and area < 10000:
            # Get circle parameters and draw it on the original image
            (x,y), radius = cv2.minEnclosingCircle(contour)
            center = [int(x),int(y)]
            print("cen",center)
            balls.append(center)
            radius = int(radius)
            cv2.circle(img,center,radius,(0,255,0),2)

    print("balls",balls)           


    img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)                 


    cv2.imshow("sdf",img)
    cv2.waitKey(1)
    
    
    

    
def image_callback(msg):
    # Convert the image message to an OpenCV image
    img = bridge.imgmsg_to_cv2(msg,"rgb8")
    img =cv2.resize(img,(640,480))
    detect_ball(img)
    # Display the image
   # cv2.imshow("Image", img)
   # cv2.waitKey(1) # Wait for a short time to allow the image to be displayed
   

       
bridge = CvBridge()
# Initialize the ROS node
rospy.init_node("image_viewer")

# Subscribe to the image topic
rospy.Subscriber("/rs_plugin/color/image_raw", Image, image_callback)

while not rospy.is_shutdown():
    rospy.spin()

