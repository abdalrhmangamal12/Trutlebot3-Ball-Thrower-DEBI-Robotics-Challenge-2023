#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
import geometry_msgs.msg

from math import pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tf
import tf2_ros
import tf2_geometry_msgs
from python_control.srv import *
from gazebo_msgs.srv import DeleteModel

import numpy as np
import random
import subprocess

class turtlebot3Class():
    def __init__(self):
        rospy.init_node('turtlebot3_wafflepi', anonymous=True)


        # robotic arm initializer
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()


        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_end_effector_link("end_effector_link")
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.gripper.set_max_acceleration_scaling_factor(1)
        self.gripper.set_max_velocity_scaling_factor(1)

        self._robotic_arm_initial_settlement()
        rospy.loginfo('Robotic Arm Initialized.............')


        # tf2 initializer
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo('Transformation function Initialized.............')



        # Turtlebot3 Initializer
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rospy.loginfo('Turtlebot3 Nodes Initialized.............')


    def _robotic_arm_initial_settlement(self):
        joint_positions = [0, 0, 0, 0]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(wait=True)

    def look_for_balls(self, j4_value):

        j4_value = np.deg2rad(j4_value)

        j1_value = np.deg2rad(-160)
        joint_positions = [j1_value, 0, 0, j4_value]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(wait=True)


        # DURING ROTATION, NODE 2 COLLECT INFORMATION FOR:
        #   > ROTATION ANGLES FOR BALL IN THE SEARCHED WORKSPACE
        
        self.arm.set_max_acceleration_scaling_factor(0.6)
        self.arm.set_max_velocity_scaling_factor(0.6)
        
        j1_value = np.deg2rad(155)
        joint_positions = [j1_value, 0, 0, j4_value]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(wait=True)

        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
        
        # SERVICE 1, TO KNOW:
        #   > ROTATION ANGLES FOR BALLS IN THE SEARCHED WORKSPACE
        # >>>> use group.get_current_joint_values()

        rospy.wait_for_service('service_1')
        service_1 = rospy.ServiceProxy('service_1', JointAngle)
        feedback = service_1(1)

        if feedback is not False:


            rospy.loginfo("Located ball in robots workspace ..........")
            
            j1_value = np.deg2rad(feedback.joint_angles[0]-20)
            joint_positions = [j1_value, 0, 0, j4_value]
            self.arm.set_joint_value_target(joint_positions)
            self.arm.go(wait=True)


            # SERVICE 2, TO KNOW:
            #   > LOCATION OF BALL

            service_2 = rospy.ServiceProxy('service_2', LocationOfBall)
            ball_loc = service_2(1)



            location = self._transform_location(px = ball_loc.point.x, py = ball_loc.point.y,\
                pz = ball_loc.point.z, target_frame = "map",source_frame = "link5",condition = 1)

            return location
        
        else:
            return False


    def _transform_location(self,px,py,pz,target_frame,source_frame,condition=0):


        point_to_be_transformed = geometry_msgs.msg.PoseStamped()
        point_to_be_transformed.header.frame_id = source_frame
        point_to_be_transformed.header.stamp = rospy.Time.now()
        point_to_be_transformed.pose.position.z = pz


        point_to_be_transformed.pose.orientation.x = 0
        point_to_be_transformed.pose.orientation.y = 0
        point_to_be_transformed.pose.orientation.z = 0
        point_to_be_transformed.pose.orientation.w = 1


        if condition == 1:
            yaw = np.arctan2(py, px)
            hyp = np.hypot(py,px)
            hyp -= 0.26
            point_to_be_transformed.pose.position.x = hyp*np.cos(yaw)
            point_to_be_transformed.pose.position.y = hyp*np.sin(yaw)

        else:
            point_to_be_transformed.pose.position.x = px
            point_to_be_transformed.pose.position.y = py

        source_to_target = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
        transformation = tf2_geometry_msgs.do_transform_pose(point_to_be_transformed, source_to_target)

        return transformation


    def go_to_pose(self,located_ball):


        j4_value = np.deg2rad(70)
        joint_positions = [0, 0, 0, j4_value]
        self.arm.set_joint_value_target(joint_positions)
        
        x = located_ball.pose.position.x
        y = located_ball.pose.position.y
        yaw = np.rad2deg(np.arctan2(y, x))


        self.move_turtlebot(x,y,yaw)
        self.arm.go(wait=True)




        # SERVICE 2, TO KNOW:
        #   > LOCATION OF BALL

        rospy.wait_for_service('service_2')
        service_2 = rospy.ServiceProxy('service_2', LocationOfBall)
        ball_loc = service_2(1)


        rospy.loginfo('Looking at ball .............')

        gripping_location = self._transform_location(px = ball_loc.point.x, py = ball_loc.point.y\
            ,pz = ball_loc.point.z ,target_frame = "base_footprint",source_frame = "link5")
        ball_position = [ gripping_location.pose.position.x\
        ,gripping_location.pose.position.y ,gripping_location.pose.position.z]
        
        self._open_grip()
        self.arm.set_position_target(ball_position)
        self.arm.go(wait=True)
        self._close_grip()
        rospy.loginfo('Ball gripped .............')
        self.arm.set_joint_value_target([0,0,0,0])
        self.arm.go(wait=True)





    def move_turtlebot(self,x,y,yaw):
        
        yaw = np.deg2rad(yaw)
        quat = tf.quaternion_from_euler(0, 0, yaw)
        
        # Set the goal target pose
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0

        self.goal.target_pose.pose.orientation.x = quat[0]
        self.goal.target_pose.pose.orientation.y = quat[1]
        self.goal.target_pose.pose.orientation.z = quat[2]
        self.goal.target_pose.pose.orientation.w = quat[3]
        # Send the goal to the action server
        self.client.send_goal(self.goal)

        # Wait for the robot to reach the goal
        self.client.wait_for_result()
        rospy.loginfo('Turtlebot3 reached location.............')


    def throw_ball(self):
        self._close_grip()

        # joint_positions = [0, np.deg2rad(70), np.deg2rad(-50), np.deg2rad(-20)]
        joint_positions = [0, 0, 0, np.deg2rad(-90)]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(wait=True)
        

        joint_positions = [0, 0, 0, 0]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(wait=False)
        rospy.sleep(1)
        self._open_grip()


        rospy.loginfo("Ball released---------------------")

    def _close_grip(self):

        step = self.gripper.get_current_joint_values()
        step[0] = 0.00525
        self.gripper.set_joint_value_target(step)
        self.gripper.go(wait=True)



    
    def _open_grip(self):

        step = self.gripper.get_current_joint_values()
        step[0] = 0.019
        self.gripper.set_joint_value_target(step)
        self.gripper.go(wait=True)





if __name__ == '__main__':
    try:
  
        motion = turtlebot3Class()
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # calling ball
        subprocess.run("rosrun python_control spawner.py", shell=True)                     
        j4_value = 25
       
        while not rospy.is_shutdown():
            located_ball = motion.look_for_balls(j4_value)
            
            if located_ball is not False:
                motion.go_to_pose(located_ball)
                motion.move_turtlebot(1.55,random.uniform(0, -1.75),0)
                motion.throw_ball()
                rospy.loginfo('Turtlebot3 reached end location.............')
                motion.move_turtlebot(0,0,0)
                delete_model('ball_model')
                # calling ball
                subprocess.run("rosrun python_control spawner.py", shell=True)                     

                j4_value = 0

            else:
                if j4_value == 60:
                    j4_value = 0
                else:
                    j4_value+=10



    except rospy.ROSInterruptException:
        pass

