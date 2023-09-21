#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tf



class turtlebot3Class():
    def __init__(self):

        # robotic arm initializer
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('waffle_turtlebot3', anonymous=True)

        self.robot = moveit_commander.RobotCommander()


        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_end_effector_link("end_effector_link")
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.gripper.set_max_acceleration_scaling_factor(1)
        self.gripper.set_max_velocity_scaling_factor(1)

        rospy.loginfo('Robotic Arm Initialized.............')



        # Turtlebot3 Initializer

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        rospy.loginfo('Turtlebot3 Initialized.............')


    def initial_settlment(self):
        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        move = Twist()
        move.angular.z = 0.785
        rospy.sleep(1)

        pub.publish(move)
        rospy.sleep(5)


        # # Stop the robot
        move.angular.z = 0
        pub.publish(move)
        rospy.sleep(1)

        rospy.loginfo("AMCL optimized...............")


    def go_to_pose(self,x,y,yaw):
        quat = tf.quaternion_from_euler(0, 0, yaw)
        # Set the goal target pose
        self.goal.target_pose.header.frame_id = 'odom'
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
        rospy.loginfo('Turtlebot3 reached pose.............')

    def push_ball(self):
        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        move = Twist()
        move.linear.x = 0.5
        rospy.sleep(0.1)
        pub.publish(move)
        rospy.sleep(2)


        # # Stop the robot
        move.linear.x = 0
        rospy.sleep(0.1)
        pub.publish(move)
        rospy.sleep(1)

        move.linear.x = -0.5
        rospy.sleep(0.1)
        pub.publish(move)
        rospy.sleep(3)
        
        # Stop the robot
        move.linear.x = 0
        rospy.sleep(0.1)
        pub.publish(move)
        rospy.sleep(1)

        rospy.loginfo("...............")

if __name__ == '__main__':
    try:
        motion = turtlebot3Class()


        motion.initial_settlment()
        motion.go_to_pose(0.5,0,0)
        motion.push_ball()

        motion.go_to_pose(0.5,-0.4,0)
        motion.push_ball()

        motion.go_to_pose(0.5,-0.8,0)
        motion.push_ball()

        motion.go_to_pose(0,-1.5,0)


    except rospy.ROSInterruptException:
        pass

