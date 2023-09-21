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
        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

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



    def _go_to_pose(self,x,y,yaw):
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
        rospy.loginfo('Turtlebot3 reached start location.............')



    def _close_grip(self):
        # rospy.sleep(2)
        succeeded = False
        step = self.gripper.get_current_joint_values()
        step[0] = 0.00525
        self.gripper.set_joint_value_target(step)
        self.gripper.go(wait=True)



    
    def _open_grip(self):
        # rospy.sleep(2)
        succeeded = False

        step = self.gripper.get_current_joint_values()
        step[0] = 0.019
        self.gripper.set_joint_value_target(step)
        self.gripper.go(wait=True)


   
    def _manipulate_arm(self,joint_positions, ball):


        if ball != 0:

            # if ball == "ball1":

            #     pose = [0.4,-2,0.0275]
            
            # if ball == "ball2":
            #     pose = [0.8,-2,0.0275]

            # if ball == "ball3":
            #     pose = [1.2,-2,0.0275]



            self.arm.set_position_target([0.221,0,0.0275])

        if joint_positions != 0:
            self.arm.set_joint_value_target(joint_positions)
        

        self.arm.go(wait=True)


    
    def arm_control(self, required, ball = "no ball"):            
        if required == "take_ball":
            self._open_grip()
            self._manipulate_arm(joint_positions = 0, ball = ball)
            self._close_grip()
            self._manipulate_arm(joint_positions= [0, -1,0.3,0.7], ball = 0)
            rospy.loginfo("ball gripped---------------------")


        if required == "release_ball":
            self._close_grip()
            self._manipulate_arm(joint_positions= [0, 70*pi/180,-50*pi/180,-20*pi/180], ball = 0)
            self._open_grip()
            self._manipulate_arm(joint_positions= [0, -1,0.3,0.7], ball = "no ball")
            rospy.loginfo("ball released---------------------")


    def position(self,required_place):
        if required_place == "home":
            pose = (0,0,0)

        elif required_place == "end":
            pose = (1.47,-1,0)

        elif required_place == "ball1":
            pose = (0.4,-1.778,-90*pi/180)
        
        elif required_place == "ball2":
            pose = (0.8,-1.778,-90*pi/180)
        
        elif required_place == "ball3":
            pose = (1.2,-1.778, -90*pi/180)
        
        self._go_to_pose(pose[0],pose[1],pose[2])




if __name__ == '__main__':
    try:
        motion = turtlebot3Class()

        # motion.initial_settlment()

        for i in range(3):
            # home, ball1, ball2, ball3, end
            ball = "ball"+str(i+1)
            motion.position(required_place = ball)
            
            # take_ball or release_ball
            motion.arm_control(required = "take_ball",ball = ball)
            motion.position("end")
            motion.arm_control(required = "release_ball",ball = "no ball")
            rospy.loginfo(ball+"is Done.....")


        # motion.position("home")



    except rospy.ROSInterruptException:
        pass

