we using ros noetic on ubuntu 20.04
# OpenManipulator with TurtleBot3
1.installing the open_manipulator_with_tb3 package, install turtlebot3 and open_manipulator packages on the Remote PC 
1.1 The TurtleBot3 Simulation Package requires turtlebot3 and turtlebot3_msgs packages as prerequisite. Without these prerequisite packages, the Simulation cannot be launched.
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make

2.Download and build the package using the following commands in order to use assembled OpenMANIPULATOR-X.
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ sudo apt install ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit*
$ cd ~/catkin_ws && catkin_make

''''''''''''after this installation now you have all package required ''''''''''''''

 -Load TurtleBot3 with OpenMANIPULATOR-X into Gazebo world using this command
            roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
-In order to use Moveit feature, launch move_group node. If you press [▶] button in Gazebo to start simulation, use the following command.
With a successful launch, “You can start planning now!” message will be printed on the terminal.
           roslaunch turtlebot3_manipulation_moveit_config move_group.launch
           
-Use Moveit feature in RViz by reading moveit.rviz file where Moveit enviroment data is configured.
You can control the mounted manipulator using an interactive marker, and simulate the motion of goal position, which helps preventing a possible physical contact by simulating the motion in advance.
          roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
if all above is okay that is agreat step now let`s build our own world and move 3 balls from predefine location to the another side of our map
first you should add our package turtlebot3_manipulation_simulations to your src in your catkin_ws and then run these command
           1-     cd catkin_ws 
           2-     catkin build 
1-launch turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/full.launch 
it will open gazebo with our world and rviz to make nivagation then 
1-in gazebo add path from model_editor_model add debi_balls file that you find in our package copy it and paste in model_editor_model  
2- then add balls in the predefine locations for ball_1( 0.5,0,0)  ,ball_2 (0.5,-0.4,0) and ball_3   (0.5,-0.8,0)
3- rivz open navigation node after launch full.launch it launch navigation.launch file in the upper off rviz screen click on 2D pose stimate and make the arrow  direction to top of screen 
4- then run the trail_l python file to start move ball from robot side to another side using this comand 
first navigate to turtlebot3_manipulation_gazebo/scripts then open in terminal and run  rosrun trail_1.py or using python3 trail_1.py 
finally you see robot move 3 ball from its side to the other side 

