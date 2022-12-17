# Robotic Dartboard Project
The goal of this project is to get the robot to catch darts in the middle of a dartboard.

## Current Progress
We have implemented the analytically derived inverse kinematic model. We also perform forward kinematics on the generated joint angles for sanity checking. We have written the code to track projectiles using the depth camera plugin, and to predict the trajectory for the projectile using a small averaging window. We solve for the intersection of this trajectory (a standard parabola) and the catching plane.

We have also modified the robot model to have a dartboard, and modified the default world to contain our depth camera and dart projectile with an initial velocity. The robot will convert the calculated intersection into world space coordinates, and then use our inverse kinematic model to move to catch the dart.

## How to Run
### Simulator

#### Terminal 1
$ catkin_make  (Only once)  
$ source devel/setup.bash  
$ roslaunch ur3_driver ur3_gazebo.launch  

#### Terminal 2
$ source devel/setup.bash
$ rosrun lab2pkg_py project_exec.py  
