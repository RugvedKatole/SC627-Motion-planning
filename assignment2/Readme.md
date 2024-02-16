# SC627 Assignment 2: Dynamic Obstacle Avoidance

## Description

In this assignment, you will implement a collision avoidance algorithm for the robot to move from its starting position to it goals whiles avoiding dynamic and static obstaceles. The code repository contains a ros noetic package which includes launch files, worlds, and planner.py script. You have to edit the planner.py file inside src folder which should navigate to its goal avoiding all the obstacles. 

You are free to use any type of potential field functions. The launch file launchs the dynamic_obstacle world with a turtlebot. The code can be adapted to use the launch the static world as well. Turtlebot has a lidar mounted which has to be used to detect the obstacle and plan the trajectory accordingly. The simulations are to be replicated in with turtlebot3 burger in the arms lab.

## Task

The planner.py file will host the code for collision avoidance algorithm. The planner should be able to take a goal position and navigate to the goal without any collisions. 

Execute following to launch gazebo world with turtlebot

`roslaunch assignment2 assignment.launch`

Your task is to write a collision avoidance algorithm based on artificial potential field to avoid static and dyanmic obstacles. The algorithm has to be tested in simulation as well as in the hardware. 

### Steps before using launch
The plugin for dynamic obstacles resides inside the worlds folder and has to be build before use, follow the steps to build the plugin.

Run setup.sh in worlds folder file it will take care of everthing
`./setup.sh`

If you face problems with setup.sh, follow the steps given below.

```
roscd assignment2/worlds
mkdir build
cd build
cmake ../
make
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
# Check if its working; you should see a box moving around near orign
gazebo dynamic_obstacle.world
```

## Report
Your report should include the following
1. A brief explaination of your approach including all the equations used. 
2. Reasoning behind the used of choosen potential function.
3. What are the limitations of your approach
4. How can you improve the planner

**Extra Credits**:

Provide a heatmap of your potential function and how it replans the trajectory in case of dynamic obstacles. 

## Grading 
Your grade will depend on:
1. The correctness of your implementations
2. Finding solutions within a fixed reasonable budget of time.
3. Simulation Results
4. Experimental Results
5. Report

## Submission
Same as previous assignment