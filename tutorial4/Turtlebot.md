# TurtleBot3 Burger Hardware Setup

[Setup Link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)


Set ROSMASTER and ROS HOSTNAME on Laptops/PC
By pasting following command in ~/.bashrc

`export ROS_MASTER_URI=http://192.168.0.7:11311/`


`export ROS_HOSTNAME=192.168.0.7`

change 192.168.0.7 to your IP, using if config


Connecting to the Robot

`ssh ubuntu@<IP of the Robot>`
Password: turtlebot

Run Bringup launch

`roslaunch turtlebot3_bringup turtlebot3_robot.launch`