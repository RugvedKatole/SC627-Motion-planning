#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Planner:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_avoidance')
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.move_cmd = Twist()

    def laser_callback(self, scan_msg):
        # Retrieve LiDAR data and store it for obstacle avoidance
        self.scan_data = scan_msg

    def planner(self):
        # Implement dynamic obstacle avoidance using APF logic here based on LiDAR data
        # For example, let's say we stop if there's an obstacle within 0.5 meters
        if min(self.scan_data.ranges) < 0.5:
            self.move_cmd.linear.x = 0.0
        else:
            self.move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s

    def run(self):
        while not rospy.is_shutdown():
            self.planner()  # Call the obstacle avoidance function
            self.velocity_publisher.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Planner = Planner()
        Planner.run()
    except rospy.ROSInterruptException:
        pass
