#!/usr/bin/env python3

from tutorial4.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
import math
import numpy as np

class OBS:

    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        #Initialize client
        self.client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
        self.client.wait_for_server()
        self.ns = rospy.get_namespace()
        self.bot_location = Pose2D()
        self.abs_x = float(self.ns.split('_')[-1].split('/')[0])

    def callback_odom(self, data):
        self.bot_location.x = data.pose.pose.position.x
        self.bot_location.y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        if yaw < 0:
            yaw += 2 * math.pi
        self.bot_location.theta = yaw

    def run(self):
        while True: #replace true with termination condition
            wp = MoveXYGoal()
            wp.pose_dest.x = self.abs_x
            if self.bot_location.y < 0:
                wp.pose_dest.y = np.random.random()
            else:
                wp.pose_dest.y = -np.random.random()
            wp.pose_dest.theta = 1.57 #theta is the orientation of robot in radians (0 to 2pi)

            #send waypoint to turtlebot3 via move_xy server
            self.client.send_goal(wp)

            self.client.wait_for_result()

            #getting updated robot location
            result = self.client.get_result()

            #write to output file (replacing the part below)
            # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)

if __name__ == '__main__':
    rospy.init_node('obs', anonymous= True)
    obs = OBS()
    obs.run()
    rospy.spin()