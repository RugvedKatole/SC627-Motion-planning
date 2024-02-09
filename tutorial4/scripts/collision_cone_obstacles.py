#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial4.msg import ObsData, Obs
from tf.transformations import euler_from_quaternion


class Obstacles:

    def __init__(self):

        self.obs_pub = rospy.Publisher('/obs_data', ObsData, queue_size = 10)
        rospy.Subscriber('/bot_2/odom', Odometry, self.callback_odom, 'bot_2')
        rospy.Subscriber('/bot_3/odom', Odometry, self.callback_odom, 'bot_3')
        rospy.Subscriber('/bot_4/odom', Odometry, self.callback_odom, 'bot_4')
        self.obs = {}
        self.obs['bot_2'] = Obs()
        self.obs['bot_3'] = Obs()
        self.obs['bot_4'] = Obs()

    def callback_odom(self, data, bot_id):
        self.obs[bot_id].obs = bot_id
        self.obs[bot_id].pose_x = data.pose.pose.position.x
        self.obs[bot_id].pose_y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        yaw = math.atan2(math.sin(yaw),math.cos(yaw)) # normalizing to get yaw in -pi to pi
        # self.obs[bot_id].pose.theta = yaw

        lin_vel = data.twist.twist.linear.x
        self.obs[bot_id].vel_x = lin_vel * math.cos(yaw)
        self.obs[bot_id].vel_y = lin_vel * math.sin(yaw)

if __name__ == '__main__':
    rospy.init_node('collision_cone_obstacles', anonymous = True)
    s = Obstacles()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        obs = []
        for i, n in enumerate(s.obs.values()):
            obs.append(n)

        obs_data = ObsData()
        obs_data.obstacles = obs
        print(obs_data)
        s.obs_pub.publish(obs_data)
        r.sleep() 

