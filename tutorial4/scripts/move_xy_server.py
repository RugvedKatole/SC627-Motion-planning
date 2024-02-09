#!/usr/bin/env python3

import rospy
import actionlib
import math
from tutorial4.msg import MoveXYAction, MoveXYResult, MoveXYFeedback
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveXY:

    def __init__(self):
        self._feedback = MoveXYFeedback()
        self._result = MoveXYResult()

        self.epsilon_dist = 3e-02 # 3 cm
        self.epsilon_ang = 0.0872665 # 5 degrees

        self.a1 = 0.5
        self.a2 = 0.005

        self.d1 = 0.1
        self.d2 = 0.005

        self.bot_location = Pose2D()
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self._as = actionlib.SimpleActionServer("move_xy", MoveXYAction, self.callback_move, False)
        self._as.start()
        
    def callback_odom(self, data):
        self.bot_location.x = data.pose.pose.position.x
        self.bot_location.y = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        if yaw < 0:
            yaw += 2 * math.pi
        self.bot_location.theta = yaw

    def ang_err(self, pose1, pose2):
        err = pose1.theta - pose2.theta
        if err > math.pi:
            err -= 2 * math.pi
        if err < -math.pi:
            err += 2 * math.pi
        return err

    def dist_err(self, pose1, pose2):
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)


    def head_err(self, pose1, pose2):
        head = math.atan2(pose1.y - pose2.y, pose1.x - pose2.x)
        if head < 0:
            head += 2 * math.pi
        err = head - pose2.theta
        if err > math.pi:
            err -= 2 * math.pi
        if err < -math.pi:
            err += 2 * math.pi
        return err

    def callback_move(self, goal):
        r = rospy.Rate(30)
        success = False
        err_ang_old = 0
        err_dist_old = 0
        err_head_old = 0
        dist = False
        ang = False
        head = False

        while not success:

            self._feedback.pose_mid = self.bot_location

            err_ang = self.ang_err(goal.pose_dest, self.bot_location)
            err_dist = self.dist_err(goal.pose_dest, self.bot_location)
            err_head = self.head_err(goal.pose_dest, self.bot_location)

            if abs(err_ang) < self.epsilon_ang and err_dist < self.epsilon_dist:
                success = True
                pub_msg = Twist()
                self.cmd.publish(pub_msg)
                self._result.pose_final = self.bot_location
                self._as.set_succeeded(self._result)
                break

            elif self._as.is_preempt_requested():
                pub_msg = Twist()
                self.cmd.publish(pub_msg)
                self._as.set_preempted()
                break
            
            self._as.publish_feedback(self._feedback)

            if abs(err_dist) > self.epsilon_dist and (not dist):
                if abs(err_head) > self.epsilon_ang and (not head):
                    pub_msg = Twist()
                    pub_msg.linear.x = 0
                    pub_msg.angular.z = min(0.5, max(-0.5, self.a1 * err_head + self.a2 * (err_head - err_head_old) * 30))
                    # print('head')
                else:                       
                    pub_msg = Twist()
                    pub_msg.angular.z = 0
                    pub_msg.linear.x = max(0.1, self.d1 * err_dist, self.d2 * (err_dist - err_dist_old) * 30)
                    # print('dist')

            elif abs(err_ang) > self.epsilon_ang and (not ang):
                dist = True
                head = True
                pub_msg = Twist()
                pub_msg.linear.x = 0
                pub_msg.angular.z = min(0.5, max(-0.5, self.a1 * err_ang + self.a2 * (err_ang - err_ang_old) * 30))
                # print('ang')
            
            else:
                ang = True
                dist = True
                head = True

            self.cmd.publish(pub_msg)
            err_ang_old = err_ang
            err_dist_old = err_dist
            err_head_old = err_head
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('move_xy_server')
    server = MoveXY()
    rospy.spin()