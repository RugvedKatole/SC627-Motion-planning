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

        self.bot_location = Pose2D()
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.ns = rospy.get_namespace()
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
        dist = False
        ang = False
        head = False
        direction = 1
        while not success:

            self._feedback.pose_mid = self.bot_location

            err_ang = self.ang_err(goal.pose_dest, self.bot_location)
            err_dist = self.dist_err(goal.pose_dest, self.bot_location)
            err_head = self.head_err(goal.pose_dest, self.bot_location)
    
            if err_head > math.pi/2:
                err_head = - math.pi + err_head
                direction = -1
            elif err_head < -math.pi/2:
                err_head =  math.pi + err_head
                direction = -1
            else:
                direction = 1

            print(self.ns, direction)
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
                pub_msg = Twist()
                pub_msg.linear.x = direction * 0.22
                pub_msg.angular.z = min(0.5, max(-0.5, err_head))

            elif abs(err_ang) > self.epsilon_ang and (not ang):
                dist = True
                # head = True
                pub_msg = Twist()
                pub_msg.linear.x = 0
                pub_msg.angular.z = min(0.5, max(-0.5, err_ang))
                # print('ang')
            
            else:
                ang = True
                dist = True
                # head = True

            self.cmd.publish(pub_msg)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('move_xy_server')
    server = MoveXY()
    rospy.spin()