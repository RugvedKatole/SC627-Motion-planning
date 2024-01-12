#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def cirle():
    # Create a subscriber with appropriate topic
    # rospy.Publisher('Topic_name', Type/Class, queue_size=10)
    # Assign values to linear and angular velocities before publishing
    # 
    #<start your code here>
        
        
    #<code end>

    rospy.init_node('Circle', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # publish your message to topic publisher.publish
        #<start your code here>
        
        
        #<code end>
        rate.sleep()

if __name__ == '__main__':
    try:
        cirle()
    except rospy.ROSInterruptException:
        pass