#! /usr/bin/env python3
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the /mobile_base_controller/cmd_vel action, including the
# goal message and the result message.

from geometry_msgs.msg import Twist, Vector3

def move_base():
    rospy.init_node('robot_behaviour', anonymous=True)
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5) # 10hz
    linear = Vector3(0, 0, 0)
    angular = Vector3(0, 0, -0.5)
    cmd_vel = Twist(linear, angular)
    for i in range(8):
        rospy.loginfo(cmd_vel)
        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_base()
    except rospy.ROSInterruptException:
        pass
