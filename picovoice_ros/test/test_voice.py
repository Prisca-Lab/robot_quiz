#!/usr/bin/env python
import roslib

import sys
import rostest
import unittest
import rospy

class TestVoice(unittest.TestCase):

    def test_1(self):
        name = rospy.get_name()
        print("{} stdout".format(name))
        # sys.stderr.write("{} stderr\n".format(name))
        rospy.logdebug("{} debug".format(name))
        rospy.loginfo("{} info".format(name))
        rospy.logwarn("{} warn".format(name))
        rospy.logerr("{} err".format(name))

if __name__ == '__main__':
    import rosunit
    rospy.init_node("py_test")
    rostest.rosrun('picovoice_ros', 'test_voice', TestVoice)