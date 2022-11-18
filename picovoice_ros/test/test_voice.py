#!/usr/bin/env python
import roslib

import sys
import rostest
import unittest
import rospy
import wave
from picovoice_ros.srv import AskUser

TIME_OUT = 10.0  # seconds
INTENT_OPTIONS = ['risposta_uno', 'risposta_due',
                  'risposta_tre', 'risposta_quattro']

class TestVoice(unittest.TestCase):

    def test_1(self):
        name = rospy.get_name()
        print("{} stdout".format(name))
        # sys.stderr.write("{} stderr\n".format(name))
        rospy.logdebug("{} debug".format(name))
        rospy.loginfo("{} info".format(name))
        rospy.logwarn("{} warn".format(name))
        rospy.logerr("{} err".format(name))

    def test_intent(self):
        rospy.logdebug(self.__class__.__name__)
        rospy.loginfo(f'Waiting {TIME_OUT} seconds for user response')
        try:
            rospy.wait_for_service('ask_user', timeout=TIME_OUT)
        except rospy.ROSException as e:
            rospy.logerr("ask_user service not available")
            exit(1)

        proxy = rospy.ServiceProxy('ask_user', AskUser)
        proxy_response = proxy("")
        response = proxy_response.user_intent

        rospy.loginfo(f"user response {response}")
        if response in INTENT_OPTIONS:
            rospy.loginfo("Test passed")
        
if __name__ == '__main__':
    import rosunit
    rospy.init_node("py_test")
    rostest.rosrun('picovoice_ros', 'test_voice', TestVoice)