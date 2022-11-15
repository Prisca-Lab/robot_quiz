#!/usr/bin/env python3

import logging
import rospy
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behavior_lib.eyes_reproducer import Eyes
from behavior_lib.gesture_reproducer import Gesture
from behavior_lib.speech_reproducer import Speech
from robot_behavior.srv import ExecuteBehavior, ExecuteBehaviorResponse

try:
    loglevel = os.environ["LOG_LEVEL"]
except KeyError:
    loglevel = "DEBUG"

log = logging.getLogger(__name__)
logging.basicConfig(format='%(levelname)s:%(message)s', level=loglevel)


class BehaviourNode:
    """
  This is a module to reproduce a robot action combining speech,
  eyes expression and gesture.
  """

    def __init__(self):

        self.say = Speech("it_IT")
        self.move_body = Gesture()
        self.move_eyes = Eyes()
        self.modes = [self.say, self.move_body, self.move_eyes]
        self.execute_behavior = rospy.Service(
            'behaviour', ExecuteBehavior, self.cb)

    def cb(self, msg):
        rospy.loginfo("Received service request: %s", msg)
        self.say.data = msg.robot_behavior.text
        self.move_eyes.data = msg.robot_behavior.eyes
        self.move_body.data = msg.robot_behavior.body
        self.run()

    def run(self):
        try:
            for m in self.modes:
                m.execute()
            return ExecuteBehaviorResponse(True)
        except Exception as e:
            rospy.logerr(e)
            self.stop()
            return ExecuteBehaviorResponse(False)

    def stop(self):
        for m in self.modes:
            m.stop()


def default_modes():

    behaviour = BehaviourNode()

    behaviour.say.data = "Ciao sono Antonio"
    behaviour.move_body.data = "nodding.yaml"
    behaviour.move_eyes.data = "sad"


if __name__ == "__main__":

    rospy.init_node("robot_behaviour")
    b = BehaviourNode()
    # default_modes()
    rospy.spin()
