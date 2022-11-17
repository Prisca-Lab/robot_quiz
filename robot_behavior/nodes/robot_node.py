#!/usr/bin/env python3

import logging
import rospy
import sys
import os
from threading import Thread

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

        self.modes = [Speech, Gesture, Eyes]
        self.active_modes = []
        self.execute_behavior = rospy.Service(
            'behaviour', ExecuteBehavior, self.cb)

    def cb(self, msg):
        rospy.loginfo("Received service request: %s", msg)

        # create instance of a BehaviorMode only if it is requested by the service call
        # if not, the instance is not created

        if msg.robot_behavior.text:
            speech = self.modes[0]("it_IT")
            speech.data = msg.robot_behavior.text
            self.active_modes.append(speech)

        if msg.robot_behavior.body:
            body = self.modes[1]()
            body.data = msg.robot_behavior.body
            self.active_modes.append(body)

        if msg.robot_behavior.eyes:
            eyes = self.modes[2]()
            eyes.data = msg.robot_behavior.eyes
            self.active_modes.append(eyes)

        return self.run()

    def run(self):
        threads = []
        try:
            # each thread executes the method execute of an active behavior mode
            for m in self.active_modes:
                threads.append(Thread(target=m.execute))

            for t in threads:
                t.start()

            for t in threads:
                t.join()

            res = True
        except Exception as e:
            rospy.logerr(e)
            self.stop()
            res = False
        self.clear()
        return ExecuteBehaviorResponse(res)

    def stop(self):
        for m in self.active_modes:
            m.stop()

    def clear(self):
        self.active_modes = []


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
