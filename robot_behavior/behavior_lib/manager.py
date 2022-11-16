# UNUSED CHECK NODE
# coding=utf-8
from robot_behaviour.eyes_reproducer import Eyes
from robot_behaviour.gesture_reproducer import Gesture
from robot_behaviour.speech_reproducer import Speech

from robot_behaviour.abstract_behaviour import BehaviourMode
import rospy

class PlayBehaviour:
    """
  This is a module to reproduce a robot action combining speech,
  eyes expression and gesture.
  """

    def __init__(self, modes = [BehaviourMode]):
        # rospy.init_node("manager", anonymous=True)

        self.say = Speech("it_IT")
        self.move_body = Gesture()
        self.move_eyes = Eyes()
        self.modes = [self.say, self.move_body, self.move_eyes]

        
        # if modes == None:
        # else:
        #     for m in modes:
        #         assert isinstance(m, BehaviourMode), "mode is not a BehaviourMode"
        #     self.modes = modes


    def run(self):
        """
    it reproduces a complex robot behaviour by
    combining multi-modal interactions such as voice,
    gesture and eyes expression

    sentence: the text the robot has to reproduce
    gesture_name: the name of the prerecorded motion
    the robot has to play using play_motion
    eyes_expression_name: the name of the eyes expression
    """

        for m in self.modes:
            m.execute()

    def stop(self):
        for m in self.modes:
            m.stop()



def main():
    speech = Speech("it_IT")
    gesture = Gesture()
    eyes = Eyes()

    modes = [speech, gesture, eyes]
    behaviour = PlayBehaviour(modes)

    speech.data = "Ciao sono Antonio"
    gesture.data = "nodding.yaml"
    eyes.data = "sad"

    behaviour.run()


def default_modes():

    behaviour = PlayBehaviour()

    behaviour.say.data = "Ciao sono Antonio"
    behaviour.move_body.data = "nodding.yaml"
    behaviour.move_eyes.data = "sad"
    
    behaviour.run()

if __name__ == "__main__":
    main()
