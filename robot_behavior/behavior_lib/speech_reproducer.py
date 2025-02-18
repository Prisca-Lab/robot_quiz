# import ROS headers
import rospy
import actionlib
# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from sound_play.libsoundplay import SoundClient
from behavior_lib.abstract_behaviour import BehaviourMode
import time


class Speech(BehaviourMode):

    def __init__(self, language):
        name = self.__class__.__name__
        super().__init__(name)
        # rospy.init_node(name, anonymous=True)
        # change it to tts if you're using from the tiago
        self.client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.sound_client = SoundClient()
        self.language = language

    def execute(self):
        # assert type(data) is str, "Error, data input must be a string"
        if self.data is not None:
            try:
                self.client.wait_for_server(timeout=rospy.Duration(5))
            except Exception as e:
                rospy.logerr("Cannot connect to action server /tts")
                exit(1)
            goal = TtsGoal()
            goal.rawtext.text = self.data
            goal.rawtext.lang_id = self.language
            start = time.time()
            self.client.send_goal(goal)
        else:
            rospy.logerr("Please set data before executing")
        # we wait until the action won't finish
        try:
            self.client.wait_for_result()
        except Exception as e:
            rospy.logerr("Cannot retrieve action server result /tts")
            exit(1)
        
        rospy.loginfo(f"{self.name} lasted: {time.time() - start:.3f} seconds")
        return

    def stop(self):
        rospy.loginfo("canceling...")
        self.data = ""
        self.execute()
        rospy.loginfo("goal has been canceled")


def main():
    speech = Speech("it_IT")
    speech.data = "Ciao"
    speech.execute()


if __name__ == "__main__":
    main()
