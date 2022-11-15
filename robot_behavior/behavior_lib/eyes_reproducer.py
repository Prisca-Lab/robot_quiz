import rospy
import std_msgs.msg
from hri_msgs.msg import Expression
from behavior_lib.abstract_behaviour import BehaviourMode
import time

class Eyes(BehaviourMode):

    def __init__(self):
        name = self.__class__.__name__
        super().__init__(name)
        
        self.eyes_pub = rospy.Publisher(
            '/eyes/expression', Expression, queue_size=10)

    # define publisher for changing robot facial expression
    def execute(self):
        if self.data is not None:            
            print(f"executing {self.name} at {time.time()}")
            expression_msg = Expression()
            # expression_msg.header = std_msgs.msg.Header() #uncomment for version > 0.7.0
            expression_msg.expression = self.data
            rospy.loginfo(expression_msg)
            self.eyes_pub.publish(expression_msg)
        else:
            rospy.logerr("Please set data before executing")

    def stop(self):
        expression_msg = Expression()
        # expression_msg.header = std_msgs.msg.Header() #uncomment for version > 0.7.0
        expression_msg.expression = "neutral"
        rospy.loginfo(expression_msg)
        self.eyes_pub.publish(expression_msg)


def main():
    eyes = Eyes()
    eyes.data = "happy"
    eyes.execute()


if __name__ == "__main__":
    main()
