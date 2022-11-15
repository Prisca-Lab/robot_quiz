
# System imports
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from behavior_lib.abstract_behaviour import BehaviourMode


class Gesture(BehaviourMode):

    def __init__(self):
        name = self.__class__.__name__
        super().__init__(name)
        self.client = SimpleActionClient('/play_motion', PlayMotionAction)

    def execute(self):
        """Reproduce action passed as input action_name
        """
        if self.data is not None:
            action_name = self.data
            rospy.loginfo("Starting run_motion_python application...")
            self.wait_for_valid_time(10.0)
            rospy.loginfo("Waiting for Action Server...")
            self.client.wait_for_server()

            goal = PlayMotionGoal()
            goal.motion_name = action_name
            goal.skip_planning = False
            goal.priority = 0  # Optional

            rospy.loginfo(f'Sending goal with motion: {action_name}')
            self.client.send_goal(goal)
            rospy.loginfo("Execute action without waiting for result...")
        else:
            rospy.logerr("No action name passed as input")

    def stop(self):
        self.client.cancel_all_goals()

    def wait_for_valid_time(self, timeout):
        """Wait for a valid time (non-zero), this is important  when using a simulated clock
        """
        # Loop until:
        # * ros master shutdowns
        # * control+C is pressed (handled in is_shutdown())
        # * timeout is achieved
        # * time is valid
        start_time = time.time()
        while not rospy.is_shutdown():
            if not rospy.Time.now().is_zero():
                return
            if time.time() - start_time > timeout:
                rospy.logerr("Timed-out waiting for valid time.")
                exit(0)
            time.sleep(0.1)
        # If control+C is pressed the loop breaks, we can exit
        exit(0)

def main():
    gesture = Gesture()
    gesture.data = "Ciao"
    gesture.execute()

if __name__ == "__main__":
    main()