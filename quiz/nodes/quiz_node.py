#!/usr/bin/env python3

import rospy
import logging
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from script.state_manager import StateManager
from script.conditions import ExperimentConditions
from script.fs_utils import load_file, load_quiz_questions, Personality, LogResult

try:
    loglevel = os.environ["LOG_LEVEL"]
except KeyError:
    loglevel = "DEBUG"

log = logging.getLogger(__name__)
logging.basicConfig(format='%(levelname)s:%(message)s', level=loglevel)


class QuizNode(object):
    def __init__(self, user_id: int, condition: ExperimentConditions):
        self.user_id = user_id
        self.condition = condition

        self.questions = load_quiz_questions()

        story = load_file("story.txt")

        self.personality = Personality(self.condition.personality.name)

        log = LogResult(self.user_id, self.condition)

        data_in = {"log": log, "story": story, "quiz_questions": self.questions,
                   "personality": self.personality, "condition": self.condition}

        self.state_manager = StateManager(data_in)

        rospy.loginfo(
            f'User id: {self.user_id} \t Condition: {self.condition}')
        rospy.logdebug('Quiz node initialized')

    def run(self):
        rospy.loginfo('Quiz node running')
        outcome = self.state_manager.sm.execute()
        rospy.signal_shutdown("Quiz node finished")


if __name__ == "__main__":
    rospy.init_node('quiz_node')
    try:
        user_id = rospy.get_param("user_id")
        condition_int = rospy.get_param("condition")
    except KeyError:
        rospy.logerr(
            "Have you set the <user_id> and <condition> ros parameters?")
        sys.exit(1)
    try:
        # convert int to enum
        condition = ExperimentConditions.value_of(condition_int)

        quiz_node = QuizNode(user_id, condition)

        quiz_node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start quiz node.')
