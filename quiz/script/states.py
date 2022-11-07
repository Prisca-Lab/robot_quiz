#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python client library
from smach import State, StateMachine  # State machine library
import smach_ros  # Extensions for SMACH library to integrate it with ROS
from time import sleep  # Handle time
from std_msgs.msg import String


TIME_OUT = 2
STATE_INIT_SLEEP = 1
class Initial(State):
    """Read Initial story and initiate the quiz"""
    def __init__(self):
        State.__init__(self, outcomes=['start'],  input_keys=['data_in'],
                       output_keys=['data_out'])

    def execute(self, userdata):
        pub = rospy.Publisher('robot_initial_speech', String, queue_size=10)
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        pub.publish(userdata.data_in['story'])
        return 'start'


class Quiz(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "finish"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in


        userdata.data_out = data_dict_out


class RobotSpeak(State):
    def __init__(self):
        State.__init__(self, outcomes=["to_human"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in


        userdata.data_out = data_dict_out



class HumanTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=["answer", "no_answer", "request_hint"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        userdata.data_out = data_dict_out


class RepeatTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "next_question"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        userdata.data_out = data_dict_out


class Hint(State):
    def __init__(self):
        State.__init__(self, outcomes=["question"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        question = data_dict_out['current_question']
        hinted_question = question.get_hinted()

        userdata.data_out = data_dict_out



class CheckAnswer(State):
    """comment according to the personality type
        if answer is correct:
            comment positively with personality type
        if answer is incorrect:
            comment negatively with personality type
    Args:
        State (_type_): _description_
    """
    def __init__(self):
        State.__init__(self, outcomes=["next_question"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        userdata.data_out = data_dict_out


class Final(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=[
                       'data_in'], output_keys=['data_out'])

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        userdata.data_out = data_dict_out


        return 'succeeded'


def main():

    # Initialize the node
    rospy.init_node('fsm')

    # Create a SMACH state machine container
    sm = StateMachine(outcomes=['succeeded', 'failed'])

    # Set user data for the finite state machine
    # pass in quiz questions and personality traits

    sm.userdata.sm_input = "Hello"
    # Open the state machine container. A state machine container holds a number of states.
    with sm:

        StateMachine.add('Initial', Initial(), transitions={'start': 'Quiz'}, remapping={'data_in': 'sm_input',
                                                                                         'data_out': 'sm_input'})
        StateMachine.add('Quiz', Quiz(), transitions={
                         'question': 'RobotSpeak', 'finish': 'Final'}, remapping={'data_in': 'sm_input',
                                                                                  'data_out': 'sm_input'})

        StateMachine.add('RobotSpeak', RobotSpeak(), transitions={'to_human': 'HumanTurn'}, remapping={'data_in': 'sm_input',
                                                                                                       'data_out': 'sm_input'})

        StateMachine.add('HumanTurn', HumanTurn(), transitions={
                         'answer': 'CheckAnswer', 'no_answer': 'RepeatTurn', 'request_hint': 'Hint'}, remapping={'data_in': 'sm_input',
                                                                                                                 'data_out': 'sm_input'})

        StateMachine.add('RepeatTurn', RepeatTurn(), transitions={
                         'question': 'RobotSpeak', 'next_question': 'Quiz'}, remapping={'data_in': 'sm_input',
                                                                                        'data_out': 'sm_input'})

        StateMachine.add('Hint', Hint(), transitions={
                         'question': 'RobotSpeak'}, remapping={'data_in': 'sm_input',
                                                               'data_out': 'sm_input'})

        StateMachine.add('CheckAnswer', CheckAnswer(), transitions={
                         'next_question': 'Quiz'}, remapping={'data_in': 'sm_input',
                                                              'data_out': 'sm_input'})

        StateMachine.add('Final', Final(), transitions={
                         'succeeded': 'succeeded'}, remapping={'data_in': 'sm_input',
                                                               'data_out': 'sm_input'})

    # View our state transitions using ROS by creating and starting the instrospection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
