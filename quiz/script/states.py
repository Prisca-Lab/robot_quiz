#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python client library
from smach import State, StateMachine  # State machine library
import smach_ros  # Extensions for SMACH library to integrate it with ROS
from time import sleep  # Handle time
from std_msgs.msg import String


TIME_OUT = 2
STATE_INIT_SLEEP = 1

INTENT_OPTIONS = ['risposta_uno','risposta_due','risposta_tre','risposta_quattro']
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
        questions = data_dict_out['quiz_questions']
        
        for q in questions:
            if q.done == False:
                data_dict_out['counter_current_question'] = 0
                data_dict_out['current_question'] = q
                userdata.data_out = data_dict_out
                return 'question'
        
        userdata.data_out = data_dict_out
        return 'finish'

class RobotSpeak(State):
    def __init__(self):
        State.__init__(self, outcomes=["to_human"], input_keys=[
            "data_in"], output_keys=["data_out"])
        
        self.pub = rospy.Publisher('/tts', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        question = data_dict_out['current_question'].question
        rospy.loginfo(question)
        self.pub.publish(question)

        userdata.data_out = data_dict_out
        return 'to_human'



class HumanTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=["answer", "no_answer", "request_hint"], input_keys=[
            "data_in"], output_keys=["data_out"])
        #subscribe to a node that returns the intent of the user
        self.sub = rospy.Subscriber('/answer', String, self.callback)
        self.answer = None

    def callback(self, data):
        self.answer = data.data
        rospy.loginfo(self.answer)

    def execute(self, userdata):
        rospy.loginfo(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        try:
            rospy.loginfo(f'Waiting {TIME_OUT} seconds for user response')
            response = rospy.wait_for_message(
                'human_response', String, timeout=TIME_OUT).data
            
            if response in INTENT_OPTIONS:
                data_dict_out['answer'] = response
                userdata.data_out = data_dict_out
                return 'answer'

            elif response == 'suggerimento':
                return 'request_hint'

        except rospy.ROSException as e:
            rospy.logerr(e)
            return 'no_answer'

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

        data_dict_out['counter_current_question'] += 1
        userdata.data_out = data_dict_out
        if data_dict_out['counter_current_question'] < 2:
            return 'question'
        else:
            return 'next_question'



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
        data_dict_out['current_question'] = hinted_question
        userdata.data_out = data_dict_out
        return 'question'


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

        # check if answer is correct
        # data_dict_out['response']
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
