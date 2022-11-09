#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python client library
from smach import State, StateMachine  # State machine library
import smach_ros  # Extensions for SMACH library to integrate it with ROS
from time import sleep  # Handle time
from std_msgs.msg import String
from picovoice_ros.srv import AskUser
from script.pal_speech_client import Speech

TIME_OUT = 10
STATE_INIT_SLEEP = 0.5

INTENT_OPTIONS = ['risposta_uno', 'risposta_due',
                  'risposta_tre', 'risposta_quattro']


# INTENT_OPTIONS = ['RISP1', 'RISP2', 'RISP3', 'RISP4']


class Initial(State):
    """Read Initial story and initiate the quiz"""

    def __init__(self):
        State.__init__(self, outcomes=['start'],  input_keys=['data_in'],
                       output_keys=['data_out'])
        self.client = Speech("it_IT")

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        self.client.text_to_speech("Iniziamo!")
        # story = userdata.data_in['story']
        # self.client.text_to_speech(story)
        return 'start'


class Quiz(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "finish"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        questions = data_dict_out['quiz_questions']

        for q in questions:
            if q.done == False:
                data_dict_out['tentative'] = 1
                data_dict_out['current_question'] = q
                userdata.data_out = data_dict_out
                return 'question'

        userdata.data_out = data_dict_out
        return 'finish'


class RobotSpeak(State):
    def __init__(self):
        State.__init__(self, outcomes=["to_human"], input_keys=[
            "data_in"], output_keys=["data_out"])
        self.client = Speech("it_IT")

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        question = data_dict_out['current_question']
        rospy.loginfo(question.question.values)
        rospy.loginfo(question.available_answers.values)
        self.client.text_to_speech(question.text)
        # self.pub.publish(question.question.values[0])

        userdata.data_out = data_dict_out
        return 'to_human'


class HumanTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=["answer", "no_answer", "request_hint"], input_keys=[
            "data_in"], output_keys=["data_out"])
        # subscribe to a node that returns the intent of the user
        self.answer = None

    def callback(self, data):
        self.answer = data.data
        rospy.loginfo(self.answer)

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        try:
            rospy.loginfo(f'Waiting {TIME_OUT} seconds for user response')
            rospy.wait_for_service('ask_user', timeout=TIME_OUT)
            proxy = rospy.ServiceProxy('ask_user', AskUser)

            proxy_response = proxy("")
            response = proxy_response.user_intent
            rospy.loginfo(response)
            if response in INTENT_OPTIONS:
                data_dict_out['answer'] = response
                userdata.data_out = data_dict_out
                return 'answer'

            elif response == "suggerimento":
                return 'request_hint'

            elif response == "no_answer":
                rospy.loginfo("User did not answer")
                return "no_answer"
            else:
                rospy.logerr("Intent not recognized")

        except rospy.ROSException as e:
            rospy.logerr(e)
            return 'no_answer'

        userdata.data_out = data_dict_out


class RepeatTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "next_question"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        rospy.loginfo(f"Tentative: {data_dict_out['tentative']}")
        userdata.data_out = data_dict_out
        if data_dict_out['tentative'] < 2:
            data_dict_out['tentative'] += 1
            return 'question'
        else:
            data_dict_out['current_question'].done = True
            return 'next_question'


class Hint(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "next_question"], input_keys=[
            "data_in"], output_keys=["data_out"])
        self.client = Speech("it_IT")

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        question = data_dict_out['current_question']
        hint = data_dict_out['personality'].get_hint()
        self.client.text_to_speech(hint)
        if question.type == "question":
            hinted_question = question.get_hinted()
            data_dict_out['current_question'] = hinted_question
            userdata.data_out = data_dict_out
            return 'question'
        else:
            rospy.logerr('Questions can only be hinted one time')
            data_dict_out['current_question'].done = True
            return 'next_question'


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
        self.client = Speech("it_IT")

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        # check if answer is correct
        is_answer_correct = data_dict_out['current_question'].check(
            data_dict_out['answer'])

        if is_answer_correct == True:
            # say positive feedback
            rospy.logerr('Last question was correct')
            sentence = "Risposta corretta."
            sentence += userdata.data_in['personality'].get_positive()
            rospy.loginfo(sentence)
        elif is_answer_correct == False:
            # say negative feedback
            rospy.logerr('Last question was wrong')
            sentence = "Risposta sbagliata!"
            sentence += userdata.data_in['personality'].get_negative()
            rospy.loginfo(sentence)

        self.client.text_to_speech(sentence)
        data_dict_out['current_question'].done = True
        userdata.data_out = data_dict_out

        return 'next_question'


class Final(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=[
                       'data_in'], output_keys=['data_out'])

    def execute(self, userdata):
        rospy.logdebug(
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
                         'question': 'RobotSpeak', 'next_question': 'Quiz'}, remapping={'data_in': 'sm_input',
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
