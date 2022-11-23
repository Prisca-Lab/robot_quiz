#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python client library
from smach import State, StateMachine  # State machine library
import smach_ros  # Extensions for SMACH library to integrate it with ROS
from time import sleep  # Handle time
from robot_behavior.msg import Behavior
from robot_behavior.srv import ExecuteBehavior
from picovoice_ros.srv import AskUser
from script.conditions import BodyConditions
from script.ctrl_robot_base import rotate_base
from itertools import cycle

TIME_OUT = 10
STATE_INIT_SLEEP = 0.3

INTENT_OPTIONS = ['risposta_uno', 'risposta_due',
                  'risposta_tre', 'risposta_quattro']

ALIVE = cycle(["alive_1", "alive_2", "alive_5", "alive_6"])

ROT_ALPHA = 0.5
class Initial(State):
    """Read Initial story and initiate the quiz"""

    def __init__(self):
        State.__init__(self, outcomes=['start'],  input_keys=['data_in'],
                       output_keys=['data_out'])

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)

        rospy.wait_for_service('behaviour', timeout=TIME_OUT)
        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)
        proxy_response = proxy(
            Behavior(text="Iniziamo!", body="open_arms_provocative", eyes="neutral"))

        response = proxy_response.success

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

        log = data_dict_out['log']

        if "current_question" in data_dict_out:
            # append to list of log
            log.result.extend([data_dict_out["current_question"].id, data_dict_out["is_answer_correct"],
                              data_dict_out["tentative"], data_dict_out["current_question"].is_hinted])

        # TODO add backup loop (if user not answer => insert key)
        # TODO disable the touchscreen
        # TODO add movement of the robot and eye expression in hints
        # TODO Check the answer and give feedback
        # TODO Change last sentence with personality (with score?)
        # TODO ? Idle behavior during questions?
        # TODO Activate gaze tracker?

        for q in questions:
            if q.done == False:
                data_dict_out['tentative'] = 1
                data_dict_out['current_question'] = q
                userdata.data_out = data_dict_out
                rospy.loginfo(f"Domanda n. {data_dict_out['current_question'].id}")

                if BodyConditions.SIDE == data_dict_out['condition'].body:
                    # call function to rotate the body!
                    if int(data_dict_out['current_question'].id) % 2 == 0:
                        rotate_base(ROT_ALPHA)
                    else:
                        rotate_base(-ROT_ALPHA)
                    rospy.loginfo("Rotating the base")

                return 'question'

        userdata.data_out = data_dict_out
        return 'finish'


class RobotSpeak(State):
    def __init__(self):
        State.__init__(self, outcomes=["to_human"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        question = data_dict_out['current_question']
        rospy.logdebug(question.available_answers.values)
        text = question.get_text()
        rospy.loginfo(f"robot say: {text}")
        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)
        proxy_response = proxy(
            Behavior(text=text, eyes="neutral", body=next(ALIVE)))

        if question.id == 5:
            sleep(STATE_INIT_SLEEP)
            proxy_response = proxy(
                Behavior(text=question.get_pre_answer_hint_text()))

        rospy.logdebug("behavior has been executed")
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
            try:
                rospy.wait_for_service('ask_user', timeout=TIME_OUT)
            except rospy.ROSException as e:
                rospy.logerr("ask_user service not available")
                exit(1)

            proxy = rospy.ServiceProxy('ask_user', AskUser)
            proxy_response = proxy("")
            response = proxy_response.user_intent

            rospy.loginfo(f"user response {response}")
            if response in INTENT_OPTIONS:
                data_dict_out['answer'] = response
                userdata.data_out = data_dict_out
                return 'answer'

            elif response == "suggerimento":
                return 'request_hint'

            elif response == "no_answer":
                rospy.loginfo("User did not answer or answer was not recognized, mark as wrong")
                data_dict_out['answer'] = response
                userdata.data_out = data_dict_out
                return "answer"

            elif response == "ripeti":
                rospy.loginfo("User asked to repeat the question")
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

        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)
        # can repeat the same question only 2 times
        if data_dict_out['tentative'] < 2:
            data_dict_out['tentative'] += 1
            return 'question'
        else:
            data_dict_out['current_question'].done = True
            data_dict_out['is_answer_correct'] = False
            rospy.logerr('Questions can only be repeated two times')
            proxy(Behavior(text="Posso ripetere la domanda solo due volte.", body=next(ALIVE)))
            return 'next_question'


class Hint(State):
    def __init__(self):
        State.__init__(self, outcomes=["question", "next_question"], input_keys=[
            "data_in"], output_keys=["data_out"])

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in
        hint = data_dict_out['personality'].get_hint()

        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)

        if data_dict_out['current_question'].type == "question":
            proxy(Behavior(text=hint, body=next(ALIVE)))
            hinted_question = data_dict_out['current_question'].get_hinted()
            data_dict_out['current_question'] = hinted_question
            userdata.data_out = data_dict_out
            return 'question'
        else:
            rospy.logerr('Questions can only be hinted one time')
            proxy(Behavior(text="Posso fornire suggerimenti solo una volta.", body=next(ALIVE)))
            data_dict_out['current_question'].done = True
            data_dict_out['is_answer_correct'] = False
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

    def execute(self, userdata):
        rospy.logdebug(
            f'In {self.__class__.__name__} state for {STATE_INIT_SLEEP} seconds')
        sleep(STATE_INIT_SLEEP)
        data_dict_out = userdata.data_in

        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)
        personality = data_dict_out['personality']

        # check if answer is correct
        is_answer_correct = data_dict_out['current_question'].check(
            data_dict_out['answer'])
        data_dict_out['is_answer_correct'] = is_answer_correct

        if is_answer_correct == True:
            # say positive feedback
            rospy.logerr('Last question was correct')
            sentence = "Risposta corretta. "
            sentence += personality.get_positive()

            if personality.name == "AGREEABLENESS":
                body_motion = "middle_arms_side_to_side"
                eyes_motion = "happy"
            elif personality.name == "ANTAGONIST":
                body_motion = ""
                eyes_motion = "bored"

            rospy.loginfo(sentence)

        elif is_answer_correct == False:
            # say negative feedback
            rospy.logerr('Last question was wrong')
            sentence = "Risposta sbagliata! "
            sentence += personality.get_negative()

            if personality.name == "AGREEABLENESS":
                body_motion = "open_arms_provocative"
                eyes_motion = "sad"
            elif personality.name == "ANTAGONIST":
                body_motion = "cover_face_provocative"
                eyes_motion = "angry"

            rospy.loginfo(sentence)

        proxy(Behavior(text=sentence, body=body_motion, eyes=eyes_motion))
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
        proxy = rospy.ServiceProxy('behaviour', ExecuteBehavior)
        rotate_base(ROT_ALPHA)
        proxy(Behavior(
            text="Prima di salutarci, vorrei sapere se riesci a imitare il mio movimento.", eyes="neutral"))
        proxy(Behavior(body="angel_position"))

        proxy(Behavior(text="Grazie per aver giocato con me!",
              body="nod", eyes="neutral"))

        data_dict_out["log"].write()
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
