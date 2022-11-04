#!/usr/bin/env python3

# Import the necessary libraries
import rospy  # Python client library
from smach import State, StateMachine  # State machine library
import smach_ros  # Extensions for SMACH library to integrate it with ROS
from time import sleep  # Handle time
from std_msgs.msg import String


"""
userdata.response_type:
    'init' - Initial state responded
    'answer' - User answered a question
    'not_answer' - User did not answer a question (timeout) try 2 times repeat the same question then move on
    'hint' - User asked for a hint (remove 2 options from the question)
"""


class Initial(State):
    def __init__(self):
        State.__init__(self, outcomes=['start'],  input_keys=['data_in'],
                       output_keys=['data_out', 'response_type'])

    def execute(self, userdata):
        pub = rospy.Publisher('robot_initial_speech', String, queue_size=10)
        sec = 2
        rospy.loginfo(f'In {self.__class__.__name__} state for {sec} seconds')
        sleep(sec)
        # rospy.loginfo(userdata.data_in)
        userdata.response_type = 'init'
        pub.publish(userdata.data_in['story'])
        # Robot reads the Initial greeting sentence in the experiment
        return 'start'


class HumanTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['respond_to_robot'], input_keys=[
                       'data_in'], output_keys=['data_out', 'response_type'])

    def execute(self, userdata):
        sec = 5
        rospy.loginfo(f'In {self.__class__.__name__} state for {sec} seconds')
        sleep(sec)

        data_dict_out = userdata.data_in
        data_dict_out['current_question'] = userdata.data_in['current_question']
        try:
            timeout = 10
            userdata.response_type = 'answer'
            rospy.loginfo(f'Waiting {timeout} seconds for user response')

            data_dict_out['response'] = rospy.wait_for_message(
                'human_response', String, timeout=timeout).data
            # rospy.loginfo(userdata.data_out['response'])
            # msg = rospy.wait_for_message('response', String, timeout=10)
        except rospy.ROSException as e:
            rospy.logerr(e)
            userdata.response_type = 'not_answer'
            return 'respond_to_robot'

        userdata.data_out = data_dict_out
        return 'respond_to_robot'


class RobotTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['question', 'finish'], input_keys=[
                       'data_in', 'response_type'], output_keys=['data_out'])
        self.count_not_answer = 0
        self.pub = rospy.Publisher('say', String, queue_size=10)

    def execute(self, userdata):
        sec = 5
        rospy.loginfo(f'In {self.__class__.__name__} state for {sec} seconds')
        sleep(sec)

        data_dict_out = {}
        data_dict_out = userdata.data_in

        if userdata.response_type == 'answer':
            current_question = userdata.data_in['current_question']
            current_question.check(userdata.data_in['response'])

        elif userdata.response_type == 'not_answer':
            self.count_not_answer += 1
            if self.count_not_answer == 2:
                current_question.done == True
            print("User did not answer, what to do?")
        
        elif userdata.response_type == 'hint':
            print("User asked for a hint!")

        for q in userdata.data_in['quiz_questions']:
            if q.done == False:
                rospy.loginfo(q.question)
                self.pub.publish(q.question)
                data_dict_out['current_question'] = q
                userdata.data_out = data_dict_out
                self.count_not_answer = 0
                return 'question'
        
        rospy.loginfo('All questions done')
        return 'finish'

        # # userdata.data_in['quiz_questions']

        # self.pub.publish('I am the robot and i am talking')
        # if self.c <=10:
        #     return 'question'
        # else:
        #     return 'finish'


class Final(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=[
                       'data_in'], output_keys=['data_out'])

    def execute(self, userdata):
        sec = 2
        rospy.loginfo(f'In {self.__class__.__name__} state for {sec} seconds')
        sleep(sec)
        # rospy.loginfo('The user entered: ' + str(userdata.data_in))
        return 'succeeded'

# Main method


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

        StateMachine.add('Initial', Initial(), transitions={'start': 'RobotTurn'}, remapping={'data_in': 'sm_input',
                                                                                              'data_out': 'sm_input'})
        StateMachine.add('RobotTurn', RobotTurn(), transitions={
                         'question': 'HumanTurn', 'finish': 'Final'}, remapping={'data_in': 'sm_input',
                                                                                 'data_out': 'sm_input'})
        StateMachine.add('HumanTurn', HumanTurn(), transitions={
                         'respond_to_robot': 'RobotTurn'}, remapping={'data_in': 'sm_input',
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
