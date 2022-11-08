
import smach_ros
from smach import StateMachine
import rospy
from script.states import Initial, HumanTurn, Final, CheckAnswer, RobotSpeak, Quiz, RepeatTurn, Hint


class StateManager:
    def __init__(self, data_in) -> None:
        self.sm = StateMachine(outcomes=['succeeded', 'failed'])
        self.sm.userdata.sm_input = data_in

        with self.sm:

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
        self.sis = smach_ros.IntrospectionServer(
            'server_name', self.sm, '/SM_ROOT')
        self.sis.start()

    def __del__(self):
        rospy.logdebug('Destroy state machine')
        self.sis.stop()