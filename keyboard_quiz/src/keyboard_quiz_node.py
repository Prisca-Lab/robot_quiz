import rospy
from picovoice_ros.srv import AskUser, AskUserResponse
from pynput import keyboard



def get_key():
    with keyboard.Events() as events:
        for event in events:
            if event.key == keyboard.Key.esc:
                break
            else:
                pass
                # print('Received event {}'.format(event))
            return event.key


def get_intent():
    rospy.logwarn("Press on keyboard to get intent [1, 2, 3, 4, s, r]")
    key = None
    res = None
    while not res:
        key_raw = get_key()
        if hasattr(key_raw, 'char'):
            key = key_raw.char
            if key == "1":
                res = "risposta_uno"
            elif key == "2":
                res = "risposta_due"
            elif key == "3":
                res = "risposta_tre"
            elif key == "4":
                res = "risposta_quattro"
            elif key == "s":
                res = "suggerimento"
            elif key == "r":
                res = "ripeti"
            else:
                res = "no_answer"
            return res
        else:
            rospy.logdebug("Not a char")


class KeyboardQuizNode:
    def __init__(self) -> None:
        self.service = rospy.Service('ask_user', AskUser, self.handle_user)

    def handle_user(self, req):
        val = get_intent()
        rospy.loginfo(f'Intent: {val}')
        return AskUserResponse(val)



if __name__ == '__main__':
    rospy.init_node('keyboard_node')

    rospy.loginfo("Intent from Keyboard")
    K = KeyboardQuizNode()
    rospy.spin()
