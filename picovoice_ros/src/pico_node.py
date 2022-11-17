#!/usr/bin/env python
import os
import sys

from picovoice_ros.srv import AskUser, AskUserResponse

import pvrhino
from pvrecorder import PvRecorder
import wave
import struct
import time

import rospy


MODEL = os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), 'picovoice/rhino/', 'rhino_params_it.pv')
CONTEXT = os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), 'picovoice/rhino/', 'quiz_it_linux_v2_1_0.rhn')


class IntentRecognition:

    def __init__(
            self,
            access_key="30uXi1g4+23OWdquCouuIax2CfKWmTzfvC9IFkSrYmhKTAapX1q4Zg==",
            library_path=pvrhino.LIBRARY_PATH,
            model_path=MODEL,
            context_path=CONTEXT,
            endpoint_duration_sec=1.0,
            require_endpoint=True,
            audio_device_index=-1,
            output_path=None):

        self._access_key = access_key
        self._library_path = library_path
        self._model_path = model_path
        self._context_path = context_path
        self._endpoint_duration_sec = endpoint_duration_sec
        self._require_endpoint = require_endpoint
        self._audio_device_index = audio_device_index
        self._output_path = output_path
        try:
            self.rhino = pvrhino.create(
                access_key=self._access_key,
                library_path=self._library_path,
                model_path=self._model_path,
                context_path=self._context_path,
                endpoint_duration_sec=self._endpoint_duration_sec,
                require_endpoint=self._require_endpoint)
        except pvrhino.RhinoInvalidArgumentError as e:
            args = (
                self._access_key,
                self._library_path,
                self._model_path,
                self._context_path,
                self._require_endpoint
            )
            print("One or more arguments provided to Rhino is invalid: ", args)
            print("If all other arguments seem valid, ensure that '%s' is a valid AccessKey" %
                  self._access_key)
            raise e
        self.intent = "no_answer"
        print("Intent started")

    def run(self, max_seconds=10):
        """
         Creates an input audio stream, instantiates an instance of Rhino object, and infers the intent from spoken
         commands.
         """

        self.recorder = PvRecorder(
            device_index=self._audio_device_index, frame_length=self.rhino.frame_length)
        rospy.logdebug(self.recorder.get_audio_devices())
        self.recorder.start()

        self.wav_file = None

        try:
            if self._output_path is not None:
                self.wav_file = wave.open(self._output_path, "w")
                self.wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            # rospy.loginfo(self.rhino.context_info)

            rospy.loginfo(f"Using device: {self.recorder.selected_device}")
            rospy.loginfo(f"Detecting intent for {max_seconds} seconds")
            start = time.time()
            while time.time() - start < max_seconds:
                pcm = self.recorder.read()

                if self.wav_file is not None:
                    self.wav_file.writeframes(
                        struct.pack("h" * len(pcm), *pcm))

                is_finalized = self.rhino.process(pcm)
                if is_finalized:
                    inference = self.rhino.get_inference()
                    if inference.is_understood:
                        # print('{')
                        # print("  intent : '%s'" % inference.intent)
                        # print('  slots : {')
                        # for slot, value in inference.slots.items():
                        #     print("    %s : '%s'" % (slot, value))
                        # print('  }')
                        # print('}\n')
                        self.intent = inference.intent
                        self.recorder.delete()
                        return self.intent
                    else:
                        # self.intent = None
                        rospy.loginfo("Didn't understand the command.\n")
            self.recorder.delete()
            return self.intent
        except Exception as e:
            rospy.logerr(e)
        self.recorder.delete()

    def stop(self):
        if self.recorder is not None:
            self.recorder.delete()

        if self.rhino is not None:
            self.rhino.delete()

        if self.wav_file is not None:
            self.wav_file.close()


class PicoNode:
    def __init__(self, device_idx) -> None:
        self.service = rospy.Service('ask_user', AskUser, self.handle_user)
        self.intent_recognition = IntentRecognition(audio_device_index=device_idx)

    def handle_user(self, req):
        val = self.intent_recognition.run()
        rospy.loginfo(f'Intent: {val}')
        return AskUserResponse(val)


if __name__ == '__main__':
    rospy.init_node('pico_node')
    rospy.myargv(argv=sys.argv)

    device_idx = rospy.get_param("~device")
    rospy.logdebug(f'Available audio devices {PvRecorder.get_audio_devices()}')
    rospy.loginfo(f"Selected device at index: {device_idx}")
    P = PicoNode(device_idx)
    rospy.spin()
