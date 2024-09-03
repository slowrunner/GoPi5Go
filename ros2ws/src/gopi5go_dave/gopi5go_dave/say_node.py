#!/usr/bin/env python3

# FILE: say_node.py

"""
    Uses piper TTS and aplay to speak string phrases sent to /say service request

    dave_interfaces.srv.Say.srv
        string saystring
        ---
        bool spoken

    CLI:   ros2 service call /say dave_interfaces/srv/Say "saystring: 'hello'"

    DOCKER FILE REQ:
        sudo apt install \
            alsa-base \
            alsa-utils \
            libsndfile1-dev \
            libportaudio2 \

        sudo pip3 install piper-tts

    DOCKER INVOCATION REQ:
        docker run -dit --name r2hdp --net=host \
            -v /dev/snd:/dev/snd \                           <<----
            -v /home/pi:/home/pi -v /dev/input:/dev/input \
            -v /dev/bus/usb:/dev/bus/usb  \
            -e TZ=America/New_York \
            -w /home/pi/GoPi5Go/ros2ws --privileged r2hdp 

    CONTROL VOLUME: (cmds/set_vol_very_low.sh)
        #!/bin/bash
        # FILE: cmds/set_vol_very_low.sh
        amixer -D pulse sset Master 10%
        ~/GoPi5Go/ros2ws/cmds/say.sh 'Volume set very low 10 percent'

    DEVICE:  2,0 

"""

# DEBUG = True
DEBUG = False

from dave_interfaces.srv import Say

import rclpy
from rclpy.node import Node
import sys
import logging
import wave
import os
from piper.voice import PiperVoice
from time import sleep
import subprocess
import datetime as dt


filename = 'temp.say_node.wav'

DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

# voicedir = os.path.expanduser('~/GoPi5Go/models/piper-tts/') #Where onnx model files are stored on my machine
voicedir = '/home/pi/GoPi5Go/models/piper-tts/'  # Where onnx model files are stored on my machine
model = voicedir+"en_US-arctic-medium.onnx"
if DEBUG: print("say_node: model=",model)
voice = PiperVoice.load(model)
if DEBUG: print("PiperVoice Object Created")


class SayService(Node):

    def __init__(self):
        super().__init__('say')
        self.srv = self.create_service(Say, 'say', self.say_cb)
        # create logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

        self.loghandler = logging.FileHandler('/home/pi/GoPi5Go/logs/speak.log')

        logformatter = logging.Formatter('%(asctime)s|%(message)s',"%Y-%m-%d %H:%M")
        self.loghandler.setFormatter(logformatter)
        self.logger.addHandler(self.loghandler)





    def say_cb(self, request, response):
        text = request.saystring
        logStr='Say request:"{}"'.format(text)
        self.get_logger().info(logStr)
        if DEBUG: 
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,logStr)

        wav_file = wave.open(filename, 'w')
        audio = voice.synthesize(text,wav_file)
        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"say_node: file written, speaking phrase")

        # subprocess.check_output(['aplay -D plughw:2,0 -r 22050 -f S16_LE ' + filename], stderr=subprocess.STDOUT, shell=True)
        os.system('aplay -D plughw:2,0 -r 22050 -f S16_LE ' + filename)

        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"say_node: after aplay")

        os.remove(filename)
        if DEBUG:
            dtstr = dt.datetime.now().strftime(DT_FORMAT)
            print(dtstr,"say_node: file removed")

        response.spoken = True
        self.logger.info(text + " - spoken: " + str(response.spoken) )

        return response


def main():
    rclpy.init()

    say_svc = SayService()

    try:
        rclpy.spin(say_svc)
    except KeyboardInterrupt:
        pass

    # rclpy.shutdown()


if __name__ == '__main__':
    main()




