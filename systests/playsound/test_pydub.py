#!/usr/bin/env python3

from pydub import AudioSegment
from pydub.playback import play
from time import sleep

phrase = AudioSegment.from_wav('temp.say_node.wav')
print('playing sound using pydub')

play(phrase,output_device="plughw:2,0")
# play(phrase)
sleep(10)
