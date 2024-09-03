#!/usr/bin/env python3

import os
from time import sleep

phrase = 'temp.say_node.wav'
print('playing sound using os.system')

os.system('aplay -D plughw:2,0 -r 22050 -f S16_LE ' + phrase)
sleep(10)
