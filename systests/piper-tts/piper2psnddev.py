#!/bin/env python3

# REQ:
#  sudo pip3 install sounddevice --break-system-packages
#  sudo pip3 install soundfile --break-system-packages
#  sudo pip3 install piper-tts --break-system-packages


import sounddevice as sd
import soundfile as sf
import wave
import os
from piper.voice import PiperVoice


filename = 'temp.wav'

voicedir = os.path.expanduser('~/wali_pi5/c3ws/models/piper-tts/') #Where onnx model files are stored on my machine
model = voicedir+"en_US-arctic-medium.onnx"
voice = PiperVoice.load(model)
wav_file = wave.open(filename, 'w')
text = "This temporary wave file was generated from Python call to piper output to python sound device"
audio = voice.synthesize(text,wav_file)

# Extract data and sampling rate from file
data, fs = sf.read(filename, dtype='float32')
sd.play(data, fs)
status = sd.wait()  # Wait until file is done playing
os.remove(filename)
