#!/bin/env python3

import os
import wave
from piper.voice import PiperVoice

voicedir = os.path.expanduser('~/wali_pi5/c3ws/models/piper-tts/') #Where onnx model files are stored on my machine
model = voicedir+"en_US-arctic-medium.onnx"
voice = PiperVoice.load(model)
wav_file = wave.open('piper2wav.wav', 'w')
text = "This wave file was generated from Python piper to wave dot PY."
audio = voice.synthesize(text,wav_file)
