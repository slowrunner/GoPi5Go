#!/bin/env python3

# REQ:  sudo apt install python3-pyaudio
# NOTE:  ALSA will generate a million prints - ignore

import pyaudio
import wave
import os
from piper.voice import PiperVoice


filename = 'temp.wav'

voicedir = os.path.expanduser('~/wali_pi5/c3ws/models/piper-tts/') #Where onnx model files are stored on my machine
model = voicedir+"en_US-arctic-medium.onnx"
voice = PiperVoice.load(model)
wav_file = wave.open(filename, 'w')
text = "This temporary wave file was generated from Python piper to wave to pyaudio dot PY."
audio = voice.synthesize(text,wav_file)



# Set chunk size of 1024 samples per data frame
chunk = 1024  

# Open the sound file 
wf = wave.open(filename, 'rb')

# Create an interface to PortAudio
p = pyaudio.PyAudio()

# Open a .Stream object to write the WAV file to
# 'output = True' indicates that the sound will be played rather than recorded
stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
                channels = wf.getnchannels(),
                rate = wf.getframerate(),
                output = True)

num_frames = wf.getnframes()

# print("num frames: ",num_frames)

# Read data in chunks
data = wf.readframes(chunk)

frames = 1
# Play the sound by writing the audio data to the stream
# while data != '':
while frames < num_frames:
    stream.write(data)
    # print("frames written: ", frames)
    data = wf.readframes(chunk)
    frames += 1

# Close and terminate the stream
stream.close()
p.terminate()
