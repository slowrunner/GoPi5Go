#!/usr/bin/env python3

import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice

# voicedir = "./voices/" #Where onnx model files are stored on my machine
voicedir = "/home/pi/GoPi5Go/models/piper-tts/" #Where onnx model files are stored on my machine
model = voicedir+"en_US-lessac-medium.onnx"
voice = PiperVoice.load(model)
text = "This is an example of text-to-speech using Piper TTS."
print("voice.config.sample_rate:",voice.config.sample_rate)
print("sd.default.device:",sd.default.device)
# sd.default.device="USB Audio"
sd.default.device="2,0"
print("sd.default.device:",sd.default.device)
sd.default.samplerate=22050
print("sd.default.samplerate:",sd.default.samplerate)

# Setup a sounddevice OutputStream with appropriate parameters
# The sample rate and channels should match the properties of the PCM data
# stream = sd.OutputStream(samplerate=voice.config.sample_rate, channels=1, dtype='int16')
stream = sd.OutputStream(samplerate=22050, channels=1, dtype='int16')
stream.start()

for audio_bytes in voice.synthesize_stream_raw(text):
    int_data = np.frombuffer(audio_bytes, dtype=np.int16)
    stream.write(int_data)

stream.stop()
stream.close()
