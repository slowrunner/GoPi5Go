#!/bin/bash


# pushd /home/pi/GoPi5Go/systests/piper-tts
# echo $1 | piper   --model en_US-lessac-medium.onnx    --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw - 
echo $1 | piper   --model /home/pi/GoPi5Go/models/piper-tts/en_US-arctic-medium.onnx    --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw - 
# popd
