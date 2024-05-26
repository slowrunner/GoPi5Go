#!/bin/bash


# was device 2,0 before setting up usb0 networking/gadget mode
# changed to device 0,0
# $ aplay -l
# **** List of PLAYBACK Hardware Devices ****
# card 0: UACDemoV10 [UACDemoV1.0], device 0: USB Audio [USB Audio]
#   Subdevices: 1/1
#   Subdevice #0: subdevice #0

if [ "$#" -ne 1 ] ;
	then echo 'Usage:  ./say.sh "string to speak" '
	exit
fi


# echo $1 | piper   --model /home/pi/wali_pi5/c3ws/models/piper-tts/en_US-arctic-medium.onnx  --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw - 
echo $1 | piper   --model /home/pi/GoPi5Go/models/piper-tts/en_US-arctic-medium.onnx  --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw - 
