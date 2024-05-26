#!/bin/bash

mkdir -p /home/pi/GoPi5Go/models/piper-tts
echo -e "Download the arctic voice to ~/GoPi5Go/models/piper-tts"
pushd ~/GoPi5Go/models/piper-tts

echo 'Hello.  This is arctic medium voice' | piper  --model en_US-arctic-medium     --output_file /dev/null

popd
