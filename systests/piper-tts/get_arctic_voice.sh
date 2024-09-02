#!/bin/bash

# mkdir -p /home/pi/GoPi5Go/models/piper-tts
mkdir ./tmp
echo -e "Downloading the arctic voice to ./tmp"
# echo -e "Downloading the arctic voice to ~/GoPi5Go/models/piper-tts"
pushd tmp

echo 'Hello.  This is arctic medium voice' | piper  --model en_US-arctic-medium     --output_file /dev/null

popd
