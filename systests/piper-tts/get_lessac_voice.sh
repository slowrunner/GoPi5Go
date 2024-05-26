#!/bin/bash

mkdir -p /home/pi/GoPi5Go/models/piper-tts
echo -e "Download the lessac voice to ~/GoPi5Go/models/piper-tts"
pushd ~/GoPi5Go/models/piper-tts

echo 'Hello.  This is lessac medium voice' | piper  --model en_US-lessac-medium     --output_file /dev/null

popd
