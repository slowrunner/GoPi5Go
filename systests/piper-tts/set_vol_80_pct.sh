#!/bin/bash

amixer -D pulse sset Master 80%
~/GoPi5Go/systests/piper-tts/piper.sh 'Volume set to 80 percent' 
