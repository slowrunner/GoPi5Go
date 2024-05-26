#!/bin/bash

amixer -D pulse sset Master 40%
~/GoPi5Go/systests/piper-tts/piper.sh 'Volume set to 40 percent'
