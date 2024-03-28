#!/bin/bash

amixer -D pulse sset Master 80%
~/GoPi5Go/plib/speak.py 'Volume set to 80 percent' 
