#!/bin/bash

amixer -D pulse sset Master 40%
~/GoPi5Go/plib/speak.py 'Volume set to 40 percent'
