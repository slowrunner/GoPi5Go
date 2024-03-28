#!/bin/bash

amixer -D pulse sset Master 100%
~/GoPi5Go/plib/speak.py 'Volume set to 100 percent'
