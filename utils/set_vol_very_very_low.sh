#!/bin/bash

amixer -D pulse sset Master 40%
~/GoPi5Go/plib/speak.py 'Volume set very very low at 40 percent'
