#!/bin/bash

~/GoPi5Go/plib/speak.py 'Volume set to 80 percent' 80
amixer -D pulse sset Master 80%
