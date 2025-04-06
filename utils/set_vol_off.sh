#!/bin/bash

~/GoPi5Go/plib/speak.py 'Volume set off at 0 percent' 45
amixer -D pulse sset Master 0%
