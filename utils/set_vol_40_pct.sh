#!/bin/bash

~/GoPi5Go/plib/speak.py 'Volume set to 40 percent' 40
amixer -D pulse sset Master 40%
