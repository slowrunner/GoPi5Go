#!/bin/bash

~/GoPi5Go/plib/speak.py 'Volume set to 100 percent' 100
amixer -D pulse sset Master 100%
