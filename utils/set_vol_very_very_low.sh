#!/bin/bash

~/GoPi5Go/plib/speak.py 'Volume set very very low at 45 percent' 45
amixer -D pulse sset Master 45%
