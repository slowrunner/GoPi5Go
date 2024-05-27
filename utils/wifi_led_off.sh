#!/bin/bash

# turns off "WiFi Led" - the yellow low battery on GoPi5Go-Dave
python3 -c "import gopigo3;GPG=gopigo3.GoPiGo3();GPG.set_led(GPG.LED_WIFI,0,0,0)"
