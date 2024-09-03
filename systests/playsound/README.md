# python playsound in ROS 2 test

### playsound: no work

```
  pip3 install playsound
  from playsound import playsound
  playsound('file.wav')


  File "/home/pi/.local/lib/python3.10/site-packages/playsound.py", line 162, in _playsoundNix
    import gi
  ModuleNotFoundError: No module named 'gi'
```

### pydub module: no work

  sudo apt install ffmpeg libavcodec-extra
  pip3 install pydub

  from pydub import AudioSegment
  from pydub.playback import play

  song = AudioSegment.from_wav("file.wav")
  play(song)

## ONLY WAY THAT WORKS os.system
(I was using subprocess, but aplay was crashing)

import os

filename = "file.wav"
os.system('aplay -D plughw:2,0 -r 22050 -f S16_LE ' + filename)
