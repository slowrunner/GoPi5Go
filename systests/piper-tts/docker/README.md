# Piper-tts in Docker


*=== Installation
1) Add to dockerfile and rebuild container:

```
# Install package dependencies
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        alsa-base \
        alsa-utils \
        python3-pip \
        libsndfile1-dev && \
    apt-get clean

RUN pip3 install piper-tts
```

2) Create piper-tts model folder and get desired voice model file
```
mkdir -p /home/pi/GoPi5Go/models/piper-tts
cd /home/pi/GoPi5Go/models/piper-tts
# This will download the voice model
echo 'Welcome to the world of speech synthesis!' | piper --model en_US-lessac-medium --output_file welcome.wav
rm welcome.wav
```

3) Test aplay inside Docker container:  
```
./test_aplay.sh
```

4) Test piper-tts inside Docker container:  
```
./piper.sh "hello"
```


