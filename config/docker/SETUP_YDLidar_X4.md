# Setup YDLIDAR X4 in Docker on GoPi5Go-Dave

2024-08-14


=== Prereqs
- Docker  
- Linux OS: Using Ubuntu 22.04 LTS 64-bit  
- External 5v power connected to microUSB of Interface Card  
- USB A to USB C cable from RPi to Interface Card  



Add to dockerfile: 
```
RUN sudo apt-get install -y swig   (or add "swig \" to existing apt-get)
 
# Install the YDLidar-SDK

COPY ydlidar.rules /etc/udev/rules.d
COPY ydlidar-2303.rules /etc/udev/rules.d
COPY ydlidar-V2.rules /etc/udev/rules.d

ADD YDLidar-SDK /home/pi/YDLidar-SDK
WORKDIR /home/pi/YDLidar-SDK
RUN sudo mkdir /home/pi/YDLidar-SDK/build
WORKDIR /home/pi/YDLidar-SDK/build
RUN sudo cmake /home/pi/YDLidar-SDK
RUN sudo make
RUN sudo make install
WORKDIR /home/pi/YDLidar-SDK
RUN sudo pip3 install .```
```

Create mk_ydlidar_rules.sh
```
#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >ydlidar.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >ydlidar-V2.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >ydlidar-2303.rules
```

Run it once:
```
cd ../docker/
chmod +x mk_ydlidar_rules.sh
./mk_ydlidar_rules.sh
```



Add to 7_build_gopi5gor2hdp.sh
```
#!/bin/bash

# FILE: 7_build_gopi5gor2hdp.sh

# This is an extention of 7_build_gopi5gor2hdp.sh that adds the GoPi5Go API and the YDLidar-SDK to the base r2hdp container

cd ~/GoPi5Go/config/docker
if [ -d YDLidar-SDK ]; then
    echo -e "Removing existing docker/YDLidar-SDK/" 
    rm -rf YDLidar-SDK
fi
echo -e "Bring down clean YDLidar-SDK"
git clone https://github.com/YDLIDAR/YDLidar-SDK

echo -e "copy in my ydlidar_test_interactive.cpp example"
cp ~/GoPi5Go/systests/myYDLidar-SDK/ydlidar_test_interactive.cpp YDLidar-SDK/examples


# build a ros humble desktop plus navigation, slam-toolbox, localization, GoPi5Go API, with ydlidar sdk installed, tagged ydlidar
sudo docker build --no-cache . -t gopi5goROS2 -f docker_files/gopi5go_r2hdp_dockerfile

```

./7_build_gopi5gor2hdp.sh
```


run_[detached_]gopigo3r2hdp.sh:
```
#!/bin/bash

cd ~/GoPi5Go/ros2ws
# --rm    remove container after running

docker run -it --net=host \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -v /dev/ttyUSB0:/dev/ttyUSB0 \           <<-- added for ydlidar
 -v /var/lock:/var/lock  \
 -e TZ=America/New_York \
 -w /home/pi/GoPi5Go/ros2ws \
 --privileged \
 --rm \
 ydlidar

```


TEST:  IN DOCKER:
```
ubuntu@ROS2HH:~/YDLidar-SDK/startup$ ls -al /dev/ttyUSB0
crw-rw-rw- 1 root dialout 188, 0 Oct 15 23:47 /dev/ttyUSB0
```

=== Run ~/GoPi5Go/systests/myYDLidar-SDK/ydlidar_test.py

```
pi@GoPi5Go:DOCKER:~/GoPi5Go/systests/myYDLidar-SDK $ ./ydlidar_test.py 
[YDLIDAR] SDK initializing
[YDLIDAR] SDK has been initialized
[YDLIDAR] SDK Version: 1.2.5
[YDLIDAR] Lidar successfully connected [/dev/ttyUSB0:128000]
[YDLIDAR] Lidar running correctly! The health status: good
[YDLIDAR] Fail to get baseplate device information
[YDLIDAR] Lidar init success, Elapsed time 633 ms
[YDLIDAR] Start to getting intensity flag
[YDLIDAR] End to getting intensity flag
[YDLIDAR] Create thread 0xCF013120
[YDLIDAR] Successed to start scan mode, Elapsed time 4563 ms
[YDLIDAR] Single Fixed Size: 580
[YDLIDAR] Sample Rate: 5.00K
[YDLIDAR] Successed to check the lidar, Elapsed time 425 ms
[2024-05-20 01:17:10][info] [YDLIDAR] Now lidar is scanning...
scan.config.scan_time: 0.0


*** Scan received[ 1716182230710948000 ]: 574 ranges is [ 9 ]Hz
angle: -0.23561936616897583  range:  0.0
angle: -0.2255294919013977  range:  0.0
angle: -0.21462088823318481  range:  0.0
angle: -0.20371276140213013  range:  0.0
angle: -0.19280415773391724  range:  0.0
angle: -0.18189603090286255  range:  0.0
angle: -0.17098790407180786  range:  0.0
angle: -0.16007930040359497  range:  0.0
angle: -0.1488984227180481  range:  0.0
angle: -0.2740710377693176  range:  6.729000091552734
angle: -0.12708169221878052  range:  0.0
angle: -0.11617357283830643  range:  0.0
angle: -0.10526496917009354  range:  0.0
angle: -0.09435684233903885  range:  0.0
angle: -0.08317596465349197  range:  0.0
angle: -0.20562154054641724  range:  3.611999988555908
angle: -0.0613592304289341  range:  0.0
angle: -0.05045110359787941  range:  0.0
angle: -0.17316943407058716  range:  3.8010001182556152
angle: -0.16226130723953247  range:  3.7279999256134033
angle: -0.1510799527168274  range:  3.6570000648498535
angle: -0.13989907503128052  range:  3.5889999866485596
angle: -0.12899094820022583  range:  3.559999942779541
angle: -0.11808235198259354  range:  3.5299999713897705
angle: -0.10717422515153885  range:  3.5160000324249268
angle: -0.09626562148332596  range:  3.50600004196167
angle: -0.08508474379777908  range:  3.4779999256134033
angle: -0.07390386611223221  range:  3.4649999141693115
```


Run ydlidar_test_interactive 
```
pi@GoPi5Go:DOCKER:~/GoPi5Go/systests/myYDLidar-SDK $ ydlidar_test_interactive 
__   ______  _     ___ ____    _    ____  
\ \ / /  _ \| |   |_ _|  _ \  / \  |  _ \ 
 \ V /| | | | |    | || | | |/ _ \ | |_) | 
  | | | |_| | |___ | || |_| / ___ \|  _ <  
  |_| |____/|_____|___|____/_/   \_\_| \_\ 

Baudrate:
0. 115200
1. 128000
2. 153600
3. 230400
4. 512000
Please select the lidar baudrate:1
Whether the Lidar is one-way communication[yes/no]:no
Please enter the lidar scan frequency[5-12]:5
[YDLIDAR] SDK initializing
[YDLIDAR] SDK has been initialized
[YDLIDAR] SDK Version: 1.2.5
[YDLIDAR] Lidar successfully connected [/dev/ttyUSB0:128000]
[YDLIDAR] Lidar running correctly! The health status: good
[YDLIDAR] Baseplate device info
Firmware version: 1.10
Hardware version: 1
Model: X4
Serial: 2020062200002315
[YDLIDAR] Lidar init success, Elapsed time 687 ms
[YDLIDAR] Start to getting intensity flag
[YDLIDAR] End to getting intensity flag
[YDLIDAR] Create thread 0x28727100
[YDLIDAR] Successed to start scan mode, Elapsed time 4563 ms
[YDLIDAR] Fixed Size: 1020
[YDLIDAR] Sample Rate: 5.00K
[YDLIDAR] Successed to check the lidar, Elapsed time 0 ms
[2024-05-20 01:19:41][info] [YDLIDAR] Now lidar is scanning...
Scan received[1716182380991912000]: 578 ranges is [inf]Hz
Scan received[1716182381114450000]: 575 ranges is [8.160734]Hz
Scan received[1716182381229298000]: 574 ranges is [8.707161]Hz
Scan received[1716182381336961000]: 570 ranges is [9.288242]Hz
Scan received[1716182381458714000]: 575 ranges is [8.213350]Hz
Scan received[1716182381574163000]: 575 ranges is [8.661834]Hz
Scan received[1716182381688706000]: 575 ranges is [8.730346]Hz
Scan received[1716182381804711000]: 574 ranges is [8.620318]Hz
Scan received[1716182381919389000]: 574 ranges is [8.720068]Hz
Scan received[1716182382034250000]: 574 ranges is [8.706176]Hz
Scan received[1716182382148415000]: 569 ranges is [8.759252]Hz
Scan received[1716182382256073000]: 574 ranges is [9.288673]Hz
```





