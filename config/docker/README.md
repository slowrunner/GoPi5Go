#  Setup ROS 2 Humble in Docker on Pi5 PiOS Bookworm

## Resulting Execution:

Boot starts /etc/systemd/system/docker.gopi5goROS2.service  
- after 60s invokes /home/pi/GoPi5Go/ros2ws/run_docker_detached_gopi5goROS2.sh  
- which runs /home/pi/GoPi5Go/ros2ws/start_GoPi5Go-Dave.sh
- which starts:  
  - battery_node:  publishes /battery_state  
  - docking_node:  publishes /dock_status, offers /dock and /undock services  
  - dave_node:     calls /dock when vBatt<10v, and calls /undock when charge current < -175mA  
  - odometer:      records ROS all /cmd_vel movement (does not record dock/undock movement)  
  - joy_node:      handles wireless F710 joy controller to publish /cmd_vel  
  - say_node:      TTS speech server offers /say {"phrase"} service  


## Setup Process:  

REF: https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html#raspberry-pi-os-with-ros-2-in-docker  

### install Docker  

- 1_setup_docker_apt_repo.sh  
- 2__install_docker_pkgs.sh   

Hello from Docker!  
This message shows that your installation appears to be working correctly.  

# Create ros2ws/src directory  

mkdir -p ~/GoPi5Go/ros2ws/src  

### Build ROS2 Humble Desktop Plus Docker Image  

```
./6_build_humble_desktop_plus_container.sh  
```

### Run with:  ./run_docker_r2hdp.sh  

```
docker run -it --net=host  -v /home/pi:/home/pi -v /dev/input:/dev/input \
 -v /dev/bus/usb:/dev/bus/usb  -w /home/pi/GoPi5Go/ros2ws --privileged --rm r2hdp
```

### Build gopi5gor2hdp image:  GoPi5Go-Dave additions to ROS2 Humble Desktop Plus Image  

```
./7_build_gopi5gor2hdp_image.sh  
```

### Run image as gopi5goROS2 named container:  ./run_gopi5goROS2.sh  

```
docker run -dt --net=host \
 -v /dev/snd:/dev/snd \
 -v /dev/input:/dev/input \
 -v /home/pi:/home/pi \
 -v /dev/bus/usb:/dev/bus/usb \
 -v /dev/ttyUSB0:/dev/ttyUSB0 \
 -v /var/lock:/var/lock \
 -e TZ=America/New_York \
 -w /home/pi/GoPi5Go/ros2ws \
 --privileged \
 --rm \
 --name gopi5goROS2 \
 gopi5gor2hdp

```

## Setup service to auto start the gopi5goROS2 container at boot  

```
./install_docker_gopi5goROS2_dot_service.sh
sudo systemctl enable gopi5goROS2
sudo systemctl start gopi5goROS2
```

## Create /home/pi/GoPi5Go/ros2ws/start_GoPi5Go-Dave.sh  

This file should start the needed GoPi5Go-Dave nodes:  
- battery_node  publishes /batter_state  
- docking_node  publishes /dock_status, offers /dock and /undock services  
- dave_node     calls /dock when vBatt<10v, and calls /undock when charge current < -175mA  
- odometer      records ROS all /cmd_vel movement (does not record dock/undock movement)  
- say_node      TTS speech server offers /say {"phrase"} service  
- joy_node      handles wireless F710 joy controller to publish /cmd_vel  


## Setup to start GoPi5Go-Dave nodes when gopi5goROS2 container starts  

- Add to GoPi5Go/ros2ws/run_detached_gopi5goROS2.sh:  

```
 -w /home/pi/GoPi5Go/ros2ws \
```

- Add to end of ..docker/docker_files/gopi5go_r2hdp_dockerfile:  

```
CMD bash -c "source start_GoPi5Go-Dave.sh"
```


## CLEAN UP AFTER A REBUILD  

```
 $ docker image list  
REPOSITORY   TAG       IMAGE ID       CREATED       SIZE  
r2hdp        latest    a32732c7e869   2 hours ago   3.41GB  
<none>       <none>    beceb7b58a39   2 days ago    3.37GB  


$ docker image rm beceb7b58a39  
Error response from daemon: conflict: unable to delete beceb7b58a39 (must be forced) - image is being used by stopped container 9b9ae547247a  

$ docker container list -a  
pi@GoPi5Go:~/GoPi5Go/config/docker $ docker image list
REPOSITORY     TAG       IMAGE ID       CREATED       SIZE
gopi5gor2hdp   latest    0b695659bfa0   13 days ago   4.07GB
r2hdp          latest    1d518df34aa5   2 weeks ago   3.92GB

$ docker image rm -f beceb7b58a39  
Deleted: sha256:beceb7b58a39f7cd26fd32bbbc222d7fe9b591682a40a07ff9bef9be426d92d0  
```


### or use docker container prune to remove all stopped containers  

```
$ docker container rm -f 9b9ae547247a  
9b9ae547247a  

$ docker image list  
REPOSITORY   TAG       IMAGE ID       CREATED       SIZE  
r2hdp        latest    a32732c7e869   2 hours ago   3.41GB  
r2hd         latest    20eb98163bc8   2 days ago    3.23GB  


```


## Joystick access from Docker  

- Required apt packages  ```RUN apt install -y xxx yyy```  
  - joystick  
  - python3-pip  

- Since running on PiOS, evdev will need /home/pi/ to exist  
  - ```RUN useradd -s /bin/bash pi``` will create /home/pi/  

- Required python packages  
  - evdev  ```RUN sudo pip3 install evdev```  

- Copy gamepad config file into container  

```
# files to copy must be in the docker tree

COPY snes_slow.config.yaml /opt/ros/humble/share/teleop_twist_joy/config/
```

- Run in --privileged mode  

```
docker run -it --net=host  -v /home/pi:/home/pi -v /dev/input:/dev/input \
 -v /dev/bus/usb:/dev/bus/usb  -w /home/pi/pi5desk/c3ws --privileged --rm r2hdp
```


### UTILITIES:
- du_docker.sh         show how much disk space used by Docker image(s)
- list_images.sh  
- list_containers.sh
- delete_docker_images.sh
- prune_build_cache.sh
- run_r2hdp.sh
- run_gopi5goROS2.sh
- restart_gopi5goROS2.sh

