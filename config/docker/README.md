# Install ROS 2 Humble To Pi5 PiOS Bookworm In Docker

REF: https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html#raspberry-pi-os-with-ros-2-in-docker

** install Docker
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



## CLEAN UP AFTER A REBUILD  

```
 $ docker image list  
REPOSITORY   TAG       IMAGE ID       CREATED       SIZE  
r2hdp        latest    a32732c7e869   2 hours ago   3.41GB  
<none>       <none>    beceb7b58a39   2 days ago    3.37GB  
r2hd         latest    20eb98163bc8   2 days ago    3.23GB  


$ docker image rm beceb7b58a39  
Error response from daemon: conflict: unable to delete beceb7b58a39 (must be forced) - image is being used by stopped container 9b9ae547247a  

$ docker container list -a  
CONTAINER ID   IMAGE     COMMAND   CREATED   STATUS    PORTS     NAMES  
9b9ae547247a   r2hdp    ...  

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


