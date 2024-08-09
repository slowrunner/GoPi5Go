# Attempt at Docker image with GoPiGo3 API installed over Legacy PiOS to run on PiOS Bookworm/Pi5

### Build
```
./build_gopigo3_buster.sh
```

- Log File:  build.log
- Check disk space for docker image

```
sudo du -sh /var/lib/docker
df -h /
```

### Run
```
./run_gopigo3buster.sh
```

### Remove the Docker image, build cache, and local GoPiGo3 repo
```
./rm_gpg3buster_image.sh
rm -rf GoPiGo3
docker buildx prune
```

### Result

The build succeeds but the GoPiGo3 complains because pigpiod doesn't like the Pi5:
```

#29 255.0 GOPIGO3 SOFTWARE INSTALLATION SUCCESSFUL.
#29 DONE 271.1s


pi@GoPi5Go:DOCKER:~/Dexter/GoPiGo3/Software/Python/Examples $ sudo ./Read_Info.py 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Can't connect to pigpio at localhost(8888)

Did you start the pigpio daemon? E.g. sudo pigpiod


...


pi@GoPi5Go:DOCKER:~/Dexter/GoPiGo3/Software/Python/Examples $ sudo pigpiod
2024-08-09 13:01:51 gpioHardwareRevision: unknown rev code (c04170)
2024-08-09 13:01:51 initCheckPermitted: 
+---------------------------------------------------------+
|Sorry, this system does not appear to be a raspberry pi. |
|aborting.                                                |
+---------------------------------------------------------+


Can't initialise pigpio library

```
