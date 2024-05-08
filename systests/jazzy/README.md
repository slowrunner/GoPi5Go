# Process I used to test Raspberry Pi5 Dockered Jazzy Listener with AMD64 Debian Jazzy Talker
(I'm a little late to the Docker on Pi5 Testing, so wrote it up here)

### Install Docker for Raspberry Pi5 (over PiOS Bookworm Desktop 64-bit)
File 1_setup_docker_apt_repo.sh:
```
#!/bin/bash

# REF:https://docs.docker.com/engine/install/debian/#install-using-the-repository 

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

Run it:
```
chmod +x  1_setup_docker_apt_repo.sh
./ 1_setup_docker_apt_repo.sh
```

File 2_install_docker_pkgs.sh:
```
#!/bin/bash

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
```

Run it:
```
chmod +x 2_install_docker_pkgs.sh
./2_install_docker_pkgs.sh

...
Hello from Docker!
This message shows that your installation appears to be working correctly.
...

```

### Get Jazzy Test Image:
```
sudo docker pull ghcr.io/sloretz/ros-testing:jazzy-desktop
```

### Run the Jazzy Test Image:
```
sudo docker run --rm=true -it --net=host  ghcr.io/sloretz/ros-testing:jazzy-desktop

root@GoPi5Go:/# source /opt/ros/jazzy/setup.bash
root@GoPi5Go:/# ros2 run demo_nodes_cpp listener

(on networked AMD64 Debian Jazzy system:
$ ros2 run demo_nodes_py talker)

[INFO] [1715184500.569241973] [listener]: I heard: [Hello World: 0]
[INFO] [1715184501.557232599] [listener]: I heard: [Hello World: 1]
[INFO] [1715184502.553296922] [listener]: I heard: [Hello World: 2]
[INFO] [1715184503.554859055] [listener]: I heard: [Hello World: 3]
[INFO] [1715184504.555255615] [listener]: I heard: [Hello World: 4]
^C[INFO] [1715184595.875557013] [rclcpp]: signal_handler(signum=2)
```

### Check Docker Jazzy:

```
# ros2 doctor --report
...

   PLATFORM INFORMATION
system           : Linux
platform info    : Linux-6.6.20+rpt-rpi-2712-aarch64-with-glibc2.39
release          : 6.6.20+rpt-rpi-2712
processor        : aarch64

   QOS COMPATIBILITY LIST
compatibility status    : No publisher/subscriber pairs found

   RMW MIDDLEWARE
middleware name    : rmw_fastrtps_cpp

   ROS 2 INFORMATION
distribution name      : jazzy
distribution type      : ros2
distribution status    : pre-release
release platforms      : {'debian': ['bookworm'], 'rhel': ['9'], 'ubuntu': ['noble']}


/# dpkg -l | grep jazzy-desktop
ii  ros-jazzy-desktop                                0.11.0-1noble.20240503.191416       arm64        A package which extends 'ros_base' and includes high level packages like vizualization tools and demos.
```

### To exit Docker/Bash:
```
root@GoPi5Go:/# exit
```
