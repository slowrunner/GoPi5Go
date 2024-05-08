#!/bin/bash

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
echo -e "Adding user Pi to docker group to eliminate need for sudo"
sudo gpasswd -a $USER docker
echo -e "Must logout and log back in for this to take effect"


