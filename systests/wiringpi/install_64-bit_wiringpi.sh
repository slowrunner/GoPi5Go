#!/bin/bash

echo -e "install gdebi"
sudo apt install -y gdebi-core
sudo gdebi wiringpi_3.0_armhf.deb

