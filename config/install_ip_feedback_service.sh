#!/bin/bash

# === ESPEAK-NG
sudo apt install -y espeak-ng
pip3 install py-espeak-ng --break-system-packages
espeak-ng "Am I alive? Can you hear me?"


# installs and configures the ip_feedback service

chmod 777 ip_feedback.sh

echo "copying ip_feedback.sh to /home/pi"
cp ip_feedback.sh /home/pi

echo "copying ip_feedback.service to /etc/systemd/system"
sudo cp etc_systemd_system.ip_feedback.service /etc/systemd/system/ip_feedback.service
sudo systemctl daemon-reload
sudo systemctl enable ip_feedback
sudo service ip_feedback start
