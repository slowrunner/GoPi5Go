#!/bin/bash

# installs and configures the gp5go power service

chmod 777 gp5g_power.sh

echo "copying gp5g_power.service to /etc/systemd/system"
sudo cp etc_systemd_system.gp5g_power.service /etc/systemd/system/gp5g_power.service
sudo systemctl daemon-reload
sudo systemctl enable gp5g_power
sudo service gp5g_power start
