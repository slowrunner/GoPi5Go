#! /usr/bin/env bash
# This script is used to detect if GPG3 connected and run gopi5go_power.py
# This script is called by gp5g_power.service

# These lines are specific to the GoPiGo3
SCRIPTDIR="$(readlink -f $(dirname $0))"
echo "SCRIPTDIR: " $SCRIPTDIR
REPO_PATH=$(readlink -f $(dirname $0) | grep -E -o "^(.*?\\GoPi5Go)")
echo "REPO_PATH: " $REPO_PATH

# Check if gopi5go_power.py is running.
SERVICE='gopi5go_power.py'
# To be able to do an update without crapping out, we need to have power management on.  So let's directly turn it on.
if ps ax | grep -v grep | grep $SERVICE > /dev/null
then
    echo "$SERVICE service running."
else
    echo "$SERVICE is not running"
    # Run the gopigo3_power.py if GPG3 detected
    echo -e "sudo python3 $REPO_PATH/config/gopi5go_power.py"
    sudo python3 $REPO_PATH/config/gopi5go_power.py
    echo "$SERVICE started."
fi
