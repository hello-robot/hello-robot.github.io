#! /bin/bash

. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user

/usr/bin/python3 /usr/bin/hello_robot_pimu_ping.py &>> "$HOME/stretch_user/log/hello_robot_pimu_ping.`date '+%Y%m%d%H%M'`.log"
