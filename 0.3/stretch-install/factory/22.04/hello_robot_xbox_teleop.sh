#! /bin/bash

. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user
/usr/bin/python3 $HOME/.local/bin/stretch_xbox_controller_teleop.py &>> "$HOME/stretch_user/log/hello_robot_xbox_teleop.`date '+%Y%m%d%H%M'`.log"
