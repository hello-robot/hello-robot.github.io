#! /bin/bash

. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages
export HELLO_FLEET_PATH=$HOME/stretch_user
logfile="$HOME/stretch_user/log/hello_robot_xbox_teleop.`date '+%Y%m%d%H%M'`.log"

/usr/bin/python $HOME/.local/bin/xbox_dongle_init.py &>> $logfile
sleep 1
/usr/bin/python $HOME/.local/bin/stretch_xbox_controller_teleop.py &>> $logfile
