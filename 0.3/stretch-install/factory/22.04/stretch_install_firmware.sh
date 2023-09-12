#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
if getopts ":l:" opt && [[ $opt == "l" && -d $OPTARG ]]; then
    REDIRECT_LOGDIR=$OPTARG
fi
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_install_firmware.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "###########################################"
echo "INSTALLATION OF STRETCH FIRMWARE"
echo "###########################################"
source ~/.bashrc
echo "Read ttyACMx mapping"
REx_firmware_flash.py --map &>> $REDIRECT_LOGFILE
echo "Perform firmware update (this might take a while)"
REx_firmware_updater.py --install --no_prompts &>> $REDIRECT_LOGFILE
