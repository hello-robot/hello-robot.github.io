#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
AMENT_WSDIR="$HOME/ament_ws"
while getopts l:w: opt; do
    case $opt in
        l)
            if [[ -d $OPTARG ]]; then
                REDIRECT_LOGDIR=$OPTARG
            fi
            ;;
        w)
            AMENT_WSDIR=$OPTARG
            ;;
    esac
done
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_create_ament_workspace.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "###########################################"
echo "CREATING HUMBLE AMENT WORKSPACE at $AMENT_WSDIR"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "humble" ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/humble/setup.bash

if [[ -d $AMENT_WSDIR ]]; then
    echo "You are about to delete and replace the existing ament workspace. If you have any personal data in the workspace, please create a back up before proceeding."
    prompt_yes_no(){
    read -p "Do you want to continue? Press (y/n for yes/no): " x
    if [ $x = "n" ]; then
            echo "Exiting the script."
            exit 1
    elif [ $x = "y" ]; then
            echo "Continuing to create a new ament workspace."
    else
        echo "Press 'y' for yes or 'n' for no."
        prompt_yes_no
    fi
    }
    prompt_yes_no
fi

echo "Deleting $AMENT_WSDIR if it already exists..."
sudo rm -rf $AMENT_WSDIR
echo "Creating the workspace directory..."
mkdir -p $AMENT_WSDIR/src
echo "Cloning the workspace's packages..."
cd $AMENT_WSDIR/src
vcs import --input ~/stretch_install/factory/22.04/stretch_ros2_iron.repos >> $REDIRECT_LOGFILE
echo "Fetch ROS packages' dependencies (this might take a while)..."
cd $AMENT_WSDIR/
rosdep install --rosdistro=humble -iyr --skip-keys="librealsense2" --from-paths src &>> $REDIRECT_LOGFILE
echo "Compile the workspace (this might take a while)..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release &>> $REDIRECT_LOGFILE
echo "Source setup.bash file..."
source $AMENT_WSDIR/install/setup.bash
echo "Update ~/.bashrc dotfile to source workspace..."
echo "source $AMENT_WSDIR/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "Updating meshes in stretch_ros to this robot's batch..."
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=${HOME}/stretch_user
$AMENT_WSDIR/src/stretch_ros2/stretch_description/batch/update_description.py >> $REDIRECT_LOGFILE
echo "Setup uncalibrated robot URDF..."
ros2 run stretch_calibration update_uncalibrated_urdf >> $REDIRECT_LOGFILE
echo "Setup calibrated robot URDF..."
ros2 run stretch_calibration update_with_most_recent_calibration >> $REDIRECT_LOGFILE
colcon build &>> $REDIRECT_LOGFILE
