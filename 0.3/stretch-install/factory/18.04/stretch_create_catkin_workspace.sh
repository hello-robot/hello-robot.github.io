#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
CATKIN_WSDIR="$HOME/catkin_ws"
while getopts l:w: opt; do
    case $opt in
        l)
            if [[ -d $OPTARG ]]; then
                REDIRECT_LOGDIR=$OPTARG
            fi
            ;;
        w)
            CATKIN_WSDIR=$OPTARG
            ;;
    esac
done
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_create_catkin_workspace.`date '+%Y%m%d%H%M'`_redirected.txt"

echo "###########################################"
echo "CREATING MELODIC CATKIN WORKSPACE at $CATKIN_WSDIR"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "melodic" ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/melodic/setup.bash

if [[ -d $CATKIN_WSDIR ]]; then
    echo "You are about to delete and replace the existing catkin workspace. If you have any personal data in the workspace, please create a back up before proceeding."
    prompt_yes_no(){
    read -p "Do you want to continue? Press (y/n for yes/no): " x
    if [ $x = "n" ]; then
            echo "Exiting the script."
            exit 1
    elif [ $x = "y" ]; then
            echo "Continuing to create a new catkin workspace."
    else
        echo "Press 'y' for yes or 'n' for no."
        prompt_yes_no
    fi
    }
    prompt_yes_no
fi

echo "Deleting $CATKIN_WSDIR if it already exists..."
sudo rm -rf $CATKIN_WSDIR
# see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment for details
echo "Creating the workspace directory..."
mkdir -p $CATKIN_WSDIR/src
echo "Cloning the workspace's packages..."
cd $CATKIN_WSDIR/src
vcs import --input ~/stretch_install/factory/18.04/stretch_ros_melodic.repos &>> $REDIRECT_LOGFILE
echo "Fetch ROS packages' dependencies (this might take a while)..."
cd $CATKIN_WSDIR/
rosdep install -iy --skip-keys="librealsense2" --from-paths src &>> $REDIRECT_LOGFILE
echo "Make the workspace..."
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release &>> $REDIRECT_LOGFILE
echo "Source setup.bash file..."
source $CATKIN_WSDIR/devel/setup.bash
echo "Index ROS packages..."
rospack profile >> $REDIRECT_LOGFILE
echo "Update ~/.bashrc dotfile to source workspace..."
echo "source $CATKIN_WSDIR/devel/setup.bash" >> ~/.bashrc
echo "Updating meshes in stretch_ros to this robot's batch..."
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=${HOME}/stretch_user
$CATKIN_WSDIR/src/stretch_ros/stretch_description/meshes/update_meshes.py &>> $REDIRECT_LOGFILE
echo "Setup uncalibrated robot URDF..."
bash -i $CATKIN_WSDIR/src/stretch_ros/stretch_calibration/nodes/update_uncalibrated_urdf.sh >> $REDIRECT_LOGFILE
echo "Setup calibrated robot URDF..."
# TODO: will print to stderr but report exit code 0 for precalibrated robots, it should exit 1 and mitigate precalibration within script
bash -i $CATKIN_WSDIR/src/stretch_ros/stretch_calibration/nodes/update_with_most_recent_calibration.sh &>> $REDIRECT_LOGFILE
echo "Compiling FUNMAP's Cython code..."
cd $CATKIN_WSDIR/src/stretch_ros/stretch_funmap/src/stretch_funmap
./compile_cython_code.sh &>> $REDIRECT_LOGFILE
