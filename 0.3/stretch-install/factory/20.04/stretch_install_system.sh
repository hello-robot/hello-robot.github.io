#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log"
if getopts ":l:" opt && [[ $opt == "l" && -d $OPTARG ]]; then
    REDIRECT_LOGDIR=$OPTARG
fi
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/stretch_install_system.`date '+%Y%m%d%H%M'`_redirected.txt"

function install {
    sudo apt-get install -y "$@" >> $REDIRECT_LOGFILE
}

echo "###########################################"
echo "INSTALLATION OF SYSTEM WIDE PACKAGES"
echo "###########################################"
echo "Apt update & upgrade (this might take a while)"
sudo apt-add-repository universe >> $REDIRECT_LOGFILE
sudo apt-get --yes update >> $REDIRECT_LOGFILE
sudo apt-get --yes upgrade &>> $REDIRECT_LOGFILE
echo "Install zip & unzip"
install zip unzip
echo "Install Curl"
install curl
echo "Install Git"
install git
echo "Install rpl"
install rpl
echo "Install ipython3"
install ipython3
echo "Install pip3"
install python3-pip
echo "Install pyaudio"
install python3-pyaudio
echo "Install Emacs packages"
install emacs yaml-mode
echo "Install nettools"
install net-tools
echo "Install wget"
install wget
echo "Install vim"
install vim
echo "Install pyserial"
install python3-serial
echo "Install Port Audio"
install portaudio19-dev
echo "Install lm-sensors & nvme-cli"
install lm-sensors
install nvme-cli
echo "Install Cython for FUNMAP"
install cython3
echo "Install cheese for camera testing"
install cheese
echo "Install SSH Server"
install ssh
echo "Install Chromium"
install chromium-browser
echo "Install htop"
install htop
echo "Install Ubuntu Sounds"
install ubuntu-sounds
echo "Install BleachBit"
install bleachbit
echo ""

# see http://wiki.ros.org/noetic/Installation/Ubuntu for details
echo "###########################################"
echo "INSTALLATION OF ROS NOETIC"
echo "###########################################"
echo "Setting up sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Setting up keys"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &>> $REDIRECT_LOGFILE
echo "Apt update"
sudo apt-get --yes update >> $REDIRECT_LOGFILE
echo "Install ROS Noetic (this might take a while)"
install ros-noetic-desktop-full
echo "Install rosdep"
install python3-rosdep
echo "Configure rosdep"
if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
fi
sudo rosdep init >> $REDIRECT_LOGFILE
rosdep update --include-eol-distros >> $REDIRECT_LOGFILE
echo "Install vcstool"
install python3-vcstool
echo ""

echo "###########################################"
echo "INSTALLATION OF ADDITIONAL ROS NOETIC PKGS"
echo "###########################################"
echo "Install packages to work with URDFs"
install liburdfdom-tools meshlab
install ros-noetic-urdfdom-py
echo "Install joint state GUI package"
install ros-noetic-joint-state-publisher-gui
echo "Install TF2 related packages"
install ros-noetic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
install ros-noetic-rviz-imu-plugin ros-noetic-imu-filter-madgwick
echo "Install robot pose filter for use with IMU and wheel odometry"
install ros-noetic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
install ros-noetic-robot-localization
echo "Install ros_numpy package for msgs conversions"
install ros-noetic-ros-numpy
echo "Install ROS control packages (primarily for simulations with Gazebo)"
install ros-noetic-ros-control ros-noetic-ros-controllers
echo "Install ROS teleop packages"
install ros-noetic-teleop-twist-keyboard
echo "Install ROS navigation and mapping packages"
install ros-noetic-move-base ros-noetic-move-base-msgs
install ros-noetic-gmapping ros-noetic-navigation
echo "Install RPLidar A1M8 packages"
install ros-noetic-rplidar-ros ros-noetic-rplidar-ros-dbgsym
echo "Install scan tools for Canonical Scan Matching using the laser_scan_matcher"
install ros-noetic-scan-tools
echo ""

echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
echo "###########################################"
echo "Install dynamic reconfiguration"
install ros-noetic-ddynamic-reconfigure ros-noetic-ddynamic-reconfigure-python
echo "Register the librealsense APT server's public key"
function register_librealsense_apt_server {
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
}
register_librealsense_apt_server &>> $REDIRECT_LOGFILE
echo "Add the librealsense APT server to the list of APT repositories"
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u &>> $REDIRECT_LOGFILE
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Apt update"
sudo apt-get --yes update >> $REDIRECT_LOGFILE
echo "Install librealsense2 packages"
install librealsense2 librealsense2-dkms librealsense2-udev-rules librealsense2-utils librealsense2-dev librealsense2-dbg
