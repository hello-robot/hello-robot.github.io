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
sudo apt-add-repository universe -y >> $REDIRECT_LOGFILE
sudo apt-get --yes update >> $REDIRECT_LOGFILE
sudo apt-get --yes upgrade &>> $REDIRECT_LOGFILE
echo "Install zip & unzip"
install zip unzip
echo "Install Curl"
install curl
echo "Install ca-certificates"
install ca-certificates
echo "Install gnupg"
install gnupg
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
echo "Install APT HTTPS"
install apt-transport-https
echo "Install Network Security Services libraries"
install libnss3-tools
echo ""

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
echo "###########################################"
echo "INSTALLATION OF ROS 2 HUMBLE"
echo "###########################################"
echo "Setting up keys"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "Setting up sources.list"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "Apt update"
sudo apt-get --yes update >> $REDIRECT_LOGFILE
echo "Install ROS 2 Humble (this might take a while)"
install ros-humble-desktop-full
# https://discourse.ros.org/t/ros-developer-tools-now-in-binary-form/29802
echo "Install ROS 2 Dev Tools"
install ros-dev-tools
echo "Install colcon"
install python3-colcon-common-extensions
install python3-colcon-clean
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
echo "INSTALLATION OF ADDITIONAL ROS HUMBLE PKGS"
echo "###########################################"
echo "Install packages to work with URDFs"
install liburdfdom-tools meshlab
install ros-humble-urdfdom-py
echo "Install joint state GUI package"
install ros-humble-joint-state-publisher-gui
echo "Install IMU visualization plugin for RViz and IMU filter"
install ros-humble-rviz-imu-plugin ros-humble-imu-filter-madgwick
echo "Install robot localization package for use with IMU and wheel odometry"
install ros-humble-robot-localization
echo "Install teleop packages"
install ros-humble-teleop-twist-keyboard
echo ""

echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
echo "###########################################"
echo "Register the librealsense APT server's public key"
function register_librealsense_apt_server {
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
}
register_librealsense_apt_server &>> $REDIRECT_LOGFILE
echo "Add the librealsense APT server to the list of APT repositories"
function add_librealsense_apt_server {
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
}
add_librealsense_apt_server &>> $REDIRECT_LOGFILE
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Apt update"
sudo apt-get --yes update >> $REDIRECT_LOGFILE
echo "Install librealsense2 packages"
install librealsense2 librealsense2-dkms librealsense2-udev-rules librealsense2-utils librealsense2-dev librealsense2-dbg
echo ""

echo "###########################################"
echo "INSTALLATION OF WEB INTERFACE"
echo "###########################################"
echo "Register the nodesource APT server's public key"
function register_nodesource_apt_server {
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg
}
register_nodesource_apt_server &>> $REDIRECT_LOGFILE
echo "Add the nodesource APT server to the list of APT respositories"
function add_nodesource_apt_server {
    NODE_MAJOR=21
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | sudo tee /etc/apt/sources.list.d/nodesource.list
}
add_nodesource_apt_server &>> $REDIRECT_LOGFILE
echo "Apt update"
sudo apt-get --yes update >> $REDIRECT_LOGFILE
echo "Install NodeJS"
install nodejs
echo "Install PyPCL and PyKDL"
install python3-pcl python3-pykdl screen
echo "Install PM2"
sudo npm install -g pm2 &>> $REDIRECT_LOGFILE
