#!/bin/bash

echo "This script will install and update system wide packages for Stretch"

echo "###########################################"
echo "INSTALLATION OF OS PACKAGES"
# upgrade to the latest versions of Ubuntu packages
echo "Upgrading Ubuntu packages to the latest versions..."
sudo apt --yes update
sudo apt --yes upgrade
echo "Done."
echo ""
echo "Install Python"
sudo apt --yes install python ipython
echo "Install Pip"
sudo apt --yes install python-pip
echo "Install Git"
sudo apt --yes install git
echo "Install rpl via apt"
sudo apt --yes install rpl
echo "Install ipython3 via apt"
sudo apt --yes install ipython3
sudo apt --yes install python3-pip
echo "Install Emacs packages"
sudo apt --yes install emacs yaml-mode
echo "Install nettools"
sudo apt --yes install net-tools
echo "Install git and wget"
sudo apt --yes install git wget
echo "Install vim"
sudo apt --yes install vim
echo "Install Python packages"
sudo apt --yes install python-serial
echo "Install GSL for csm"
sudo apt --yes install libgsl0-dev
echo "DONE WITH MAIN INSTALLATION OF OS PACKAGES"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF HARDWARE PACKAGES"
# packages to support stretch_body
echo "Installing lm-sensors"
sudo apt-get install lm-sensors
echo "Making and installing nvme"
cd ~/
git clone https://github.com/linux-nvme/nvme-cli.git
cd nvme-cli/
make
sudo make install
cd ..
rm -rf nvme-cli
echo "DONE WITH MAIN INSTALLATION OF HARDWARE PACKAGES"
echo "###########################################"
echo ""


# Install ROS Melodic
# see http://wiki.ros.org/melodic/Installation/Ubuntu#Installation for details
echo "###########################################"
echo "MAIN INSTALLATION OF ROS MELODIC"
echo "Setting up sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "Setting up keys"
# New key as of Jun 8, 2019
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "apt update"
sudo apt --yes update
echo "Installating Desktop-Full Version"
sudo apt --yes install ros-melodic-desktop-full
echo "Initialize rosdep"
sudo apt --yes install python-rosdep
sudo rosdep init
rosdep update
echo "Install additional ROS packages"
sudo apt --yes install python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "Source .bash file"
source /opt/ros/melodic/setup.bash
echo "DONE WITH MAIN INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo ""

################ Additional packages#####################################
echo "###########################################"
echo "ADDITIONAL INSTALLATION OF ROS MELODIC"
echo "Install packages to work with URDFs"
sudo apt --yes install liburdfdom-tools meshlab
echo "Install cheese for camera testing"
sudo apt --yes install cheese
echo "Install joint state GUI package"
sudo apt --yes install ros-melodic-joint-state-publisher-gui
echo "Install TF2 related packages"
sudo apt --yes install ros-melodic-tf2-tools
echo "Install IMU visualization plugin for RViz and IMU filter"
sudo apt --yes install ros-melodic-rviz-imu-plugin ros-melodic-imu-filter-madgwick
#echo "Install robot pose filter for use with IMU and wheel odometry"
#sudo apt --yes install ros-melodic-robot-pose-ekf
echo "Install robot localization package for use with IMU and wheel odometry"
sudo apt --yes install ros-melodic-robot-localization
echo "Install ROS packages for Robotis Dynamixel actuators"
sudo apt --yes install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench
echo "Install ROS control packages (primarily for simulations with Gazebo)"
sudo apt --yes install ros-melodic-ros-control ros-melodic-ros-controllers
echo "Install ROS RGB-D package and dynamic reconfiguration package for use with Intel D435i"
sudo apt --yes install ros-melodic-rgbd-launch ros-melodic-ddynamic-reconfigure-python
echo "Install ROS teleop packages"
sudo apt --yes install ros-melodic-teleop-twist-keyboard
#echo "Install ROS navigation and mapping packages"
#sudo apt --yes install ros-melodic-move-base ros-melodic-map-server ros-melodic-amcl ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
sudo apt --yes install ros-melodic-move-base ros-melodic-move-base-msgs
sudo apt --yes install ros-melodic-gmapping ros-melodic-navigation
#echo "Install ROS MoveIt! installation"
#sudo apt --yes install ros-melodic-moveit
# SMACH VIEWER HAS A BUG, SO NOT INSTALLING FOR NOW
#echo "Install additional SMACH packages"
#sudo apt install ros-melodic-smach-viewer
echo "Install RPLidar A1M8 packages"
sudo apt --yes install ros-melodic-rplidar-ros ros-melodic-rplidar-ros-dbgsym
echo "Install Respeaker and speech recognition packages"
sudo apt --yes install ros-melodic-respeaker-ros ros-melodic-ros-speech-recognition
echo "DONE WITH ADDITIONAL INSTALLATION OF ROS MELODIC"
echo "###########################################"
echo ""


echo "###########################################"
echo "INSTALLATION OF INTEL D435i"
sudo apt --yes install ros-melodic-realsense2-camera ros-melodic-realsense2-description
# "The following NEW packages will be installed:
#  ros-melodic-ddynamic-reconfigure ros-melodic-librealsense2 ros-melodic-realsense2-camera"

# UNCLEAR IF THE FOLLOWING COMMANDS ARE STILL NEEDED
# # see https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
echo "INSTALL INTEL D435i"
echo "Register the server's public key"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
echo "Add the server to the list of repositories"
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
echo "Remove old records in case of upgrading"
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
echo "Update"
sudo apt --yes update
echo "Install D435i packages"
sudo apt --yes install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
#######
# NOTE: Use realsense-viewer to install the recommended firmware instead of the following commands.
#######
# # attempt to install D435i firmware 
# echo "Downloading D435i firmware Version: 5.11.6.250 Date: 6/4/2019"
# echo "WARNING: THIS MAY NOT BE THE LATEST FIRMWARE"
# mkdir -p ~/firmware/d435i
# cd ~/firmware/d435i
# wget https://downloadmirror.intel.com/28870/eng/D400_Series_Production_FW_5_11_6_250.zip
# unzip D400_Series_Production_FW_5_11_6_250.zip 
# echo "Installing D435i firmware installation software"
# echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
# sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
# sudo apt --yes update 
# sudo apt --yes install intel-realsense-dfu
# echo "########################################################"
# echo "WARNING: Please pay attention. This firmware upgrade script may not function properly on a new robot, since the bus and device numbers for the D435i may have changed. If this fails, you can find candidates using the following command: lsusb | grep Intel"
# echo "Attempting to update D435i firmware automatically using the following command"
# echo "intel-realsense-dfu -b 002 -d 003 -f -i Signed_Image_UVC_5_11_6_250.bin"
# intel-realsense-dfu -b 002 -d 003 -f -i Signed_Image_UVC_5_11_6_250.bin
# echo "########################################################"
echo "DONE WITH INSTALLATION OF INTEL D435i"
echo "###########################################"
echo ""


# echo "###########################################"
# echo "INSTALLATION OF NETWORKING"
# # set up SSH
# echo "Setting up SSH..."
# sudo apt --yes install ssh
# # sudo emacs -nw /etc/ssh/sshd_config
# # Change default port “#Port 22” -> “Port 32123”
# echo "Changing SSH port to 32123..."
# sudo sed -i 's/Port 22/Port 32123/g' /etc/ssh/sshd_config
# sudo service ssh restart
# echo "Done."
# echo ""

# echo "The IP address for this machine follows:"
# curl ifconfig.me
# echo ""
# echo "Make it a static IP and then use it for SSH and VNC."
# echo "Done!"
# echo "DONE WITH INSTALLATION OF NETWORKING"
# echo "###########################################"
# echo ""

