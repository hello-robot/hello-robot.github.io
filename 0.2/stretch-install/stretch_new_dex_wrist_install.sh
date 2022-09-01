#!/bin/bash

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04)$ ]]; then
    echo "Could not identify OS. Please contact Hello Robot Support."
    exit 1
fi

echo "To be run once a robot has gone through production bringup "
echo "Configures to use an attached DexWrist"
echo "Use --factory if this is a factory installation"

read -p "Proceed with installation (y/n)?" -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo "Setting Dynamixel bauds to 115200"
REx_dynamixel_set_baud.py /dev/hello-dynamixel-wrist 13 115200

echo "Configuring user YAML"
./factory/$factory_osdir/stretch_dex_wrist_yaml_configure.py $1

echo "Setting up Stretch ROS and URDF"
cd ~/catkin_ws/src/stretch_ros/
git pull

cd ~/repos
git clone https://github.com/hello-robot/stretch_tool_share
cd ~/repos/stretch_tool_share
git pull
cd ~/repos/stretch_tool_share/tool_share/stretch_dex_wrist/stretch_description
cp urdf/stretch_dex_wrist.xacro ~/catkin_ws/src/stretch_ros/stretch_description/urdf
cp urdf/stretch_description.xacro ~/catkin_ws/src/stretch_ros/stretch_description/urdf
cp meshes/*.STL ~/catkin_ws/src/stretch_ros/stretch_description/meshes


echo "Updating URDF calibration "
rosrun stretch_calibration update_urdf_after_xacro_change.sh
cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf
./export_urdf.sh

