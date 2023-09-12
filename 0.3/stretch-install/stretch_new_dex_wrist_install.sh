#!/bin/bash

source /etc/os-release
factory_osdir="$VERSION_ID"
if [[ ! $factory_osdir =~ ^(18.04|20.04|22.04)$ ]]; then
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

echo "Updating ROS or ROS 2 workspace to work with a dex wrist"
echo "---------------------------------------------------------------------------"
echo "Please source the ROS distribution you want to work with before proceeding"
echo "---------------------------------------------------------------------------"
if [ $ROS_VERSION = 1 ]
then
    echo "Setting up Stretch ROS and URDF"
    cd ~/catkin_ws/src/stretch_ros/
    git pull

    cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf
    cp stretch_description_dex.xacro stretch_description.xacro

    echo "Updating URDF calibration "
    rosrun stretch_calibration update_urdf_after_xacro_change.sh
    cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf
    ./export_urdf.sh
else
    echo "Updating Stretch ROS 2 and URDF"
    cd ~/ament_ws/src/stretch_ros2
    git pull
    
    cd ~/ament_ws
    colcon build
    source install/setup.bash

    ros2 run hello_helpers configure_wrist --dex
    colcon build
    cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
    ./export_urdf.sh
fi
