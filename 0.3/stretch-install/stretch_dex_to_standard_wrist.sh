#!/bin/bash

cd ~/stretch_install
python3 ./factory/20.04/stretch_standard_wrist_yaml_configure.py

echo "Updating ROS or ROS 2 workspace to work with a standard wrist"
echo "---------------------------------------------------------------------------"
echo "Please source the ROS distribution you want to work with before proceeding"
echo "---------------------------------------------------------------------------"
if [ $ROS_VERSION = 1 ]
then
    echo "Setting up Stretch ROS and URDF"
    cd ~/catkin_ws/src/stretch_ros/
    git pull

    cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf
    cp stretch_description_standard.xacro stretch_description.xacro

    echo "Updating URDF calibration"
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

    ros2 run hello_helpers configure_wrist --standard
    colcon build
    cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
    ./export_urdf.sh
fi