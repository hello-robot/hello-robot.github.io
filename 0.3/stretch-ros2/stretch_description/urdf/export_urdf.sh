#!/bin/bash

# Prior to running this script make sure you have used stretch_calibration to create a calibrated URDF. This script also requires rpl, which can be installed with "sudo apt install rpl".
echo "Prior to running this script make sure you have used stretch_calibration to create a calibrated URDF. This script also requires rpl, which can be installed with sudo apt install rpl." 
echo ""

# Move previous exported_urdf to exported_urdf_previous.
echo "Move previous exported_urdf to exported_urdf_previous."
echo "mv ./exported_urdf ./exported_urdf_previous"
mv ./exported_urdf ./exported_urdf_previous
echo ""

# Create new exported URDF directories.
echo "Create new exported URDF directories."
echo "mkdir -p ./exported_urdf/" 
mkdir -p ./exported_urdf/
echo "mkdir -p ./exported_urdf/meshes/"
mkdir -p ./exported_urdf/meshes/
echo ""

# Copy the mesh files and the original calibrated URDF file to the exported URDF.
echo "Copy the mesh files and the current calibrated URDF file to the exported URDF."
echo "cp ../meshes/* ./exported_urdf/meshes/"
cp ../meshes/* ./exported_urdf/meshes/
echo "cp ./stretch.urdf ./exported_urdf/"
cp ./stretch.urdf ./exported_urdf/
echo ""

# Replace the mesh file locations in the original URDF with local directories."
echo "Replace the mesh file locations in the exported URDF with local directories."
OLD_NAME="package://stretch_description/"
NEW_NAME="./"
echo "rpl -i $OLD_NAME $NEW_NAME ./exported_urdf/stretch.urdf"
rpl -i $OLD_NAME $NEW_NAME ./exported_urdf/stretch.urdf
echo ""

# Copy D435i mesh from the realsense2_description ROS package to the exported URDF.
echo "Copy D435i mesh from the realsense2_description ROS package to the exported URDF."
echo "cp `ros2 pkg prefix realsense2_description`/share/realsense2_description/meshes/d435.dae ./exported_urdf/meshes/"
cp `ros2 pkg prefix realsense2_description`/share/realsense2_description/meshes/d435.dae ./exported_urdf/meshes/
echo "rpl -i "package://realsense2_description/" "./" ./exported_urdf/stretch.urdf"
rpl -i "package://realsense2_description/" "./" ./exported_urdf/stretch.urdf
echo ""

# copy controller calibration file used by stretch ROS
echo "Copy the current controller parameter yaml file to the exported URDF."
echo "cp `ros2 pkg prefix stretch_core`/share/stretch_core/config/controller_calibration_head.yaml ./exported_urdf/"
cp `ros2 pkg prefix stretch_core`/share/stretch_core/config/controller_calibration_head.yaml ./exported_urdf/
echo ""

# copy license file
echo "Copy license file to exported URDF"
echo "cp export_urdf_license_template.md  ./exported_urdf/LICENSE.md"
cp export_urdf_license_template.md  ./exported_urdf/LICENSE.md

echo "Copy the exported URDF to the standard Stretch directory."
echo "cp -rf exported_urdf/* $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf"
cp -rf exported_urdf/* $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
echo ""
