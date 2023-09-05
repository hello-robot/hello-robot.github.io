# This script removes the file that pins RealSense packages to 2.45 and upgrades these packages
# It is intended that this scripts runs after a run of the realsense_downgrade_patch.sh

remove_melodic_downgrade() {
    echo "Remove apt pinning file from standard location"
    sudo rm /etc/apt/preferences.d/melodic_librealsense2_packages

    echo "Update latest apt indices"
    sudo apt update

    echo "Install newest librealsense2 binaries"
    sudo apt install librealsense2 librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules librealsense2-gl librealsense2-gl-dev librealsense2-gl-dbg librealsense2-net librealsense2-net-dev librealsense2-net-dbg

    echo "NOTE: Please restart the robot. Then use 'realsense-viewer' to upgrade the camera's firmware to the latest versions."
}

remove_noetic_downgrade() {
    echo "Remove apt pinning file from standard location"
    sudo rm /etc/apt/preferences.d/noetic_librealsense2_packages

    echo "Update latest apt indices"
    sudo apt update

    echo "Install newest librealsense2 binaries"
    sudo apt install librealsense2 librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules librealsense2-gl librealsense2-gl-dev librealsense2-gl-dbg librealsense2-net librealsense2-net-dev librealsense2-net-dbg

    echo "NOTE: Please restart the robot. Then use 'realsense-viewer' to upgrade the camera's firmware to the latest versions."
}


OS_VERSION=$(lsb_release -d -s)

if [[ $OS_VERSION == *18.04* ]]; then
    remove_melodic_downgrade
elif [[ $OS_VERSION == *20.04* ]]; then
    remove_noetic_downgrade
else
    echo "Could not identify OS. Please contact Hello Robot Support."
fi
