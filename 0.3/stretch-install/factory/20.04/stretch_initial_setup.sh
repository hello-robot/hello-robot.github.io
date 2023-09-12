#!/bin/bash
set -e

do_factory_install=${1:-'false'}

if $do_factory_install; then
    echo "WARNING: Running a FACTORY install. This is only meant to be run at Hello Robot HQ."
fi
echo "WARNING: Run this installation for fresh Ubuntu installs only."

echo "Checking ~/.bashrc doesn't already define HELLO_FLEET_ID..."
if [[ $HELLO_FLEET_ID ]]; then
    echo "Expecting var HELLO_FLEET_ID to be undefined. Check end of ~/.bashrc file, delete all lines in 'STRETCH BASHRC SETUP' section, and open a new terminal. Exiting."
    exit 1
fi

read -p "Plug in charger & attach clip-clamp. Ready to proceed (y/n)? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Confirmation failed. Will not proceed with installation."
    exit 1
fi

PS3="Select model type: "

select model in stretch-re1 stretch-re2
do
    echo "Selected model: $model"

  if [[ "$model" == "stretch-re1" ]]
  then
    break
  fi

 if [[ "$model" == "stretch-re2" ]]
  then
    break
  fi

done

pre=$model"-"

echo -n "Enter fleet id xxxx for $pre""xxxx> "
read id
if [[ ! $id =~ ^[0-9]{4}$ ]]; then
    echo "Input should be four digits. Exiting."
    exit 1
fi


HELLO_FLEET_ID="$pre$id"

read -p "HELLO_FLEET_ID will be $HELLO_FLEET_ID. Proceed with installation (y/n)? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Confirmation failed. Will not proceed with installation."
    exit 1
fi

DIR=`pwd`
echo "Checking Stretch Install cloned to right place..."
if [[ ! -d "$HOME/stretch_install" ]]; then
    echo "Expecting Stretch Install to be in home folder. Exiting."
    exit 1
fi

if [ $do_factory_install = 'false' ]; then
    echo "Checking robot calibration data in home folder..."
    if [[ ! -d "$HOME/$HELLO_FLEET_ID" ]]; then
        echo "Expecting robot calibration $HELLO_FLEET_ID to be present in the the home folder. Exiting."
        exit 1
    fi
fi

echo "Waiting to get online..."
while ! timeout 0.2 ping -c 1 -n google.com &> /dev/null
do
    sleep 1
    printf "%c" "."
done

echo "Checking install repo is up-to-date..."
git remote update &> /dev/null
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u})
if [ ! $LOCAL = $REMOTE ]; then
    echo "Repo not up-to-date. Please perform a 'git pull'. Exiting."
    exit 1
fi

echo "Waiting for apt lock..."
while sudo fuser /var/{lib/{dpkg,apt/lists},cache/apt/archives}/lock >/dev/null 2>&1; do
    sleep 1
    printf "%c" "."
done

echo "Setting up /etc/hello-robot directory..."
echo "HELLO_FLEET_ID=$HELLO_FLEET_ID">>hello-robot.conf
if [ -d "/etc/hello-robot" ]; then
    sudo rm -rf /etc/hello-robot
fi
sudo mkdir /etc/hello-robot
sudo mv hello-robot.conf /etc/hello-robot
sudo cp $DIR/stretch_about.png /etc/hello-robot/

if $do_factory_install; then
    echo "Fetching robot's calibration data from Github..."
    cd ~/
    git config --global credential.helper store
    git clone https://github.com/hello-robot/stretch_fleet.git
    sudo cp -rf ~/stretch_fleet/robots/$HELLO_FLEET_ID /etc/hello-robot/
    rm -rf stretch_fleet
else
    echo "Fetching robot's calibration data locally from $HOME/$HELLO_FLEET_ID directory..."
    sudo cp -rf ~/$HELLO_FLEET_ID /etc/hello-robot
    rm -rf ~/$HELLO_FLEET_ID
fi

echo "Setting up UDEV rules..."
sudo cp /etc/hello-robot/$HELLO_FLEET_ID/udev/*.rules /etc/udev/rules.d
sudo udevadm control --reload

echo "Allow shutdown without password..."
sudo cp $DIR/hello_sudoers /etc/sudoers.d/

echo "Setting up startup scripts..."
mkdir -p ~/.local/bin
sudo cp $DIR/hello_robot_audio.sh /usr/bin/
sudo cp $DIR/hello_robot_lrf_off.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.py /usr/bin/
sudo cp $DIR/hello_robot_pimu_ping.sh /usr/bin/
sudo cp $DIR/hello_robot_xbox_teleop.sh /usr/bin/

echo "Setting up apt retries..."
echo 'Acquire::Retries "3";' > 80-retries
sudo mv 80-retries /etc/apt/apt.conf.d/
