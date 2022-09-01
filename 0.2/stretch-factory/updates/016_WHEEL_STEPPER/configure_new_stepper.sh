#! /bin/bash

sudo chmod a+rw $HELLO_FLEET_PATH/$HELLO_FLEET_ID/udev/*
sudo cp *.rules $HELLO_FLEET_PATH/$HELLO_FLEET_ID/udev
sudo cp stretch_re1_factory_params.yaml $HELLO_FLEET_PATH/$HELLO_FLEET_ID/
sudo cp hello-motor*.yaml $HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_steppers
sudo cp *.rules /etc/udev/rules.d
sudo cp *.rules /etc/hello-robot/$HELLO_FLEET_ID/udev




