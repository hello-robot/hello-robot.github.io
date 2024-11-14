# Demo #2: Web Teleop

In this demo, we'll introduce the Stretch Web Teleop codebase and try out the interface.

## Background

Stretch has a website for teleoperating the robot from a web browser. This website can be set up to teleoperate the robot remotely from anywhere in the world with an internet connection, or simply eyes-off teleop from the next room on a local network. The codebase is built on ROS 2, WebRTC, Nav2, and TypeScript.

## Launching the interface

First, navigate to the folder containing the codebase using:

```{.bash .shell-prompt .copy}
colcon_cd stretch_web_teleop
```

Next, launch the interface (if you captured a map in the [previous tutorial](./demos_mapping_and_navigation.md), use `./launch_interface -m ${HELLO_FLEET_PATH}/maps/nav2_demo_map.yaml` instead):

```{.bash .shell-prompt .copy}
./launch_interface.sh
```

In the terminal, you will see output similar to:

```
Visit the URL(s) below to see the web interface:
https://localhost/operator
https://192.168.1.14/operator
```

Look for a URL like `https://<ip_address>/operator`. Visit this URL in a web browser on your personal laptop or desktop to see the web interface. Ensure your personal computer is connected to the same network as Stretch. You might see a warning that says "Your connection is not private". If you do, click `Advanced` and `Proceed`.

Once you're done with the interface, close the browser and run:

```{.bash .shell-prompt .copy}
./stop_interface.sh
```

**Note:** Only one browser can be connected to the interface at a time.

## Usage Guide

The web interface currently has a variety of control modes, displays and customization options. This tutorial will explain how to use the standard version of the interface that appears when you load it for the first time.

### Overview of Layout

There are three panels. The `Camera Views` panel contains the wide angle and gripper camera views. The second panel has three tabs: (1) `Base`, (2) `Wrist & Gripper`, and (3) `Arm & Lift`. Each of these tabs contains a button pad for controlling the respective joints. The `Safety` panel contains the run stop and battery gauge. The header contains a drop down for three action modes, the speed controls (`Slowest`, `Slow`, `Medium`, `Fast`, and `Fastest`) and a button to enable the customization mode.

### Wide-Angle Camera View
The wide angle camera is attached to the robot's head which can pan and tilt. There are four buttons bordering the camera feed the will pan and tilt the camera.
<p align="center">
    <img src="../images/pan-tilt.gif">
</p>

#### Quick Look
There are three built-in quick look options: `Look Ahead`, `Look at Base` and `Look at Gripper`. 
<p align="center">
    <img src="../images/quick-look.gif">
</p>

#### Follow Gripper
The `follow gripper` button will automatically pan/tilt the head to focus on the gripper as the arm is moved. This is can be useful when trying to pick something up.
<p align="center">
    <img src="../images/follow-gripper.gif">
</p>

#### Predictive Display
The 'predictive display' mode will overlay a trajectory over the video stream that Stretch will follow. Stretch's speed and heading will depend on the length and curve of the trajectory. Stretch will move faster the longer the trajectory is and slower the shorter the trajectory is. The trajectory will turn red when you click and the robot is moving. The robot will rotate in place when you click on the base and will move backwards when you click behind the base. In the `press-release` and `click-click` [action modes](#action-modes) you can move the cursor to update the trajectory while the robot is moving. Additionally, you can scale the speed by selecting one of the speed controls. 
<p align="center">
    <img src="../images/predictive-display.gif">
</p>

### Gripper Camera
There are two quick actions for the gripper camera view: (1) `center wrist` and (2) `stow wrist`. Center wrist will turn the wrist out and align it with the arm. Stow wrist will rotate the wrist to the stow position.
<p align="center">
    <img src="../images/quick-actions.gif">
</p>

### Button Pads
Each button pad controls a different set of joints on the robot. When you click a button the robot will move and the button will highlight blue while the robot is moving. The button will turn red when the respective joint is at its limit. 

<table align="center">
  <tr>
    <th>Drive</th>
    <td><img src="../images/drive-optimized.gif"></td>
  </tr>
  <tr>
    <th>Dex Wrist</th>
    <td><img src="../images/wrist-gripper.gif"></td>
  </tr>
  <tr>
    <th>Arm & Lift</th>
    <td><img src="../images/arm-lift.gif"></td>
  </tr>
</table>

### Action Modes
The action modes can be selected in the dropdown in the top-left corner of the interface. The action modes provides varying degrees of discrete and continuous control.

- **Step Actions**: When you click, Stretch will move a fixed distance based on the selected speed.
- **Press-Release**: Stretch will move while you are pressing and holding the button and will stop when you release.
- **Click-Click**: Stretch will start moving when you click and will stop when you click again. You can also stop Stretch by moving the cursor outside the button you clicked.

## Troubleshooting

If you're having trouble with the steps in the guide, please check the following tips:

### Stretch fails launch the interface

The interface will have created a .zip file with its logs. Please contact Hello Robot support and include the file. The log files will typically include information on what went wrong.
