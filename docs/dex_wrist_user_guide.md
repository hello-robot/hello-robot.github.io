# Stretch RE1: Dex Wrist User Guide

In this guide, we will cover the installation, configuration, and use of the Stretch Dex Wrist.

## Overview

The Stretch Dex Wrist is an optional add-on to the RE1. It adds pitch and roll degrees of freedom to the standard wrist yaw joint. It also includes a slightly modified version of the standard Stretch Compliant Gripper. 

**NOTE:** If your robot did not ship with the Stretch Dex Wrist pre-installed you will want to first proceed to the Appendix: Installation and Configuration at the end of this guide. 

## Functional Specification

![](./images/dex_wrist_specification.png)



## Working with the Dex Wrist

### Safe Use

The Dex Wrist requires added attention to safety. Its additional dexterity introduces new pinch points around the wrist pitch and roll degrees of freedom.

**NOTE:** Please review the [Robot Safety Guide](https://docs.hello-robot.com/robot_safety_guide/) prior to working with the Dex Wrist.

In addition to these precautions, the Dex Wrist requires attention to pinch points between:

* The wrist pitch and wrist yaw structures during yaw motion
* The gripper and wrist pitch structures during pitch motion

The Dex Wrist includes a pinch point safety marking as a reminder to users:

![](./images/hand_crush_rs.png)

### Avoiding Collisions

The added dexterity of the Dex Wrist introduces new opportunities for self-collision between the robot tool and the robot. These include

* Running the tool into the base during lift downward motion
* Running the tool into the ground
* Running the tool into the wrist yaw structure

We recommend becoming familiar with the potential collision points of the Dex Wrist by commanding careful motions through the `stretch_xbox_controller_teleop.py` tool. 

With Stretch Body v0.1.0 we introduce a [simple collision avoidance controller](https://github.com/hello-robot/stretch_body/blob/feature/collision_avoidance/body/stretch_body/robot_collision.py). 

The collision avoidance behavior acts to dynamically set the robot joint limits according to simple models of its kinematic state.  The avoidance behavior is defined in [`collision_model.py`](https://github.com/hello-robot/stretch_tool_share/blob/master/python/stretch_tool_share/stretch_dex_wrist/collision_model.py)

For performance reasons this collision avoidance behavior is coarse and does not prevent all self-collisions and considered 'experimental'. The collision avoidance is off by default for the standard Stretch RE1. For robots with the wrist we turn it on by default. It be turned on or off by modifying the following in your user YAML:

```yaml
robot:
  use_collision_manager: 1
```

### XBox Teleoperation

The Dex Wrist can be teleoperated using the XBox controller. When the Dex Wrist is installed the `stretch_xbox_controller_teleop.py` tool will automatically remap control of the pan-tilt head to control of the pitch-roll wrist.

```console
$ stretch_xbox_controller_teleop.py
```

The new key mapping is shown below. A printable version is available [here](./images/stretch_re1_dex_wrist_teleop_guide.pdf).

![](./images/stretch_re1_dex_wrist_teleop_guide.png)

### Stretch Body Interface

The new [WristPitch](https://github.com/hello-robot/stretch_tool_share/blob/master/python/stretch_tool_share/stretch_dex_wrist_beta/wrist_pitch.py) and [WristRoll](https://github.com/hello-robot/stretch_tool_share/blob/master/python/stretch_tool_share/stretch_dex_wrist_beta/wrist_roll.py) joints are accessed from Stretch Body in the same manner as the [WristYaw](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wrist_yaw.py) joint. 

Control of the Stretch Dex Wrist uses the same interfaces as the rest of the Stretch Body Robot joints.  For example:

```python
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

#Move arm to safe manipulation location
robot.stow()
robot.lift.move_to(0.4)
robot.push_command()
time.sleep(2.0)

#Pose the Dex Wrist
robot.end_of_arm.move_to('wrist_yaw',0)
robot.end_of_arm.move_to('wrist_pitch',0)
robot.end_of_arm.move_to('wrist_roll',0)
robot.end_of_arm.move_to('stretch_gripper',50)
time.sleep(2.0)

#Go back to stow and shutdown
robot.stow()
robot.stop()

```

You can jog the individual joints of the wrist with the Stretch Body interface using the [`stretch_dex_wrist_jog.py`](https://github.com/hello-robot/stretch_tool_share/blob/feature/master/python/bin/stretch_dex_wrist_jog.py) tool that installs with the Stretch Tool Share:

```console
$ stretch_dex_wrist_jog.py --pitch
$ stretch_dex_wrist_jog.py --yaw
$ stretch_dex_wrist_jog.py --roll
```

For reference, the parameters for the Stretch Dex Wrist (which can be overridden in the user YAML) can be seen in  [params.py](https://github.com/hello-robot/stretch_tool_share/blob/master/python/stretch_tool_share/stretch_dex_wrist_beta/params.py).

### Stretch ROS Interface

The Dex Wrist can be controlled via ROS as well, as shown in the [keyboard teleoperation code](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/keyboard_teleop). To test the interface:

```console
$ roslaunch stretch_calibration simple_test_head_calibration.launch
```

You can use Ctrl-C to exit when done. The menu interface is:

```
---------- KEYBOARD TELEOP MENU -----------
                                           
              i HEAD UP                    
 j HEAD LEFT            l HEAD RIGHT       
              , HEAD DOWN                  
                                           
                                           
 7 BASE ROTATE LEFT     9 BASE ROTATE RIGHT
 home                   page-up            
                                           
                                           
              8 LIFT UP                    
              up-arrow                     
 4 BASE FORWARD         6 BASE BACK        
 left-arrow             right-arrow        
              2 LIFT DOWN                  
              down-arrow                   
                                           
                                           
              w ARM OUT                    
 a WRIST FORWARD        d WRIST BACK       
              x ARM IN                     
                                           
                                           
 c PITCH FORWARD        v PITCH BACK       
 o ROLL FORWARD         p ROLL BACK        
                                           
                                           
              5 GRIPPER CLOSE              
              0 GRIPPER OPEN               
                                           
  step size:  b BIG, m MEDIUM, s SMALL     
                                           
              q QUIT                       
                                           
-------------------------------------------
```



![](images/dex_wrist_rviz_rs.png)



## Appendix: Installation and Configuration

Robots that did not ship with the Dex Wrist installed will require additional hardware and software installation.

### Production Batch Variation

Earlier production 'batches' of Stretch will require a hardware upgrade prior to use the Dex Wrist. To check your robot's batch, run:

```console
$ stretch_about.py
```

Refer to this table to determine what changes are required for your robot.

| Batch Name        | Upgrade Wacc Board | Update Baud Rate |
| ----------------- | ------------------ | ---------------- |
| Guthrie           | Y                  | Y                |
| Hank              | Y                  | Y                |
| Irma              | Y                  | Y                |
| Joplin            | N                  | Y                |
| Kendrick or later | N                  | N                |

#### Upgrade Wacc Board

If your robot requires a Wacc Board upgrade please follow the [instructions here](https://github.com/hello-robot/stretch_factory/tree/master/updates/013_WACC_INSTALL) with the assistance of Hello Robot support. This must be done before attaching the Dex Wrist to our robot.

#### Update Baud Rate

The new wrist requires moving to 115200 Baud communication for all Dynamixel servos from the previous 57600. Use the commands below.

```console
$ RE1_dynamixel_set_baud.py /dev/hello-dynamixel-head 11 115200
---------------------
Checking servo current baud for 57600
----
Identified current baud of 57600. Changing baud to 115200
Success at changing baud

$ RE1_dynamixel_set_baud.py /dev/hello-dynamixel-head 12 115200
---------------------
Checking servo current baud for 57600
----
Identified current baud of 57600. Changing baud to 115200
Success at changing baud

$ RE1_dynamixel_set_baud.py /dev/hello-dynamixel-wrist 13 115200
---------------------
Checking servo current baud for 57600
----
Identified current baud of 57600. Changing baud to 115200
Success at changing baud
```

### Attaching the Dex Wrist

![](./images/dex_wrist_install.png)

Power down your Stretch before installing the Dex Wrist.

The Dex Wrist mounts to the bottom of the [Stretch Wrist Tool Plate](https://docs.hello-robot.com/hardware_user_guide/#wrist-tool-plate). Installation requires

* 8 [M2x6mm Torx FHCS bolts](https://www.mcmaster.com/90236A104/) (provided)
* 4 [M2.5x4mm Torx FHCS bolts](https://www.mcmaster.com/92703A448/) (provided)
* 2 [M2.5x8mm SHCS bolts](https://www.mcmaster.com/91290A102/) (provided)
* T6 Torx wrench (provided)
* T8 Torx wrench (provided)
* 2mm Hex key (provided)

First, remove the standard Stretch Gripper if it is still attached [according to the Hardware User Guide](https://docs.hello-robot.com/hardware_user_guide/#gripper-removal). 

#### Mounting Bracket

Note where the forward direction is on the wrist yaw tool plate. The forward direction is indicated by the  additional alignment hole that is just outside the bolt pattern (shown pointing down in the image)

![](./images/dex_wrist_C_rs.png)

Using the T6 Torx wrench, attach the wrist mount bracket (A) to the bottom of the tool plate using the provided  M2x6mm bolts (B). 

**NOTE: ensure that the forward direction of the bracket (also indicated by an alignment hole) matches the forward direction of the tool plate.**



![![]](./images/dex_wrist_bracket_install_rs.png)

Now route the Dynamixel cable coming from the Stretch Wrist Yaw through the hollow bore of the wrist yaw joint.

 

**NOTE: During this step ensure the Dynamixel cable from the wrist yaw exits out the back (towards the shoulder)**


Next, raise the wrist module up vertically into the mounting bracket


![![]](./images/dex_wrist_roll_install_rs.png)


, then sliding it over horizontally so that the bearing mates onto its post. 


![![]](./images/dex_wrist_roll_install2_rs.png)



Now rotate the wrist yaw joint so the wrist pitch servo body is accessible. Attach the pitch servo to the mounting bracket using the 4 M2.5x4mm screws (C) using the T8 Torx wrench.



![](./images/dex_wrist_pitch_bracket_attach_rs.png)

Finally, route the Dynamixel cable into the wrist pitch servo (pink) and install the cable clip (D) using the M2.5x8mm bolts and the 2mm hex wrench.

 ![](./images/dex_wrist_cable_route_rs.png)

 ![](./images/dex_wrist_cable_detail.png)

### Software Configuration

Robots that did not ship with the Dex Wrist pre-installed will require their software to be updated and configured.

**NOTE:** Each user account on the robot will need to run the following steps to configure the Dex Wrist.

#### Upgrade Stretch Body

Ensure the latest version of Stretch Body and Stretch Factory are installed

```console
$ pip2 install hello-robot-stretch-body -U --no-cache-dir
$ pip2 install hello-robot-stretch-body-tools -U --no-cache-dir
$ pip2 install hello-robot-stretch-factory -U --no-cache-dir
$ pip2 install hello-robot-stretch-tool-share -U --no-cache-dir
```

#### Backup User YAML

```console
$ cd $HELLO_FLEET_PATH/$HELLO_FLEET_ID
$ cp stretch_re1_user_params.yaml stretch_re1_user_params.yaml.bak
```

#### Run Installation Script

```console
$ cd ~/repos
$ git clone https://github.com/hello-robot/stretch_install
$ cd ./stretch_install
$ git pull
$ ./factory/stretch_install_dex_wrist.sh
```

**NOTE:** The factory gripper calibration may not provide the full range of motion in some cases. If necessary you can dial in the gripper calibration with the tool `RE1_gripper_calibrate.py`

------
<div align="center"> All materials are Copyright 2020 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>
