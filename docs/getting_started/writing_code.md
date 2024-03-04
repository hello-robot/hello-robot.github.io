# Writing Code

This tutorial will introduce the two most common ways of developing software with Stretch - Python and ROS 2. The Stretch Body Robot API will be introduced, and you will write a simple Python program using commands to move the various joints of the robot.

## Background

Stretch supports two different approaches to developing software - ROS 2 and Python.

ROS 2 (Robot Operating System 2) is commonly used by robotics developers as a way to bootstrap their software development. It consists of a set of software libraries and tools for building robot applications. In technical terms, ROS is a middleware framework that is a collection of transport protocols, development and debugging tools, and open-source packages.

Python is one of the most popular programming languages in the world. It is easy to learn, flexible, and commonly used in academic and research settings. In robotics, Python is particularly popular among the machine learning community, but it is suitable for other types of projects as well and can be a great way to get used to working with Stretch.

You can learn more about the difference between these two approaches in [Developing With Stretch](../../developing/basics/). As Python for Stretch is quite straightforward and ROS 2 can have a bit of a learning curve, this guide will focus on writing a Python program for Stretch. Continuing with these tutorials, you will find examples of ROS programs in the Demos section.

## Stretch Body

The low-level Python interface for the robot is called Stretch Body, which is a Python library pre-installed on your robot. Let's write our first program exploring some of the features of this API.

Open up a Terminal. First, let's verify that the system is ready to go:

```{.bash .shell-prompt .copy}
stretch_system_check.py
```

You may need to home the robot.

```{.bash .shell-prompt .copy}
stretch_robot_home.py
```

Once the robot has homed, launch iPython. iPython is an interactive console where each line of code runs immediately.

```{.bash .shell-prompt .copy}
ipython3
```

To begin, we need to initialize the Robot class which encapsulates all devices on the robot. Run the following in your iPython window, hitting enter after each line:

```{.python .no-copy}
import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
```

The `startup()` command starts communication with the robot's devices. Next, command the robot to move. Run the following:

```{.python .no-copy}
robot.stow()
```

The robot will move to its stow position, with the shoulder lowered, arm retracted, and wrist and gripper tucked in.

You can send commands to one joint at a time. Run the following:

```{.python .no-copy}
robot.arm.move_to(0.25)
robot.push_command()
```

Move the lift joint half way up:

```{.python .no-copy}
robot.lift.move_to(0.55)
robot.push_command()
```

The `move_to()` command tells one of the robot joints to go to a specific absolute position. You can also use `move_by()` to instead command the joint to move incrementally from its current position:

```{.python .no-copy}
robot.lift.move_by(0.1)
robot.push_command()

robot.lift.move_by(-0.1)
robot.push_command()
```

You can queue up multiple movements on different joints before pushing the command to the robot. These will execute simultaneously.

```{.python .no-copy}
robot.lift.move_to(0.7)
robot.arm.move_by(0.1)
robot.push_command()
```

If you queue up two movements on the same joint, they will overwrite the previous command and only the last will execute when the command is pushed.

Next, you can  make the robot print out the status of a joint in a human-interpretable fashion. Try:

```{.python .no-copy}
robot.pretty_print()
```

As you can see, this is a lot of information. You can also call pretty_print() on just one robot subsystem at a time:

```{.python .no-copy}
robot.lift.pretty_print()
```

Moving the robot head is made easy by a number of pre-programmed positions in useful orientations. Try:

```{.python .no-copy}
robot.head.pose('tool')
robot.head.pose('ahead')
robot.head.pose('wheels')
robot.head.pose('back')
```

The wrist and gripper joints are part of `end_of_arm`. Note that the following commands execute when sent - no need for `robot.push_command()`. Try:

```{.python .no-copy}
robot.end_of_arm.move_to('wrist_yaw',0.5)
robot.end_of_arm.move_to('wrist_pitch', -0.5)
robot.end_of_arm.move_to('wrist_roll', 1)
robot.end_of_arm.move_by('wrist_roll', -1)

robot.end_of_arm.move_to('stretch_gripper',50)
robot.end_of_arm.move_to('stretch_gripper',-50)
```
When you're ready to end the session, run the following command to shut down the Robot instance and stop communication with the robot's devices. This is important as leaving this running could impede another process from accessing these devices later.

```{.python .no-copy}
robot.stop()
```

To close the iPython session, type `exit()` and press enter; or simply close the Terminal window.

Additional information on the Stretch Body Python API is available in the [Python Tutorial](../../python/moving/) series.

## Learn More

The [Developing with Stretch](../../developing/basics/) guides are a great place to start for additional helpful information on how to approach writing software for Stretch. Tutorial tracks for working with Stretch in both Python and ROS 2 are also available on Stretch Docs.

## Next Steps

In the next tutorial, [Demo #1 - Mapping & Navigation](./demos_mapping_and_navigation.md), we will look at two ways that Stretch can navigate within a map.

<!-- TODO:

---

## Troubleshooting

If you're having trouble with the steps in the guide, please check the following tips:

-->

------
<div align="center"> All materials are Copyright 2020-2024 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
