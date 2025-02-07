# Distributions & Roadmap

Stretch's "robot distributions" are bundles of software installed onto the robot's onboard computer. A distribution typically includes an [Ubuntu](https://en.wikipedia.org/wiki/Ubuntu) operating system, Stretch's Python libraries, a [ROS distribution](https://docs.ros.org/en/rolling/Releases.html) with Stretch's ROS packages, and firmware for Stretch's hardware.

## List of Distributions

| Distribution                                                                              | Release Date | Description | EOL Date |
|-------------------------------------------------------------------------------------------|--------------|-------------|----------|
| <div style='background-color: #5bdb5b; padding:10px'>Ubuntu 22.04 **(Recommended)**</div> | <div style='padding:10px'>January 2024</div> | <div style='padding:10px'>The 22.04 distribution comes with ROS2 Humble and Python3.10. It does <b>not</b> come with ROS1.</div> | <div style='padding:10px'>January 2027</div> |

### Deprecated Distributions

| Distribution                                 | Release Date | Description | EOL Date |
|----------------------------------------------|--------------|-------------|----------|
| <div style='background-color: #ed8080; padding:10px'>Ubuntu 20.04 **(Deprecated)**</div>                                              | <div style='padding:10px'>October 2022</div> | <div style='padding:10px'>The 20.04 distribution comes with ROS1 Noetic and Python3.8. This distribution is in maintenance (i.e. only receives bug-fixes, and new features aren't ported back).</div> | <div style='padding:10px'>January 2025</div> |
| <div style='background-color: #ed8080; padding:10px;'>Ubuntu 18.04 **(Deprecated)**</div> | <div style='padding:10px'>May 2020</div> | <div style='padding:10px'>18.04 was the first distribution for Stretch, and included software for ROS1 Melodic and Python2. This distribution is no longer supported.</div> | <div style='padding:10px'>August 2023</div> |

### Determining your Current Distribution

Open a terminal and run the following command. The outputs contains a description of the installed distribution.

```{.bash .shell-prompt}
lsb_release -d
Description:	Ubuntu 22.04.3 LTS
```

### Installing a Distribution

To install one of the distributions above, check out the [Performing a Robot Installation](../../stretch-install/robot_install/) guide.

## Roadmap

We may support ROS 2 Jazzy (a [future distribution](https://docs.ros.org/en/rolling/Releases.html#future-distributions) of ROS2) and Ubuntu 24.04 in the future. Reach out to [Hello Robot](mailto:support@hello-robot.com) to provide feedback on distributions you'd like to see.
