# Changing Tools

Stretch was designed to support more than one type of gripper, so you can change the tool attached to the end of the robot's arm. This tutorial will go through the different end-of-arm tools available and how configure them in software after a hardware tool change.

## Stretch Supported Tools

The supported end-of-arm tool configurations are based on your robot model and can be found in the table below.


|  Tool 	                |   Stretch 3 	|   Stretch RE2 |   Stretch RE1 |
|--------	                |------------	|-------------  |------------   |  
|   eoa_wrist_dw3_tool_sg3	|   :white_check_mark:         |       :white_check_mark:        |       :x:        |
|   eoa_wrist_dw3_tool_nil	|   :white_check_mark:         |       :white_check_mark:        |       :x:        |
|   tool_stretch_dex_wrist	|   :white_check_mark:         |       :white_check_mark:        |        :white_check_mark:        |
|   tool_stretch_gripper	|   :white_check_mark:         |       :white_check_mark:        |        :white_check_mark:        |
|   tool_none	            |   :white_check_mark:         |       :white_check_mark:        |        :white_check_mark:        |

The Stretch software has one parameter, called `robot.tool`, that decides what tool the software is currently configured for. You can run the following command in a Terminal to print out the `robot.tool` parameter:
```{.bash .shell-prompt .copy}
stretch_params.py | grep robot.tool
```
The output will look like:
```
stretch_configuration_params.yaml     param.robot.tool     eoa_wrist_dw3_tool_sg3
```

## Configuring a new Tool

Once you have completed the hardware installation of a new tool, you can use a CLI called `stretch_configure_tool.py` to configure your software for the new tool. This CLI automatically assesses the hardware found on the robot and configures the appropriate software configuration. The configuration changes would involve updating the parameter shown above and generating updated URDFs.

```commandline
~$ stretch_configure_tool.py -h

For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

usage: stretch_configure_tool.py [-h] [-v]

Configure Stretch's software for the attached tool

options:
  -h, --help     show this help message and exit
  -v, --verbose  Prints more information
```

You can use this CLI any time to verify the integrity of your software configuration for the officially supported tools.

```commandline
$ stretch_configure_tool.py

For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

Loading...
Your software is currently configured for the eoa_wrist_dw3_tool_sg3 tool
Which seems correct based on the hardware connected to your robot
Done!
```

If the CLI is unable to determine the right tool for your robot, it will let you know. In this case, reach out to Hello Robot Support for help.

```commandline
$ stretch_configure_tool.py

For use with S T R E T C H (R) from Hello Robot Inc.
---------------------------------------------------------------------

Loading...
Your software is currently configured for the eoa_wrist_dw3_tool_sg3 tool
...
Unable to find any tool that matches the hardware connected to your robot. Contact Hello Robot support for help.
Not changing anything. Exiting.
```

## Add support for a new tool

If you've created a new tool for Stretch, you may want it to be supported by the `stretch_configure_tools.py` CLI. The source code for the CLI can be [found here](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_configure_tool.py). Feel free to open a Github pull request to merge support for your tool into the CLI.
