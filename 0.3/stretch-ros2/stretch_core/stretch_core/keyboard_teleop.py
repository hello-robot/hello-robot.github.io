#!/usr/bin/env python3

import math
from .keyboard import KBHit
import argparse as ap

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

import hello_helpers.hello_misc as hm


class GetKeyboardCommands:

    def __init__(self, mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on):
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on
        self.kb = KBHit()
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0
        self.small_deg = 3.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.005  #0.02
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.big_deg = 12.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.06
        self.mode = 'position' #'manipulation' #'navigation'

    def get_deltas(self):
        if self.step_size == 'small':
            deltas = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            deltas = {'rad': self.medium_rad, 'translate': self.medium_translate} 
        if self.step_size == 'big':
            deltas = {'rad': self.big_rad, 'translate': self.big_translate} 
        return deltas

    def print_commands(self):
        print('---------- KEYBOARD TELEOP MENU -----------')
        print('                                           ')
        print('              i HEAD UP                    ')
        print(' j HEAD LEFT            l HEAD RIGHT       ')
        print('              , HEAD DOWN                  ')
        print('                                           ')
        print('                                           ')
        print(' 7 BASE ROTATE LEFT     9 BASE ROTATE RIGHT')
        print(' home                   page-up            ')
        print('                                           ')
        print('                                           ')
        print('              8 LIFT UP                    ')
        print('              up-arrow                     ')
        print(' 4 BASE FORWARD         6 BASE BACK        ')
        print(' left-arrow             right-arrow        ')
        print('              2 LIFT DOWN                  ')
        print('              down-arrow                   ')
        print('                                           ')
        print('                                           ')
        print('              w ARM OUT                    ')
        print(' a WRIST FORWARD        d WRIST BACK       ')
        print('              x ARM IN                     ')
        print('                                           ')
        print('                                           ')
        print('              5 GRIPPER CLOSE              ')
        print('              0 GRIPPER OPEN               ')
        print('                                           ')
        print('  step size:  b BIG, m MEDIUM, s SMALL     ')
        print('                                           ')
        print('              q QUIT                       ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self, node):
        command = None
        c = None

        if self.kb.kbhit(): # Returns True if any key pressed
            c = self.kb.getch()
             
        ####################################################
        ## MOSTLY MAPPING RELATED CAPABILITIES
        ## (There are non-mapping outliers.)
        ####################################################
        
        # Sequential performs a fixed number of autonomus mapping iterations
        if (c == '!') and self.mapping_on:
            number_iterations = 4
            for n in range(number_iterations):
                # Trigger a 3D scan with the D435i
                trigger_request = Trigger.Request() 
                trigger_result = node.trigger_head_scan_service(trigger_request)
                node.get_logger().info('trigger_result = {0}'.format(trigger_result))

                # Trigger driving the robot to the estimated next best place to scan
                trigger_request = Trigger.Request() 
                trigger_result = node.trigger_drive_to_scan_service(trigger_request)
                node.get_logger().info('trigger_result = {0}'.format(trigger_result))
                
        # Trigger localizing the robot to a new pose anywhere on the current map
        if ((c == '+') or (c == '=')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_global_localization_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger localizing the robot to a new pose that is near its current pose on the map
        if ((c == '-') or (c == '_')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_local_localization_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger driving the robot to the estimated next best place to perform a 3D scan
        if ((c == '\\') or (c == '|')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_drive_to_scan_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger performing a 3D scan using the D435i
        if (c == ' ') and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_head_scan_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger rotating the mobile base to align with the nearest 3D cliff detected visually
        if ((c == '[') or (c == '{')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_align_with_nearest_cliff_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # DEPRECATED: Trigger extend arm until contact
        if ((c == ']') or (c == '}')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_reach_until_contact_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # DEPRECATED: Trigger lower arm until contact
        if ((c == ':') or (c == ';')) and self.mapping_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_lower_until_contact_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))
            
        
        ####################################################
        ## OTHER CAPABILITIES
        ####################################################

        # Trigger Hello World whiteboard writing demo
        if ((c == '`') or (c == '~')) and self.hello_world_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_write_hello_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger open drawer demo with downward hook motion
        if ((c == 'z') or (c == 'Z')) and self.open_drawer_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_open_drawer_down_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger open drawer demo with upward hook motion
        if ((c == '.') or (c == '>')) and self.open_drawer_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_open_drawer_up_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger clean surface demo
        if ((c == '/') or (c == '?')) and self.clean_surface_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_clean_surface_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))
            
        # Trigger grasp object demo    
        if ((c == '\'') or (c == '\"')) and self.grasp_object_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_grasp_object_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

        # Trigger deliver object demo    
        if ((c == 'y') or (c == 'Y')) and self.deliver_object_on:
            trigger_request = Trigger.Request() 
            trigger_result = node.trigger_deliver_object_service(trigger_request)
            node.get_logger().info('trigger_result = {0}'.format(trigger_result))

            
        ####################################################
        ## BASIC KEYBOARD TELEOPERATION COMMANDS
        ####################################################
        
        # 8 or up arrow
        if c == '8' or c == '\x1b[A':
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        # 2 or down arrow
        if c == '2' or c == '\x1b[B':
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}
        if self.mode == 'manipulation':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'joint_mobile_base_translation', 'delta': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'joint_mobile_base_translation', 'delta': -self.get_deltas()['translate']}
        elif self.mode == 'position':
            # 4 or left arrow
            if c == '4' or c == '\x1b[D':
                command = {'joint': 'translate_mobile_base', 'inc': self.get_deltas()['translate']}
            # 6 or right arrow
            if c == '6' or c == '\x1b[C':
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_deltas()['translate']}
            # 1 or end key 
            if c == '7' or c == '\x1b[H':
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_deltas()['rad']}
            # 3 or pg down 5~
            if c == '9' or c == '\x1b[5':
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_deltas()['rad']}
        elif self.mode == 'navigation':
            node.get_logger().info('ERROR: Navigation mode is not currently supported.')

        if c == 'w' or c == 'W':
            command = {'joint': 'wrist_extension', 'delta': self.get_deltas()['translate']}
        if c == 'x' or c == 'X':
            command = {'joint': 'wrist_extension', 'delta': -self.get_deltas()['translate']}
        if c == 'd' or c == 'D':
            command = {'joint': 'joint_wrist_yaw', 'delta': -self.get_deltas()['rad']}
        if c == 'a' or c == 'A':
            command = {'joint': 'joint_wrist_yaw', 'delta': self.get_deltas()['rad']}
        if c == '5' or c == '\x1b[E' or c == 'g' or c == 'G':
            # grasp
            command = {'joint': 'joint_gripper_finger_left', 'delta': -self.get_deltas()['rad']}
        if c == '0' or c == '\x1b[2' or c == 'r' or c == 'R':
            # release
            command = {'joint': 'joint_gripper_finger_left', 'delta': self.get_deltas()['rad']}
        if c == 'i' or c == 'I':
            command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == ',' or c == '<':
            command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'j' or c == 'J':
            command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.get_deltas()['rad'])}
        if c == 'l' or c == 'L':
            command = {'joint': 'joint_head_pan', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if c == 'b' or c == 'B':
            node.get_logger().info('process_keyboard.py: changing to BIG step size')
            self.step_size = 'big'
        if c == 'm' or c == 'M':
            node.get_logger().info('process_keyboard.py: changing to MEDIUM step size')
            self.step_size = 'medium'
        if c == 's' or c == 'S':
            node.get_logger().info('process_keyboard.py: changing to SMALL step size')
            self.step_size = 'small'
        if c == 'q' or c == 'Q':
            node.get_logger().info('keyboard_teleop exiting...')
            node.get_logger().info('Received quit character (q), so exiting')

        ####################################################

        return command


class KeyboardTeleopNode(hm.HelloNode):

    def __init__(self, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False, grasp_object_on=False, deliver_object_on=False):
        hm.HelloNode.__init__(self)
        self.keys = GetKeyboardCommands(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on
        self.joint_state = JointState()
        
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        command = self.keys.get_command(self)
        self.send_command(command)

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            trajectory_goal = FollowJointTrajectory.Goal()
            # trajectory_goal.goal_time_tolerance = rclpy.time.Time()
            joint_name = command['joint']
            duration1 = Duration(seconds=0.0)
            duration2 = Duration(seconds=1.0)

            # for base joints
            if 'inc' in command:
                inc = command['inc']
                point1 = MultiDOFJointTrajectoryPoint()
                point2 = MultiDOFJointTrajectoryPoint()
                point1.time_from_start = duration1.to_msg()
                point2.time_from_start = duration2.to_msg()
                transform1 = Transform()
                transform2 = Transform()
                transform1.translation.x = 0.0
                transform1.rotation.w = 1.0
                transform2.translation.x = inc
                transform2.rotation.w = 1.0
                point1.transforms = [transform1]
                point2.transforms = [transform2]
                # joint_name should be 'position' and not one generated by command i.e. 'translate/rotate_mobile_base'
                joint_name = 'position'
                trajectory_goal.multi_dof_trajectory.joint_names = [joint_name]
                trajectory_goal.multi_dof_trajectory.points = [point1, point2]
                trajectory_goal.multi_dof_trajectory.header.stamp = self.get_clock().now().to_msg()
                self.trajectory_client.send_goal_async(trajectory_goal)

            # for non-base joints
            elif 'delta' in command:
                point1 = JointTrajectoryPoint()
                point2 = JointTrajectoryPoint()
                point1.time_from_start = duration1.to_msg()
                point2.time_from_start = duration2.to_msg()
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point1.positions = [joint_value]
                point2.positions = [new_value]
                trajectory_goal.trajectory.joint_names = [joint_name]
                trajectory_goal.trajectory.points = [point1, point2]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                self.trajectory_client.send_goal_async(trajectory_goal)

    def main(self):
        rclpy.init()
        hm.HelloNode.main(self, 'keyboard_teleop', 'keyboard_teleop', wait_for_first_pointcloud=False)

        if self.mapping_on: 
            self.get_logger().info('Node ' + self.node_name + ' waiting to connect to /funmap/trigger_head_scan.')

            # ROS2 template for creating a service client, must be used for all services below
            # self.trigger_head_scan_service = node.create_client(Trigger, '/funmap/trigger_head_scan')
            # while not self.trigger_head_scan_service.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().info('service not available, waiting again...')
            # self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_head_scan.')

            rospy.wait_for_service('/funmap/trigger_head_scan')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_head_scan.')
            self.trigger_head_scan_service = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_drive_to_scan.')
            self.trigger_drive_to_scan_service = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_global_localization')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_global_localization.')
            self.trigger_global_localization_service = rospy.ServiceProxy('/funmap/trigger_global_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_local_localization')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_local_localization.')
            self.trigger_local_localization_service = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')
            self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy('/funmap/trigger_align_with_nearest_cliff', Trigger)

            rospy.wait_for_service('/funmap/trigger_reach_until_contact')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
            self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger)

            rospy.wait_for_service('/funmap/trigger_lower_until_contact')
            self.get_logger().info('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
            self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        if self.hello_world_on: 
            rospy.wait_for_service('/hello_world/trigger_write_hello')
            self.get_logger().info('Node ' + self.node_name + ' connected to /hello_world/trigger_write_hello.')
            self.trigger_write_hello_service = rospy.ServiceProxy('/hello_world/trigger_write_hello', Trigger)

        if self.open_drawer_on:
            rospy.wait_for_service('/open_drawer/trigger_open_drawer_down')
            self.get_logger().info('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_down.')
            self.trigger_open_drawer_down_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_down', Trigger)

            rospy.wait_for_service('/open_drawer/trigger_open_drawer_up')
            self.get_logger().info('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_up.')
            self.trigger_open_drawer_up_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_up', Trigger)

            
        if self.clean_surface_on:
            rospy.wait_for_service('/clean_surface/trigger_clean_surface')
            self.get_logger().info('Node ' + self.node_name + ' connected to /clean_surface/trigger_clean_surface.')
            self.trigger_clean_surface_service = rospy.ServiceProxy('/clean_surface/trigger_clean_surface', Trigger)

        if self.grasp_object_on:
            rospy.wait_for_service('/grasp_object/trigger_grasp_object')
            self.get_logger().info('Node ' + self.node_name + ' connected to /grasp_object/trigger_grasp_object.')
            self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

        if self.deliver_object_on:
            rospy.wait_for_service('/deliver_object/trigger_deliver_object')
            self.get_logger().info('Node ' + self.node_name + ' connected to /deliver_object/trigger_deliver_object.')
            self.trigger_deliver_object_service = rospy.ServiceProxy('/deliver_object/trigger_deliver_object', Trigger)

        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 10)
        self.subscription

        self.keys.print_commands()
        rclpy.spin(self)
        self.keys.kb.set_normal_term()
        self.destroy_node()
        rclpy.shutdown()

def main():
    try:
        parser = ap.ArgumentParser(description='Keyboard teleoperation for stretch.')
        parser.add_argument('--mapping_on', action='store_true', help='Turn on mapping control. For example, the space bar will trigger a head scan. This requires that the mapping node be run (funmap).')
        parser.add_argument('--hello_world_on', action='store_true', help='Enable Hello World writing trigger, which requires connection to the appropriate hello_world service.')
        parser.add_argument('--open_drawer_on', action='store_true', help='Enable Open Drawer trigger, which requires connection to the appropriate open_drawer service.')
        parser.add_argument('--clean_surface_on', action='store_true', help='Enable Clean Surface trigger, which requires connection to the appropriate clean_surface service.')
        parser.add_argument('--grasp_object_on', action='store_true', help='Enable Grasp Object trigger, which requires connection to the appropriate grasp_object service.')
        parser.add_argument('--deliver_object_on', action='store_true', help='Enable Deliver Object trigger, which requires connection to the appropriate deliver_object service.')

        args, unknown = parser.parse_known_args()
        mapping_on = args.mapping_on
        hello_world_on = args.hello_world_on
        open_drawer_on = args.open_drawer_on
        clean_surface_on = args.clean_surface_on
        grasp_object_on = args.grasp_object_on
        deliver_object_on = args.deliver_object_on

        node = KeyboardTeleopNode(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()