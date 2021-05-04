#!/usr/bin/env python3
import hebi
import rospy
import time

rospy.init_node('write_gains_node', anonymous=True)

lookup = hebi.Lookup()
time.sleep(2)

family_name = "ShipBot"
module_names = ['shoulder1_joint', 'shoulder2_joint', 'elbow_joint', 'wrist1_joint', 'wrist2_joint']
# setup group
group = lookup.get_group_from_names([family_name], module_names)
if group is None:
  print('Group not found: Did you forget to set the module family and name above?')

group_command = hebi.GroupCommand(group.size)
group_command.position_limit_max = 6
group_command.position_limit_min = -6
group.send_command(group_command)
