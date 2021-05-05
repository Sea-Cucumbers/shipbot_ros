#!/usr/bin/env python2

import rospy
from shipbot_ros.srv import TravelAbs
from shipbot_ros.srv import TravelRel
from shipbot_ros.srv import InitialLocalization
from std_srvs.srv import Empty
import time

def localize(req):
  time.sleep(10)
  return True

def travel_abs(req):
  time.sleep(10)
  return True

def travel_rel(req):
  time.sleep(10)
  return True

def stop_chassis(req):
  time.sleep(10)
  return True

rospy.init_node('chassis_control_node', anonymous=True)
 
localize_service = rospy.Service('/chassis_control_node/localize', InitialLocalization, localize)
travel_abs_service = rospy.Service('/chassis_control_node/travel_abs', TravelAbs, travel_abs)
travel_rel_service = rospy.Service('/chassis_control_node/travel_rel', TravelRel, travel_rel)
stop_chassis_service = rospy.Service('/chassis_control_node/stop_chassis', Empty, stop_chassis)
rospy.spin()
