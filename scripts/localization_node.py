#!/usr/bin/env python2

import numpy as np
import rospy
from shipbot_ros.msg import ChassisFeedback
from shipbot_ros.msg import ChassisState
from shipbot_ros.msg import ChassisCommand
from kf import *

got_fbk = False 
got_cmd = False 
prev_t = 0
t = 0

# This is probably a bit confusing. The Arduino is integrating
# the z component of the gyro, and it's giving us its own current
# yaw estimate. We use the difference between the Arduino's previous
# yaw estimate and the current to propagate the robot's yaw in the motion
# model. The reason we don't use the gyro directly is that there's a decent
# amount of delay until we get the value, and we don't want to propagate
# outdated gyro values for this much time.
ardu_yaw = 0
tofs = [0, 0, 0, 0]
vx = 0
vy = 0
cmd_w = 0 # TODO: maybe use this instead of gyro?

# chassis_callback: populates the tof sensor values and the Arduino's
# integrated yaw
# ARGUMENTS
# fbk_msg: a ROS message of type shipbot_ros/ChassisFeedback
def chassis_callback(fbk_msg):
  global t
  global ardu_yaw
  global tofs
  global got_fbk

  got_fbk = True

  t = fbk_msg.header.stamp.secs 
  ardu_yaw = fbk_msg.yaw
  tofs = fbk_msg.tofs

# command_callback: populates the commanded body velocity
# ARGUMENTS
# cmd_msg: a ROS message of type shipbot_ros/ChassisCommand
def command_callback(cmd_msg):
  global vx
  global vy
  global cmd_w
  global got_cmd

  got_cmd = True

  vx = cmd_msg.vx
  vy = cmd_msg.vy
  w = cmd_msg.w

rospy.init_node('localization_node', anonymous=True)
chassis_sub = rospy.Subscriber('/shipbot/chassis_feedback', ChassisFeedback, chassis_callback)
command_sub = rospy.Subscriber('/shipbot/chassis_command', ChassisCommand, command_callback)
state_pub = rospy.Publisher('/shipbot/chassis_state', ChassisState, queue_size=1)
state_msg = ChassisState()

maxx = 1.524
maxy = 0.9144

initialized = False
yawsum = 0 

nfilters = 8
states = np.zeros((3, nfilters))
covs = np.array([np.eye(3) for i in range(nfilters)])
log_weights = np.log(np.ones(nfilters)/nfilters)
prev_t = 0
prev_yaw = 0

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
  if got_fbk:
    got_fbk = False

    if not initialized:
      for i in range(nfilters):
        states[:, i], covs[i] = init_state_given_yaw(i*np.pi/4 + 0.01, tofs)

      print(states)
      quit()
      initialized = True
      prev_t = t
      prev_yaw = ardu_yaw
      continue

    new_log_weights = np.zeros(nfilters)
    for i in range(nfilters):
      states[:, i], covs[i] = predict(states[:, i], covs[i], ardu_yaw - prev_yaw, vx, vy, t - prev_t)
      states[:, i], covs[i], new_log_weights[i] = correct(states[:, i], covs[i], tofs, log_weights[i])

    
    yawsum += abs(ardu_yaw - prev_yaw)
    if yawsum > 2*np.pi:
      log_weights = new_log_weights
      log_weights = normalize_log_weights(log_weights)

      live_filters = np.logical_and(np.logical_and(log_weights > -10, states[0] > 0), states[1] > 0)
      live_filters = np.logical_and(live_filters, np.logical_and(states[0] < maxx, states[1] < maxy))
      if np.any(live_filters):
        states = states[:, live_filters]
        covs = covs[live_filters]
        log_weights = normalize_log_weights(log_weights[live_filters])
        nfilters = len(covs)

    state = np.matmul(states, np.exp(log_weights))

    state_msg.x = state[0]
    state_msg.y = state[1]
    state_msg.yaw = state[2]
    state_msg.header.stamp = rospy.Time().now()
    state_pub.publish(state_msg)

    state_to_print = np.copy(state)
    state_to_print[2] *= 180/np.pi
    state_to_print[:2] /= 0.0254
    print(np.trunc(state_to_print))
    prev_t = t
    prev_yaw = ardu_yaw

  rate.sleep()

