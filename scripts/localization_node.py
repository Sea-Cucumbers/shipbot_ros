#!/usr/bin/env python2

import numpy as np
import rospy
from shipbot_ros.msg import ChassisFeedback
from kf import *

got_fbk = False 
prev_t = 0
t = 0
fbk_yaw = 0
tofs = [0, 0, 0, 0]

def chassis_callback(fbk_msg):
  global t
  global fbk_yaw
  global tofs
  global got_fbk

  got_fbk = True

  t = fbk_msg.header.stamp.secs 
  fbk_yaw = fbk_msg.yaw
  tofs = fbk_msg.tofs

rospy.init_node('localization_node', anonymous=True)
chassis_sub = rospy.Subscriber('/shipbot/chassis_feedback', ChassisFeedback, chassis_callback)

maxx = 152.4
maxy = 91.44

initialized = False
yawsum = 0 

nfilters = 8
states = np.zeros((5, nfilters))
covs = np.array([np.eye(5) for i in range(nfilters)])
log_weights = np.log(np.ones(nfilters)/nfilters)
prev_t = 0
prev_yaw = 0

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
  if got_fbk:
    got_fbk = False

    if not initialized:
      for i in range(nfilters):
        states[:, i], covs[i] = init_state_given_yaw(i*np.pi/4, tofs)

      initialized = True
      prev_t = t
      prev_yaw = fbk_yaw
      continue

    new_log_weights = np.zeros(nfilters)
    for i in range(nfilters):
      states[:, i], covs[i] = predict(states[:, i], covs[i], fbk_yaw - prev_yaw, t - prev_t)
      states[:, i], covs[i], new_log_weights[i] = correct(states[:, i], covs[i], tofs, log_weights[i])

    
    yawsum += abs(fbk_yaw - prev_yaw)
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
    state_to_print = np.copy(state)
    state_to_print[2] *= 180/np.pi
    state_to_print[:2] /= 2.54
    print(np.trunc(state_to_print))
    prev_t = t
    prev_yaw = fbk_yaw

  rate.sleep()

