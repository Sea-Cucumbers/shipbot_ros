#!/usr/bin/env python2

import numpy as np
import rospy
from shipbot_ros.msg import ChassisFeedback
from shipbot_ros.msg import ChassisState
from shipbot_ros.msg import ChassisCommand
from kf import *
import threading

got_fbk = False 
got_cmd = False 
t = 0

lock = threading.Lock()

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

  lock.acquire()
  got_fbk = True

  t = fbk_msg.header.stamp.secs + fbk_msg.header.stamp.nsecs/1000000000.0
  ardu_yaw = fbk_msg.yaw
  tofs = fbk_msg.tofs
  lock.release()

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

collect_idx = 0
ncollect = 50
collected = False
collection = np.zeros((4, ncollect))
valid_sensors = np.zeros(4, dtype=np.bool)
initialized = False
yawsum = 0 

nfilters = 8
states = np.zeros((3, nfilters))
covs = np.array([np.eye(3) for i in range(nfilters)])
log_weights = np.log(np.ones(nfilters)/nfilters)
prev_t = 0
cull_t = 0
prev_yaw = 0

print('Waiting for mission control node')
rospy.wait_for_service('/mission_control_node/chassis_done')
chassis_done_client = rospy.ServiceProxy('/mission_control_node/chassis_done', Empty)

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
  if got_fbk:
    lock.acquire()
    got_fbk = False

    if not collected:
      collection[:, collect_idx] = np.array(tofs)
      collect_idx += 1
      if collect_idx >= ncollect:
        var = np.var(collection, axis=1)
        sort_idx = np.argsort(var)
        valid_sensors[sort_idx[0]] = True
        valid_sensors[sort_idx[1]] = True
        collected = True

    if not initialized and collected:
      for i in range(nfilters):
        states[:, i], covs[i] = init_state_given_yaw(i*np.pi/4 + 0.01, np.array(tofs), valid_sensors)

      initialized = True
      prev_t = t
      cull_t = t
      prev_yaw = ardu_yaw
      chassis_done_client()
      lock.release()
      continue

    if initialized:
      new_log_weights = np.zeros(nfilters)
      for i in range(nfilters):
        states[:, i], covs[i] = predict(states[:, i], covs[i], vx, vy, ardu_yaw - prev_yaw, t - prev_t)
        states[:, i], covs[i], new_log_weights[i] = correct(states[:, i], covs[i], np.array(tofs), log_weights[i])
      
      yawsum += abs(ardu_yaw - prev_yaw)
      if t > 2 + cull_t:
        cull_t = t
        log_weights = new_log_weights
        log_weights = normalize_log_weights(log_weights)

        live_filters = np.logical_and(np.logical_and(log_weights > -10, states[0] > 0), states[1] > 0)
        live_filters = np.logical_and(live_filters, np.logical_and(states[0] < maxx, states[1] < maxy))
        if np.any(live_filters):
          states = states[:, live_filters]
          covs = covs[live_filters]
          log_weights = normalize_log_weights(log_weights[live_filters])
          nfilters = len(covs)

          if nfilters < 8:
            nnew = 2*nfilters
            new_states = np.zeros((3, nnew))
            new_covs = np.array([np.eye(3) for i in range(nnew)])
            new_log_weights = np.zeros(nnew)

            for i in range(nfilters):
              new_idx = 2*i
              new_states[:, new_idx] = states[:, i].copy()
              new_states[2, new_idx] += np.pi/8
              new_covs[new_idx] = covs[i].copy()
              new_log_weights[new_idx] = log_weights[i]

              new_idx += 1
              new_states[:, new_idx] = states[:, i].copy()
              new_states[2, new_idx] -= np.pi/8
              new_covs[new_idx] = covs[i].copy()
              new_log_weights[new_idx] = log_weights[i]

            states = np.concatenate((states, new_states), axis=1)
            covs = np.concatenate((covs, new_covs), axis=0)
            log_weights = np.concatenate((log_weights, new_log_weights), axis=0)
            log_weights = normalize_log_weights(log_weights)
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
    lock.release()

  rate.sleep()

