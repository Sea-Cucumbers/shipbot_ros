#!/usr/bin/env python2

import numpy as np
import rospy
from shipbot_ros.msg import ChassisFeedback
from shipbot_ros.msg import ChassisState
from shipbot_ros.msg import ChassisCommand
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from pf import *
import threading, Queue
import tf2_ros

robot_width = 0.1905
minx = robot_width
miny = robot_width
long_wall = 1.568
short_wall = 0.95
maxx = long_wall - robot_width
maxy = short_wall - robot_width

got_fbk = False 
got_cmd = False 
t = 0

lock = threading.Lock()

def dummy():
  pass

def update_weights(queue, particles, log_weights, tofs):
  queue.put(get_new_log_weights(particles, log_weights, tofs))

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

print('Waiting for mission control node')
rospy.wait_for_service('/localization_mux_node/loc_prep_done_pf')

chassis_sub = rospy.Subscriber('/shipbot/chassis_feedback', ChassisFeedback, chassis_callback)
command_sub = rospy.Subscriber('/shipbot/chassis_command', ChassisCommand, command_callback)
state_pub = rospy.Publisher('/shipbot/chassis_state_pf', ChassisState, queue_size=1)
state_msg = ChassisState()

collect_idx = 0
ncollect = 50
collected = False
collection = np.zeros((4, ncollect))
valid_sensors = np.zeros(4, dtype=np.bool)
initialized = False

nparticles = 200
particles = np.zeros((3, nparticles))
nyaws = 8
log_weights = np.log(np.ones(nyaws*50)/(nyaws*50))
prev_t = 0
prev_yaw = 0

queue = Queue.Queue()
pf_thread = threading.Thread(target=dummy)

# Only needed for rviz
'''
br = tf2_ros.TransformBroadcaster()
robot_tf = TransformStamped()

robot_tf.header.stamp = rospy.Time.now()
robot_tf.header.frame_id = 'world'
robot_tf.child_frame_id = 'shipbot'
robot_tf.transform.translation.x = 0
robot_tf.transform.translation.y = 0
robot_tf.transform.translation.z = 0
robot_tf.transform.rotation.x = 0
robot_tf.transform.rotation.y = 0
robot_tf.transform.rotation.z = 0
robot_tf.transform.rotation.w = 1
br.sendTransform(robot_tf)

particle_marker = Marker()
particle_marker.header.frame_id = 'world'
particle_marker.type = Marker.POINTS
particle_marker.action = Marker.ADD
particle_marker.pose.position.x = 0
particle_marker.pose.position.y = 0
particle_marker.pose.position.z = 0
particle_marker.pose.orientation.w = 1
particle_marker.pose.orientation.x = 0
particle_marker.pose.orientation.y = 0
particle_marker.pose.orientation.z = 0
particle_marker.scale.x = 0.01
particle_marker.scale.y = 0.01
particle_marker.scale.z = 0.01
particle_marker.color.r = 1
particle_marker.color.g = 0
particle_marker.color.b = 0
particle_marker.color.a = 1
particle_marker_pub = rospy.Publisher('/shipbot/particles', Marker, queue_size=1)

guiderail_marker = Marker()
guiderail_marker.header.frame_id = 'world'
guiderail_marker.type = Marker.LINE_LIST
guiderail_marker.action = Marker.ADD
guiderail_marker.pose.position.x = 0
guiderail_marker.pose.position.y = 0
guiderail_marker.pose.position.z = 0
guiderail_marker.pose.orientation.w = 1
guiderail_marker.pose.orientation.x = 0
guiderail_marker.pose.orientation.y = 0
guiderail_marker.pose.orientation.z = 0
guiderail_marker.scale.x = 0.01
guiderail_marker.color.r = 0
guiderail_marker.color.g = 1
guiderail_marker.color.b = 0
guiderail_marker.color.a = 1
guiderail_pub = rospy.Publisher('/shipbot/guiderail', Marker, queue_size=1)
guiderail_marker.points = [Point(1.524, 0, 0), Point(0, 0, 0), Point(0, 0, 0), Point(0, 0.9144, 0)]
'''

loc_prep_done_client = rospy.ServiceProxy('/localization_mux_node/loc_prep_done_pf', Empty)

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
      states = np.zeros((3, nyaws))
      for i in range(nyaws):
        states[:, i] = init_state_given_yaw(i*np.pi/4 + 0.01, np.array(tofs), valid_sensors)

      particles = np.tile(states, (1, 50)) + np.random.normal(scale=0.1, size=(3, nyaws*50))
      particles[0, particles[0] < minx] = minx
      particles[0, particles[0] > maxx] = maxx
      particles[1, particles[1] < miny] = miny
      particles[1, particles[1] > maxy] = maxy

      initialized = True
      prev_t = t
      prev_yaw = ardu_yaw
      loc_prep_done_client()
      lock.release()
      continue

    if initialized:
      particles = motion_model(particles, vx, vy, ardu_yaw - prev_yaw, t - prev_t)

      if not pf_thread.is_alive():
        if not queue.empty():
          log_weights = queue.get()
          Neff = 1.0/np.exp(2*log_weights).sum()
          if Neff < nparticles/2.0:
            particles, log_weights = resample(particles, log_weights, nparticles)
        pf_thread = threading.Thread(target=update_weights, args=(queue, particles.copy(), log_weights, np.array(tofs)))
        pf_thread.start()

      state = np.zeros(3)
      weights = np.exp(log_weights)
      state[:2] = np.matmul(particles[:2], weights)
      directions = np.zeros((2, particles.shape[1]))
      directions[0] = np.cos(particles[2])
      directions[1] = np.sin(particles[2])
      av_dir = np.matmul(directions, weights)
      state[2] = np.arctan2(av_dir[1], av_dir[0])

      state_msg.x = state[0]
      state_msg.y = state[1]
      state_msg.yaw = state[2]
      state_msg.header.stamp = rospy.Time().now()
      state_pub.publish(state_msg)

      state_to_print = np.copy(state)
      state_to_print[2] *= 180/np.pi
      state_to_print[:2] /= 0.0254
      #print(np.trunc(state_to_print))
      prev_t = t
      prev_yaw = ardu_yaw

      # Only needed for rviz
      '''
      particle_marker.points = [Point(particles[0, p], particles[1, p], 0) for p in range(particles.shape[1])]
      particle_marker.header.stamp = rospy.Time().now()
      particle_marker_pub.publish(particle_marker)
      guiderail_pub.publish(guiderail_marker)

      robot_tf.transform.translation.x = state[0]
      robot_tf.transform.translation.y = state[1]
      robot_tf.transform.rotation.z = np.sin(state[2]/2)
      robot_tf.transform.rotation.w = np.cos(state[2]/2)
      robot_tf.header.stamp = rospy.Time.now()
      br.sendTransform(robot_tf)
      '''

    lock.release()

  rate.sleep()

