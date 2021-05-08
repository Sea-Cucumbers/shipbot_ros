from angle_mod import *
import numpy as np

robot_width = 0.1905

minx = robot_width
miny = robot_width

maxx = 1.568 - robot_width
maxy = 0.95 - robot_width

def clamp_state(state):
  ret = state.copy()
  if ret[0] < minx:
    ret[0] = minx
  if ret[0] > maxx:
    ret[0] = maxx
  if ret[1] < miny:
    ret[1] = miny
  if ret[1] > maxy:
    ret[1] = maxy

  return ret

# STATE DEFINITION
# state is [x, y, theta].
# (x, y) is the location of the center of the chassis.
# If the camera is facing away from the short wall of the guiderail, theta = 0. 
# (x, y) = (0, 0) is at the intersection of guiderails. +x is left (along the long wall),
# +y is backwards (along the short wall).
# theta is in radians, x and y are in meters
# Front of the robot is sensor 0, and sensor numbers increase counterclockwise.
# Positive theta velocity is about the vertical axis

# ray_intersect_segment: get distance along a ray 
# where it intersects a line segment, or -1
# if it never intersects
# ARGUMENTS
# r0: initial point of the ray
# dr: unit vector indicating ray direction
# s0: initial point of the line segment
# s1: final point of the line segment
# RETURN: distance along the ray 
# where it intersects the line segment, or -1
# if it never intersects
def ray_intersect_segment(r0, dr, s0, s1):
  ds = s1 - s0
  LHS = np.zeros((2, 2))
  LHS[:, 0] = dr
  LHS[:, 1] = -ds
  det = np.linalg.det(LHS)

  # Ray is parallel to the line segment
  if det == 0:
    return -1
  
  soln = np.linalg.solve(LHS, s0 - r0)

  # soln[0] is the distance along the ray, so it
  # can't be negative. soln[1] is the parameter
  # for the line segment, which ranges from
  # 0 to 1
  if soln[0] < 0 or soln[1] < 0 or soln[1] > 1:
    return -1

  return soln[0]

# normalize_log_weights: normalizes log_weights
# using the log-sum-exp trick
# ARGUMENTS
# log_weights: natural log of a vector of numbers
# RETURN: a new vector such that when we sum the exponents
# of the weights, we get 1
def normalize_log_weights(log_weights):
  weights = np.exp(log_weights - np.max(log_weights))
  weights /= np.sum(weights)
  return np.log(weights)

corner = np.array([0, 0])
long_wall_end = np.array([1.524, 0])
short_wall_end = np.array([0, 0.9144])

# Displacements [x, y] of sensors in chassis' local frame, where
# chassis' local frame aligns with the world frame at theta = 0
t0 = np.array([0.1651, -0.09525])
t1 = np.array([-0.1016, 0.15875])
t2 = np.array([-0.1651, 0.1143])
t3 = np.array([0.1016, -0.15875])

# Ray vectors for each sensor in the chassis' local frame
dr0 = np.array([1, 0])
dr1 = np.array([0, 1])
dr2 = np.array([-1, 0])
dr3 = np.array([0, -1])

ts = [t0, t1, t2, t3]
drs = [dr0, dr1, dr2, dr3]

sensor_info = zip(ts, drs)

# init_state_given_yaw: determines what the state should be if we have
# a certain yaw an the sensors are reading the values given by obs
# ARGUMENTS
# yaw: robot's yaw
# obs: sensor readings, going from sensor 0 to sensor 3
# RETURN: robot state. theta is modded to be in the range [0, 2*pi]
def init_state_given_yaw(yaw, obs, valid_sensors):
  yaw = angle_mod(yaw)
  state = np.zeros(3)
  cov = np.eye(3)
  cov[2, 2] = 0.09
  state[2] = yaw

  min_resid_norm = 1000
  best_state = np.array([maxx/2, maxy/2, 0])
  for xidx in range(50):
    for yidx in range(25):
      state[0] = xidx*maxx/50
      state[1] = yidx*maxy/50
      zhat = sensor_model(state)
      resid_norm = np.linalg.norm(obs[valid_sensors] - zhat[valid_sensors])
      if resid_norm < min_resid_norm:
        best_state = state.copy()
        min_resid_norm = resid_norm

  return best_state, cov

# predict: propagates state using motion model. New position
# is the previous position plus the velocity control input.
# New theta is the previous theta plus the integral of the gyro
# values since the previous time step
# ARGUMENTS
# state: current state estimate
# cov: covariance of current state estimate
# vx: x component of commanded robot body velocity
# vy: y component of commanded robot body velocity
# dyaw: when the Arduino starts up, it sets its local yaw to zero,
# and starts integrating the gyro. Since we last
# got a message from the Arduino, how much has its local yaw estimate
# changed? This is how much we predict the robot's yaw changed
# dt: time since we've last predicted
# RETURN: predicted state and its covariance
def predict(state, cov, vx, vy, dyaw, dt):
  c = np.cos(state[2])
  s = np.sin(state[2])

  # Transform commanded body velocity into world frame and propagate
  # positino
  state[0] += (vx*c - vy*s)*dt
  state[1] += (vx*s + vy*c)*dt

  state[2] += dyaw
  state[2] = angle_mod(state[2])

  # 1 cm standard deviation in new position estimate, 0.1
  # radians standard deviation in new theta estimate
  Q = 0.0001*np.eye(3)
  Q[2, 2] = 0.01

  # Derivative of each state element wrt its previous value is 1,
  # since we're simply adding something to the previous value
  F_t = np.eye(3)

  # Derivative of new position estimate wrt theta
  F_t[0, 2] = (-vx*s - vy*c)*dt
  F_t[1, 2] = (vx*c - vy*s)*dt
  cov = np.matmul(np.matmul(F_t, cov), F_t.transpose()) + Q
  return state, cov

# sensor_model: calculates expected sensor readings given the state
# ARGUMENTS
# state: robot state
# RETURN: expected sensor readings in meters. If a sensor is expected
# not to be facing a wall, the expected reading is returned as 2
def sensor_model(state):
  zhat = 2*np.ones(4)

  c = np.cos(state[2])
  s = np.sin(state[2])
  rot = np.array([[c, -s], [s, c]])
  for sensor, (t, dr) in enumerate(sensor_info):
    # Recall that t is the local displacement of the sensor in the chassis frame
    # Here, we calculate the sensor's position in the world frame
    sensor_pos = state[:2] + np.matmul(rot, t)

    # Recall that dr is the unit direction vector of the sensor in the chassis frame
    # Here, we calculate the sensor's direction in the world frame
    d = np.matmul(rot, dr)

    # Check distance to long wall and short wall
    long_dist = ray_intersect_segment(sensor_pos, d, corner, long_wall_end)
    short_dist = ray_intersect_segment(sensor_pos, d, corner, short_wall_end)

    if long_dist < 0 and short_dist < 0:
      pass
    elif long_dist < 0:
      # Sensor ray doesn't intersect long wall, so we use the distance to the short wall
      zhat[sensor] = short_dist
    elif short_dist < 0:
      # Sensor ray doesn't intersect short wall, so we use the distance to the long wall
      zhat[sensor] = long_dist

  return zhat

# dh: calculate sensor model Jacobian (derivative of expected sensor reading wrt robot state)
# ARGUMENTS
# state: robot state
# RETURN: sensor model Jacobian
def dh(state):
  H_t = np.zeros((4, 3))

  zhat = sensor_model(state)

  # Numerical Jacobian computation
  for i in range(3):
    cpy = state.copy()
    cpy[i] += 0.0001   
    zhat_plus = sensor_model(cpy)
    H_t[:, i] = (zhat_plus - zhat)/0.0001

  return H_t

# correct: corrects the state using sensor measurements, and updates its weight
# using measurement likelihood (assuming it's being used in a Gaussian sum
# ARGUMENTS
# state: current state estimate
# cov: current state estimate's covariance
# obs: sensor readings in METERS
# log_weight: current weight of this filter, assuming it's being used in a
# Gaussian sum
# RETURN: corrected state and covariance
def correct(state, cov, obs, log_weight):
  R = 0.01*np.eye(4)
  zhat = sensor_model(state)
  for i in range(4):
    if zhat[i] > 1.25:
      # Sensor isn't looking at a wall, so don't trust it
      R[i, i] = 16
    if obs[i] > 1.25:
      R[i, i] = 16
      obs[i] = 2
  sort_idx = np.argsort(obs)
  R[sort_idx[2], sort_idx[2]] = 16
  R[sort_idx[3], sort_idx[3]] = 16
  obs[sort_idx[2]] = 2
  obs[sort_idx[3]] = 2
      
  H_t = dh(state)

  resid = obs - zhat
  inn_cov = np.matmul(np.matmul(H_t, cov), H_t.transpose()) + R

  # Check if any sensors are surprisingly off. If so, don't trust them
  for sensor in range(4):
    if resid[sensor]*resid[sensor]/inn_cov[sensor, sensor] > 9:
      inn_cov[sensor, sensor] -= R[sensor, sensor]
      R[sensor, sensor] = 16
      inn_cov[sensor, sensor] += R[sensor, sensor]
      obs[sensor] = 2

  K = np.linalg.solve(inn_cov, np.matmul(H_t, cov)).transpose()
  state = state + np.matmul(K, resid)
  state[2] = angle_mod(state[2])
  cov = np.matmul(np.eye(3) - np.matmul(K, H_t), cov)

  dist = np.matmul(resid, np.linalg.solve(inn_cov, resid))
  log_weight -= 0.5*dist + 0.5*np.log(np.linalg.det(inn_cov))

  return state, cov, log_weight
