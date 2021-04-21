from angle_mod import *
import numpy as np

# Get distance along ray where intersection occurs, or -1
# if it never intersects
def ray_intersect_segment(r0, dr, s0, s1):
  ds = s1 - s0
  LHS = np.zeros((2, 2))
  LHS[:, 0] = dr
  LHS[:, 1] = -ds
  det = np.linalg.det(LHS)
  if det == 0:
    return -1
  
  soln = np.linalg.solve(LHS, s0 - r0)
  if soln[0] < 0 or soln[1] < 0 or soln[1] > 1:
    return -1

  return soln[0]

def normalize_log_weights(log_weights):
  weights = np.exp(log_weights - np.max(log_weights))
  weights /= np.sum(weights)
  return np.log(weights)

corner = np.array([0, 0])
long_wall_end = np.array([1.524, 0])
short_wall_end = np.array([0, 0.9144])

# Coordinates [x, y] of sensors in chassis' local frame, where
# chassis' local frame aligns with the world frame at yaw = 0
t0 = np.array([-0.1, -0.155])
t1 = np.array([0.15, 0.1])
t2 = np.array([0.105, 0.16])
t3 = np.array([-0.155, -0.105])

dr0 = np.array([0, -1])
dr1 = np.array([1, 0])
dr2 = np.array([0, 1])
dr3 = np.array([-1, 0])

ts = [t0, t1, t2, t3]
drs = [dr0, dr1, dr2, dr3]

sensor_info = zip(ts, drs)

# What would the state be if we had a certain yaw and the sensors were reading
# the values given by obs?
def init_state_given_yaw(yaw, obs):
  yaw = angle_mod(yaw)
  state = np.zeros(5)
  cov = np.eye(5)
  cov[2, 2] = 0.04
  cov[3, 3] = 0.0001
  cov[4, 4] = 0.0001
  state[2] = yaw
  if yaw <= np.pi/4 or yaw > 7*np.pi/4:
    # Facing forward
    c = np.cos(yaw)
    s = np.sin(yaw)
    
    # y position of sensor facing long wall
    y0 = obs[0]*c

    # Could be sensing one of two walls with right sensor. Assume
    # short wall
    x3 = obs[3]*c

    c = np.cos(yaw)
    s = np.sin(yaw)

    state[0] = x3 - (t3[0]*c - t3[1]*s)
    state[1] = y0 - (t0[0]*s + t0[1]*c)

  elif state[2] > np.pi/4 and state[2] <= 3*np.pi/4:
    # Facing left
    c = np.cos(yaw - np.pi/2)
    
    # y position of sensor facing long wall
    y3 = obs[3]*c

    # Could be sensing one of two walls with right sensor. Assume
    # short wall
    x2 = obs[2]*c

    c = np.cos(yaw)
    s = np.sin(yaw)

    state[0] = x2 - (t2[0]*c - t2[1]*s)
    state[1] = y3 - (t3[0]*s + t3[1]*c)

  elif state[2] > 3*np.pi/4 and state[2] <= 5*np.pi/4:
    # Facing backward
    c = np.cos(yaw - np.pi)

    # y position of sensor facing long wall
    y2 = obs[2]*c

    # Could be sensing one of two walls with right sensor. Assume
    # short wall
    x1 = obs[1]*c

    c = np.cos(yaw)
    s = np.sin(yaw)

    state[0] = x1 - (t1[0]*c - t1[1]*s)
    state[1] = y2 - (t2[0]*s + t2[1]*c)

  elif state[2] >= 11*np.pi/8 and state[2] <= 13*np.pi/8:
    # Facing right
    c = np.cos(yaw - 3*np.pi/2)

    # y position of sensor facing long wall
    y1 = obs[1]*c

    # Could be sensing one of two walls with right sensor. Assume
    # short wall
    x0 = obs[0]*c

    c = np.cos(yaw)
    s = np.sin(yaw)

    state[0] = x0 - (t0[0]*c - t0[1]*s)
    state[1] = y1 - (t1[0]*s + t1[1]*c)

  return state, cov

# state is [x, y, yaw, vx, vy]. (x, y) is the location of sensor 0.
# If we're facing long side of guiderail, theta = 0. (x, y) = (0, 0)
# is at the intersection of guiderails. +x is left, +y is backwards.
# theta is in radians, x and y are in cm, vx and vy are in meters per second.
# Front of the robot is sensor 0. Positive yaw velocity
# is about the vertical axis
def predict(state, cov, dyaw, vx, vy, dt):
  F_t = np.eye(5)
  F_t[0, 3] = dt
  F_t[1, 4] = dt
  state = np.matmul(F_t, state)
  state[3] = vx*np.cos(state[2]) - vy*np.sin(state[2])
  state[4] = vx*np.sin(state[2]) + vy*np.cos(state[2])
  F_t[3, 2] = -vx*np.sin(state[2]) - vy*np.cos(state[2])
  F_t[4, 2] = vx*np.cos(state[2]) - vy*np.sin(state[2])
  state[2] += dyaw
  state[2] = angle_mod(state[2])

  Q = 0.01*np.eye(5)
  Q[2, 2] = 0.001
  cov = np.matmul(np.matmul(F_t, cov), F_t.transpose()) + Q
  return state, cov

def sensor_model(state):
  R = 0.0016*np.eye(4)
  zhat = 0.2*np.ones(4)

  c = np.cos(state[2])
  s = np.sin(state[2])
  rot = np.array([[c, -s], [s, c]])
  for sensor, (t, dr) in enumerate(sensor_info):
    sensor_pos = state[:2] + np.matmul(rot, t)
    d = np.matmul(rot, dr)
    long_dist = ray_intersect_segment(sensor_pos, d, corner, long_wall_end)
    short_dist = ray_intersect_segment(sensor_pos, d, corner, short_wall_end)

    if long_dist < 0 and short_dist < 0:
      R[sensor, sensor] = 4
    elif long_dist < 0:
      zhat[sensor] = short_dist
    elif short_dist < 0:
      zhat[sensor] = long_dist

  return zhat, R

def dh(state):
  cpy = state.copy()
  H_t = np.zeros((4, 5))
  for i in range(len(state)):
    cpy[i] += 0.001   
    zhat_plus, R = sensor_model(cpy)
    cpy[i] -= 0.002 
    zhat_minus, R = sensor_model(cpy)

    H_t[:, i] = (zhat_plus - zhat_minus)/0.002

  return H_t

# Sensor 0 is on the same side as the ethernet cable. 1-3 go counterclockwise
def correct(state, cov, obs, log_weight):
  zhat, R = sensor_model(state)
  H_t = dh(state)

  resid = obs - zhat
  inn_cov = np.matmul(np.matmul(H_t, cov), H_t.transpose()) + R

  # Check if any sensors are surprisingly off. If so, don't trust them
  for sensor in range(4):
    if resid[sensor]*resid[sensor]/inn_cov[sensor, sensor] > 9:
      inn_cov[sensor, sensor] = 4

  Ktmp = np.matmul(cov, H_t.transpose())
  state = state + np.matmul(Ktmp, np.linalg.solve(inn_cov, resid))
  state[2] = angle_mod(state[2])
  cov = np.matmul(np.eye(5) - np.matmul(Ktmp, np.linalg.solve(inn_cov, H_t)), cov)

  dist = np.matmul(resid, np.linalg.solve(inn_cov, resid))
  log_weight -= 0.5*dist + 0.5*np.log(np.linalg.det(inn_cov))
  return state, cov, log_weight
