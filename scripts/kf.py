from angle_mod import *
import numpy as np

def normalize_log_weights(log_weights):
  weights = np.exp(log_weights - np.max(log_weights))
  weights /= np.sum(weights)
  return np.log(weights);

# Coordinates [x, y] of sensors in sensor 0's local frame, where sensor
# 0's local frame aligns with the world frame at yaw = 0
loc1 = [9.75*2.54, 10*2.54]
loc2 = [7.75*2.54, 12.5*2.54]
loc3 = [-2.5*2.54, 2.625*2.54]

def init_state_given_yaw(yaw, obs):
  state = np.zeros(5)
  cov = np.eye(5)
  cov[2, 2] = 0.04
  state[2] = yaw
  if yaw >= 7*np.pi/4 or yaw <= np.pi/4:
    # Facing forward
    c = np.cos(yaw)
    state[0] = obs[3]*c
    state[1] = obs[0]*c

  elif state[2] >= 3*np.pi/8 and state[2] <= 5*np.pi/8:
    # Facing left
    c = np.cos(yaw - np.pi/2)
    state[0] = obs[2]*c
    state[1] = obs[3]*c

  elif state[2] >= 7*np.pi/8 and state[2] <= 9*np.pi/8:
    # Facing backward
    c = np.cos(yaw - np.pi)
    state[0] = obs[1]*c
    state[1] = obs[2]*c

  elif state[2] >= 11*np.pi/8 and state[2] <= 13*np.pi/8:
    # Facing right
    c = np.cos(yaw - 3*np.pi/2)
    state[0] = obs[0]*c
    state[1] = obs[1]*c

  return state, cov

# state is [x, y, yaw, vx, vy]. (x, y) is the location of sensor 0.
# If we're facing long side of guiderail, theta = 0. (x, y) = (0, 0)
# is at the intersection of guiderails. +x is left, +y is backwards.
# theta is in radians, x and y are in cm, vx and vy are in meters per second.
# Front of the robot is sensor 0. Positive yaw velocity
# is about the vertical axis
def predict(state, cov, dyaw, dt):
  F_t = np.eye(5)
  F_t[0, 3] = dt
  F_t[1, 4] = dt
  state = np.matmul(F_t, state)
  state[2] += dyaw
  state[2] = angle_mod(state[2])

  Q = np.eye(5)
  Q[2, 2] = 0.001
  cov = np.matmul(np.matmul(F_t, cov), F_t.transpose()) + Q
  return state, cov

# Sensor 0 is on the same side as the ethernet cable. 1-3 go counterclockwise
def correct(state, cov, obs, log_weight):
  R = 0.25*np.eye(4)
  zhat = 200*np.ones(4)
  H_t = np.zeros((4, 5))

  if state[2] >= 7*np.pi/4 or state[2] <= np.pi/4:
    # Facing forward
    theta = state[2]
    c = np.cos(theta)
    ooc = 1/c
    zhat[3] = state[0]*ooc + loc3[0]
    zhat[0] = state[1]*ooc
    R[1, 1] = 1000
    R[2, 2] = 1000

    H_t[3, 0] = ooc
    H_t[0, 0] = ooc
    ddyaw = np.sin(theta)/(c*c)
    H_t[3, 2] = state[0]*ddyaw
    H_t[0, 2] = state[1]*ddyaw

  elif state[2] >= np.pi/4 and state[2] <= 3*np.pi/4:
    # Facing left
    theta = state[2] - np.pi/2
    c = np.cos(theta)
    ooc = 1/c
    zhat[2] = state[0]*ooc - loc2[1]
    zhat[3] = state[1]*ooc + loc3[0]

    R[0, 0] = 1000
    R[1, 1] = 1000

    H_t[2, 0] = ooc
    H_t[3, 1] = ooc
    ddyaw = np.sin(theta)/(c*c)
    H_t[2, 2] = state[0]*ddyaw
    H_t[3, 2] = state[1]*ddyaw

  elif state[2] >= 3*np.pi/4 and state[2] <= 5*np.pi/4:
    # Facing backward
    theta = state[2] - np.pi
    c = np.cos(theta)
    ooc = 1/c
    zhat[1] = state[0]*ooc - loc1[0]
    zhat[2] = state[1]*ooc - loc2[1]

    R[0, 0] = 1000
    R[3, 3] = 1000

    H_t[1, 0] = ooc
    H_t[2, 1] = ooc
    ddyaw = np.sin(theta)/(c*c)
    H_t[1, 2] = state[0]*ddyaw
    H_t[2, 2] = state[1]*ddyaw

  elif state[2] >= 5*np.pi/4 and state[2] <= 7*np.pi/4:
    # Facing right
    theta = state[2] - 3*np.pi/2
    c = np.cos(theta)
    ooc = 1/c
    zhat[0] = state[0]*ooc
    zhat[1] = state[1]*ooc - loc1[0]

    R[2, 2] = 1000
    R[3, 3] = 1000

    H_t[0, 0] = ooc
    H_t[1, 1] = ooc
    ddyaw = np.sin(theta)/(c*c)
    H_t[0, 2] = state[0]*ddyaw
    H_t[1, 2] = state[1]*ddyaw

  resid = obs - zhat
  inn_cov = np.matmul(np.matmul(H_t, cov), H_t.transpose()) + R

  # Check if any sensors are surprisingly off. If so, don't trust them
  for sensor in range(4):
    if resid[sensor]*resid[sensor]/inn_cov[sensor, sensor] > 9:
      inn_cov[sensor, sensor] = 1000

  Ktmp = np.matmul(cov, H_t.transpose())
  state = state + np.matmul(Ktmp, np.linalg.solve(inn_cov, resid))
  state[2] = angle_mod(state[2])
  cov = np.matmul(np.eye(5) - np.matmul(Ktmp, np.linalg.solve(inn_cov, H_t)), cov)

  dist = np.matmul(resid, np.linalg.solve(inn_cov, resid))
  log_weight -= 0.5*dist + 0.5*np.log(np.linalg.det(inn_cov))
  return state, cov, log_weight
