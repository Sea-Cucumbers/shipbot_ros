from angle_mod import *
import numpy as np

robot_width = 0.1905
minx = robot_width
miny = robot_width
long_wall = 1.568
short_wall = 0.95
maxx = long_wall - robot_width
maxy = short_wall - robot_width

corner = np.array([0, 0])
long_wall_end = np.array([long_wall, 0])
short_wall_end = np.array([0, short_wall])

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
  return np.log(weights + 0.000001)

# init_state_given_yaw: determines what the state should be if we have
# a certain yaw an the sensors are reading the values given by obs
# ARGUMENTS
# yaw: robot's yaw
# obs: sensor readings, going from sensor 0 to sensor 3
# RETURN: robot state. theta is modded to be in the range [0, 2*pi]
def init_state_given_yaw(yaw, obs, valid_sensors):
  yaw = angle_mod(yaw)
  state = np.zeros(3)
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

  return best_state

def motion_model(particles, vx, vy, dyaw, dt):
  c = np.cos(particles[2])
  s = np.sin(particles[2])

  # Transform commanded body velocity into world frame and propagate
  # position
  particles[0] += (vx*c - vy*s)*dt
  particles[1] += (vx*s + vy*c)*dt

  particles[2] += dyaw
  particles[2] = np.mod(particles[2], 2*np.pi)

  particles += np.random.normal(scale=0.01, size=particles.shape)

  return particles

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

def get_new_log_weights(particles, log_weights, obs):
  new_log_weights = np.zeros(log_weights.shape)
  sort_idx = np.argsort(obs)
  good_idx = sort_idx[:2]
  for p in range(particles.shape[1]):
    zhat = sensor_model(particles[:, p])
    resid = zhat - obs
    new_log_weights[p] = log_weights[p] - 0.5*np.dot(resid[good_idx], resid[good_idx])*100

  new_log_weights = normalize_log_weights(new_log_weights)
  best_idx = np.argmax(new_log_weights)
  return new_log_weights

def resample(particles, log_weights, nparticles):
  live_particles = log_weights > -10

  particles = particles[:, live_particles]
  log_weights = normalize_log_weights(log_weights[live_particles])
  
  new_particles = np.zeros((3, nparticles))
  new_log_weights = np.zeros(nparticles)
  stop_idx = np.cumsum(np.exp(log_weights))*nparticles
  i = 0
  for p in range(nparticles):
    while i < len(stop_idx) and p > stop_idx[i]:
      i += 1

    new_particles[:, p] = particles[:, i]
    new_log_weights[p] = 1.0/nparticles

  log_weights = np.log(new_log_weights)
  return new_particles, log_weights
