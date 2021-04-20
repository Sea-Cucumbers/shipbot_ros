import math

def fmodp(x, y):
  ret = math.fmod(x, y)
  if ret < 0:
    ret = y + ret
  return ret

def angle_mod(theta):
  return fmodp(theta, 2*math.pi)
