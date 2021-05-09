#include "angle_mod.h"
#include <math.h>

double fmodp(double x, double y) {
  double ret = fmod(x, y);
  if (ret < 0) {
    ret = y + ret;
  }
  return ret;
}

double angle_mod(double theta) {
  return fmodp(theta, 2*M_PI);
}

double angdiff(double th1, double th2) {
  th1 = angle_mod(th1);
  th2 = angle_mod(th2);

  if (th2 - th1 > M_PI) {
    th2 -= 2*M_PI;
  } else if (th2 - th1 < -M_PI) {
    th2 += 2*M_PI;
  }

  return th2 - th1;
}
