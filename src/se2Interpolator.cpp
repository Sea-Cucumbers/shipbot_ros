#include "se2Interpolator.h"
#include <iostream>
#include "angle_mod.h"

using namespace std;

SE2Interpolator::SE2Interpolator(const Vector3d &start, const Vector3d &end, double start_time, double end_time) : start(start), end(end), start_time(start_time), end_time(end_time), delta_t(end_time - start_time) {
  vel = error(start, end)/delta_t;
}

Vector3d SE2Interpolator::eval(double t) {
  if (t > end_time) {
    t = end_time;
  } else if (t < start_time) {
    t = start_time;
  }

  return start + vel*(t - start_time);
}

Vector3d SE2Interpolator::deriv1(double t) {
  if (t > end_time || t < start_time) {
    return Vector3d::Zero();
  } 

  return vel;
}

Vector3d SE2Interpolator::error(const Vector3d &state1, const Vector3d &state2) {
  Vector3d ret;
  ret.head<2>() = state2.head<2>() - state1.head<2>();
  ret(2) = angdiff(state1(2), state2(2));
  return ret;
}

double SE2Interpolator::get_start_time() {
  return start_time;
}

double SE2Interpolator::get_end_time() {
  return end_time;
}

const Vector3d &SE2Interpolator::get_end() {
  return end;
}

bool SE2Interpolator::contains_time(double t) {
  return t >= start_time && t <= end_time;
}
