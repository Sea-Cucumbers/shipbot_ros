#include "minJerkInterpolator.h"
#include <iostream>

using namespace std;

MinJerkInterpolator::MinJerkInterpolator(const VectorXd &start, const VectorXd &end, double start_time, double end_time) : start(start), end(end), start_time(start_time), end_time(end_time), delta_t(end_time - start_time) {}

VectorXd MinJerkInterpolator::eval(double t) {
  if (t > end_time) {
    t = end_time;
  } else if (t < start_time) {
    t = start_time;
  }

  t = (t - start_time)/delta_t;
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t3*t;
  double t5 = t4*t;
  return start + (start - end)*(15*t4 - 6*t5 - 10*t3);
}

VectorXd MinJerkInterpolator::deriv1(double t) {
  if (t > end_time) {
    t = end_time;
  } else if (t < start_time) {
    t = start_time;
  }

  t = (t - start_time)/delta_t;
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t3*t;
  return (start - end)*(60*t3 - 30*t4 - 30*t2);
}

VectorXd MinJerkInterpolator::deriv2(double t) {
  if (t > end_time) {
    t = end_time;
  } else if (t < start_time) {
    t = start_time;
  }

  t = (t - start_time)/delta_t;
  double t2 = t*t;
  double t3 = t2*t;
  return (start - end)*(180*t2 - 120*t3 - 60*t);
}

double MinJerkInterpolator::get_start_time() {
  return start_time;
}

double MinJerkInterpolator::get_end_time() {
  return end_time;
}

bool MinJerkInterpolator::contains_time(double t) {
  return t >= start_time && t <= end_time;
}
