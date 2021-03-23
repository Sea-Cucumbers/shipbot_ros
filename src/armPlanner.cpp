#include "armPlanner.h"
#include <iostream>

using namespace std;

ArmPlanner::ArmPlanner() : plan_exists(false) {}

void ArmPlanner::plan(const VectorXd &start, const VectorXd &end, double start_time, double end_time) {
  delta_t = end_time - start_time;
  this->start = start;
  this->end = end;
  this->start_time = start_time;
  this->end_time = end_time;
  plan_exists = true;
}

VectorXd ArmPlanner::eval(double t) {
  if (!plan_exists) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }

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

VectorXd ArmPlanner::deriv1(double t) {
  if (!plan_exists) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }

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

VectorXd ArmPlanner::deriv2(double t) {
  if (!plan_exists) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  
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
