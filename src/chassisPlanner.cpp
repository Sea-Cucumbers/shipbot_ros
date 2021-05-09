#include "chassisPlanner.h"
#include <iostream>
#include "geometry.h"

using namespace std;

ChassisPlanner::ChassisPlanner(double linear_speed,
                               double angular_speed) : linear_speed(linear_speed),
                                                       angular_speed(angular_speed) {}

Vector3d ChassisPlanner::eval(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->eval(t);
}

Vector3d ChassisPlanner::deriv1(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->deriv1(t);
}

void ChassisPlanner::retrieve_segment(double t) {
  if (t > segments.back().get_end_time()) {
    current_segment = prev(segments.end());
  } else if (t < segments.front().get_start_time()) {
    current_segment = segments.begin();
  } else {
    while (!current_segment->contains_time(t)) {
      ++current_segment;
      if (current_segment == segments.end()) {
        current_segment = segments.begin();
      }
    }
  }
}
    
bool ChassisPlanner::planned() {
  return segments.size() > 0;
}

double ChassisPlanner::get_end_time() {
  return segments.back().get_end_time();
}

void ChassisPlanner::add_waypoint(const Vector3d &waypoint) {
  if (segments.size() == 0) {
    cout << "Cannot call add_waypoint on empty plan" << endl;
    return;
  }

  Vector3d start = segments.back().get_end();

  Vector3d err = SE2Interpolator::error(start, waypoint);
  double start_time = segments.back().get_end_time(); 
  double end_time = start_time + max(err.head<2>().norm()/linear_speed, abs(err(2))/angular_speed);

  segments.push_back(SE2Interpolator(start, waypoint, start_time, end_time));
}

void ChassisPlanner::plan(double start_time, const Vector3d &start, const Vector3d &goal) {
  segments.clear();

  Vector3d err = SE2Interpolator::error(start, goal);
  double end_time = start_time + max(err.head<2>().norm()/linear_speed, abs(err(2))/angular_speed);

  // Construct and interpolator and check if we intersect any obstacles. If we do, translate to
  // the center of the testbed first
  SE2Interpolator interpolator(start, goal, start_time, end_time);

  bool collision = false;
  for (double t = start_time; t < end_time; t += 0.1) {
    Vector3d state = interpolator.eval(t);

    // Long wall
    if (rectIntersectsLineSegment(robot_width, robot_width, state(2), state(0), state(1),
                                  0, long_wall, 0, 0)) {
      collision = true;
      break;
    }

    // Short wall
    if (rectIntersectsLineSegment(robot_width, robot_width, state(2), state(0), state(1),
                                  0, 0, 0, short_wall)) {
      collision = true;
      break;
    }
  }

  if (collision) {
    Vector3d center(long_wall/2, short_wall/2, start(2));
    err = SE2Interpolator::error(start, center);
    end_time = start_time + max(err.head<2>().norm()/linear_speed, abs(err(2))/angular_speed);
    segments.push_back(SE2Interpolator(start, center, start_time, end_time));
    add_waypoint(goal);
  } else {
    segments.push_back(interpolator);
  }

  current_segment = segments.begin();
}

Vector3d ChassisPlanner::error(const Vector3d &state1, const Vector3d &state2) {
  return SE2Interpolator::error(state1, state2);
}
