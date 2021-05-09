#include "armPlanner.h"
#include <iostream>

using namespace std;

ArmPlanner::ArmPlanner(double seconds_per_meter,
                       double seconds_per_degree,
                       const VectorXd &reset_config,
                       double horizontal_pause_back,
                       double vertical_pause_back,
                       double vertical_pause_above,
                       double grip_wait) : seconds_per_meter(seconds_per_meter),
                                           seconds_per_degree(seconds_per_degree),
                                           reset_config(reset_config),
                                           horizontal_pause_back(horizontal_pause_back),
                                           vertical_pause_back(vertical_pause_back),
                                           vertical_pause_above(vertical_pause_above),
                                           grip_wait(grip_wait) {}

void ArmPlanner::sample_points(vector<geometry_msgs::Point> &points) {
  points.clear();
  if (segments.size() == 0) {
    cout << "Tried sampling trajectory but no trajectory was planned!" << endl;
    return;
  }
  bool dummy;
  for (double t = segments[0].second.get_start_time();
       t < segments.back().second.get_end_time();
       t += 0.1) {
    geometry_msgs::Point point;
    VectorXd tppt = eval(t, dummy);
    point.x = tppt(0);
    point.y = tppt(1);
    point.z = tppt(2);
    points.push_back(point);
  }
}

void ArmPlanner::reset_arm(const VectorXd &start,
                           double start_time) {
  start_plan(start_time, false, start, reset_config);
  current_segment = segments.begin();
}

void ArmPlanner::stop_arm(const VectorXd &start,
                          double start_time) {
  start_plan(start_time, false, start, start);
  current_segment = segments.begin();
}

void ArmPlanner::spin_rotary(const VectorXd &start,
                             const Vector3d &position,
                             bool vertical_spin_axis,
                             double degrees,
                             double start_time) {
  // If vertical axis, pause 10 cm above. If horizontal axis, pause 10 cm back
  VectorXd waypoint = VectorXd::Zero(5);
  waypoint.head<3>() = position;

  if (vertical_spin_axis) {
    waypoint(2) += vertical_pause_above;
    waypoint(1) += vertical_pause_back;
    waypoint(3) = -M_PI/2;
  } else {
    waypoint(1) -= horizontal_pause_back;
  }

  start_plan(start_time, false, start, waypoint);
                                         
  if (vertical_spin_axis) {
    // If vertical axis, move forward, then down
    waypoint(1) += vertical_pause_back;
    add_waypoint(waypoint);
    waypoint(2) -= vertical_pause_above;
    add_waypoint(waypoint);
  } else {
    // If horizontal axis, move forward
    waypoint(1) += horizontal_pause_back;
    add_waypoint(waypoint);
  }

  // Wait for the gripper to grip
  add_grip_phase(true);

  // Spin the valve
  waypoint(4) -= degrees*M_PI/180;
  add_waypoint(waypoint);

  // Wait for the gripper to ungrip
  add_grip_phase(true);

  if (vertical_spin_axis) {
    // If vertical axis, move up then back
    waypoint(2) += vertical_pause_above;
    add_waypoint(waypoint);
    waypoint(1) -= vertical_pause_back;
    add_waypoint(waypoint);
  } else {
    // If horizontal axis, move back
    waypoint(1) -= horizontal_pause_back;
  }

  current_segment = segments.begin();
}

void ArmPlanner::spin_shuttlecock(const VectorXd &start,
                                  const Vector3d &position, 
                                  bool vertical_spin_axis,
                                  bool do_open,
                                  double start_time) {
  VectorXd waypoint = VectorXd::Zero(5);
  waypoint.head<3>() = position;
  if (vertical_spin_axis) {
    waypoint(3) = -M_PI/2;
  }

  if (do_open) {
    if (vertical_spin_axis) {
      // The handle is pointing toward us. Stop above it and backward
      waypoint(1) -= vertical_pause_back;
      waypoint(2) += vertical_pause_above;
      
      start_plan(start_time, false, start, waypoint);

      // Move forward
      waypoint(1) += vertical_pause_back;
      add_waypoint(waypoint);

      // Harden end-effector
      add_grip_phase(true);

      // Move down
      waypoint(2) -= vertical_pause_above;
      add_waypoint(waypoint);

      // Move forward and right
      waypoint(0) += shuttlecock_length;
      waypoint(1) += shuttlecock_length;
      add_waypoint(waypoint);

      // Move up
      waypoint(2) += vertical_pause_above;
      add_waypoint(waypoint);

      // Unharden end-effector
      add_grip_phase(false);
    } else {
      // The handle is pointing right. Pause backward from it
      waypoint(1) -= horizontal_pause_back;
      
      start_plan(start_time, false, start, waypoint);

      // Harden end-effector
      add_grip_phase(true);

      // Move forward
      waypoint(1) += horizontal_pause_back;
      add_waypoint(waypoint);

      // Move up and left
      waypoint(0) -= shuttlecock_length;
      waypoint(2) += shuttlecock_length;
      add_waypoint(waypoint);

      // Move back
      waypoint(1) -= horizontal_pause_back;
      add_waypoint(waypoint);

      // Unharden end-effector
      add_grip_phase(false);
    }
  } else {
    if (vertical_spin_axis) {
      // The handle is pointing right. Stop above it and backward
      waypoint(1) -= vertical_pause_back;
      waypoint(2) += vertical_pause_above;
      
      start_plan(start_time, false, start, waypoint);

      // Move forward
      waypoint(1) += vertical_pause_back;
      add_waypoint(waypoint);

      // Harden end-effector
      add_grip_phase(true);

      // Move down
      waypoint(2) -= vertical_pause_above;
      add_waypoint(waypoint);

      // Move back and left
      waypoint(0) -= shuttlecock_length;
      waypoint(1) -= shuttlecock_length;
      add_waypoint(waypoint);

      // Move up
      waypoint(2) += vertical_pause_above;
      add_waypoint(waypoint);

      // Unharden end-effector
      add_grip_phase(false);
    } else {
      // The handle is pointing up. Pause backward from it
      waypoint(1) -= horizontal_pause_back;
      
      start_plan(start_time, false, start, waypoint);

      // Harden end-effector
      add_grip_phase(true);

      // Move forward
      waypoint(1) += horizontal_pause_back;
      add_waypoint(waypoint);

      // Move down and right
      waypoint(0) += shuttlecock_length;
      waypoint(2) -= shuttlecock_length;
      add_waypoint(waypoint);

      // Move back
      waypoint(1) -= horizontal_pause_back;
      add_waypoint(waypoint);

      // Unharden end-effector
      add_grip_phase(true);
    }
  }

  current_segment = segments.begin();
}

void ArmPlanner::switch_breaker(const VectorXd &start,
                                const Vector3d &position,
                                bool push_up,
                                double start_time) {
  // Pause back and up/down depending on which way we're pushing
  VectorXd waypoint = VectorXd::Zero(5);
  waypoint.head<3>() = position;
  waypoint(1) -= horizontal_pause_back;
  waypoint(2) += push_up ? -horizontal_pause_back : horizontal_pause_back;
  waypoint(3) = push_up ? M_PI/4 : -M_PI/4;

  start_plan(start_time, false, start, waypoint);
                                         
  // Move forward and upward/downward depending on which way we're pushing 
  waypoint(1) += horizontal_pause_back;
  waypoint(2) = waypoint(2) + (push_up ? horizontal_pause_back : -horizontal_pause_back);
  add_waypoint(waypoint);

  // Grip
  add_grip_phase(true);

  // Push the switch
  waypoint(2) = waypoint(2) + (push_up ? horizontal_pause_back : -horizontal_pause_back);
  add_waypoint(waypoint);

  // Ungrip
  add_grip_phase(false);

  // Move back
  waypoint(1) -= horizontal_pause_back;
  add_waypoint(waypoint);

  current_segment = segments.begin();
}

VectorXd ArmPlanner::eval(double t, bool &grip) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  grip = current_segment->first;
  return current_segment->second.eval(t);
}

VectorXd ArmPlanner::deriv1(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->second.deriv1(t);
}

VectorXd ArmPlanner::deriv2(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);
  
  return current_segment->second.deriv2(t);
}

void ArmPlanner::retrieve_segment(double t) {
  if (t > segments.back().second.get_end_time()) {
    current_segment = prev(segments.end());
  } else if (t < segments.front().second.get_start_time()) {
    current_segment = segments.begin();
  } else {
    while (!current_segment->second.contains_time(t)) {
      ++current_segment;
      if (current_segment == segments.end()) {
        current_segment = segments.begin();
      }
    }
  }
}
    
bool ArmPlanner::planned() {
  return segments.size() > 0;
}

double ArmPlanner::get_end_time() {
  return segments.back().second.get_end_time();
}

void ArmPlanner::add_grip_phase(bool grip) {
  if (segments.size() == 0) {
    cout << "Cannot call add_waypoint on empty plan" << endl;
    return;
  }
  VectorXd start = segments.back().second.get_end();
  double start_time = segments.back().second.get_end_time(); 
  double end_time = start_time + grip_wait;
  segments.push_back(make_pair(grip, MinJerkInterpolator(start,
                                                         start,
                                                         start_time,
                                                         end_time)));
}

void ArmPlanner::add_waypoint(const VectorXd &waypoint) {
  if (segments.size() == 0) {
    cout << "Cannot call add_waypoint on empty plan" << endl;
    return;
  }

  VectorXd start = segments.back().second.get_end();
  double start_time = segments.back().second.get_end_time(); 
  Vector3d diff = waypoint.head<3>() - start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  bool grip = segments.back().first;
  segments.push_back(make_pair(grip, MinJerkInterpolator(start,
                                                         waypoint,
                                                         start_time,
                                                         end_time)));
}

void ArmPlanner::start_plan(double start_time, bool grip, const VectorXd &start, const VectorXd &waypoint) {
  segments.clear();
  Vector3d diff = waypoint.head<3>() - start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(make_pair(grip, MinJerkInterpolator(start,
                                                         waypoint,
                                                         start_time,
                                                         end_time)));
}
