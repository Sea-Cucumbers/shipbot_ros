#include "armPlanner.h"
#include <iostream>
#include "angle_mod.h"

using namespace std;

ArmPlanner::ArmPlanner(double seconds_per_meter,
                       double seconds_per_degree,
                       const VectorXd &reset_config,
                       double horizontal_pause_back,
                       double vertical_pause_back,
                       double vertical_pause_above,
                       double pause_side,
                       double grip_delay,
                       double press_delay,
                       double shuttlecock_force_h,
                       double shuttlecock_force_v,
                       double breaker_force_up,
                       double breaker_force_down,
                       double rotary_force_h,
                       double rotary_force_v) : seconds_per_meter(seconds_per_meter),
                                                seconds_per_degree(seconds_per_degree),
                                                reset_config(reset_config),
                                                horizontal_pause_back(horizontal_pause_back),
                                                vertical_pause_back(vertical_pause_back),
                                                vertical_pause_above(vertical_pause_above),
                                                pause_side(pause_side),
                                                grip_delay(grip_delay),
                                                press_delay(press_delay),
                                                shuttlecock_force_h(shuttlecock_force_h),
                                                shuttlecock_force_v(shuttlecock_force_v),
                                                breaker_force_up(breaker_force_up),
                                                breaker_force_down(breaker_force_down),
                                                rotary_force_h(rotary_force_h),
                                                rotary_force_v(rotary_force_v) {}

void ArmPlanner::sample_points(vector<geometry_msgs::Point> &points) {
  points.clear();
  if (segments.size() == 0) {
    cout << "Tried sampling trajectory but no trajectory was planned!" << endl;
    return;
  }
  bool grip_dummy;
  Vector3d force_dummy;
  for (double t = segments[0].interpolator.get_start_time();
       t < segments.back().interpolator.get_end_time();
       t += 0.1) {
    geometry_msgs::Point point;
    VectorXd tppt = eval(t, grip_dummy, force_dummy);
    point.x = tppt(0);
    point.y = tppt(1);
    point.z = tppt(2);
    points.push_back(point);
  }
}

void ArmPlanner::reset_arm(const VectorXd &start,
                           double start_time) {
  start_plan(start_time, false, Vector3d::Zero(), start, reset_config);
  current_segment = segments.begin();
}

void ArmPlanner::stop_arm(const VectorXd &start,
                          double start_time) {
  start_plan(start_time, false, Vector3d::Zero(), start, start);
  current_segment = segments.begin();
}

void ArmPlanner::spin_rotary(const VectorXd &start,
                             const Vector3d &position,
                             bool vertical_spin_axis,
                             double degrees,
                             double start_time) {
  // If vertical axis, pause above. If horizontal axis, pause back
  VectorXd waypoint = VectorXd::Zero(5);
  waypoint.head<3>() = position;

  if (vertical_spin_axis) {
    waypoint(2) += vertical_pause_above;
    waypoint(1) += vertical_pause_back;
    waypoint(3) = -M_PI/2;
  } else {
    waypoint(1) -= horizontal_pause_back;
  }

  start_plan(start_time, false, Vector3d::Zero(), start, waypoint);
                                         
  if (vertical_spin_axis) {
    // If vertical axis, move forward, then down
    waypoint(1) += vertical_pause_back;
    add_waypoint(waypoint);
    waypoint(2) -= vertical_pause_above;
    add_waypoint(waypoint);

    // Start pressing
    change_force(Vector3d(0, 0, rotary_force_v));
  } else {
    // If horizontal axis, move forward
    waypoint(1) += horizontal_pause_back;
    add_waypoint(waypoint);

    // Start pressing
    change_force(Vector3d(0, 0, rotary_force_h));
  }

  // Wait for the gripper to grip
  add_grip_phase(true);

  // Spin the valve
  waypoint(4) -= degrees*M_PI/180;
  add_waypoint(waypoint);

  // Stop pressing
  change_force(Vector3d(0, 0, 0));

  // Wait for the gripper to ungrip
  add_grip_phase(false);

  if (vertical_spin_axis) {
    // If vertical axis, move up then back
    waypoint(2) += vertical_pause_above;
    add_waypoint(waypoint);
    waypoint(1) -= vertical_pause_back;
    add_waypoint(waypoint);
  } else {
    // If horizontal axis, move back
    waypoint(1) -= horizontal_pause_back;
    add_waypoint(waypoint);
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

  if (do_open) {
    if (vertical_spin_axis) {
      // The handle is pointing toward us. Stop to the left
      waypoint(0) -= pause_side;
      waypoint(2) -= 0.04;
      waypoint(3) = -M_PI/2;
      
      start_plan(start_time, false, Vector3d::Zero(), start, waypoint);

      // Move forward and right
      waypoint(0) += shuttlecock_tip_to_joint + pause_side;
      waypoint(1) += shuttlecock_handle_center_to_joint;
      waypoint(4) += 3*M_PI/4;
      add_waypoint(waypoint);
    } else {
      // The handle is pointing right. Pause below and to the right
      waypoint(0) += 0.0508;
      waypoint(1) -= 0.08;
      waypoint(2) -= vertical_pause_above;
      waypoint(3) = M_PI/2;
      
      start_plan(start_time, false, Vector3d::Zero(), start, waypoint);

      // Move up and left
      waypoint(2) += vertical_pause_above + 0.05;
      waypoint(0) -= 0.06;
      add_waypoint(waypoint);
    }
  } else {
    if (vertical_spin_axis) {
      // The handle is pointing right. Stop backward
      waypoint(1) -= vertical_pause_back;
      waypoint(2) -= shuttlecock_depth;
      
      start_plan(start_time, false, Vector3d::Zero(), start, waypoint);

      // Move forward
      waypoint(1) += vertical_pause_back;
      add_waypoint(waypoint);

      // Face down
      waypoint(3) = -M_PI/3;
      add_waypoint(waypoint);

      // Start pressing
      change_force(Vector3d(0, 0, shuttlecock_force_v));

      // Harden end-effector
      add_grip_phase(true);

      // Spin
      waypoint(4) -= 3*M_PI/4;
      waypoint(0) -= shuttlecock_tip_to_joint;
      waypoint(1) -= shuttlecock_tip_to_joint;
      add_waypoint(waypoint);

      // Stop pressing
      change_force(Vector3d(0, 0, 0));

      // Unharden end-effector
      add_grip_phase(false);

      // Move up and back
      waypoint(2) += vertical_pause_above + shuttlecock_depth;
      add_waypoint(waypoint);
    } else {
      // The handle is pointing up. Pause backward from it
      waypoint(1) -= horizontal_pause_back;
      
      start_plan(start_time, false, Vector3d::Zero(), start, waypoint);

      // Harden end-effector
      add_grip_phase(true);

      // Move forward
      waypoint(1) += horizontal_pause_back;
      add_waypoint(waypoint);

      // Move down and right
      waypoint(0) += shuttlecock_tip_to_joint;
      waypoint(2) -= shuttlecock_tip_to_joint;
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
                                int num,
                                double start_time) {
  double pause_vertical = push_up ? -horizontal_pause_back : horizontal_pause_back;
  double angle = push_up ? M_PI/4 : -M_PI/4;
  double force = push_up ? breaker_force_up : breaker_force_down;

  int pause_horizontal = 0;
  if (num == 1) {
    pause_horizontal = pause_side;
  } else if (num == 2) {
    pause_horizontal = 0;
  } else if (num == 3) {
    pause_horizontal = -pause_side;
  } else {
    return;
  }

  // Pause back and up/down depending on which way we're pushing, and left/right
  // depending on which switch we're pushing
  VectorXd waypoint = VectorXd::Zero(5);
  waypoint.head<3>() = position;
  waypoint(0) += pause_horizontal;
  waypoint(1) -= horizontal_pause_back;
  waypoint(2) += pause_vertical;
  waypoint(3) = angle;

  start_plan(start_time, false, Vector3d::Zero(), start, waypoint);
                                         
  // Move forward and upward/downward depending on which way we're pushing,
  // and leftward/rightward depending on which switch we're pushing
  waypoint(0) -= pause_horizontal;
  waypoint(1) += horizontal_pause_back;
  waypoint(2) = waypoint(2) - pause_vertical;
  add_waypoint(waypoint);

  // Start pressing
  change_force(Vector3d(0, 0, force));

  // Grip
  add_grip_phase(true);

  // Press up if we're pushing up
  if (push_up) {
    change_force(Vector3d(0, -breaker_force_up, breaker_force_up)/sqrt(2));
  }

  // Push the switch
  waypoint(2) = waypoint(2) + (push_up ? 0.1 : -0.1);
  add_waypoint(waypoint);

  // Stop pressing
  change_force(Vector3d(0, 0, 0));

  // Ungrip
  add_grip_phase(false);

  // Move back
  waypoint(1) -= horizontal_pause_back;
  add_waypoint(waypoint);

  current_segment = segments.begin();
}

VectorXd ArmPlanner::eval(double t, bool &grip, Vector3d &force) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  grip = current_segment->grip;
  force = current_segment->force;
  return current_segment->interpolator.eval(t);
}

VectorXd ArmPlanner::deriv1(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->interpolator.deriv1(t);
}

VectorXd ArmPlanner::deriv2(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);
  
  return current_segment->interpolator.deriv2(t);
}

void ArmPlanner::retrieve_segment(double t) {
  if (t > segments.back().interpolator.get_end_time()) {
    current_segment = prev(segments.end());
  } else if (t < segments.front().interpolator.get_start_time()) {
    current_segment = segments.begin();
  } else {
    while (!current_segment->interpolator.contains_time(t)) {
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
  return segments.back().interpolator.get_end_time();
}

void ArmPlanner::add_grip_phase(bool grip) {
  if (segments.size() == 0) {
    cout << "Cannot call add_grip_phase on empty plan" << endl;
    return;
  }
  VectorXd start = segments.back().interpolator.get_end();
  double start_time = segments.back().interpolator.get_end_time(); 
  double end_time = start_time + grip_delay;
  const Vector3d &force = segments.back().force;
  segments.push_back(Segment(grip, force, MinJerkInterpolator(start,
                                                              start,
                                                              start_time,
                                                              end_time)));
}

void ArmPlanner::change_force(const Vector3d &force) {
  if (segments.size() == 0) {
    cout << "Cannot call add_force on empty plan" << endl;
    return;
  }
  VectorXd start = segments.back().interpolator.get_end();
  double start_time = segments.back().interpolator.get_end_time(); 
  double end_time = start_time + press_delay;
  bool grip = segments.back().grip;
  segments.push_back(Segment(grip, force, MinJerkInterpolator(start,
                                                              start,
                                                              start_time,
                                                              end_time)));
}

void ArmPlanner::add_waypoint(const VectorXd &waypoint) {
  if (segments.size() == 0) {
    cout << "Cannot call add_waypoint on empty plan" << endl;
    return;
  }

  VectorXd start = segments.back().interpolator.get_end();
  double start_time = segments.back().interpolator.get_end_time(); 
  Vector3d diff = waypoint.head<3>() - start.head<3>();
  double angle_diff = max(abs(angdiff(waypoint(3), start(3))), abs(angdiff(waypoint(4), start(4))));
  double end_time = start_time + max(seconds_per_meter*diff.norm(), seconds_per_degree*angle_diff*180/M_PI);
  bool grip = segments.back().grip;
  const Vector3d &force = segments.back().force;
  segments.push_back(Segment(grip, force, MinJerkInterpolator(start,
                                                              waypoint,
                                                              start_time,
                                                              end_time)));
}

void ArmPlanner::start_plan(double start_time, bool grip, const Vector3d &force, const VectorXd &start, const VectorXd &waypoint) {
  segments.clear();
  Vector3d diff = waypoint.head<3>() - start.head<3>();
  double angle_diff = max(abs(angdiff(waypoint(3), start(3))), abs(angdiff(waypoint(4), start(4))));
  double end_time = start_time + max(seconds_per_meter*diff.norm(), seconds_per_degree*angle_diff*180/M_PI);
  segments.push_back(Segment(grip, force, MinJerkInterpolator(start,
                                                              waypoint,
                                                              start_time,
                                                              end_time)));
}
