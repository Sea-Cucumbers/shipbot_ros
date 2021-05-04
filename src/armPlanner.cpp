#include "armPlanner.h"
#include <iostream>

using namespace std;

ArmPlanner::ArmPlanner(double seconds_per_meter,
                       double seconds_per_degree) : seconds_per_meter(seconds_per_meter),
                                                    seconds_per_degree(seconds_per_degree) {
 reset_config = VectorXd::Zero(5);
 reset_config(0) = -0.135214;
 reset_config(1) = 0.18142;
 reset_config(2) = 0.05839807;
 reset_config(3) = -1.52432;
 reset_config(4) = 2.67237;

}

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
  segments.clear();
  VectorXd seg_start = start;
  VectorXd seg_end = reset_config;
  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));
                                         
  current_segment = segments.begin();
}

void ArmPlanner::spin_rotary(const VectorXd &start,
                             const Vector3d &position,
                             bool vertical_spin_axis,
                             double degrees,
                             double start_time) {
  segments.clear();
  VectorXd seg_start = start;
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = position;
  if (vertical_spin_axis) {
    // If vertical axis, pause 10 cm above
    seg_end(2) += pause_dist;
    seg_end(3) = -M_PI/2;
  } else {
    // If horizontal axis, pause 10 cm back
    seg_end(1) -= pause_dist;
  }
  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));
                                         
 
  seg_start = seg_end;
  if (vertical_spin_axis) {
    // If vertical axis, move 10 cm down
    seg_end(2) -= pause_dist + add_dist;
  } else {
    // If horizontal axis, move 10 cm forward
    seg_end(1) += pause_dist + add_dist;
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Wait for the gripper to grip
  start_time = end_time;
  end_time += grip_wait;
  seg_start = seg_end;
  segments.push_back(make_pair(true, MinJerkInterpolator(seg_start,
                                                         seg_end,
                                                         start_time,
                                                         end_time)));

  // Spin the valve
  start_time = end_time;
  end_time += seconds_per_degree*degrees;
  seg_start = seg_end;
  seg_end(4) -= degrees*M_PI/180;
  segments.push_back(make_pair(true, MinJerkInterpolator(seg_start,
                                                         seg_end,
                                                         start_time,
                                                         end_time)));

  // Wait for the gripper to ungrip
  start_time = end_time;
  end_time += grip_wait;
  seg_start = seg_end;
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Move back
  seg_start = seg_end;
  if (vertical_spin_axis) {
    // If vertical axis, move 10 cm up
    seg_end(2) += pause_dist + add_dist;
  } else {
    // If horizontal axis, move 10 cm back
    seg_end(1) -= pause_dist + add_dist;
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  current_segment = segments.begin();
}

void ArmPlanner::spin_shuttlecock(const VectorXd &start,
                                  const Vector3d &position, 
                                  bool vertical_spin_axis,
                                  bool do_open,
                                  double start_time) {
  segments.clear();
  VectorXd seg_start = start;
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = position;
  if (vertical_spin_axis) {
    seg_end(3) = -M_PI/2;
  }

  if (do_open) {
    if (vertical_spin_axis) {
      // The handle is pointing toward us. Stop to the left of it
      seg_end(0) -= pause_dist;
      
      Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
      double end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));

      // Move forward and right
      start_time = end_time;
      seg_start = seg_end;
      seg_end(0) += 2*pause_dist;
      seg_end(1) += shuttlecock_length;
      diff = seg_end.head<3>() - seg_start.head<3>();
      end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));
    } else {
      // The handle is currently pointing right. Stop below it
      seg_end(2) -= pause_dist;

      Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
      double end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));

      // Move up and left
      start_time = end_time;
      seg_start = seg_end;
      seg_end(0) -= shuttlecock_length;
      seg_end(2) += 2*pause_dist;
      diff = seg_end.head<3>() - seg_start.head<3>();
      end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));
    }
  } else {
    if (vertical_spin_axis) {
      // The handle is currently pointing right. Stop above it and forward
      seg_end(1) += pause_dist;
      seg_end(2) += pause_dist;

      Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
      double end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));

      // Move down 
      start_time = end_time;
      seg_start = seg_end;
      seg_end(2) -= pause_dist;
      diff = seg_end.head<3>() - seg_start.head<3>();
      end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));

      // Pull back and left
      start_time = end_time;
      seg_start = seg_end;
      seg_end(0) -= shuttlecock_length;
      seg_end(1) -= 2*pause_dist;
      diff = seg_end.head<3>() - seg_start.head<3>();
      end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));
    } else {
      // The handle is currently pointing up, stop to the left of it
      seg_end(0) -= pause_dist;

      Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
      double end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));

      // Move down and right
      seg_start = seg_end;
      seg_end(0) += shuttlecock_length;
      seg_end(2) -= 2*pause_dist;
      diff = seg_end.head<3>() - seg_start.head<3>();
      end_time = start_time + seconds_per_meter*diff.norm();
      segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                              seg_end,
                                                              start_time,
                                                              end_time)));
    }
  }

  current_segment = segments.begin();
}

void ArmPlanner::switch_breaker(const VectorXd &start,
                                const Vector3d &position,
                                bool push_up,
                                double start_time) {
  segments.clear();

  // Pause 10 cm back
  VectorXd seg_start = start;
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = position;
  seg_end(1) -= pause_dist;

  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                           end_time)));
                                         
  // Move forward
  seg_start = seg_end;
  seg_end(1) += pause_dist + add_dist;
  diff = seg_end.head<3>() - seg_start.head<3>();
  start_time = end_time;
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Wait for the gripper to grip
  start_time = end_time;
  end_time += 3;
  seg_start = seg_end;
  segments.push_back(make_pair(true, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Push the switch
  seg_start = seg_end;
  seg_end(2) = seg_end(2) + (push_up ? 0.05 : -0.05);
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(make_pair(true, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Wait for the gripper to ungrip
  start_time = end_time;
  end_time += 3;
  seg_start = seg_end;
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

  // Move back
  seg_start = seg_end;
  seg_end(1) -= pause_dist + add_dist;
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(make_pair(false, MinJerkInterpolator(seg_start,
                                                          seg_end,
                                                          start_time,
                                                          end_time)));

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
