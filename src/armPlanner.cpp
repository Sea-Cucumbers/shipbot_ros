#include "armPlanner.h"
#include <iostream>

using namespace std;

ArmPlanner::ArmPlanner(double seconds_per_meter,
                       double seconds_per_degree) : seconds_per_meter(seconds_per_meter),
                                                    seconds_per_degree(seconds_per_degree),
                                                    reset_position(0, 0.4, 0.5) {}

void ArmPlanner::sample_points(vector<geometry_msgs::Point> &points) {
  points.clear();
  if (segments.size() == 0) {
    cout << "Tried sampling trajectory but no trajectory was planned!" << endl;
    return;
  }
  for (double t = segments[0].get_start_time();
       t < segments.back().get_end_time();
       t += 0.1) {
    geometry_msgs::Point point;
    VectorXd tppt = eval(t);
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
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = reset_position;
  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));
                                         
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
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));
                                         
 
  seg_start = seg_end;
  if (vertical_spin_axis) {
    // If vertical axis, move 10 cm down
    seg_end(2) -= pause_dist;
  } else {
    // If horizontal axis, move 10 cm forward
    seg_end(1) += pause_dist;
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  // Spin the valve
  start_time = end_time;
  end_time += seconds_per_degree*degrees;
  seg_start = seg_end;
  seg_end(4) -= degrees*M_PI/180;

  // Move back
  seg_start = seg_end;
  if (vertical_spin_axis) {
    // If vertical axis, move 10 cm up
    seg_end(2) += pause_dist;
  } else {
    // If horizontal axis, move 10 cm back
    seg_end(1) -= pause_dist;
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  current_segment = segments.begin();
}

void ArmPlanner::spin_shuttlecock(const VectorXd &start,
                                  const Vector3d &position, 
                                  const Vector3d &handle_end,
                                  bool vertical_spin_axis,
                                  bool clockwise,
                                  double start_time) {
  segments.clear();
  VectorXd seg_start = start;
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = position;
  if (vertical_spin_axis) {
    seg_end(3) = -M_PI/2;
  }

  if (clockwise) {
    if (vertical_spin_axis) {
      if (handle_end(1) > position(1)) {
        seg_end(0) -= pause_dist;
      } else {
        seg_end(0) += pause_dist;
      }
    } else {
      if (handle_end(2) > position(2)) {
        seg_end(0) -= pause_dist;
      } else {
        seg_end(0) += pause_dist;
      }
    }
  } else {
    if (vertical_spin_axis) {
      if (handle_end(1) > position(1)) {
        seg_end(0) += pause_dist;
      } else {
        seg_end(0) -= pause_dist;
      }
    } else {
      if (handle_end(2) > position(2)) {
        seg_end(0) += pause_dist;
      } else {
        seg_end(0) -= pause_dist;
      }
    }
  }

  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));
                                         
 
  seg_start = seg_end;
  if (clockwise) {
    if (vertical_spin_axis) {
      if (handle_end(1) > position(1)) {
        seg_end(0) += shuttlecock_length;
        seg_end(1) -= shuttlecock_length;
      } else {
        seg_end(0) -= shuttlecock_length;
        seg_end(1) += shuttlecock_length;
      }
    } else {
      if (handle_end(2) > position(2)) {
        seg_end(0) += shuttlecock_length;
        seg_end(2) -= shuttlecock_length;
      } else {
        seg_end(0) -= shuttlecock_length;
        seg_end(2) += shuttlecock_length;
      }
    }
  } else {
    if (vertical_spin_axis) {
      if (handle_end(1) > position(1)) {
        seg_end(0) -= shuttlecock_length;
        seg_end(1) -= shuttlecock_length;
      } else {
        seg_end(0) += shuttlecock_length;
        seg_end(1) += shuttlecock_length;
      }
    } else {
      if (handle_end(2) > position(2)) {
        seg_end(0) -= shuttlecock_length;
        seg_end(2) -= shuttlecock_length;
      } else {
        seg_end(0) += shuttlecock_length;
        seg_end(2) += shuttlecock_length;
      }
    }
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  // Move back
  seg_start = seg_end;
  if (vertical_spin_axis) {
    // If vertical axis, move 10 cm up
    seg_end(2) += pause_dist;
  } else {
    // If horizontal axis, move 10 cm back
    seg_end(1) -= pause_dist;
  }
  start_time = end_time;
  diff = seg_end.head<3>() - seg_start.head<3>();
  end_time += seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  current_segment = segments.begin();
}

void ArmPlanner::switch_breaker(const VectorXd &start,
                                const Vector3d &position,
                                bool push_up,
                                double start_time) {
  segments.clear();
  VectorXd seg_start = start;
  VectorXd seg_end = VectorXd::Zero(5);
  seg_end.head<3>() = position;
  seg_end(2) -= pause_dist;

  Vector3d diff = seg_end.head<3>() - seg_start.head<3>();
  double end_time = start_time + seconds_per_meter*diff.norm();
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));
                                         
 
  seg_start = seg_end;
  seg_end(2) += pause_dist;
  start_time = end_time;
  end_time += seconds_per_meter*pause_dist;
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  // Move back
  seg_start = seg_end;
  seg_end(1) -= pause_dist;
  start_time = end_time;
  end_time += seconds_per_meter*pause_dist;
  segments.push_back(MinJerkInterpolator(seg_start,
                                         seg_end,
                                         start_time,
                                         end_time));

  current_segment = segments.begin();
}

VectorXd ArmPlanner::eval(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->eval(t);
}

VectorXd ArmPlanner::deriv1(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);

  return current_segment->deriv1(t);
}

VectorXd ArmPlanner::deriv2(double t) {
  if (segments.size() == 0) {
    cout << "No plan to evaluate!" << endl;
    exit(1);
  }
  retrieve_segment(t);
  
  return current_segment->deriv2(t);
}

void ArmPlanner::retrieve_segment(double t) {
  if (t > segments.back().get_end_time()) {
    current_segment = prev(segments.end());
  } else if (t > segments.back().get_end_time()) {
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
    
