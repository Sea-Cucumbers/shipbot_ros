#include "ikSolver.h"
#include <pinocchio/container/boost-container-limits.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

IKSolver::IKSolver(const string &urdf_file) {
  pin::urdf::buildModel(urdf_file, model);
  model_data = make_unique<pin::Data>(model);

  forwardKinematics(model, *model_data, pin::neutral(model));
  pin::updateFramePlacements(model, *model_data);

  for (pin::JointIndex joint_id = 0;
       joint_id < model.joints.size();
       ++joint_id) {
    if (model.names[joint_id].rfind("rotary") != string::npos) {
      actuator_names.push_back(model.names[joint_id]);
    }
  }

  pin::FrameIndex l2_fid = model.getFrameId("link2");
  pin::FrameIndex l3_fid = model.getFrameId("link3");
  pin::FrameIndex l4_fid = model.getFrameId("link4");
  pin::FrameIndex l5_fid = model.getFrameId("link5");
  D = model_data->oMf[l5_fid].translation()(0);

  l2 = model_data->oMf[l3_fid].translation()(1) - model_data->oMf[l2_fid].translation()(1);
  l3 = model_data->oMf[l4_fid].translation()(1) - model_data->oMf[l3_fid].translation()(1);
  double l4 = model_data->oMf[l5_fid].translation()(1) - model_data->oMf[l4_fid].translation()(1);
  double l5 = 4.4386e-02; // Would have to use the urdf library to get this, but I'm feeling a bit lazy
  ee = l4 + l5;
}

void IKSolver::solve(unordered_map<string, std_msgs::Float64> &cmd_msgs, double x, double y, double z) {
  double th1 = atan2(y, x) + asin(D/sqrt(x*x + y*y));

  double xp = x - D*sin(th1);
  double yp = y + D*cos(th1);

  // If we wanted a nonzero pitch, we'd have to subtract ee*cos(pitch) here
  double a = sqrt(xp*xp + yp*yp) - ee;
  // If we wanted a nonzero pitch, we'd have to subtract ee*sin(pitch) here
  double zp = z; 

  double th3 = -acos((a*a + zp*zp - l2*l2 - l3*l3)/(2*l2*l3));
  double th2 = atan2(zp, a) - atan2(l3*sin(th3), l2 + l3*cos(th3));

  // If we wanted a nonzero pitch, we'd have to add pitch here
  double th4 = -th3 - th2;

  cmd_msgs["rotary1"].data = th1 - M_PI/2;
  cmd_msgs["rotary2"].data = -th2;
  cmd_msgs["rotary3"].data = th3;
  cmd_msgs["rotary4"].data = -th4;
}
