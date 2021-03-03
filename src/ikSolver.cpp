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

  pin::FrameIndex l1_fid = model.getFrameId("link1");
  pin::FrameIndex l2_fid = model.getFrameId("link2");
  pin::FrameIndex l3_fid = model.getFrameId("link3");
  pin::FrameIndex l4_fid = model.getFrameId("link4");
  pin::FrameIndex l5_fid = model.getFrameId("link5");
  ee_fid = model.getFrameId("end_effector");
  D = model_data->oMf[ee_fid].translation()(0);

  l0 = model_data->oMf[l1_fid].translation()(2);
  l1 = model_data->oMf[l2_fid].translation()(2) - model_data->oMf[l1_fid].translation()(2);
  l2 = model_data->oMf[l3_fid].translation()(1) - model_data->oMf[l2_fid].translation()(1);
  l2_loc = model_data->oMf[l2_fid].translation()(1);
  l3 = model_data->oMf[l4_fid].translation()(1) - model_data->oMf[l3_fid].translation()(1);
  l4 = model_data->oMf[l5_fid].translation()(1) - model_data->oMf[l4_fid].translation()(1);
  l5 = model_data->oMf[ee_fid].translation()(1) - model_data->oMf[l5_fid].translation()(1);
  ee = l4 + l5;
}

void IKSolver::solve(unordered_map<string, std_msgs::Float64> &cmd_msgs, double x, double y, double z) {
  double th1 = atan2(y, x) + asin(D/sqrt(x*x + y*y));

  double xp = x - D*sin(th1);
  double yp = y + D*cos(th1);

  // If we wanted a nonzero pitch, we'd have to subtract ee*cos(pitch) here
  double a = sqrt(xp*xp + yp*yp) - ee + l2_loc;
  // If we wanted a nonzero pitch, we'd have to subtract ee*sin(pitch) here
  double zp = z - l0 - l1; 

  double th3 = -acos((a*a + zp*zp - l2*l2 - l3*l3)/(2*l2*l3));
  double th2 = atan2(zp, a) - atan2(l3*sin(th3), l2 + l3*cos(th3));

  // If we wanted a nonzero pitch, we'd have to add pitch here
  double th4 = -th3 - th2;

  cmd_msgs["rotary1"].data = th1 - M_PI/2;
  cmd_msgs["rotary2"].data = -th2;
  cmd_msgs["rotary3"].data = th3;
  cmd_msgs["rotary4"].data = -th4;
}

void IKSolver::fk(Vector3d &position, Quaterniond &orientation, shared_ptr<sensor_msgs::JointState> &joints_ptr) {
  size_t njoints_msg = joints_ptr->name.size();
  VectorXd config(2*njoints_msg);
  VectorXd vel(njoints_msg);
  for (size_t j = 0; j < njoints_msg; ++j) {
    pin::JointIndex jidx = model.getJointId(joints_ptr->name[j]) - 1;
    config(2*jidx) = cos(joints_ptr->position[j]);
    config(2*jidx + 1) = sin(joints_ptr->position[j]);
    vel(jidx) = joints_ptr->velocity[j];
  }

  pin::forwardKinematics(model, *model_data, config, vel);
  pin::updateFramePlacements(model, *model_data);
  position = model_data->oMf[ee_fid].translation();
  orientation = Quaterniond(model_data->oMf[ee_fid].rotation());
}
