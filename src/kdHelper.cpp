#include "kdHelper.h"
#include <pinocchio/container/boost-container-limits.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <Eigen/Dense>
#include <iostream>

KDHelper::KDHelper(const string &urdf_file) {
  pin::urdf::buildModel(urdf_file, model);
  model_data = make_unique<pin::Data>(model);

  config = pin::neutral(model);
  // This is to account for the fact that joint 0 = PI at neutral
  config(0) = -1;
  config(1) = 0;
  vel = VectorXd::Zero(model.nv);
  forwardKinematics(model, *model_data, config);
  pin::updateFramePlacements(model, *model_data);

  // Start at 1 because 0 is universe
  for (pin::JointIndex joint_id = 1;
       joint_id < model.joints.size();
       ++joint_id) {
    actuator_names.push_back(model.names[joint_id]);
  }

  pin::FrameIndex l1_fid = model.getFrameId("shoulder1_joint");
  pin::FrameIndex l2_fid = model.getFrameId("shoulder2_joint");
  pin::FrameIndex l3_fid = model.getFrameId("elbow_joint");
  pin::FrameIndex l4_fid = model.getFrameId("wrist1_joint");
  pin::FrameIndex l5_fid = model.getFrameId("wrist2_joint");
  ee_fid = model.getFrameId("ee_tip");
  D = model_data->oMf[ee_fid].translation()(0);

  l0 = model_data->oMf[l1_fid].translation()(2);
  l1 = model_data->oMf[l2_fid].translation()(2) - l0;
  l2_loc = model_data->oMf[l2_fid].translation()(1);
  l2 = model_data->oMf[l3_fid].translation()(1) - l2_loc;
  l3 = model_data->oMf[l4_fid].translation()(1) - model_data->oMf[l3_fid].translation()(1);
  l4 = model_data->oMf[l5_fid].translation()(1) - model_data->oMf[l4_fid].translation()(1);
  l5 = model_data->oMf[ee_fid].translation()(1) - model_data->oMf[l5_fid].translation()(1);
  ee = l4 + l5;
}

void KDHelper::update_state(const VectorXd &positions, const VectorXd &velocities, const VectorXd &efforts) {
  for (size_t j = 0; j < actuator_names.size(); ++j) { 
    pin::JointIndex jidx = model.getJointId(actuator_names[j]);
    int qidx = model.joints[jidx].idx_q();
    int vidx = model.joints[jidx].idx_v();
    config(qidx) = cos(positions(j));
    config(qidx + 1) = sin(positions(j));
    vel(vidx) = velocities(j);
  }

  pin::forwardKinematics(model, *model_data, config, vel);
  pin::computeJointJacobians(model, *model_data);
  pin::updateFramePlacements(model, *model_data);
}

bool KDHelper::ik(VectorXd &joint_positions, const VectorXd& task_config) {
  double x = task_config(0);
  double y = task_config(1);
  double z = task_config(2);
  double pitch = task_config(3);
  double roll = task_config(4);

  double th1 = atan2(y, x) + asin(D/sqrt(x*x + y*y));

  double xp = x - D*sin(th1);
  double yp = y + D*cos(th1);

  double a = sqrt(xp*xp + yp*yp) - ee*cos(pitch) + l2_loc;
  double zp = z - l0 - l1 - ee*sin(pitch); 

  double c = (a*a + zp*zp - l2*l2 - l3*l3)/(2*l2*l3);
  if (c > 1 || c < -1) {
    return false;
  }
  double th3 = abs(c - 1) < 0.01 ? 0 : (abs(c + 1) < 0.01 ? M_PI : -acos(c));
  double th2 = atan2(zp, a) - atan2(l3*sin(th3), l2 + l3*cos(th3));
  double th4 = pitch - th3 - th2;

  joint_positions(0) = th1 - M_PI/2 - M_PI;
  joint_positions(1) = -th2;
  joint_positions(2) = th3;
  joint_positions(3) = -th4;
  joint_positions(4) = -roll;

  return true;
}

void KDHelper::fk(Vector3d &position, Quaterniond &orientation, double &pitch, double &roll) {
  position = model_data->oMf[ee_fid].translation();
  orientation = Quaterniond(model_data->oMf[ee_fid].rotation());
  pitch = -atan2(config(3), config(2)) + atan2(config(5), config(4)) - atan2(config(7), config(6));
  roll = -atan2(config(9), config(8));

  //cout << model_data->oMf[model.getFrameId("elbow_joint")].translation()/0.0254 << endl << endl;
}

void KDHelper::grav_comp(VectorXd &joint_torques) {
  joint_torques = pin::computeGeneralizedGravity(model, *model_data, config);
}

void KDHelper::tsid(VectorXd &joint_torques, const VectorXd &acc) {
  MatrixXd J = MatrixXd::Zero(5, 5);
  MatrixXd dJ = MatrixXd::Zero(5, 5);
  MatrixXd Jee = MatrixXd::Zero(6, 5);
  MatrixXd dJee = MatrixXd::Zero(6, 5);
  pin::getFrameJacobian(model, *model_data, ee_fid, pin::ReferenceFrame::WORLD, Jee);
  pin::getFrameJacobianTimeVariation(model, *model_data, ee_fid, pin::ReferenceFrame::WORLD, dJee);
  J.topRows<3>() = Jee.topRows<3>();
  J(3, 1) = -1;
  J(3, 2) = 1;
  J(3, 3) = -1;
  J(4, 4) = 1;
  dJ.topRows<3>() = dJee.topRows<3>();
  joint_torques = pin::rnea(model, *model_data, config, vel, J.completeOrthogonalDecomposition().solve(acc - dJ*vel));
}

void KDHelper::idk(VectorXd &joint_velocities, const VectorXd &task_velocity) {
  MatrixXd J = MatrixXd::Zero(5, 5);
  MatrixXd dJ = MatrixXd::Zero(5, 5);
  MatrixXd Jee = MatrixXd::Zero(6, 5);
  MatrixXd dJee = MatrixXd::Zero(6, 5);
  pin::getFrameJacobian(model, *model_data, ee_fid, pin::ReferenceFrame::WORLD, Jee);
  J.topRows<3>() = Jee.topRows<3>();
  J(3, 1) = -1;
  J(3, 2) = 1;
  J(3, 3) = -1;
  J(4, 4) = 1;
  joint_velocities = J.completeOrthogonalDecomposition().solve(task_velocity);
}
