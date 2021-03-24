#include "kdHelper.h"
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <chrono>
#include <thread>
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_control_node");
  ros::NodeHandle nh("~");

  string urdf_file;
  double rate = 20;
  
  nh.getParam("urdf", urdf_file);
  nh.getParam("rate", rate);
  KDHelper kd(urdf_file);

  const vector<string> &actuator_names = kd.get_actuator_names();
  VectorXd position_cmds = VectorXd::Zero(actuator_names.size());
  VectorXd velocity_cmds = VectorXd::Zero(actuator_names.size());
  VectorXd effort_cmds = VectorXd::Zero(actuator_names.size());

  tf2_ros::TransformBroadcaster pose_br;
  geometry_msgs::TransformStamped pose;
  pose.header.frame_id = "world";
  pose.child_frame_id = "end effector pose";

  // HEBI initialization
  hebi::Lookup lookup;
  auto group = lookup.getGroupFromNames({"ShipBot"}, actuator_names);

  if (!group) {
    std::cout
      << "Group not found! Check that the family and name of a module on the network" << std::endl
      << "matches what is given in the source file." << std::endl;
    return -1;
  }

  hebi::GroupCommand group_command(group->size());
  hebi::GroupFeedback group_feedback(group->size());

  VectorXd position_fbk = VectorXd::Zero(group->size());
  VectorXd velocity_fbk = VectorXd::Zero(group->size());
  VectorXd effort_fbk = VectorXd::Zero(group->size());
  
  Vector3d position(0, 0, 0);
  Quaterniond orientation(1, 0, 0, 0);
  double pitch = 0;
  double roll = 0;

  ros::Rate r(rate);
  while (ros::ok())
  {
    group->getNextFeedback(group_feedback);
    group_feedback.getPosition(position_fbk);
    group_feedback.getVelocity(velocity_fbk);
    group_feedback.getEffort(effort_fbk);

    kd.update_state(position_fbk, velocity_fbk, effort_fbk);

    kd.fk(position, orientation, pitch, roll);

    double t = ros::Time::now().toSec();

    VectorXd task_config(5);
    task_config.head<3>() = position;
    task_config(3) = pitch;
    task_config(4) = roll;
    kd.ik(position_cmds, task_config);
    kd.grav_comp(effort_cmds);

    // These don't seem to help
    //kd.tsid(effort_cmds, task_acc);
    //kd.idk(velocity_cmds, task_vel);

    // Send commands to HEBI modules
    //group_command.setPosition(position_cmds);
    //group_command.setVelocity(velocity_cmds);
    group_command.setEffort(effort_cmds);
    group->sendCommand(group_command);

    pose.transform.translation.x = position(0);
    pose.transform.translation.y = position(1);
    pose.transform.translation.z = position(2);
    pose.transform.rotation.x = orientation.x();
    pose.transform.rotation.y = orientation.y();
    pose.transform.rotation.z = orientation.z();
    pose.transform.rotation.w = orientation.w();

    pose.header.stamp = ros::Time::now();
    pose_br.sendTransform(pose);

    r.sleep();
    ros::spinOnce();
  }
}

