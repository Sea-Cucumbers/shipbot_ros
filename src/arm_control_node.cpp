#include "kdHelper.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
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
  
  // This controller makes the end effector move along a circle with the following
  // parameters
  double cx;
  double cy;
  double cz;
  double radius;

  nh.getParam("urdf", urdf_file);
  nh.getParam("rate", rate);
  nh.getParam("cx", cx);
  nh.getParam("cy", cy);
  nh.getParam("cz", cz);
  nh.getParam("radius", radius);
  KDHelper kd(urdf_file);

  const vector<string> &actuator_names = kd.get_actuator_names();
  VectorXd position_cmds = VectorXd::Zero(actuator_names.size());
  VectorXd velocity_cmds = VectorXd::Zero(actuator_names.size());
  VectorXd effort_cmds = VectorXd::Zero(actuator_names.size());

  tf2_ros::TransformBroadcaster pose_br;
  geometry_msgs::TransformStamped pose;
  pose.header.frame_id = "world";
  pose.child_frame_id = "end effector pose";

  // Publish trajectory to rviz
  ros::Publisher trajectory_pub = nh.advertise<visualization_msgs::Marker>("/shipbot/desired_trajectory", 1);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.ns = "shipbot";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.02f;
  marker.color.g = 1.0f;
  marker.color.a = 1.0;
  marker.header.stamp = ros::Time::now();
  for (double t = 0; t < 2*M_PI + 0.1; t += 0.1) {
    geometry_msgs::Point p;
    p.x = cx + radius*cos(t);
    p.y = cy;
    p.z = cz + radius*sin(t);

    marker.points.push_back(p);
 
  }
  trajectory_pub.publish(marker);

  // HEBI initialization
  hebi::Lookup lookup;
  auto group = lookup.getGroupFromNames({"SEA Cucumbers"}, actuator_names);

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

  double start_t = ros::Time::now().toSec();
  ros::Rate r(rate);
  while (ros::ok())
  {
    double t = ros::Time::now().toSec();
    double x = cx + radius*cos(t);
    double z = cz + radius*sin(t);

    group->getNextFeedback(group_feedback);
    group_feedback.getPosition(position_fbk);
    group_feedback.getVelocity(velocity_fbk);
    group_feedback.getEffort(effort_fbk);

    kd.update_state(position_fbk, velocity_fbk, effort_fbk);

    kd.ik(position_cmds, x, cy, z, 0);

    // Send commands to HEBI modules
    group_command.setPosition(position_cmds);
    group->sendCommand(group_command);

    Vector3d position(0, 0, 0);
    Quaterniond orientation(1, 0, 0, 0);
    kd.fk(position, orientation);
    pose.transform.translation.x = position(0);
    pose.transform.translation.y = position(1);
    pose.transform.translation.z = position(2);
    pose.transform.rotation.x = orientation.x();
    pose.transform.rotation.y = orientation.y();
    pose.transform.rotation.z = orientation.z();
    pose.transform.rotation.w = orientation.w();

    pose.header.stamp = ros::Time::now();
    pose_br.sendTransform(pose);

    marker.header.stamp = ros::Time::now();
    trajectory_pub.publish(marker);

    r.sleep();
    ros::spinOnce();
  }
}

