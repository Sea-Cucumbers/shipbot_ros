#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "shipbot_ros/spin_rotary.h"
#include "shipbot_ros/spin_shuttlecock.h"
#include "shipbot_ros/switch_breaker.h"
#include "shipbot_ros/reset_arm.h"
#include "shipbot_ros/QueryWheel.h"
#include "shipbot_ros/QuerySpigot.h"
#include "shipbot_ros/QueryShuttlecock.h"
#include "shipbot_ros/QueryBreaker.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void string_split(vector<string> &ssplit, string s, string del) {
  ssplit.clear();
  int start = 0;
  int end = s.find(del);
  while (end != -1) {
    ssplit.push_back(s.substr(start, end - start));
    start = end + del.size();
    end = s.find(del, start);
  }
  ssplit.push_back(s.substr(start, end - start));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_control_node");
  ros::NodeHandle nh("~");

  // Read mission file
  string mission_file;
  nh.getParam("mission_file", mission_file);
  ifstream f(mission_file);
  string mission_str;
  if (f) {
    ostringstream ss;
    ss << f.rdbuf();
    mission_str = ss.str();
  }

  // Split on commas
  vector<string> commands;
  string_split(commands, mission_str, ", ");
  for (vector<string>::iterator it = commands.begin(); it != commands.end(); ++it) {
    cout << *it << endl;
  }

  // For now, just do the first task in the file
  int cmd_idx = 0;

  // Wait for vision services and arm services
  ros::service::waitForService("/realsense_node/query_wheel", -1);
  ros::service::waitForService("/realsense_node/query_spigot", -1);
  ros::service::waitForService("/realsense_node/query_shuttlecock", -1);
  ros::service::waitForService("/realsense_node/query_breaker", -1);
  ros::service::waitForService("/arm_control_node/spin_rotary", -1);
  ros::service::waitForService("/arm_control_node/spin_shuttlecock", -1);
  ros::service::waitForService("/arm_control_node/switch_breaker", -1);
  ros::service::waitForService("/arm_control_node/reset_arm", -1);

  // Initialize vision service clients and srvs
  ros::ServiceClient query_wheel_client = nh.serviceClient<shipbot_ros::QueryWheel>("/realsense_node/query_wheel");
  ros::ServiceClient query_spigot_client = nh.serviceClient<shipbot_ros::QuerySpigot>("/realsense_node/query_spigot");
  ros::ServiceClient query_shuttlecock_client = nh.serviceClient<shipbot_ros::QueryShuttlecock>("/realsense_node/query_shuttlecock");
  ros::ServiceClient query_breaker_client = nh.serviceClient<shipbot_ros::QueryBreaker>("/realsense_node/query_breaker");
  shipbot_ros::QueryWheel query_wheel_srv;
  shipbot_ros::QuerySpigot query_spigot_srv;
  shipbot_ros::QueryShuttlecock query_shuttlecock_srv;
  shipbot_ros::QueryBreaker query_breaker_srv;

  // Initialize arm service clients and srvs
  ros::ServiceClient spin_rotary_client = nh.serviceClient<shipbot_ros::spin_rotary>("/arm_control_node/spin_rotary");
  ros::ServiceClient spin_shuttlecock_client = nh.serviceClient<shipbot_ros::spin_shuttlecock>("/arm_control_node/spin_shuttlecock");
  ros::ServiceClient switch_breaker_client = nh.serviceClient<shipbot_ros::switch_breaker>("/arm_control_node/switch_breaker");
  ros::ServiceClient reset_arm_client = nh.serviceClient<shipbot_ros::reset_arm>("/arm_control_node/reset_arm");
  shipbot_ros::spin_rotary spin_rotary_srv;
  shipbot_ros::spin_shuttlecock spin_shuttlecock_srv;
  shipbot_ros::switch_breaker switch_breaker_srv;
  shipbot_ros::reset_arm reset_arm_srv;

  // Get extrinsics
  vector<double> t_cam_armv;
  vector<double> R_cam_armv;
  nh.getParam("/t_cam_arm", t_cam_armv);
  nh.getParam("/R_cam_arm", R_cam_armv);

  Matrix3d R_cam_arm;
  R_cam_arm << R_cam_armv[0], R_cam_armv[1], R_cam_armv[2],
               R_cam_armv[3], R_cam_armv[4], R_cam_armv[5],
               R_cam_armv[6], R_cam_armv[7], R_cam_armv[8];
  Vector3d t_cam_arm(t_cam_armv[0], t_cam_armv[1], t_cam_armv[2]);

  /*
  if (reset_arm_client.call(reset_arm_srv)) {
    ROS_INFO("Commanded arm to reset");
  } else {
    ROS_ERROR("Failed to command arm to reset");
    return 1;
  }
  */

  // Just do first command for now
  char station = commands[cmd_idx][0];
  string rest = commands[cmd_idx].substr(1, commands[cmd_idx].size());
  vector<string> tokens;
  string_split(tokens, rest, " ");

  if (tokens[0] == "V1") {
    // Query device
    if (query_wheel_client.call(query_wheel_srv)) {
      ROS_INFO("Queried wheel");
    } else {
      ROS_ERROR("Failed to query wheel");
      return 1;
    }

    if (!query_wheel_srv.response.state.visible) {
      cout << "Couldn't find wheel valve" << endl;
      return 1; // TODO: don't return 
    }

    // Transform position into arm frame
    Vector3d dev_pos(query_wheel_srv.response.state.position.x,
                     query_wheel_srv.response.state.position.y,
                     query_wheel_srv.response.state.position.z);
    dev_pos = R_cam_arm*dev_pos + t_cam_arm;

    // Command arm
    spin_rotary_srv.request.position.x = dev_pos(0);
    spin_rotary_srv.request.position.y = dev_pos(1);
    spin_rotary_srv.request.position.z = dev_pos(2);
    spin_rotary_srv.request.vertical_spin_axis = false;
    spin_rotary_srv.request.degrees = stoi(tokens[1]);
    if (spin_rotary_client.call(spin_rotary_srv)) {
      ROS_INFO("Commanded arm to spin wheel valve");
    } else {
      ROS_ERROR("Failed to command arm to spin wheel valve");
      return 1;
    }
  } else if (tokens[0] == "V2") {
    // Query device
    if (query_spigot_client.call(query_spigot_srv)) {
      ROS_INFO("Successfully queried spigot");
    } else {
      ROS_ERROR("Failed to query spigot");
      return 1;
    }

    if (!query_spigot_srv.response.state.visible) {
      cout << "Couldn't find spigot valve" << endl;
      return 1; // TODO: don't return 
    }

    // Transform position into arm frame
    Vector3d dev_pos(query_spigot_srv.response.state.position.x,
                     query_spigot_srv.response.state.position.y,
                     query_spigot_srv.response.state.position.z);
    dev_pos = R_cam_arm*dev_pos + t_cam_arm;

    // Command arm
    spin_rotary_srv.request.position.x = dev_pos(0);
    spin_rotary_srv.request.position.y = dev_pos(1);
    spin_rotary_srv.request.position.z = dev_pos(2);
    spin_rotary_srv.request.vertical_spin_axis = query_spigot_srv.response.state.vertical;
    spin_rotary_srv.request.degrees = stoi(tokens[1]);
    if (spin_rotary_client.call(spin_rotary_srv)) {
      ROS_INFO("Commanded arm to spin spigot valve");
    } else {
      ROS_ERROR("Failed to command arm to spin spigot valve");
      return 1;
    }
  } else if (tokens[0] == "V3") {
    // Query device
    if (query_shuttlecock_client.call(query_shuttlecock_srv)) {
      ROS_INFO("Successfully queried shuttlecock");
    } else {
      ROS_ERROR("Failed to query shuttlecock");
      return 1;
    }

    if (!query_shuttlecock_srv.response.state.visible) {
      cout << "Couldn't find shuttlecock valve" << endl;
      return 1; // TODO: don't return 
    }

    if (tokens[1] == "0" && query_shuttlecock_srv.response.state.open) {
      cout << "Shuttlecock already open, doing nothing" << endl;
      return 0; // TODO: don't return 
    }
    if (tokens[1] == "1" && !query_shuttlecock_srv.response.state.open) {
      cout << "Shuttlecock already closed, doing nothing" << endl;
      return 0; // TODO: don't return 
    }

    // Transform position into arm frame
    Vector3d dev_pos(query_shuttlecock_srv.response.state.position.x,
                     query_shuttlecock_srv.response.state.position.y,
                     query_shuttlecock_srv.response.state.position.z);
    dev_pos = R_cam_arm*dev_pos + t_cam_arm;

    cout << "Shuttlecock isn't fully implemented so we're returning" << endl;
    return 0; // TODO: don't return 

    // Command arm
    spin_shuttlecock_srv.request.position.x = dev_pos(0);
    spin_shuttlecock_srv.request.position.y = dev_pos(1);
    spin_shuttlecock_srv.request.position.z = dev_pos(2);
    spin_shuttlecock_srv.request.vertical_spin_axis = query_shuttlecock_srv.response.state.vertical;
    // TODO: other shuttlecock stuff

    if (spin_shuttlecock_client.call(spin_shuttlecock_srv)) {
      ROS_INFO("Commanded arm to spin shuttlecock valve");
    } else {
      ROS_ERROR("Failed to command arm to spin shuttlecock valve");
      return 1;
    }
  } else if (tokens[0] == "A" || tokens[0] == "B") {
    // Query device
    if (query_breaker_client.call(query_breaker_srv)) {
      ROS_INFO("Successfully queried breaker");
    } else {
      ROS_ERROR("Failed to query breaker");
      return 1;
    }

    if (!query_breaker_srv.response.state1.visible) {
      cout << "Couldn't find middle breaker switch" << endl;
      return 1; // TODO: don't return 
    }
    if (!query_breaker_srv.response.state2.visible) {
      cout << "Couldn't find middle breaker switch" << endl;
      return 1; // TODO: don't return 
    }
    if (!query_breaker_srv.response.state3.visible) {
      cout << "Couldn't find rightmost breaker switch" << endl;
      return 1; // TODO: don't return 
    }

    shipbot_ros::SwitchState state;
    if (tokens[1] == "B1") {
      state = query_breaker_srv.response.state1;
    } else if (tokens[1] == "B2") {
      state = query_breaker_srv.response.state2;
    } else if (tokens[1] == "B3") {
      state = query_breaker_srv.response.state3;
    }

    if (tokens[2] == "U" && state.up) {
      cout << "Switch already up, doing nothing" << endl;
      return 0; // TODO: don't return 
    }
    if (tokens[2] == "D" && !state.up) {
      cout << "Switch already down, doing nothing" << endl;
      return 0; // TODO: don't return 
    }

    // Transform position into arm frame
    Vector3d dev_pos(state.position.x,
                     state.position.y,
                     state.position.z);
    dev_pos = R_cam_arm*dev_pos + t_cam_arm;

    // Command arm
    switch_breaker_srv.request.position.x = dev_pos(0);
    switch_breaker_srv.request.position.y = dev_pos(1);
    switch_breaker_srv.request.position.z = dev_pos(2);
    switch_breaker_srv.request.push_up = tokens[2] == "U";

    if (switch_breaker_client.call(switch_breaker_srv)) {
      ROS_INFO("Commanded arm to switch a breaker switch");
    } else {
      ROS_ERROR("Failed to command arm to switch a breaker switch");
      return 1;
    }
  }
  
  /*
  ros::Rate r(20);
  while (ros::ok()) {

    r.sleep();
    ros::spinOnce();
  }
  */
}

