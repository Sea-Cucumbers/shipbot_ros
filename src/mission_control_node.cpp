#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "shipbot_ros/SpinRotary.h"
#include "shipbot_ros/SpinShuttlecock.h"
#include "shipbot_ros/SwitchBreaker.h"
#include "shipbot_ros/ResetArm.h"
#include "shipbot_ros/QueryWheel.h"
#include "shipbot_ros/QuerySpigot.h"
#include "shipbot_ros/QueryShuttlecock.h"
#include "shipbot_ros/QueryBreaker.h"
#include "shipbot_ros/InitialLocalization.h"
#include "shipbot_ros/TravelCL.h"
#include "shipbot_ros/TravelOL.h"
#include "shipbot_ros/StopChassis.h"
#include "shipbot_ros/ChassisDone.h"
#include "shipbot_ros/ArmDone.h"
#include "shipbot_ros/ChassisState.h"
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

void wait_for_completion(ros::Rate &r, shared_ptr<bool> done_ptr) {
  while (ros::ok() && !(*done_ptr)) {
    r.sleep();
    ros::spinOnce();
  }
  *done_ptr = false;
}

class chassis_done {
  private:
    shared_ptr<bool> done_ptr;

  public:
    /*
     * chassis_done: constructor
     * ARGUMENTS
     * _stopped: pointer to done variable
     */
     chassis_done(shared_ptr<bool> _done_ptr) : done_ptr(_done_ptr) {}

    /*
     * operator (): realizes that the chassis is done with its current task
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::ChassisDone::Request &req,
                      shipbot_ros::ChassisDone::Response &res) {
      *done_ptr = true;
      return true;
    }
};

class arm_done {
  private:
    shared_ptr<bool> done_ptr;

  public:
    /*
     * arm_done: constructor
     * ARGUMENTS
     * _stopped: pointer to done variable
     */
     arm_done(shared_ptr<bool> _done_ptr) : done_ptr(_done_ptr) {}

    /*
     * operator (): realizes that the arm is done with its current task
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::ArmDone::Request &req,
                      shipbot_ros::ArmDone::Response &res) {
      *done_ptr = true;
      return true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_control_node");
  ros::NodeHandle nh("~");

  // While testing, we might only try manipulation, or only locomotion.
  // Figure out which ones we're doing
  bool do_locomotion = false;
  bool do_manipulation = false;
  nh.getParam("do_locomotion", do_locomotion);
  nh.getParam("do_manipulation", do_manipulation);

  // Get extrinsics between arm and camera
  vector<double> t_cam_armv;
  vector<double> R_cam_armv;
  nh.getParam("/t_cam_arm", t_cam_armv);
  nh.getParam("/R_cam_arm", R_cam_armv);

  Matrix3d R_cam_arm;
  R_cam_arm << R_cam_armv[0], R_cam_armv[1], R_cam_armv[2],
               R_cam_armv[3], R_cam_armv[4], R_cam_armv[5],
               R_cam_armv[6], R_cam_armv[7], R_cam_armv[8];
  Vector3d t_cam_arm(t_cam_armv[0], t_cam_armv[1], t_cam_armv[2]);

  // Get station locations
  vector<double> locA;
  vector<double> locB;
  vector<double> locC;
  vector<double> locD;
  vector<double> locE;
  vector<double> locF;
  vector<double> locG;
  vector<double> locH;
  nh.getParam("/locA", locA);
  nh.getParam("/locB", locB);
  nh.getParam("/locC", locC);
  nh.getParam("/locD", locD);
  nh.getParam("/locE", locE);
  nh.getParam("/locF", locF);
  nh.getParam("/locG", locG);
  nh.getParam("/locH", locH);

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

  if (do_manipulation) {
    // Wait for vision services
    ros::service::waitForService("/realsense_node/query_wheel", -1);
    ros::service::waitForService("/realsense_node/query_spigot", -1);
    ros::service::waitForService("/realsense_node/query_shuttlecock", -1);
    ros::service::waitForService("/realsense_node/query_breaker", -1);

    // Wait for arm services
    ros::service::waitForService("/arm_control_node/spin_rotary", -1);
    ros::service::waitForService("/arm_control_node/spin_shuttlecock", -1);
    ros::service::waitForService("/arm_control_node/switch_breaker", -1);
    ros::service::waitForService("/arm_control_node/reset_arm", -1);
  }


  if (do_locomotion) {
    // Wait for chassis services
    ros::service::waitForService("/chassis_control_node/stop_chassis", -1);
    ros::service::waitForService("/chassis_control_node/travel_cl", -1);
    ros::service::waitForService("/chassis_control_node/travel_ol", -1);
    ros::service::waitForService("/chassis_control_node/localize", -1);
  }

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
  ros::ServiceClient spin_rotary_client = nh.serviceClient<shipbot_ros::SpinRotary>("/arm_control_node/spin_rotary");
  ros::ServiceClient spin_shuttlecock_client = nh.serviceClient<shipbot_ros::SpinShuttlecock>("/arm_control_node/spin_shuttlecock");
  ros::ServiceClient switch_breaker_client = nh.serviceClient<shipbot_ros::SwitchBreaker>("/arm_control_node/switch_breaker");
  ros::ServiceClient reset_arm_client = nh.serviceClient<shipbot_ros::ResetArm>("/arm_control_node/reset_arm");
  shipbot_ros::SpinRotary spin_rotary_srv;
  shipbot_ros::SpinShuttlecock spin_shuttlecock_srv;
  shipbot_ros::SwitchBreaker switch_breaker_srv;
  shipbot_ros::ResetArm reset_arm_srv;

  // Initialize chassis service clients and srvs
  ros::ServiceClient stop_chassis_client = nh.serviceClient<shipbot_ros::StopChassis>("/chassis_control_node/stop_chassis");
  ros::ServiceClient travel_cl_client = nh.serviceClient<shipbot_ros::TravelCL>("/chassis_control_node/travel_cl");
  ros::ServiceClient travel_ol_client = nh.serviceClient<shipbot_ros::TravelOL>("/chassis_control_node/travel_ol");
  ros::ServiceClient localize_client = nh.serviceClient<shipbot_ros::InitialLocalization>("/chassis_control_node/localize");

  shipbot_ros::StopChassis stop_chassis_srv;
  shipbot_ros::TravelCL travel_cl_srv;
  shipbot_ros::TravelOL travel_ol_srv;
  shipbot_ros::InitialLocalization localize_srv;

  // Advertise services for the chassis and arm to call when they're done whatever they're doing
  shared_ptr<bool> chassis_done_ptr = make_shared<bool>(false);
  ros::ServiceServer chassis_done_service = nh.advertiseService<shipbot_ros::ChassisDone::Request, shipbot_ros::ChassisDone::Response>("chassis_done", chassis_done(chassis_done_ptr));

  shared_ptr<bool> arm_done_ptr = make_shared<bool>(false);
  ros::ServiceServer arm_done_service = nh.advertiseService<shipbot_ros::ArmDone::Request, shipbot_ros::ArmDone::Response>("arm_done", arm_done(arm_done_ptr));

  // Phew, setup done
  ros::Rate r(20);

  // INITIAL LOCALIZATION PHASE

  if (do_manipulation) {
    // Reset the arm
    if (reset_arm_client.call(reset_arm_srv)) {
      ROS_INFO("Commanded arm to reset");
    } else {
      ROS_ERROR("Failed to command arm to reset");
      return 1;
    }
    wait_for_completion(r, arm_done_ptr);
  }
  
  if (do_locomotion) {
    // Wait for localization to initialize
    wait_for_completion(r, chassis_done_ptr);

    // Localize
    if (localize_client.call(localize_srv)) {
      ROS_INFO("Commanded chassis to perform localization routine");
    } else {
      ROS_ERROR("Failed to command chassis to perform localization routine");
      return 1;
    }
    wait_for_completion(r, chassis_done_ptr);
  }

  for (int cmd = 0; cmd < commands.size(); ++cmd) {
    char station = commands[cmd][0];

    // LOCOMOTION PHASE
    if (do_locomotion) {
      // Travel to station 
      vector<double> station_loc;
      double station_theta;
      switch (station) {
        case 'A':
          station_loc = locA;
          station_theta = -M_PI/2;
          break;
        case 'B':
          station_loc = locB;
          station_theta = -M_PI/2;
          break;
        case 'C':
          station_loc = locC;
          station_theta = -M_PI/2;
          break;
        case 'D':
          station_loc = locD;
          station_theta = -M_PI/2;
          break;
        case 'E':
          station_loc = locE;
          station_theta = -M_PI/2;
          break;
        case 'F':
          station_loc = locF;
          station_theta = M_PI;
          break;
        case 'G':
          station_loc = locG;
          station_theta = M_PI;
          break;
        case 'H':
          station_loc = locH;
          station_theta = M_PI;
          break;
      }

      travel_cl_srv.request.x = station_loc[0];
      travel_cl_srv.request.y = station_loc[1];
      travel_cl_srv.request.theta = station_theta;
      if (travel_cl_client.call(travel_cl_srv)) {
        ROS_INFO("Commanded chassis to travel to station");
      } else {
        ROS_ERROR("Failed to command chassis to travel to station");
        return 1;
      }
      wait_for_completion(r, chassis_done_ptr);
    }

    // MANIPULATION PHASE
    string rest = commands[cmd].substr(1, commands[cmd].size());
    vector<string> tokens;
    string_split(tokens, rest, " ");

    if (do_manipulation) {
      if (tokens[0] == "V1") {
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
        wait_for_completion(r, arm_done_ptr);
      } else if (tokens[0] == "V2") {
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
        wait_for_completion(r, arm_done_ptr);
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
        } else if (tokens[1] == "1" && !query_shuttlecock_srv.response.state.open) {
          cout << "Shuttlecock already closed, doing nothing" << endl;
        } else {
          // Transform position into arm frame
          Vector3d dev_pos(query_shuttlecock_srv.response.state.position.x,
                           query_shuttlecock_srv.response.state.position.y,
                           query_shuttlecock_srv.response.state.position.z);
          dev_pos = R_cam_arm*dev_pos + t_cam_arm;

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
          wait_for_completion(r, arm_done_ptr);
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
        } else if (tokens[2] == "D" && !state.up) {
          cout << "Switch already down, doing nothing" << endl;
        } else {
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
          wait_for_completion(r, arm_done_ptr);
        }
      }

      // Reset the arm
      if (reset_arm_client.call(reset_arm_srv)) {
        ROS_INFO("Commanded arm to reset");
      } else {
        ROS_ERROR("Failed to command arm to reset");
        return 1;
      }
      wait_for_completion(r, arm_done_ptr);
    }
  }  
}

