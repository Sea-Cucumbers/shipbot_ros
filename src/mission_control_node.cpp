#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include "shipbot_ros/SpinRotary.h"
#include "shipbot_ros/SpinShuttlecock.h"
#include "shipbot_ros/SwitchBreaker.h"
#include "shipbot_ros/WheelState.h"
#include "shipbot_ros/SpigotState.h"
#include "shipbot_ros/BreakerState.h"
#include "shipbot_ros/SwitchState.h"
#include "shipbot_ros/ShuttlecockState.h"
#include "shipbot_ros/FindDevice.h"
#include "shipbot_ros/TravelAbs.h"
#include "shipbot_ros/TravelRel.h"
#include "shipbot_ros/ChassisState.h"
#include <std_srvs/Empty.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

static const double xee = -0.129243;
static const double centering_offset = -0.035;

/*
 * string_split: splits the string s into a vector of substrings separated
 * by delimiter del
 * ARGUMENTS
 * ssplit: populated with substrings
 * s: string to split
 * del: delimiter
 */
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

/*
 * spin_until_completion: spin until done_ptr points to true
 * ARGUMENTS
 * r: rate we use to sleep while spinning
 * done_ptr: spin until this points to true
 */
void spin_until_completion(ros::Rate &r, shared_ptr<bool> done_ptr) {
  while (ros::ok() && !(*done_ptr)) {
    r.sleep();
    ros::spinOnce();
  }
  *done_ptr = false;
}

/*
 * minor_strafe: translate the robot left or right, from the perspective of the camera,
 * and spin until the chassis is done
 * ARGUMENTS
 * delta: how much to translate
 * travel_rel_client: relative control client
 * r: rate we use to sleep while spinning
 * chassis_done_ptr: pointer we check to see if the chassis is done
 */
void minor_strafe(double delta, ros::ServiceClient &travel_rel_client, ros::Rate &r, shared_ptr<bool> chassis_done_ptr) {
  shipbot_ros::TravelRel travel_rel_srv;
  travel_rel_srv.request.delta_x = 0;
  travel_rel_srv.request.delta_y = -delta;
  travel_rel_srv.request.delta_theta = 0;

  if (travel_rel_client.call(travel_rel_srv)) {
    ROS_INFO("Commanded chassis to center arm with device");
  } else {
    ROS_ERROR("Failed to command chassis to center arm with device");
  }
  spin_until_completion(r, chassis_done_ptr);
}

class handle_done {
  private:
    shared_ptr<bool> done_ptr;

  public:
    /*
     * handle_done: constructor
     * ARGUMENTS
     * _stopped: pointer to done variable
     */
     handle_done(shared_ptr<bool> _done_ptr) : done_ptr(_done_ptr) {}

    /*
     * operator (): sets done_ptr to true
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      *done_ptr = true;
      return true;
    }
};

class start_mission {
  private:
    shared_ptr<bool> start_ptr;

  public:
    /*
     * start_mission: constructor
     * ARGUMENTS
     * _start_ptr: pointer to start variable
     */
     start_mission(shared_ptr<bool> _start_ptr) : start_ptr(_start_ptr) {}

    /*
     * operator (): realizes that we should start the mission
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      *start_ptr = true;
      return true;
    }
};

class stop_mission {
  private:
    ros::ServiceClient *stop_chassis_client;
    ros::ServiceClient *stop_arm_client;

  public:
    /*
     * stop_mission: constructor
     * ARGUMENTS
     * _start_ptr: pointer to start variable
     */
     stop_mission(ros::ServiceClient *stop_chassis_client,
                  ros::ServiceClient *stop_arm_client) : stop_chassis_client(stop_chassis_client),
                                                         stop_arm_client(stop_arm_client) {}

    /*
     * operator (): realizes that we should stop the mission
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      std_srvs::Empty empty_srv;
      stop_chassis_client->call(empty_srv);
      stop_arm_client->call(empty_srv);
      ros::spin();
      return true;
    }
};

/*
 * handle_wheel: wheel state message handler
 */
class handle_wheel {
  private:
    shared_ptr<shipbot_ros::WheelState> state_ptr;

  public:
    /*
     * handle_wheel: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     */
    handle_wheel(shared_ptr<shipbot_ros::WheelState> _state_ptr) : state_ptr(_state_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::WheelStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
    }
};

/*
 * handle_spigot: spigot state message handler
 */
class handle_spigot {
  private:
    shared_ptr<shipbot_ros::SpigotState> state_ptr;

  public:
    /*
     * handle_spigot: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     */
    handle_spigot(shared_ptr<shipbot_ros::SpigotState> _state_ptr) : state_ptr(_state_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::SpigotStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
    }
};

/*
 * handle_breaker: breaker state message handler
 */
class handle_breaker {
  private:
    shared_ptr<shipbot_ros::BreakerState> state_ptr;

  public:
    /*
     * handle_breaker: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     */
    handle_breaker(shared_ptr<shipbot_ros::BreakerState> _state_ptr) : state_ptr(_state_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::BreakerStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
    }
};

/*
 * handle_shuttlecock: shuttlecock state message handler
 */
class handle_shuttlecock {
  private:
    shared_ptr<shipbot_ros::ShuttlecockState> state_ptr;

  public:
    /*
     * handle_shuttlecock: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     */
    handle_shuttlecock(shared_ptr<shipbot_ros::ShuttlecockState> _state_ptr) : state_ptr(_state_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::ShuttlecockStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_control_node");
  ros::NodeHandle nh("~");
  ros::Rate r(20);

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

  // This service starts the mission
  shared_ptr<bool> start_ptr = make_shared<bool>(false);
  ros::ServiceServer start_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("start_mission", start_mission(start_ptr));
  spin_until_completion(r, start_ptr);

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
  sort(commands.begin(), commands.end());
  for (vector<string>::iterator it = commands.begin(); it != commands.end(); ++it) {
    cout << *it << endl;
  }

  double target_time = stod(commands.back());
  commands.pop_back();

  if (do_manipulation) {
    // Wait for vision services
    ros::service::waitForService("/realsense_node/find_device", -1);

    // Wait for arm services
    ros::service::waitForService("/arm_control_node/spin_rotary", -1);
    ros::service::waitForService("/arm_control_node/spin_shuttlecock", -1);
    ros::service::waitForService("/arm_control_node/switch_breaker", -1);
    ros::service::waitForService("/arm_control_node/reset_arm", -1);
  }


  // Wait for chassis services
  ros::service::waitForService("/chassis_control_node/stop_chassis", -1);
  ros::service::waitForService("/chassis_control_node/travel_abs", -1);
  ros::service::waitForService("/chassis_control_node/travel_rel", -1);
  ros::service::waitForService("/chassis_control_node/localize", -1);

  // Initialize vision service clients and srvs
  ros::ServiceClient find_device_client = nh.serviceClient<shipbot_ros::FindDevice>("/realsense_node/find_device");
  shipbot_ros::FindDevice find_device_srv;

  // Initialize vision subscribers
  shared_ptr<shipbot_ros::WheelState> wheel_ptr = make_shared<shipbot_ros::WheelState>();
  shared_ptr<shipbot_ros::SpigotState> spigot_ptr = make_shared<shipbot_ros::SpigotState>();
  shared_ptr<shipbot_ros::BreakerState> breaker_ptr = make_shared<shipbot_ros::BreakerState>();
  breaker_ptr->switches = vector<shipbot_ros::SwitchState>(3);
  shared_ptr<shipbot_ros::ShuttlecockState> shuttlecock_ptr = make_shared<shipbot_ros::ShuttlecockState>();
  ros::Subscriber wheel_sub = nh.subscribe<shipbot_ros::WheelState>("/shipbot/wheel_state", 1, handle_wheel(wheel_ptr));
  ros::Subscriber spigot_sub = nh.subscribe<shipbot_ros::SpigotState>("/shipbot/spigot_state", 1, handle_spigot(spigot_ptr));
  ros::Subscriber shuttlecock_sub = nh.subscribe<shipbot_ros::ShuttlecockState>("/shipbot/shuttlecock_state", 1, handle_shuttlecock(shuttlecock_ptr));
  ros::Subscriber breaker_sub = nh.subscribe<shipbot_ros::BreakerState>("/shipbot/breaker_state", 1, handle_breaker(breaker_ptr));

  // Initialize arm service clients and srvs
  ros::ServiceClient spin_rotary_client = nh.serviceClient<shipbot_ros::SpinRotary>("/arm_control_node/spin_rotary");
  ros::ServiceClient spin_shuttlecock_client = nh.serviceClient<shipbot_ros::SpinShuttlecock>("/arm_control_node/spin_shuttlecock");
  ros::ServiceClient switch_breaker_client = nh.serviceClient<shipbot_ros::SwitchBreaker>("/arm_control_node/switch_breaker");
  ros::ServiceClient reset_arm_client = nh.serviceClient<std_srvs::Empty>("/arm_control_node/reset_arm");
  ros::ServiceClient stop_arm_client = nh.serviceClient<std_srvs::Empty>("/arm_control_node/stop_arm");
  shipbot_ros::SpinRotary spin_rotary_srv;
  shipbot_ros::SpinShuttlecock spin_shuttlecock_srv;
  shipbot_ros::SwitchBreaker switch_breaker_srv;
  std_srvs::Empty reset_arm_srv;

  // Initialize chassis service clients and srvs
  ros::ServiceClient stop_chassis_client = nh.serviceClient<std_srvs::Empty>("/chassis_control_node/stop_chassis");
  ros::ServiceClient travel_abs_client = nh.serviceClient<shipbot_ros::TravelAbs>("/chassis_control_node/travel_abs");
  ros::ServiceClient travel_rel_client = nh.serviceClient<shipbot_ros::TravelRel>("/chassis_control_node/travel_rel");
  ros::ServiceClient localize_client = nh.serviceClient<std_srvs::Empty>("/chassis_control_node/localize");

  std_srvs::Empty stop_chassis_srv;
  shipbot_ros::TravelAbs travel_abs_srv;
  shipbot_ros::TravelRel travel_rel_srv;
  std_srvs::Empty localize_srv;

  // Advertise services for the chassis and arm to call when they're done whatever they're doing
  shared_ptr<bool> chassis_done_ptr = make_shared<bool>(false);
  ros::ServiceServer chassis_done_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("chassis_done", handle_done(chassis_done_ptr));

  shared_ptr<bool> loc_prep_done_ptr = make_shared<bool>(false);
  ros::ServiceServer loc_prep_done_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("loc_prep_done", handle_done(loc_prep_done_ptr));

  shared_ptr<bool> arm_done_ptr = make_shared<bool>(false);
  ros::ServiceServer arm_done_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("arm_done", handle_done(arm_done_ptr));

  // This service aborts the mission
  ros::ServiceServer stop_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("stop_mission", stop_mission(&stop_chassis_client, &stop_arm_client));

  // INITIAL LOCALIZATION PHASE

  if (do_manipulation) {
    // Reset the arm
    if (reset_arm_client.call(reset_arm_srv)) {
      ROS_INFO("Commanded arm to reset");
    } else {
      ROS_ERROR("Failed to command arm to reset");
    }
    spin_until_completion(r, arm_done_ptr);

    find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
    if (find_device_client.call(find_device_srv)) {
      ROS_INFO("Told vision node to find nothing");
    } else {
      ROS_ERROR("Failed to tell vision node to find nothing");
    }
  }
  
  if (do_locomotion) {
    // Wait for localization to initialize
    spin_until_completion(r, loc_prep_done_ptr);

    // Localize
    if (localize_client.call(localize_srv)) {
      ROS_INFO("Commanded chassis to perform localization routine");
    } else {
      ROS_ERROR("Failed to command chassis to perform localization routine");
    }
    spin_until_completion(r, chassis_done_ptr);
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

      travel_abs_srv.request.x = station_loc[0];
      travel_abs_srv.request.y = station_loc[1];
      travel_abs_srv.request.theta = station_theta;
      if (travel_abs_client.call(travel_abs_srv)) {
        ROS_INFO("Commanded chassis to travel to station");
      } else {
        ROS_ERROR("Failed to command chassis to travel to station");
      }
      spin_until_completion(r, chassis_done_ptr);
    }

    // MANIPULATION PHASE
    string rest = commands[cmd].substr(1, commands[cmd].size());
    vector<string> tokens;
    string_split(tokens, rest, " ");

    if (do_manipulation) {
      if (tokens[0] == "V1") {
        // Query device
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::SPIGOT;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Successfully called find_device");
        } else {
          ROS_ERROR("Failed to call find_device");
        }

        double start_time = ros::Time::now().toSec();
        while (!spigot_ptr->visible && ros::ok() && ros::Time::now().toSec() - start_time < 20) {
          cout << "Couldn't find spigot valve" << endl;
          r.sleep();
          ros::spinOnce();
        }

        cout << "Found device" << endl;

        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Told vision node to find nothing");
        } else {
          ROS_ERROR("Failed to tell vision node to find nothing");
        }

        // Transform position into arm frame
        Vector3d dev_pos(spigot_ptr->position.x,
                         spigot_ptr->position.y,
                         spigot_ptr->position.z);
        dev_pos = R_cam_arm*dev_pos + t_cam_arm;

        cout << "device is " << dev_pos(0) << " m in the x from the base" << endl;
        minor_strafe(dev_pos(0) - xee + centering_offset, travel_rel_client, r, chassis_done_ptr);
        dev_pos(0) = xee; // TODO: this is a very strong assumption!

        // Command arm
        spin_rotary_srv.request.position.x = dev_pos(0);
        spin_rotary_srv.request.position.y = dev_pos(1);
        spin_rotary_srv.request.position.z = dev_pos(2);
        spin_rotary_srv.request.vertical_spin_axis = spigot_ptr->vertical;
        spin_rotary_srv.request.degrees = stoi(tokens[1]);
        if (spin_rotary_client.call(spin_rotary_srv)) {
          ROS_INFO("Commanded arm to spin spigot valve");
        } else {
          ROS_ERROR("Failed to command arm to spin spigot valve");
        }
        spin_until_completion(r, arm_done_ptr);
      } else if (tokens[0] == "V2") {
        // Query device
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::WHEEL;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Successfully called find_device");
        } else {
          ROS_ERROR("Failed to call find_device");
        }

        double start_time = ros::Time::now().toSec();
        while (!wheel_ptr->visible && ros::ok() && ros::Time::now().toSec() - start_time < 20) {
          cout << "Couldn't find wheel valve" << endl;
          r.sleep();
          ros::spinOnce();
        }

        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Told vision node to find nothing");
        } else {
          ROS_ERROR("Failed to tell vision node to find nothing");
        }

        // Transform position into arm frame
        Vector3d dev_pos(wheel_ptr->position.x,
                         wheel_ptr->position.y,
                         wheel_ptr->position.z);

        dev_pos = R_cam_arm*dev_pos + t_cam_arm;

        cout << "device is " << dev_pos(0) << " m in the x from the base" << endl;

        if (station != 'E' && station != 'F') {
          minor_strafe(dev_pos(0) - xee + centering_offset, travel_rel_client, r, chassis_done_ptr);
          dev_pos(0) = xee; // TODO: this is a very strong assumption!
        }

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
        }
        spin_until_completion(r, arm_done_ptr);
      } else if (tokens[0] == "V3") {
        // Query device
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::SHUTTLECOCK;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Successfully called find_device");
        } else {
          ROS_ERROR("Failed to call find_device");
        }

        double start_time = ros::Time::now().toSec();
        while (!shuttlecock_ptr->visible && ros::ok() && ros::Time::now().toSec() - start_time < 20) {
          cout << "Couldn't find shuttlecock valve" << endl;
          r.sleep();
          ros::spinOnce();
        }

        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Told vision node to find nothing");
        } else {
          ROS_ERROR("Failed to tell vision node to find nothing");
        }

        if (tokens[1] == "0" && shuttlecock_ptr->open) {
          cout << "Shuttlecock already open, doing nothing" << endl;
        } else if (tokens[1] == "1" && !shuttlecock_ptr->open) {
          cout << "Shuttlecock already closed, doing nothing" << endl;
        } else {
          // Transform position into arm frame
          Vector3d dev_pos(shuttlecock_ptr->position.x,
                           shuttlecock_ptr->position.y,
                           shuttlecock_ptr->position.z);
          dev_pos = R_cam_arm*dev_pos + t_cam_arm;

          cout << "device is " << dev_pos(0) << " m in the x from the base" << endl;

          if (station != 'E' && station != 'F') {
            minor_strafe(dev_pos(0) - xee + centering_offset, travel_rel_client, r, chassis_done_ptr);
            dev_pos(0) = xee; // TODO: this is a very strong assumption!
          }

          // Command arm
          spin_shuttlecock_srv.request.position.x = dev_pos(0);
          spin_shuttlecock_srv.request.position.y = dev_pos(1);
          spin_shuttlecock_srv.request.position.z = dev_pos(2);
          spin_shuttlecock_srv.request.vertical_spin_axis = shuttlecock_ptr->vertical;
          spin_shuttlecock_srv.request.do_open = tokens[1] == "0";

          if (spin_shuttlecock_client.call(spin_shuttlecock_srv)) {
            ROS_INFO("Commanded arm to spin shuttlecock valve");
          } else {
            ROS_ERROR("Failed to command arm to spin shuttlecock valve");
          }
          spin_until_completion(r, arm_done_ptr);
        }
      } else if (tokens[0] == "A" || tokens[0] == "B") {
        // Query device
        find_device_srv.request.device_type = (tokens[0] == "A") ? shipbot_ros::FindDevice::Request::BREAKERA : shipbot_ros::FindDevice::Request::BREAKERB;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Successfully called find_device");
        } else {
          ROS_ERROR("Failed to call find_device");
        }

        double start_time = ros::Time::now().toSec();
        while ((!breaker_ptr->switches[0].visible ||
                !breaker_ptr->switches[1].visible ||
                !breaker_ptr->switches[2].visible) && ros::ok()
                && ros::Time::now().toSec() - start_time < 20) {
          cout << "Couldn't find all the breaker switches" << endl;
          r.sleep();
          ros::spinOnce();
        }

        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
        if (find_device_client.call(find_device_srv)) {
          ROS_INFO("Told vision node to find nothing");
        } else {
          ROS_ERROR("Failed to tell vision node to find nothing");
        }

        shipbot_ros::SwitchState state;
        if (tokens[1] == "B1") {
          state = breaker_ptr->switches[0];
        } else if (tokens[1] == "B2") {
          state = breaker_ptr->switches[1];
        } else if (tokens[1] == "B3") {
          state = breaker_ptr->switches[2];
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

          cout << "device is " << dev_pos(0) << " m in the x from the base" << endl;

          if (station != 'E' && station != 'F') {
            minor_strafe(dev_pos(0) - xee + centering_offset, travel_rel_client, r, chassis_done_ptr);
            dev_pos(0) = xee; // TODO: this is a very strong assumption!
          }

          // Command arm
          switch_breaker_srv.request.position.x = dev_pos(0);
          switch_breaker_srv.request.position.y = dev_pos(1);
          switch_breaker_srv.request.position.z = dev_pos(2);
          switch_breaker_srv.request.push_up = tokens[2] == "U";

          if (switch_breaker_client.call(switch_breaker_srv)) {
            ROS_INFO("Commanded arm to switch a breaker switch");
          } else {
            ROS_ERROR("Failed to command arm to switch a breaker switch");
          }
          spin_until_completion(r, arm_done_ptr);
        }
      }

      // Reset the arm
      if (reset_arm_client.call(reset_arm_srv)) {
        ROS_INFO("Commanded arm to reset");
      } else {
        ROS_ERROR("Failed to command arm to reset");
      }
      spin_until_completion(r, arm_done_ptr);
    }
  }  

  ros::spin();
}

