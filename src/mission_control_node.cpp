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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace Eigen;

static const double xee = -0.129243;

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

/*
 * handle_chassis_state: chassis state message handler
 */
class handle_chassis_state {
  private:
    shared_ptr<shipbot_ros::ChassisState> state_ptr;

  public:
    /*
     * handle_chassis_state: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     */
    handle_chassis_state(shared_ptr<shipbot_ros::ChassisState> _state_ptr) : state_ptr(_state_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::ChassisStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_control_node");
  ros::NodeHandle nh("~");
  ros::Rate r(20);

  // While testing, we might only try manipulation, or only locomotion.
  // Figure out which ones we're doing
  bool do_localization_routine = false;
  bool do_locomotion = false;
  bool do_manipulation = false;
  nh.getParam("do_localization_routine", do_localization_routine);
  nh.getParam("do_locomotion", do_locomotion);
  nh.getParam("do_manipulation", do_manipulation);

  // Get extrinsics between arm and camera
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped cam_wrt_arm ;
  geometry_msgs::TransformStamped arm_wrt_chassis ;
  bool got_tfs = false;
  while (!got_tfs) {
    try {
      cam_wrt_arm = tf_buffer.lookupTransform("arm_base", "camera", ros::Time(0));
      arm_wrt_chassis = tf_buffer.lookupTransform("chassis", "arm_base", ros::Time(0));
      got_tfs = true;
    } catch (tf2::TransformException ex) {
    }
  }

  Matrix4d H_cam_arm = Matrix4d::Identity();
  H_cam_arm.topLeftCorner<3, 3>() = Quaterniond(cam_wrt_arm.transform.rotation.w,
                                                cam_wrt_arm.transform.rotation.x,
                                                cam_wrt_arm.transform.rotation.y,
                                                cam_wrt_arm.transform.rotation.z).toRotationMatrix();
  H_cam_arm.topRightCorner<3, 1>() = Vector3d(cam_wrt_arm.transform.translation.x, cam_wrt_arm.transform.translation.y, cam_wrt_arm.transform.translation.z);

  Matrix4d H_arm_chassis = Matrix4d::Identity();
  H_arm_chassis.topLeftCorner<3, 3>() = Quaterniond(arm_wrt_chassis.transform.rotation.w,
                                                    arm_wrt_chassis.transform.rotation.x,
                                                    arm_wrt_chassis.transform.rotation.y,
                                                    arm_wrt_chassis.transform.rotation.z).toRotationMatrix();
  H_arm_chassis.topRightCorner<3, 1>() = Vector3d(arm_wrt_chassis.transform.translation.x, arm_wrt_chassis.transform.translation.y, arm_wrt_chassis.transform.translation.z);

  // Get station locations
  vector<double> poseA;
  vector<double> poseB;
  vector<double> poseC;
  vector<double> poseD;
  vector<double> poseE;
  vector<double> poseF;
  vector<double> poseG;
  vector<double> poseH;
  nh.getParam("/poseA", poseA);
  nh.getParam("/poseB", poseB);
  nh.getParam("/poseC", poseC);
  nh.getParam("/poseD", poseD);
  nh.getParam("/poseE", poseE);
  nh.getParam("/poseF", poseF);
  nh.getParam("/poseG", poseG);
  nh.getParam("/poseH", poseH);

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
  for (vector<string>::iterator it = commands.begin(); it != commands.end(); ++it) {
    cout << *it << endl;
  }

  double target_time = stod(commands.back());
  commands.pop_back();

  sort(commands.begin(), commands.end());

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

  // Initialize chassis subscriber
  shared_ptr<shipbot_ros::ChassisState> chassis_state_ptr = make_shared<shipbot_ros::ChassisState>();
  ros::Subscriber chassis_state_sub = nh.subscribe<shipbot_ros::ChassisState>("/shipbot/chassis_state", 1, handle_chassis_state(chassis_state_ptr));

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

  // Wait for localization to initialize
  spin_until_completion(r, loc_prep_done_ptr);
  cout << "Localization is ready" << endl;
  
  if (do_localization_routine) {
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
      vector<double> station_pose;
      switch (station) {
        case 'A':
          station_pose = poseA;
          break;
        case 'B':
          station_pose = poseB;
          break;
        case 'C':
          station_pose = poseC;
          break;
        case 'D':
          station_pose = poseD;
          break;
        case 'E':
          station_pose = poseE;
          break;
        case 'F':
          station_pose = poseF;
          break;
        case 'G':
          station_pose = poseG;
          break;
        case 'H':
          station_pose = poseH;
          break;
        default:
          cout << "Bad mission file" << endl;
          exit(1);
      }

      travel_abs_srv.request.x = station_pose[0];
      travel_abs_srv.request.y = station_pose[1];
      travel_abs_srv.request.theta = station_pose[2];
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
      // Query device
      if (tokens[0] == "V1") {
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::SPIGOT;
      } else if (tokens[0] == "V2") {
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::WHEEL;
      } else if (tokens[0] == "V3") {
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::SHUTTLECOCK;
      } else if (tokens[0] == "A") {
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::BREAKERA;
      } else if (tokens[0] == "B") {
        find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::BREAKERB;
      } else {
        cout << "Bad mission file" << endl;
        exit(1);
      }
      if (find_device_client.call(find_device_srv)) {
        ROS_INFO("Successfully called find_device");
      } else {
        ROS_ERROR("Failed to call find_device");
      }

      // Wait until we find it
      double start_time = ros::Time::now().toSec();
      while (ros::ok() && ros::Time::now().toSec() - start_time < 20) {
        if ((tokens[0] == "V1" && spigot_ptr->visible) ||
            (tokens[0] == "V2" && wheel_ptr->visible) ||
            (tokens[0] == "V3" && shuttlecock_ptr->visible) ||
            ((tokens[0] == "A" || tokens[0] == "B") &&
             breaker_ptr->switches[0].visible &&
             breaker_ptr->switches[1].visible &&
             breaker_ptr->switches[2].visible)) {
          break;
        }
        cout << "Haven't found device yet" << endl;
        r.sleep();
        ros::spinOnce();
      }

      cout << "Found device" << endl;

      find_device_srv.request.device_type = shipbot_ros::FindDevice::Request::NONE;
      if (find_device_client.call(find_device_srv)) {
        ROS_INFO("Told vision node to pause");
      } else {
        ROS_ERROR("Failed to tell vision node to pause");
      }

      // Read device position
      Vector4d dev_pos;
      dev_pos(3) = 1;
      if (tokens[0] == "V1") {
        dev_pos(0) = spigot_ptr->position.x;
        dev_pos(1) = spigot_ptr->position.y;
        dev_pos(2) = spigot_ptr->position.z;
      } else if (tokens[0] == "V2") {
        dev_pos(0) = wheel_ptr->position.x;
        dev_pos(1) = wheel_ptr->position.y;
        dev_pos(2) = wheel_ptr->position.z;
      } else if (tokens[0] == "V3") {
        if (tokens[1] == "0" && shuttlecock_ptr->open) {
          cout << "Shuttlecock already open, doing nothing" << endl;
          continue;
        } else if (tokens[1] == "1" && !shuttlecock_ptr->open) {
          cout << "Shuttlecock already closed, doing nothing" << endl;
          continue;
        }

        dev_pos(0) = shuttlecock_ptr->position.x;
        dev_pos(1) = shuttlecock_ptr->position.y;
        dev_pos(2) = shuttlecock_ptr->position.z;
      } else if (tokens[0] == "A" || tokens[0] == "B") {
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
          continue;
        } else if (tokens[2] == "D" && !state.up) {
          cout << "Switch already down, doing nothing" << endl;
          continue;
        }

        dev_pos(0) = state.position.x;
        dev_pos(1) = state.position.y;
        dev_pos(2) = state.position.z;
      }

      // Transform device position into chassis frame and f
      // figure out how much to shift along the arm's x axis such that
      // the device is centered with the arm
      dev_pos = H_cam_arm*dev_pos;
      Vector4d shift = Vector4d::Zero();
      shift(0) = dev_pos(0) - xee;

      // Transform device position into world frame
      Matrix4d H_chassis_world = Matrix4d::Identity();
      H_chassis_world.topRightCorner<3, 1>() = Vector3d(chassis_state_ptr->x, chassis_state_ptr->y, 0);
      H_chassis_world.topLeftCorner<3, 3>() = AngleAxisd(chassis_state_ptr->yaw, Vector3d(0, 0, 1)).toRotationMatrix();
      dev_pos = H_chassis_world*H_arm_chassis*dev_pos;

      // If we can, translate the chassis to center the arm
      if (station != 'E' && station != 'F') {
        Vector4d goal = H_chassis_world.col(3) + H_chassis_world*H_arm_chassis*shift;

        travel_abs_srv.request.x = goal(0);
        travel_abs_srv.request.y = goal(1);
        travel_abs_srv.request.theta = chassis_state_ptr->yaw;
        if (travel_abs_client.call(travel_abs_srv)) {
          ROS_INFO("Commanded chassis to center the arm with the device");
        } else {
          ROS_ERROR("Failed to command chassis to center the arm with the device");
        }
        spin_until_completion(r, chassis_done_ptr);
      }

      // Now get the new device position in the arm frame
      H_chassis_world.topRightCorner<3, 1>() = Vector3d(chassis_state_ptr->x, chassis_state_ptr->y, 0);
      H_chassis_world.topLeftCorner<3, 3>() = AngleAxisd(chassis_state_ptr->yaw, Vector3d(0, 0, 1)).toRotationMatrix();
      dev_pos = H_arm_chassis.inverse()*H_chassis_world.inverse()*dev_pos;

      // Command arm
      if (tokens[0] == "V1") {
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
      } else if (tokens[0] == "V2") {
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
      } else if (tokens[0] == "V3") {
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
      } else if (tokens[0] == "A" || tokens[0] == "B") {
        switch_breaker_srv.request.position.x = dev_pos(0);
        switch_breaker_srv.request.position.y = dev_pos(1);
        switch_breaker_srv.request.position.z = dev_pos(2);
        switch_breaker_srv.request.push_up = tokens[2] == "U";

        if (switch_breaker_client.call(switch_breaker_srv)) {
          ROS_INFO("Commanded arm to switch a breaker switch");
        } else {
          ROS_ERROR("Failed to command arm to switch a breaker switch");
        }
      }

      spin_until_completion(r, arm_done_ptr);

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

