#include "kdHelper.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
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
#include "armPlanner.h"
#include "shipbot_ros/SpinRotary.h"
#include "shipbot_ros/SpinShuttlecock.h"
#include "shipbot_ros/SwitchBreaker.h"
#include <std_srvs/Empty.h>

using namespace std;

// TODO: shouldn't be static, got lazy
static visualization_msgs::Marker marker;
static double start_time;
static bool called_done;

class reset_arm {
  private:
    shared_ptr<ArmPlanner> planner;
    shared_ptr<VectorXd> task_space_config;

  public:
    /*
     * reset_arm: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _task_space_config: pointer to task-space configuration
     */
     reset_arm(shared_ptr<ArmPlanner> _planner,
               shared_ptr<VectorXd> _task_space_config) : planner(_planner),
                                                          task_space_config(_task_space_config) {}

    /*
     * operator (): plan trajectory to reset arm
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      planner->reset_arm(*task_space_config,
                         ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
      called_done = false;
      return true;
    }
};

class stop_arm {
  private:
    shared_ptr<ArmPlanner> planner;
    shared_ptr<VectorXd> task_space_config;

  public:
    /*
     * stop_arm: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _task_space_config: pointer to task-space configuration
     */
     stop_arm(shared_ptr<ArmPlanner> _planner,
               shared_ptr<VectorXd> _task_space_config) : planner(_planner),
                                                          task_space_config(_task_space_config) {}

    /*
     * operator (): stop the arm
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      planner->stop_arm(*task_space_config,
                         ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
      called_done = false;
      return true;
    }
};

class spin_rotary {
  private:
    shared_ptr<ArmPlanner> planner;
    shared_ptr<VectorXd> task_space_config;

  public:
    /*
     * spin_rotary: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _task_space_config: pointer to task-space configuration
     */
     spin_rotary(shared_ptr<ArmPlanner> _planner,
                 shared_ptr<VectorXd> _task_space_config) : planner(_planner),
                                                            task_space_config(_task_space_config) {}

    /*
     * operator (): plan trajectory to manipulate device
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::SpinRotary::Request &req,
                      shipbot_ros::SpinRotary::Response &res) {
      planner->spin_rotary(*task_space_config,
                           Vector3d(req.position.x, req.position.y, req.position.z),
                           req.vertical_spin_axis,
                           (double)req.degrees,
                           ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
      called_done = false;
      return true;
    }
};

class spin_shuttlecock {
  private:
    shared_ptr<ArmPlanner> planner;
    shared_ptr<VectorXd> task_space_config;

  public:
    /*
     * spin_shuttlecock: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _task_space_config: pointer to task-space configuration
     */
     spin_shuttlecock(shared_ptr<ArmPlanner> _planner,
                      shared_ptr<VectorXd> _task_space_config) : planner(_planner),
                                                                 task_space_config(_task_space_config) {}

    /*
     * operator (): plan trajectory to manipulate device
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::SpinShuttlecock::Request &req,
                      shipbot_ros::SpinShuttlecock::Response &res) {
      planner->spin_shuttlecock(*task_space_config,
                                Vector3d(req.position.x, req.position.y, req.position.z),
                                req.vertical_spin_axis,
                                req.do_open,
                                ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
      called_done = false;
      return true;
    }
};

class switch_breaker {
  private:
    shared_ptr<ArmPlanner> planner;
    shared_ptr<VectorXd> task_space_config;

  public:
    /*
     * switch_breaker: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _task_space_config: pointer to task-space configuration
     */
     switch_breaker(shared_ptr<ArmPlanner> _planner,
                    shared_ptr<VectorXd> _task_space_config) : planner(_planner),
                                                               task_space_config(_task_space_config) {}

    /*
     * operator (): plan trajectory to manipulate device
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::SwitchBreaker::Request &req,
                      shipbot_ros::SwitchBreaker::Response &res) {
      planner->switch_breaker(*task_space_config,
                              Vector3d(req.position.x, req.position.y, req.position.z),
                              req.push_up, req.num,
                              ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
      called_done = false;
      return true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_control_node");
  ros::NodeHandle nh("~");

  string urdf_file;
  double rate = 20;
  double seconds_per_meter = 6;
  double seconds_per_degree = 0.011;
  double horizontal_pause_back = 0.1;
  double vertical_pause_back = 0.1;
  double vertical_pause_above = 0.1;
  double pause_side = 0.0254;
  double grip_delay = 5;
  double press_delay = 2;
  double shuttlecock_force_h;
  double shuttlecock_force_v;
  double breaker_force_up;
  double breaker_force_down;
  double rotary_force_h;
  double rotary_force_v;
  vector<double> reset_configv;
  reset_configv.push_back(-0.135214);
  reset_configv.push_back(0.18142);
  reset_configv.push_back(0.05839807);
  reset_configv.push_back(-1.52432);
  reset_configv.push_back(2.67237);
  vector<double> encoder_offsetsv(5, 0);
  
  nh.getParam("urdf", urdf_file);
  nh.getParam("rate", rate);
  nh.getParam("seconds_per_meter", seconds_per_meter);
  nh.getParam("seconds_per_degree", seconds_per_degree);
  nh.getParam("horizontal_pause_back", horizontal_pause_back);
  nh.getParam("vertical_pause_back", vertical_pause_back);
  nh.getParam("vertical_pause_above", vertical_pause_above);
  nh.getParam("pause_side", pause_side);
  nh.getParam("grip_delay", grip_delay);
  nh.getParam("press_delay", press_delay);
  nh.getParam("shuttlecock_force_h", shuttlecock_force_h);
  nh.getParam("shuttlecock_force_v", shuttlecock_force_v);
  nh.getParam("breaker_force_up", breaker_force_up);
  nh.getParam("breaker_force_down", breaker_force_down);
  nh.getParam("rotary_force_h", rotary_force_h);
  nh.getParam("rotary_force_v", rotary_force_v);
  nh.getParam("/reset_config", reset_configv);
  nh.getParam("/encoder_offsets", encoder_offsetsv);

  VectorXd reset_config(5);
  VectorXd encoder_offsets(5);

  for (int i = 0; i < 5; ++i) {
    reset_config(i) = reset_configv[i];
    encoder_offsets(i) = encoder_offsetsv[i];
  }

  KDHelper kd(urdf_file);

  const vector<string> &actuator_names = kd.get_actuator_names();

  tf2_ros::TransformBroadcaster pose_br;
  geometry_msgs::TransformStamped pose;
  pose.header.frame_id = "world";
  pose.child_frame_id = "end effector pose";

  // Publish trajectory to rviz
  ros::Publisher trajectory_pub = nh.advertise<visualization_msgs::Marker>("/shipbot/desired_trajectory", 1);
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
  trajectory_pub.publish(marker);

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

  // Write gains
  
  cout << "group size : " << group->size() << endl;

  string gains_file;
  nh.getParam("gains_file", gains_file);
  /*
  group_command.readGains(gains_file);
  bool success = group->sendCommandWithAcknowledgement(group_command);
  if (!success) return 1;
  cout << "wrote gains" << endl;
  */
  

  VectorXd position_fbk = VectorXd::Zero(group->size());
  VectorXd velocity_fbk = VectorXd::Zero(group->size());
  VectorXd effort_fbk = VectorXd::Zero(group->size());
  
  shared_ptr<ArmPlanner> planner = make_shared<ArmPlanner>(seconds_per_meter, seconds_per_degree, reset_config, horizontal_pause_back, vertical_pause_back, vertical_pause_above, pause_side, grip_delay, press_delay, shuttlecock_force_h, shuttlecock_force_v, breaker_force_up, breaker_force_down, rotary_force_h, rotary_force_v);

  Vector3d position(0, 0, 0);
  Quaterniond orientation(1, 0, 0, 0);
  double pitch;
  double roll;
  group->getNextFeedback(group_feedback);
  group_feedback.getPosition(position_fbk);
  group_feedback.getVelocity(velocity_fbk);
  group_feedback.getEffort(effort_fbk);
  kd.update_state(position_fbk, velocity_fbk, effort_fbk);
  kd.fk(position, orientation, pitch, roll);
  VectorXd current_task_config = VectorXd::Zero(5);
  current_task_config.head<3>() = position;
  current_task_config(3) = pitch;
  current_task_config(4) = roll;
  shared_ptr<VectorXd> config_ptr(&current_task_config);

  VectorXd position_cmds = position_fbk;
  VectorXd velocity_cmds = velocity_fbk;
  VectorXd effort_cmds = effort_fbk;

  ros::ServiceServer spin_rotary_service = nh.advertiseService<shipbot_ros::SpinRotary::Request, shipbot_ros::SpinRotary::Response>("spin_rotary", spin_rotary(planner, config_ptr));
  ros::ServiceServer spin_shuttlecock_service = nh.advertiseService<shipbot_ros::SpinShuttlecock::Request, shipbot_ros::SpinShuttlecock::Response>("spin_shuttlecock", spin_shuttlecock(planner, config_ptr));
  ros::ServiceServer switch_breaker_service = nh.advertiseService<shipbot_ros::SwitchBreaker::Request, shipbot_ros::SwitchBreaker::Response>("switch_breaker", switch_breaker(planner, config_ptr));
  ros::ServiceServer reset_arm_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("reset_arm", reset_arm(planner, config_ptr));
  ros::ServiceServer stop_arm_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("stop_arm", stop_arm(planner, config_ptr));

  ros::ServiceClient done_client = nh.serviceClient<std_srvs::Empty>("/mission_control_node/arm_done");
  std_srvs::Empty done_srv;
  called_done = true;

  ros::Publisher grip_pub = nh.advertise<std_msgs::Bool>("/shipbot/grip", 1);
  std_msgs::Bool grip_msg;
  grip_msg.data = false;

  start_time = ros::Time::now().toSec();

  ros::Rate r(rate);
  while (ros::ok())
  {
    group->getNextFeedback(group_feedback);
    group_feedback.getPosition(position_fbk);
    position_fbk += encoder_offsets;
    group_feedback.getVelocity(velocity_fbk);
    group_feedback.getEffort(effort_fbk);

    kd.update_state(position_fbk, velocity_fbk, effort_fbk);

    kd.fk(position, orientation, pitch, roll);
    current_task_config.head<3>() = position;
    current_task_config(3) = pitch;
    current_task_config(4) = roll;

    double t = ros::Time::now().toSec() - start_time;

    if (planner->planned()) {
      bool grip = false;
      Vector3d force(0, 0, 0);
      VectorXd task_config = planner->eval(t, grip, force);
      grip_msg.data = grip;
      VectorXd task_vel = planner->deriv1(t);
      VectorXd task_acc = planner->deriv2(t);

      kd.ik(position_cmds, task_config);

      /*
      for (size_t i = 0; i < 5; ++i) {
        while (position_fbk(i) - position_cmds(i) > 2*M_PI) {
          position_cmds(i) += 2*M_PI;
        }
        while (position_fbk(i) - position_cmds(i) < -2*M_PI) {
          position_cmds(i) -= 2*M_PI;
        }
      }
      */

      kd.grav_comp(effort_cmds);

      //Vector3d linear_velocity(0, 0, 0);
      //Vector3d angular_velocity(0, 0, 0);
      ///kd.fvk(linear_velocity, angular_velocity);

      VectorXd press_torque = VectorXd::Zero(5);
      kd.apply_force(press_torque, force);

      VectorXd pos_err = task_config.head<3>() - current_task_config.head<3>();
      if (force.norm() > 0.1 && pos_err.norm() > 0.3) {
        planner->reset_arm(current_task_config,
                           ros::Time::now().toSec() - start_time);
      } else {
        effort_cmds += press_torque;
      }

      // These don't seem to help
      //kd.tsid(effort_cmds, task_acc);
      //kd.idk(velocity_cmds, task_vel);

      // Send commands to HEBI modules
      position_cmds -= encoder_offsets;
      group_command.setPosition(position_cmds);
      //group_command.setVelocity(velocity_cmds);
      group_command.setEffort(effort_cmds);
      group->sendCommand(group_command);

      if (!called_done && t > planner->get_end_time()) {
        VectorXd err = task_config - current_task_config;
        // We have the + 5 here because the arm might be stuck
        if (err.head<3>().norm() < 0.1 || t > planner->get_end_time() + 5) {
          done_client.call(done_srv);
          called_done = true;
        }
      }
    }

    grip_pub.publish(grip_msg);

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

