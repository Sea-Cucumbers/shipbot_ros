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
#include "armPlanner.h"
#include "shipbot_ros/spin_rotary.h"
#include "shipbot_ros/spin_shuttlecock.h"
#include "shipbot_ros/switch_breaker.h"
#include "shipbot_ros/reset_arm.h"

using namespace std;

// TODO: shouldn't be static, got lazy
static visualization_msgs::Marker marker;
static double start_time;

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
    bool operator () (shipbot_ros::reset_arm::Request &req,
                      shipbot_ros::reset_arm::Response &res) {
      planner->reset_arm(*task_space_config,
                         ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
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
    bool operator () (shipbot_ros::spin_rotary::Request &req,
                      shipbot_ros::spin_rotary::Response &res) {
      planner->spin_rotary(*task_space_config,
                           Vector3d(req.position.x, req.position.y, req.position.z),
                           req.vertical_spin_axis,
                           (double)req.degrees,
                           ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
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
    bool operator () (shipbot_ros::spin_shuttlecock::Request &req,
                      shipbot_ros::spin_shuttlecock::Response &res) {
      planner->spin_shuttlecock(*task_space_config,
                                Vector3d(req.position.x, req.position.y, req.position.z),
                                Vector3d(req.handle_end.x, req.handle_end.y, req.handle_end.z),
                                req.vertical_spin_axis,
                                req.clockwise,
                                ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
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
    bool operator () (shipbot_ros::switch_breaker::Request &req,
                      shipbot_ros::switch_breaker::Response &res) {
      planner->switch_breaker(*task_space_config,
                              Vector3d(req.position.x, req.position.y, req.position.z),
                              req.push_up,
                              ros::Time::now().toSec() - start_time);
      planner->sample_points(marker.points);
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
  
  nh.getParam("urdf", urdf_file);
  nh.getParam("rate", rate);
  nh.getParam("seconds_per_meter", seconds_per_meter);
  nh.getParam("seconds_per_degree", seconds_per_degree);
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

  VectorXd position_fbk = VectorXd::Zero(group->size());
  VectorXd velocity_fbk = VectorXd::Zero(group->size());
  VectorXd effort_fbk = VectorXd::Zero(group->size());
  
  shared_ptr<ArmPlanner> planner = make_shared<ArmPlanner>(seconds_per_meter, seconds_per_degree);

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

  ros::ServiceServer spin_rotary_service = nh.advertiseService<shipbot_ros::spin_rotary::Request, shipbot_ros::spin_rotary::Response>("spin_rotary", spin_rotary(planner, config_ptr));
  ros::ServiceServer spin_shuttlecock_service = nh.advertiseService<shipbot_ros::spin_shuttlecock::Request, shipbot_ros::spin_shuttlecock::Response>("spin_shuttlecock", spin_shuttlecock(planner, config_ptr));
  ros::ServiceServer switch_breaker_service = nh.advertiseService<shipbot_ros::switch_breaker::Request, shipbot_ros::switch_breaker::Response>("switch_breaker", switch_breaker(planner, config_ptr));
  ros::ServiceServer reset_arm_service = nh.advertiseService<shipbot_ros::reset_arm::Request, shipbot_ros::reset_arm::Response>("reset_arm", reset_arm(planner, config_ptr));

  start_time = ros::Time::now().toSec();

  ros::Rate r(rate);
  while (ros::ok())
  {
    group->getNextFeedback(group_feedback);
    group_feedback.getPosition(position_fbk);
    group_feedback.getVelocity(velocity_fbk);
    group_feedback.getEffort(effort_fbk);

    kd.update_state(position_fbk, velocity_fbk, effort_fbk);

    kd.fk(position, orientation, pitch, roll);
    current_task_config.head<3>() = position;
    current_task_config(3) = pitch;
    current_task_config(4) = roll;

    double t = ros::Time::now().toSec() - start_time;

    if (planner->planned()) {
      VectorXd task_config = planner->eval(t);
      VectorXd task_vel = planner->deriv1(t);
      VectorXd task_acc = planner->deriv2(t);

      kd.ik(position_cmds, task_config);

      kd.grav_comp(effort_cmds);

      // These don't seem to help
      //kd.tsid(effort_cmds, task_acc);
      //kd.idk(velocity_cmds, task_vel);

      // Send commands to HEBI modules
      group_command.setPosition(position_cmds);
      //group_command.setVelocity(velocity_cmds);
      group_command.setEffort(effort_cmds);
      group->sendCommand(group_command);
    }

    cout << position/0.0254 << endl << endl << endl;
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

