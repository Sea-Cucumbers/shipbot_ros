#include "kdHelper.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

static bool got_joints;

/* 
 * handle_joints: joint message handler
 */
class handle_joints {
  private:
    shared_ptr<sensor_msgs::JointState> msg_ptr;
    
  public:
    /*
     * handle_joints: constructor
     * ARGUMENTS
     * _msg_ptr: the pointer we should populate with the received message
     */
    handle_joints(shared_ptr<sensor_msgs::JointState> _msg_ptr) : msg_ptr(_msg_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * joints_ptr: pointer to received message
     */
    void operator () (const sensor_msgs::JointStateConstPtr &joints_ptr) {
      *msg_ptr = *joints_ptr;
      got_joints = true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_control_node");
  ros::NodeHandle nh("~");

  string urdf_file;
  double rate;
  
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
  vector<std_msgs::Float64> cmd_msgs;
  vector<ros::Publisher> cmd_pubs;
  VectorXd position_cmds = VectorXd::Zero(actuator_names.size());
  for (size_t j = 0; j < actuator_names.size(); ++j) {
    cmd_pubs.push_back(nh.advertise<std_msgs::Float64>("/shipbot/" +
                                                       actuator_names[j] +
                                                       "_controller/command", 1));
    cmd_msgs.push_back(std_msgs::Float64());
    cmd_msgs[j].data = 0;
  }

  shared_ptr<sensor_msgs::JointState> joints_ptr = make_shared<sensor_msgs::JointState>();
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("/shipbot/joint_states", 1, handle_joints(joints_ptr));

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

  ros::Rate r(100);
  while (!got_joints && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }

  VectorXd positions = VectorXd::Zero(actuator_names.size());
  VectorXd velocities = VectorXd::Zero(actuator_names.size());
  VectorXd efforts = VectorXd::Zero(actuator_names.size());

  double start_t = ros::Time::now().toSec();
  while (ros::ok())
  {
    double t = ros::Time::now().toSec();
    double x = cx + radius*cos(t);
    double z = cz + radius*sin(t);

    for (size_t j = 0; j < actuator_names.size(); ++j) {
      positions(j) = joints_ptr->position[j];
      velocities(j) = joints_ptr->velocity[j];
      efforts(j) = joints_ptr->effort[j];
    }
      
    kd.update_state(positions, velocities, efforts);

    kd.ik(position_cmds, x, cy, z, 0);

    for (size_t j = 0; j < actuator_names.size(); ++j) {
      cmd_msgs[j].data = position_cmds(j);
      cmd_pubs[j].publish(cmd_msgs[j]);
    }

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

