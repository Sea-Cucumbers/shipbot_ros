#include "ikSolver.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>

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
  IKSolver solver(urdf_file);

  unordered_map<string, std_msgs::Float64> cmd_msgs;
  unordered_map<string, ros::Publisher> cmd_pubs;
  const vector<string> &actuator_names = solver.get_actuator_names();
  for (vector<string>::const_iterator it = actuator_names.begin();
       it != actuator_names.end(); ++it) {
    cmd_pubs[*it] = nh.advertise<std_msgs::Float64>("/shipbot/" + *it + "_controller/command", 1);
    cmd_msgs[*it] = std_msgs::Float64();
    cmd_msgs.at(*it).data = 0;
  }

  shared_ptr<sensor_msgs::JointState> joints_ptr = make_shared<sensor_msgs::JointState>();
  ros::Subscriber joint_sub = nh.subscribe<sensor_msgs::JointState>("/shipbot/joint_states", 1, handle_joints(joints_ptr));

  ros::Rate r(100);
  while (!got_joints && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }

  double start_t = ros::Time::now().toSec();
  while (ros::ok())
  {
    double t = ros::Time::now().toSec();
    double x = cx + radius*cos(t);
    double z = cz + radius*sin(t);
    solver.solve(cmd_msgs, x, cy, z);

    for (vector<string>::const_iterator it = actuator_names.begin();
         it != actuator_names.end(); ++it) {
      cmd_pubs.at(*it).publish(cmd_msgs.at(*it));
    }
    r.sleep();
    ros::spinOnce();
  }
}

