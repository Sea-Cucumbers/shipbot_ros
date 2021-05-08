#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "shipbot_ros/ChassisState.h"
#include "shipbot_ros/ChassisFeedback.h"
#include "std_srvs/Empty.h"
#include "angle_mod.h"

using namespace std;

class loc_prep_done {
  private:
    shared_ptr<bool> done_ptr;

  public:
    /*
     * loc_prep_done: constructor
     * ARGUMENTS
     * _done_ptr: pointer to set when we're done
     */
     loc_prep_done(shared_ptr<bool> _done_ptr) : done_ptr(_done_ptr) {}

    /*
     * operator (): tell the chassis controller to stop
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

/*
 * handle_state: state message handler
 */
class handle_state {
  private:
    shared_ptr<shipbot_ros::ChassisState> state_ptr;
    shared_ptr<bool> got_state;

  public:
    /*
     * handle_state: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     * _got_state: the pointer we should populate when we get a message
     */
    handle_state(shared_ptr<shipbot_ros::ChassisState> _state_ptr,
                 shared_ptr<bool> _got_state) : state_ptr(_state_ptr), got_state(_got_state) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::ChassisStateConstPtr &msg_ptr) {
      *state_ptr = *msg_ptr;
      *got_state = true;
    }
};

class handle_feedback {
  private:
    shared_ptr<shipbot_ros::ChassisFeedback> fbk_ptr;
    shared_ptr<bool> got_fbk;

  public:
    /*
     * handle_state: constructor
     * ARGUMENTS
     * _fbk_ptr: the pointer we should populate with the received message
     * _got_state: the pointer we should populate when we get a message
     */
    handle_feedback(shared_ptr<shipbot_ros::ChassisFeedback> _fbk_ptr,
                    shared_ptr<bool> _got_fbk) : fbk_ptr(_fbk_ptr), got_fbk(_got_fbk) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::ChassisFeedbackConstPtr &msg_ptr) {
      *fbk_ptr = *msg_ptr;
      *got_fbk = true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_mux_node");
  ros::NodeHandle nh("~");

  double rate = 20;

  shared_ptr<shipbot_ros::ChassisState> state_ptr = make_shared<shipbot_ros::ChassisState>();
  shared_ptr<bool> got_state = make_shared<bool>(false);
  ros::Subscriber state_sub = nh.subscribe<shipbot_ros::ChassisState>("/shipbot/chassis_state_pf", 1, handle_state(state_ptr, got_state));

  shared_ptr<bool> loc_prep_done_ptr = make_shared<bool>(false);
  ros::ServiceServer loc_prep_done_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("loc_prep_done_pf", loc_prep_done(loc_prep_done_ptr));

  shared_ptr<shipbot_ros::ChassisFeedback> fbk_ptr = make_shared<shipbot_ros::ChassisFeedback>();
  shared_ptr<bool> got_fbk = make_shared<bool>(false);
  ros::Subscriber fbk_sub = nh.subscribe<shipbot_ros::ChassisFeedback>("/shipbot/chassis_feedback", 1, handle_feedback(fbk_ptr, got_fbk));

  ros::ServiceClient loc_prep_done_client = nh.serviceClient<std_srvs::Empty>("/mission_control_node/loc_prep_done");
  std_srvs::Empty loc_prep_done_srv;

  ros::Publisher state_pub = nh.advertise<shipbot_ros::ChassisState>("/shipbot/chassis_state", 1);
  shipbot_ros::ChassisState state_msg;

  ros::Rate r(rate);
  while ((!(*loc_prep_done_ptr) || !(*got_state) || !(*got_fbk)) && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }

  loc_prep_done_client.call(loc_prep_done_srv);

  // Sensor geometry
  array<double, 2> t0 = {0.1651, -0.09525};
  array<double, 2> t1 = {-0.1016, 0.15875};
  array<double, 2> t2 = {-0.1651, 0.1143};
  array<double, 2> t3 = {0.1016, -0.15875};

  while (ros::ok())
  {
    if (angdiff(state_ptr->yaw, 0) < M_PI/16) {
      // Facing away from short wall. Use sensors 2 and 3
      state_msg.x = fbk_ptr->tofs[2] - t2[0];
      state_msg.y = fbk_ptr->tofs[3] - t3[1];
    } else if (angdiff(state_ptr->yaw, M_PI/2) < M_PI/16) {
      // Facing away from long wall. Use sensors 1 and 2
      state_msg.x = fbk_ptr->tofs[1] + t1[1];
      state_msg.y = fbk_ptr->tofs[2] - t2[0];
    } else if (angdiff(state_ptr->yaw, M_PI) < M_PI/16) {
      // Facing toward short wall. Use sensors 0 and 1
      state_msg.x = fbk_ptr->tofs[0] + t0[0];
      state_msg.y = fbk_ptr->tofs[1] + t1[1];
    } else if (angdiff(state_ptr->yaw, 3*M_PI/2) < M_PI/16) {
      // Facing toward long wall. Use sensors 3 and 0
      state_msg.x = fbk_ptr->tofs[3] - t3[1];
      state_msg.y = fbk_ptr->tofs[0] + t0[0];
    } else {
      state_msg.x = state_ptr->x;     
      state_msg.y = state_ptr->y;     
    }
    state_msg.yaw = state_ptr->yaw;
    state_pub.publish(state_msg);
    r.sleep();
    ros::spinOnce();
  }
}

