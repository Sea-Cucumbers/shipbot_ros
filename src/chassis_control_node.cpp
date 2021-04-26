#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "shipbot_ros/ChassisState.h"
#include "shipbot_ros/Travel.h"
#include "shipbot_ros/InitialLocalization.h"
#include "shipbot_ros/ChassisCommand.h"
#include "shipbot_ros/StopChassis.h"
#include "se2Interpolator.h"
#include <Eigen/Dense>
#include "shipbot_ros/ChassisDone.h"

using namespace std;
using namespace Eigen;

// TODO: shouldn't be static, got lazy
static double start_time;

class stop {
  private:
    shared_ptr<bool> stopped;

  public:
    /*
     * stop: constructor
     * ARGUMENTS
     * _stopped: pointer to stop variable
     */
     stop(shared_ptr<bool> _stopped) : stopped(_stopped) {}

    /*
     * operator (): tell the chassis controller to stop
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::StopChassis::Request &req,
                      shipbot_ros::StopChassis::Response &res) {
      *stopped = true;
      return true;
    }
};

class localize {
  private:
    shared_ptr<bool> start_localization;
    shared_ptr<bool> stopped;
    shared_ptr<int> safe_direction;

  public:
    /*
     * localize: constructor
     * ARGUMENTS
     * _start_localization: pointer to localization variable
     * _stopped: pointer to stopped variable
     */
     localize(shared_ptr<bool> _start_localization,
              shared_ptr<bool> _stopped,
              shared_ptr<int> _safe_direction) : start_localization(_start_localization),
                                                 stopped(_stopped),
                                                 safe_direction(_safe_direction) {}

    /*
     * operator (): tell the chassis controller to move to localize the chassis (or not) 
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::InitialLocalization::Request &req,
                      shipbot_ros::InitialLocalization::Response &res) {
      *start_localization = true;
      *stopped = false;
      *safe_direction = req.safe_direction;
      return true;
    }
};

class travel {
  private:
    shared_ptr<SE2Interpolator> planner;
    shared_ptr<Vector3d> cur_state;
    shared_ptr<bool> stopped;

  public:
    /*
     * travel: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _cur_state: pointer to current state
     * _stopped: pointer to stopped
     */
     travel(shared_ptr<SE2Interpolator> _planner,
            shared_ptr<Vector3d> _cur_state,
            shared_ptr<bool> _stopped) : planner(_planner),
                                         cur_state(_cur_state),
                                         stopped(_stopped) {}

    /*
     * operator (): plan trajectory to waypoint
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::Travel::Request &req,
                      shipbot_ros::Travel::Response &res) {
      Vector3d end = VectorXd::Zero(3);
      end(0) = req.x;
      end(1) = req.y;
      end(2) = req.theta;

      Vector3d err = planner->error(*cur_state, end);

      double traj_start = ros::Time::now().toSec() - start_time;
      //double traj_time = max(err.head<2>().norm()*4, 8*abs(err(2))/M_PI);
      double traj_time = 8*abs(err(2))/M_PI;
      *planner = SE2Interpolator(*cur_state, end, traj_start, traj_start + traj_time);
      *stopped = false;
      return true;
    }
};

/*
 * handle_state: state message handler
 */
class handle_state {
  private:
    shared_ptr<Vector3d> state_ptr;
    shared_ptr<bool> got_state;

  public:
    /*
     * handle_state: constructor
     * ARGUMENTS
     * _state_ptr: the pointer we should populate with the received message
     * _got_state: the pointer we should populate when we get a message
     */
    handle_state(shared_ptr<Vector3d> _state_ptr,
                 shared_ptr<bool> _got_state) : state_ptr(_state_ptr), got_state(_got_state) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * msg_ptr: pointer to received message
     */
    void operator () (const shipbot_ros::ChassisStateConstPtr &msg_ptr) {
      (*state_ptr)(0) = msg_ptr->x;
      (*state_ptr)(1) = msg_ptr->y;
      (*state_ptr)(2) = msg_ptr->yaw;
      *got_state = true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "chassis_control_node");
  ros::NodeHandle nh("~");

  double rate = 20;

  shared_ptr<Vector3d> state_ptr = make_shared<Vector3d>();
  shared_ptr<bool> got_state = make_shared<bool>(false);
  ros::Subscriber state_sub = nh.subscribe<shipbot_ros::ChassisState>("/shipbot/chassis_state", 1, handle_state(state_ptr, got_state));

  ros::Rate r(rate);
  while (!got_state && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }
  
  start_time = ros::Time::now().toSec();

  shared_ptr<SE2Interpolator> planner = make_shared<SE2Interpolator>(*state_ptr, *state_ptr, 0, 0);

  shared_ptr<bool> stopped = make_shared<bool>(true);

  ros::ServiceServer travel_service = nh.advertiseService<shipbot_ros::Travel::Request, shipbot_ros::Travel::Response>("travel", travel(planner, state_ptr, stopped));

  shared_ptr<bool> start_localization = make_shared<bool>(false);
  shared_ptr<int> safe_direction = make_shared<int>(0);
  ros::ServiceServer initial_localization_service = nh.advertiseService<shipbot_ros::InitialLocalization::Request, shipbot_ros::InitialLocalization::Response>("localize", localize(start_localization, stopped, safe_direction));

  ros::ServiceServer stop_service = nh.advertiseService<shipbot_ros::StopChassis::Request, shipbot_ros::StopChassis::Response>("stop_chassis", stop(stopped));

  ros::Publisher cmd_pub = nh.advertise<shipbot_ros::ChassisCommand>("/shipbot/chassis_command", 1);
  shipbot_ros::ChassisCommand cmd_msg;

  ros::ServiceClient done_client = nh.serviceClient<shipbot_ros::ChassisDone>("/mission_control/chassis_done");
  shipbot_ros::ChassisDone done_srv;

  Vector3d kp = Vector3d::Ones(3);
  vector<double> kpv;
  nh.getParam("kp", kpv);
  kp(0) = kp[0];
  kp(1) = kp[1];
  kp(2) = kp[2];

  bool doing_localization = false;
  double loc_start_time = 0;
  double loc_w = 0;

  while (ros::ok())
  {
    double t = ros::Time::now().toSec() - start_time;

    if (*stopped) {
      cmd_msg.vx = 0;
      cmd_msg.vy = 0;
      cmd_msg.w = 0;
    } else if (*start_localization) {
      *start_localization = false;
      loc_start_time = t;
      doing_localization = true;
    } else if (doing_localization) {
      double delta_t = t - loc_start_time;
      if (delta_t < 4) {
        switch (*safe_direction) {
          case shipbot_ros::InitialLocalization::Request::FORWARD:
            cmd_msg.vx = 0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::BACKWARD:
            cmd_msg.vx = -0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::LEFT:
            cmd_msg.vx = 0;
            cmd_msg.vy = 0.05;
            break;
          case shipbot_ros::InitialLocalization::Request::RIGHT:
            cmd_msg.vx = 0;
            cmd_msg.vy = -0.05;
            break;
        }
      } else if (delta_t < 8) {
        switch (*safe_direction) {
          case shipbot_ros::InitialLocalization::Request::FORWARD:
            cmd_msg.vx = -0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::BACKWARD:
            cmd_msg.vx = 0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::LEFT:
            cmd_msg.vx = 0;
            cmd_msg.vy = -0.05;
            break;
          case shipbot_ros::InitialLocalization::Request::RIGHT:
            cmd_msg.vx = 0;
            cmd_msg.vy = 0.05;
            break;
        }
      } else if (delta_t < 12) {
        switch (*safe_direction) {
          case shipbot_ros::InitialLocalization::Request::FORWARD:
            cmd_msg.vx = 0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::BACKWARD:
            cmd_msg.vx = -0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::LEFT:
            cmd_msg.vx = 0;
            cmd_msg.vy = 0.05;
            break;
          case shipbot_ros::InitialLocalization::Request::RIGHT:
            cmd_msg.vx = 0;
            cmd_msg.vy = -0.05;
            break;
        }
      } else if (delta_t < 16) {
        switch (*safe_direction) {
          case shipbot_ros::InitialLocalization::Request::FORWARD:
            cmd_msg.vx = -0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::BACKWARD:
            cmd_msg.vx = 0.05;
            cmd_msg.vy = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::LEFT:
            cmd_msg.vx = 0;
            cmd_msg.vy = -0.05;
            break;
          case shipbot_ros::InitialLocalization::Request::RIGHT:
            cmd_msg.vx = 0;
            cmd_msg.vy = 0.05;
            break;
        }
      } else if (delta_t < 24) {
        cmd_msg.vx = 0;
        cmd_msg.vy = 0;
        cmd_msg.w = M_PI/8;
      } else if (delta_t < 32) {
        cmd_msg.vx = 0;
        cmd_msg.vy = 0;
        cmd_msg.w = -M_PI/8;
      } else {
        cmd_msg.vx = 0;
        cmd_msg.vy = 0;
        cmd_msg.w = 0;
        doing_localization = false;
        *stopped = true;

        done_client.call(done_srv);
      }
    } else {
      VectorXd des_state = planner->eval(t);
      VectorXd des_vel = planner->deriv1(t);

      VectorXd err = planner->error(*state_ptr, des_state);
      if (t >= planner->get_end_time() && err.head<2>().norm() < 0.02 && abs(err(2)) < 0.1) {
        *stopped = true;
        done_client.call(done_srv);
      }

      VectorXd cmd_vel = des_vel;// + kp.cwiseProduct(err);
      double theta = (*state_ptr)(2);
      double c = cos(theta);
      double s = sin(theta);
      cmd_msg.vx = c*cmd_vel(0) + s*cmd_vel(1);
      cmd_msg.vy = -s*cmd_vel(0) + c*cmd_vel(1);
      cmd_msg.w = cmd_vel(2);
      cmd_msg.vx = 0;
      cmd_msg.vy = 0;
      cout << cmd_vel << endl << endl;
    }
    cmd_pub.publish(cmd_msg);
    r.sleep();
    ros::spinOnce();
  }
}

