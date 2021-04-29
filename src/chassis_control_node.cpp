#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "shipbot_ros/ChassisState.h"
#include "shipbot_ros/TravelCL.h"
#include "shipbot_ros/TravelOL.h"
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

enum ControlStatus {STOP, LOCALIZE, CL, OL};

class stop {
  private:
    shared_ptr<ControlStatus> status;

  public:
    /*
     * stop: constructor
     * ARGUMENTS
     * _status: pointer to control status
     */
     stop(shared_ptr<ControlStatus> _status) : status(_status) {}

    /*
     * operator (): tell the chassis controller to stop
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::StopChassis::Request &req,
                      shipbot_ros::StopChassis::Response &res) {
      *status = STOP;
      return true;
    }
};

class localize {
  private:
    shared_ptr<ControlStatus> status;
    shared_ptr<int> safe_direction;

  public:
    /*
     * localize: constructor
     * ARGUMENTS
     * _status: pointer to control status
     */
     localize(shared_ptr<ControlStatus> _status,
              shared_ptr<int> _safe_direction) : status(_status),
                                                 safe_direction(_safe_direction) {}

    /*
     * operator (): tell the chassis controller to move to localize the chassis
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::InitialLocalization::Request &req,
                      shipbot_ros::InitialLocalization::Response &res) {
      *status = LOCALIZE;
      *safe_direction = req.safe_direction;
      return true;
    }
};

class travel_cl {
  private:
    shared_ptr<SE2Interpolator> planner;
    shared_ptr<Vector3d> cur_state;
    shared_ptr<ControlStatus> status;

  public:
    /*
     * travel_cl: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _cur_state: pointer to current state
     * _status: pointer to control status
     */
     travel_cl(shared_ptr<SE2Interpolator> _planner,
               shared_ptr<Vector3d> _cur_state,
               shared_ptr<ControlStatus> _status) : planner(_planner),
                                                    cur_state(_cur_state),
                                                    status(_status) {}

    /*
     * operator (): plan trajectory to waypoint
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::TravelCL::Request &req,
                      shipbot_ros::TravelCL::Response &res) {
      Vector3d end = VectorXd::Zero(3);
      end(0) = req.x;
      end(1) = req.y;
      end(2) = req.theta;

      Vector3d err = planner->error(*cur_state, end);

      double traj_start = ros::Time::now().toSec() - start_time;
      double traj_time = max(err.head<2>().norm()*8, 8*abs(err(2))/M_PI);
      *planner = SE2Interpolator(*cur_state, end, traj_start, traj_start + traj_time);
      *status = CL;
      return true;
    }
};

class travel_ol {
  private:
    shared_ptr<SE2Interpolator> planner;
    shared_ptr<Vector3d> cur_state;
    shared_ptr<ControlStatus> status;

  public:
    /*
     * travel_ol: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _status: pointer to control status
     */
     travel_ol(shared_ptr<SE2Interpolator> _planner,
               shared_ptr<Vector3d> _cur_state,
               shared_ptr<ControlStatus> _status) : planner(_planner),
                                                    cur_state(_cur_state),
                                                    status(_status) {}

    /*
     * operator (): plan open-loop trajectory
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::TravelOL::Request &req,
                      shipbot_ros::TravelOL::Response &res) {
      Vector3d end = VectorXd::Zero(3);
      end(0) = (*cur_state)(0) + req.delta_x;
      end(1) = (*cur_state)(1) + req.delta_y;
      end(2) = (*cur_state)(2) + req.delta_theta;

      Vector3d err = planner->error(*cur_state, end);

      double traj_start = ros::Time::now().toSec() - start_time;
      double traj_time = max(err.head<2>().norm()*8, 8*abs(err(2))/M_PI);
      *planner = SE2Interpolator(*cur_state, end, traj_start, traj_start + traj_time);
      *status = OL;
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

  shared_ptr<ControlStatus> status = make_shared<ControlStatus>(STOP);

  ros::ServiceServer travel_cl_service = nh.advertiseService<shipbot_ros::TravelCL::Request, shipbot_ros::TravelCL::Response>("travel_cl", travel_cl(planner, state_ptr, status));
  ros::ServiceServer travel_ol_service = nh.advertiseService<shipbot_ros::TravelOL::Request, shipbot_ros::TravelOL::Response>("travel_cl", travel_ol(planner, state_ptr, status));

  shared_ptr<bool> start_localization = make_shared<bool>(false);
  shared_ptr<int> safe_direction = make_shared<int>(0);
  ros::ServiceServer initial_localization_service = nh.advertiseService<shipbot_ros::InitialLocalization::Request, shipbot_ros::InitialLocalization::Response>("localize", localize(status, safe_direction));

  ros::ServiceServer stop_service = nh.advertiseService<shipbot_ros::StopChassis::Request, shipbot_ros::StopChassis::Response>("stop_chassis", stop(status));

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
  double loc_vx1 = 0;
  double loc_vy1 = 0;
  double loc_vx2 = 0;
  double loc_vy2 = 0;

  while (ros::ok())
  {
    double t = ros::Time::now().toSec() - start_time;

    if (*status == STOP) {
      cmd_msg.vx = 0;
      cmd_msg.vy = 0;
      cmd_msg.w = 0;
    } else if (*status == LOCALIZE) {
      if (!doing_localization) {
        loc_start_time = t;

        switch (*safe_direction) {
          case shipbot_ros::InitialLocalization::Request::FORWARD:
            loc_vx1 = 0.05;
            loc_vy1 = 0;
            loc_vx2 = -0.05;
            loc_vy2 = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::BACKWARD:
            loc_vx1 = -0.05;
            loc_vy1 = 0;
            loc_vx2 = 0.05;
            loc_vy2 = 0;
            break;
          case shipbot_ros::InitialLocalization::Request::LEFT:
            loc_vx1 = 0;
            loc_vy1 = 0.05;
            loc_vx2 = 0;
            loc_vy2 = -0.05;
            break;
          case shipbot_ros::InitialLocalization::Request::RIGHT:
            loc_vx1 = 0;
            loc_vy1 = -0.05;
            loc_vx2 = 0;
            loc_vy2 = 0.05;
            break;
        }

        doing_localization = true;
      } else {
        double delta_t = t - loc_start_time;
        if (delta_t < 4) {
          cmd_msg.vx = loc_vx1;
          cmd_msg.vy = loc_vy1;
          cmd_msg.w = 0;
        } else if (delta_t < 8) {
          cmd_msg.vx = loc_vx2;
          cmd_msg.vy = loc_vy2;
          cmd_msg.w = 0;
        } else if (delta_t < 12) {
          cmd_msg.vx = loc_vx1;
          cmd_msg.vy = loc_vy1;
          cmd_msg.w = 0;
        } else if (delta_t < 16) {
          cmd_msg.vx = loc_vx2;
          cmd_msg.vy = loc_vy2;
          cmd_msg.w = 0;
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
          *status = STOP;

          //done_client.call(done_srv);
        }
      }
    } else if (*status == CL) {
      VectorXd des_state = planner->eval(t);
      VectorXd des_vel = planner->deriv1(t);

      VectorXd err = planner->error(*state_ptr, des_state);
      if (t >= planner->get_end_time() && err.head<2>().norm() < 0.02 && abs(err(2)) < 0.1) {
        *status = STOP;
        //done_client.call(done_srv);
      }

      VectorXd cmd_vel = des_vel + kp.cwiseProduct(err);
      double theta = (*state_ptr)(2);
      double c = cos(theta);
      double s = sin(theta);
      cmd_msg.vx = c*cmd_vel(0) + s*cmd_vel(1);
      cmd_msg.vy = -s*cmd_vel(0) + c*cmd_vel(1);
      cmd_msg.w = cmd_vel(2);
    } else if (*status == OL) {
      VectorXd des_state = planner->eval(t);
      VectorXd cmd_vel = planner->deriv1(t);
      double theta = des_state(2);
      double c = cos(theta);
      double s = sin(theta);
      cmd_msg.vx = c*cmd_vel(0) + s*cmd_vel(1);
      cmd_msg.vy = -s*cmd_vel(0) + c*cmd_vel(1);
      cmd_msg.w = cmd_vel(2);

      if (t >= planner->get_end_time()) {
        *status = STOP;
        //done_client.call(done_srv);
      }
    }
    cmd_pub.publish(cmd_msg);
    r.sleep();
    ros::spinOnce();
  }
}

