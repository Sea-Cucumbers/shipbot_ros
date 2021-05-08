#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "shipbot_ros/ChassisState.h"
#include "shipbot_ros/TravelAbs.h"
#include "shipbot_ros/TravelRel.h"
#include "shipbot_ros/ChassisCommand.h"
#include "std_srvs/Empty.h"
#include "se2Interpolator.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// TODO: shouldn't be static, got lazy
static double start_time;

enum ControlStatus {STOP, LOCALIZE, CL};

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
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      *status = STOP;
      return true;
    }
};

class localize {
  private:
    shared_ptr<ControlStatus> status;

  public:
    /*
     * localize: constructor
     * ARGUMENTS
     * _status: pointer to control status
     */
     localize(shared_ptr<ControlStatus> _status) : status(_status) {}

    /*
     * operator (): tell the chassis controller to move to localize the chassis
     * ARGUMENTS
     * req: request, not used
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res) {
      *status = LOCALIZE;
      return true;
    }
};

class travel_abs {
  private:
    shared_ptr<SE2Interpolator> planner;
    shared_ptr<Vector3d> cur_state;
    shared_ptr<ControlStatus> status;
    double linear_speed;
    double angular_speed;

  public:
    /*
     * travel_abs: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _cur_state: pointer to current state
     * _status: pointer to control status
     */
     travel_abs(shared_ptr<SE2Interpolator> _planner,
               shared_ptr<Vector3d> _cur_state,
               shared_ptr<ControlStatus> _status,
               double _linear_speed,
               double _angular_speed) : planner(_planner),
                                        cur_state(_cur_state),
                                        status(_status),
                                        linear_speed(_linear_speed),
                                        angular_speed(_angular_speed) {}

    /*
     * operator (): plan trajectory to waypoint
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::TravelAbs::Request &req,
                      shipbot_ros::TravelAbs::Response &res) {
      Vector3d end = VectorXd::Zero(3);
      end(0) = req.x;
      end(1) = req.y;
      end(2) = req.theta;

      Vector3d err = planner->error(*cur_state, end);

      double traj_start = ros::Time::now().toSec() - start_time;
      double traj_time = max(err.head<2>().norm()/linear_speed, abs(err(2))/angular_speed);
      *planner = SE2Interpolator(*cur_state, end, traj_start, traj_start + traj_time);
      *status = CL;
      return true;
    }
};

class travel_rel {
  private:
    shared_ptr<SE2Interpolator> planner;
    shared_ptr<Vector3d> cur_state;
    shared_ptr<ControlStatus> status;
    double linear_speed;
    double angular_speed;

  public:
    /*
     * travel_rel: constructor
     * ARGUMENTS
     * _planner: pointer to planner
     * _status: pointer to control status
     */
     travel_rel(shared_ptr<SE2Interpolator> _planner,
                shared_ptr<Vector3d> _cur_state,
                shared_ptr<ControlStatus> _status,
                double _linear_speed,
                double _angular_speed) : planner(_planner),
                                         cur_state(_cur_state),
                                         status(_status),
                                         linear_speed(_linear_speed),
                                         angular_speed(_angular_speed) {}

    /*
     * operator (): plan open-loop trajectory
     * ARGUMENTS
     * req: request
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::TravelRel::Request &req,
                      shipbot_ros::TravelRel::Response &res) {
      Vector3d end = VectorXd::Zero(3);
      double theta = (*cur_state)(2);
      double c = cos(theta);
      double s = sin(theta);
      end(0) = (*cur_state)(0) + req.delta_x*c - req.delta_y*s;
      end(1) = (*cur_state)(1) + req.delta_x*s + req.delta_y*c;
      end(2) = theta + req.delta_theta;

      Vector3d err = planner->error(*cur_state, end);

      double traj_start = ros::Time::now().toSec() - start_time;
      double traj_time = max(err.head<2>().norm()/linear_speed, abs(err(2))/angular_speed);
      *planner = SE2Interpolator(*cur_state, end, traj_start, traj_start + traj_time);
      *status = CL;
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

  double linear_speed = 0;
  double angular_speed = 0;
  nh.getParam("linear_speed", linear_speed);
  nh.getParam("angular_speed", angular_speed);

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

  ros::ServiceServer travel_abs_service = nh.advertiseService<shipbot_ros::TravelAbs::Request, shipbot_ros::TravelAbs::Response>("travel_abs", travel_abs(planner, state_ptr, status, linear_speed, angular_speed));
  ros::ServiceServer travel_rel_service = nh.advertiseService<shipbot_ros::TravelRel::Request, shipbot_ros::TravelRel::Response>("travel_rel", travel_rel(planner, state_ptr, status, linear_speed, angular_speed));
  ros::ServiceServer localization_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("localize", localize(status));
  ros::ServiceServer stop_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("stop_chassis", stop(status));

  ros::Publisher cmd_pub = nh.advertise<shipbot_ros::ChassisCommand>("/shipbot/chassis_command", 1);
  shipbot_ros::ChassisCommand cmd_msg;

  ros::ServiceClient done_client = nh.serviceClient<std_srvs::Empty>("/mission_control_node/chassis_done");
  std_srvs::Empty done_srv;

  Vector3d kp = Vector3d::Ones(3);
  vector<double> kpv;
  nh.getParam("kp", kpv);
  kp(0) = kp[0];
  kp(1) = kp[1];
  kp(2) = kp[2];


  bool doing_localization = false;
  double loc_start_time = 0;
  
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
        doing_localization = true;
      } else {
        double delta_t = t - loc_start_time;
        if (delta_t < 8) {
          cmd_msg.vx = 0;
          cmd_msg.vy = 0;
          cmd_msg.w = M_PI/8;
        } else if (delta_t < 16) {
          cmd_msg.vx = 0;
          cmd_msg.vy = 0;
          cmd_msg.w = -M_PI/8;
        } else {
          cmd_msg.vx = 0;
          cmd_msg.vy = 0;
          cmd_msg.w = 0;
          doing_localization = false;
          *status = STOP;

          done_client.call(done_srv);
        }
      }
    } else if (*status == CL) {
      VectorXd des_state = planner->eval(t);
      VectorXd des_vel = planner->deriv1(t);

      VectorXd err = planner->error(*state_ptr, des_state);
      if (t >= planner->get_end_time() && err.head<2>().norm() < 0.02 && abs(err(2)) < 0.1) {
        *status = STOP;
        done_client.call(done_srv);
      }

      VectorXd cmd_vel = des_vel + kp.cwiseProduct(err);
      double theta = (*state_ptr)(2);
      double c = cos(theta);
      double s = sin(theta);
      cmd_msg.vx = c*cmd_vel(0) + s*cmd_vel(1);
      cmd_msg.vy = -s*cmd_vel(0) + c*cmd_vel(1);
      cmd_msg.w = cmd_vel(2);
    }
    cmd_pub.publish(cmd_msg);
    r.sleep();
    ros::spinOnce();
  }
}

