#ifndef KD_HELPER_H
#define KD_HELPER_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <std_msgs/Float64.h>
#include <unordered_map>
#include <memory>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace Eigen;

namespace pin = pinocchio;

class KDHelper {
  public:
    /*
     * constructor: initializes internal kinematics/dynamics object
     * ARGUMENTS
     * urdf_file: urdf modeling robot
     */
    KDHelper(const string &urdf_file);

    
    inline const vector<string> &get_actuator_names() {
      return actuator_names;
    }

    /*
     * ik: determine joint positions given end-effector position
     * ARGUMENTS
     * cmd_msgs: populated with messages to send
     * x, y, z: we place the end-effector at this position with pitch zero
     * RETURN: true if a solution was found, false if not
     */
    bool ik(unordered_map<string, std_msgs::Float64> &cmd_msgs, double x, double y, double z, double pitch);

    /*
     * fk: get forward kinematics of end-effector
     * ARGUMENTS
     * position: populated with end-effector position
     * orientation: populated with end-effector orientation
     * joints_ptr: pointer to joint state message
     */
    void fk(Vector3d &position, Quaterniond &orientation, shared_ptr<sensor_msgs::JointState> &joints_ptr);

   private:
    pin::Model model;
    unique_ptr<pin::Data> model_data;

    // Names of actuators corresponding to each element of cmd_msgs
    vector<string> actuator_names;

    // Horizontal distance between end-effector and arm's origin
    double D;

    // Link lengths
    double l0;
    double l1;
    double l2;
    double l2_loc;
    double l3;
    double l4;
    double l5;
    double ee;

    pin::FrameIndex ee_fid;
};

#endif