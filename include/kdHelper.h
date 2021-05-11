#ifndef KD_HELPER_H
#define KD_HELPER_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <unordered_map>
#include <memory>

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
     * update_state: updates the internal state of the kinematics/dynamics object
     * ARGUMENTS
     * position: joint positions (order should correspond to order of actuator_names)
     * velocity: joint velocities (order should correspond to order of actuator_names)
     * effort: joint efforts (order should correspond to order of actuator_names)
     */
    void update_state(const VectorXd &positions, const VectorXd &velocities, const VectorXd &efforts);

    /*
     * ik: determine joint positions given task-space configuration
     * ARGUMENTS
     * joint_positions: populated with joint positions
     * task_config: task-space configuration (x, y, z, pitch, roll)
     * RETURN: true if a solution was found, false if not
     */
    bool ik(VectorXd &joint_positions, const VectorXd &task_config);

    /*
     * fk: get forward kinematics of end-effector. update_state must be called beforehand
     * ARGUMENTS
     * position: populated with end-effector position
     * orientation: populated with end-effector orientation
     * pitch: populated with pitch
     * roll: populated with roll
     */
    void fk(Vector3d &position, Quaterniond &orientation, double &pitch, double &roll);

    /*
     * grav_comp: get torques needed to counteract gravity. update_state must be called beforehand
     * ARGUMENTS
     * joint_torques: populated with joint torques
     */
    void grav_comp(VectorXd &joint_torques);

    /*
     * apply_force: get torques needed to apply some amount of force at the end effector,
     * expressed in the unrolled end-effector frame (i.e. the frame where the last joint
     * is at zero)
     * ARGUMENTS
     * joint_torques: populated with joint torques
     * force: apply this much force in Newtons in the unrolled end-effector frame
     */
    void apply_force(VectorXd &joint_torques, const Vector3d &force);

    /*
     * tsid: get torques needed to achieve desired task-space acceleration, where the task
     * space is defined as x, y, z, pitch, roll
     * ARGUMENTS
     * joint_torques: populated with joint torques
     */
    void tsid(VectorXd &joint_torques, const VectorXd &acc);

    /*
     * idk: determine joint velocities given task velocity
     * ARGUMENTS
     * joint_velocities: populated with joint velocities
     * task_velocity: task velocity
     */
    void idk(VectorXd &joint_velocities, const VectorXd &task_velocity);

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

    VectorXd config;
    VectorXd vel;
};

#endif
