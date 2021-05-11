#include <Eigen/Dense>
#include <vector>
#include "minJerkInterpolator.h"
#include <geometry_msgs/Point.h>

using namespace Eigen;
using namespace std;

struct Segment {
  bool grip;
  Vector3d force;
  MinJerkInterpolator interpolator;

  Segment(bool grip, const Vector3d &force, const MinJerkInterpolator &interpolator) : grip(grip), force(force),
                                                                                       interpolator(interpolator) {}
};

/*
 * ArmPlanner: currently just generates min-jerk straight lines between a given start and end
 */
class ArmPlanner {
  private:
    vector<Segment> segments;
    const double seconds_per_meter;
    const double seconds_per_degree;

    const double horizontal_pause_back;
    const double vertical_pause_back;
    const double vertical_pause_above;
    const double pause_side;

    // How long do we wait for the jammer to grip/ungrip
    const double grip_delay;

    // How long do we wait when we change the amount of force we apply
    const double press_delay;

    const double shuttlecock_force_h;
    const double shuttlecock_force_v;
    const double breaker_force_up;
    const double breaker_force_down;
    const double rotary_force_h;
    const double rotary_force_v;

    const double shuttlecock_tip_to_joint = 0.092075;
    const double shuttlecock_handle_center_to_joint = 0.0635;
    const double shuttlecock_depth = 0.024;

    VectorXd reset_config;

    vector<Segment>::iterator current_segment;

    /*
     * retrieve_segment: populated current_segment with the segment
     * containing this value of t. If t is farther into the future
     * than the time of all segments, use the last segment. If
     * it's farther into the past than the time of all segments,
     * use the first segment
     * ARGUMENTS
     * t: time
     */
    void retrieve_segment(double t);

    /*
     * add_waypoint: augment our current plan to take us from
     * the current end to the new waypoint. The plan now
     * ends at the given waypoint. This segment adopts the
     * grip status and force of the previous segment
     * ARGUMENTS
     * waypoint: waypoint which we append to trajectory
     */
    void add_waypoint(const VectorXd &waypoint);

    /*
     * add_grip_phase: augment our current plan with a segment where
     * we grip or ungrip
     * ARGUMENTS
     * grip: if true, we grip, if false, we ungrip
     */
    void add_grip_phase(bool grip);

    /*
     * change_force: augment our current plan with a segment where
     * we change the amount of force we apply, expressed in local
     * coordinates in the unrolled end-effector frame (i.e.
     * expressed in the end-effector frame when the last joint has
     * zero rotation)
     * ARGUMENTS
     * force: how much force to apply
     */
    void change_force(const Vector3d &force);

    /*
     * start_plan: clears the plan and begins a new
     * plan starting at the given configuration at the given
     * time and ending at the given waypoint
     * ARGUMENTS
     * start_time: starting time of first segment
     * grip: do we grip during the first segment?
     * force: how much force to apply during the first segment
     * start: starting configuration of first segment
     * waypoint: ending configuration of first segment
     */
    void start_plan(double start_time, bool grip, const Vector3d &force, const VectorXd &start, const VectorXd &waypoint);

  public:
    /*
     * constructor
     * ARGUMENTS
     * seconds_per_meter: motions are constructed as a series of line segments.
     * how many seconds should a meter long segment take?
     * seconds_per_degree: if we're spinning a rotary valve, how many seconds
     * per degree do we spin?
     * reset_config: [x, y, z, pitch, roll] of arm at the reset configuration
     * horizontal_pause_back: how far back do we pause before engaging horizontal devices?
     * vertical_pause_back: how far back do we pause before engaging vertical devices?
     * vertical_pause_above: how far above do we pause before engaging vertical devices?
     * pause_side: how far to the left do we pause when pushing something?
     * grip_delay: how long do we wait for the gripper to grip/ungrip
     * press_delay: how long do we wait for the gripper to change the amount of force it's applying?
     * shuttlecock_force_h: amount of force to press horizontal shuttlecock with
     * shuttlecock_force_v: amount of force to press vertical shuttlecock with
     * breaker_force_up: amount of force to push breakers up with
     * breaker_force_down: amount of force to push breakers down with
     * rotary_force_h: amount of force to press horizontal rotary valves with
     * rotary_force_v: amount of force to press vertical rotary valves with
     */
    ArmPlanner(double seconds_per_meter,
               double seconds_per_degree,
               const VectorXd &reset_config,
               double horizontal_pause_back,
               double vertical_pause_back,
               double vertical_pause_above,
               double pause_side,
               double grip_delay,
               double press_delay,
               double shuttlecock_force_h,
               double shuttlecock_force_v,
               double breaker_force_up,
               double breaker_force_down,
               double rotary_force_h,
               double rotary_force_v);

    /*
     * sample_points: sample points along current trajectory
     * ARGUMENTS
     * points: populated with samples
     */
    void sample_points(vector<geometry_msgs::Point> &points);

    /*
     * reset_arm: generate task-space motion plan to reset the arm
     * start: starting task-space configuration
     * start_time: start time for trajectory
     */
    void reset_arm(const VectorXd &start,
                   double start_time);
                   
    /*
     * stop_arm: stops the arm
     * start: task-space configuration to stop at
     * start_time: start time for trajectory
     */
    void stop_arm(const VectorXd &start,
                  double start_time);


    /*
     * spin_rotary: generate task-space motion plan to manipulate a rotary valve
     * start: starting task-space configuration
     * position: position of valve center
     * vertical_spin_axis: if true, valve spin axis points up. Otherwise forward
     * degrees: relative number of counterclockwise degrees to spin the valve
     * start_time: start time for trajectory
     */
    void spin_rotary(const VectorXd &start,
                     const Vector3d &position,
                     bool vertical_spin_axis,
                     double degrees,
                     double start_time);

    /*
     * spin_shuttlecock: generate task-space motion plan to manipulate a shuttlecock valve
     * start: starting task-space configuration
     * position: position of valve center
     * vertical_spin_axis: if true, valve spin axis points up. Otherwise forward
     * do_open: if true, spin the valve 90 deg counterclockwise, otherwise clockwise
     * start_time: start time for trajectory
     */
    void spin_shuttlecock(const VectorXd &start,
                          const Vector3d &position, 
                          bool vertical_spin_axis,
                          bool do_open,
                          double start_time);

    /*
     * switch_breaker: generate task-space motion plan to push a breaker switch
     * start: starting task-space configuration
     * position: position of switch
     * push_up: if true, we push the switch up. If false, push it down
     * num: switch 1 is leftmost, switch 2 is middle, switch 3 is rightmost
     * start_time: start time for trajectory
     */
    void switch_breaker(const VectorXd &start,
                        const Vector3d &position,
                        bool push_up,
                        int num,
                        double start_time);

    /*
     * eval: returns the desired task configuration. If t is outside the
     * time bounds, this returns the configuration evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * grip: populated with true if we're supposed to be gripping, false otherwise
     * force: populated with how much force to apply in the unrolled end-effector frame
     * RETURN: task state at time t
     */
    VectorXd eval(double t, bool &grip, Vector3d &force);

    /*
     * deriv1: returns the desired task velocity at time t. If t is outside the
     * time bounds, this returns the velocity evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * RETURN: velocity at time t
     */
    VectorXd deriv1(double t);

    /*
     * deriv2: returns the desired task acceleration at time t. If t is outside the 
     * time bounds, this returns the acceleration evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * RETURN: acceleration at time t
     */
    VectorXd deriv2(double t);

    /*
     * planned: returns true if plan exists, false otherwise
     * RETURN: true if plan exists, false otherwise
     */
    bool planned();

    /*
     * get_end_time: return ending time of last trajectory segment
     * RETURN: return ending time of last trajectory segment
     */
    double get_end_time();
};
