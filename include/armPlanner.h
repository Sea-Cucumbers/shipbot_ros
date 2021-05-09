#include <Eigen/Dense>
#include <vector>
#include "minJerkInterpolator.h"
#include <geometry_msgs/Point.h>

using namespace Eigen;
using namespace std;

/*
 * ArmPlanner: currently just generates min-jerk straight lines between a given start and end
 */
class ArmPlanner {
  private:
    vector<pair<bool, MinJerkInterpolator>> segments;
    const double seconds_per_meter;
    const double seconds_per_degree;

    const double horizontal_pause_back;
    const double vertical_pause_back;
    const double vertical_pause_above;

    // How long do we wait for the jammer to grip/ungrip
    double grip_wait;

    const double shuttlecock_length = 0.08174;

    VectorXd reset_config;

    vector<pair<bool, MinJerkInterpolator>>::iterator current_segment;

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
     * grip status of the previous segment
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
     * start_plan: clears the plan and begins a new
     * plan starting at the given configuration at the given
     * time and ending at the given waypoint
     * ARGUMENTS
     * start_time: starting time of first segment
     * grip: do we grip during the first segment?
     * start: starting configuration of first segment
     * waypoint: ending configuration of first segment
     */
    void start_plan(double start_time, bool grip, const VectorXd &start, const VectorXd &waypoint);

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
     * grip_wait: how long do we wait for the gripper to grip/ungrip
     */
     ArmPlanner(double seconds_per_meter,
                double seconds_per_degree,
                const VectorXd &reset_config,
                double horizontal_pause_back,
                double vertical_pause_back,
                double vertical_pause_above,
                double grip_wait);

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
     * switch_breaker: generate task-space motion plan to manipulate a rotary valve
     * start: starting task-space configuration
     * position: position of valve center
     * push_up: if true, we push the switch up. If false, push it down
     * start_time: start time for trajectory
     */
    void switch_breaker(const VectorXd &start,
                        const Vector3d &position,
                        bool push_up,
                        double start_time);

    /*
     * eval: returns the desired task configuration. If t is outside the
     * time bounds, this returns the configuration evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * grip: populated with true if we're supposed to be gripping, false otherwise
     * RETURN: task state at time t
     */
    VectorXd eval(double t, bool &grip);

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
