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

    // How far back do we stop before engaging a rotary valve? How far to the side do
    // we stop before pushing a shuttlecock valve? How far above/below do we stop
    // before pushing a breaker switch?
    const double pause_dist = 0.1; 

    // How long do we wait for the jammer to grip/ungrip
    const double grip_wait = 5;

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

  public:
    /*
     * constructor
     * ARGUMENTS
     * seconds_per_meter: motions are constructed as a series of line segments.
     * how many seconds should a meter long segment take?
     * seconds_per_degree: if we're spinning a rotary valve, how many seconds
     * per degree do we spin?
     */
    ArmPlanner(double seconds_per_meter,
               double seconds_per_degree);

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
