#include <Eigen/Dense>
#include <vector>
#include "se2Interpolator.h"

using namespace Eigen;
using namespace std;

/*
 * ChassisPlanner: interpolates between a start and an end. If the interpolation would
 * cause a collision since we're near a wall, moves to the center of the testbed first
 */
class ChassisPlanner {
  private:
    vector<SE2Interpolator> segments;
    const double linear_speed;
    const double angular_speed;
    const double robot_width = 0.381;

    const double long_wall = 1.568;
    const double short_wall = 0.95;

    vector<SE2Interpolator>::iterator current_segment;

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
     * ends at the given waypoint
     * ARGUMENTS
     * waypoint: waypoint which we append to plan
     */
    void add_waypoint(const Vector3d &waypoint);

  public:
    /*
     * constructor
     * ARGUMENTS
     * linear_speed: max linear speed between waypoints
     * angular_speed: max rotation speed between waypoints (rad/s)
     */
     ChassisPlanner(double linear_speed,
                    double angular_speed);

    /*
     * eval: returns the desired configuration. If t is outside the
     * time bounds, this returns the configuration evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * RETURN: task state at time t
     */
    Vector3d eval(double t);

    /*
     * deriv1: returns the desired velocity at time t. If t is outside the
     * time bounds, this returns the velocity evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * RETURN: velocity at time t
     */
    Vector3d deriv1(double t);

    /*
     * plan: clears the plan and begins a new
     * plan starting at the given configuration at the given
     * time and ending at the given goal
     * ARGUMENTS
     * start_time: starting time of first segment
     * start: starting configuration of first segment
     * waypoint: ending configuration of first segment
     */
    void plan(double start_time, const Vector3d &start, const Vector3d &goal);

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

    /*
     * error: returns the error between two states
     * ARGUMENTS
     * state1, state2: compute the errors between these states
     * RETURN: state2 - state1, but we do an angdiff for the third element
     */
    static Vector3d error(const Vector3d &state1, const Vector3d &state2);
};
