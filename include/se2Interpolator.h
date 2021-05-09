#include <Eigen/Dense>

using namespace Eigen;

/*
 * SE2Interpolator: generates straight lines between a given start and end in SE(2)
 * Configurations are represented as 3-vectors (x, y, theta) 
 */
class SE2Interpolator {
  private:
    Vector3d start;
    Vector3d end;
    Vector3d vel;
    double start_time;
    double end_time;
    double delta_t;

  public:
    /*
     * constructor: generates linear interpolator between start and end. Ensures that
     * the shortest distance is taken in orientation space
     * start, end: starting and ending configurations
     * start_time, end_time: start and end times
     */
    SE2Interpolator(const Vector3d &start, const Vector3d &end, double start_time, double end_time);

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
     * error: returns the error between two states
     * ARGUMENTS
     * state1, state2: compute the errors between these states
     * RETURN: state2 - state1, but we do an angdiff for the third element
     */
    static Vector3d error(const Vector3d &state1, const Vector3d &state2);

    /*
     * get_start_time: get trajectory start time
     */
    double get_start_time();

    /*
     * get_end_time: get trajectory end time
     */
    double get_end_time();

    /*
     * get_end: get trajectory end point
     */
    const Vector3d &get_end();

    /*
     * contains_time: returns true if the given time is within the time bounds
     * RETURN: returns true if the given time is within the time bounds
     */
    bool contains_time(double t);
};
