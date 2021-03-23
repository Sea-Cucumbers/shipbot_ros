#include <Eigen/Dense>

using namespace Eigen;

/*
 * ArmPlanner: currently just generates min-jerk straight lines between a given start and end
 */
class ArmPlanner {
  private:
    VectorXd start;
    VectorXd end;
    double start_time;
    double end_time;
    double delta_t;
    bool plan_exists;

  public:
    /*
     * constructor
     */
    ArmPlanner();

    /*
     * plan: generate task-space motion plan between start and end
     * start, end: starting and ending (x, y, z, pitch, roll)
     * start_time, end_time: start and end times
     */
    void plan(const VectorXd &start, const VectorXd &end, double start_time, double end_time);

    /*
     * eval: returns the desired task configuration. If t is outside the
     * time bounds, this returns the configuration evaluated at the nearest time bound
     * ARGUMENTS
     * t: time to evaluate
     * RETURN: task state at time t
     */
    VectorXd eval(double t);

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
};
