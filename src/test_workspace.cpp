#include "kdHelper.h"
#include <ros/ros.h>
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_workspace");
  ros::NodeHandle nh("~");

  string urdf_file;
  double left;
  double right;
  double top;
  double bottom;
  double front;
  double back;

  nh.getParam("urdf", urdf_file);
  nh.getParam("left", left);
  nh.getParam("right", right);
  nh.getParam("top", top);
  nh.getParam("bottom", bottom);
  nh.getParam("front", front);
  nh.getParam("back", back);
  KDHelper kd(urdf_file);

  VectorXd position_cmds = VectorXd::Zero(kd.get_actuator_names().size());

  VectorXd task_config = VectorXd::Zero(5);
  for (double x = left; x < right; x += 0.01) {
    task_config(0) = x;
    for (double y = bottom; y < top; y += 0.01) {
      task_config(0) = y;
      for (double z = front; z < back; z += 0.01) {
        task_config(2) = z;
        task_config(3) = 0;
        if (!kd.ik(position_cmds, task_config)) {
          cout << "Can't reach " << x << " " << y << " " << z << " with pitch 0." << endl;
        }
        task_config(3) = -M_PI/2;
        if (!kd.ik(position_cmds, task_config)) {
          cout << "Can't reach " << x << " " << y << " " << z << " with pitch -90 degrees." << endl;
        }
      }
    }
  }
  cout << "Reached everything in the workspace!" << endl;
}

