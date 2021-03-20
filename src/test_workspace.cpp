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

  for (double x = left; x < right; x += 0.01) {
    for (double y = bottom; y < top; y += 0.01) {
      for (double z = front; z < back; z += 0.01) {
        if (!kd.ik(position_cmds, x, y, z, 0)) {
          cout << "Can't reach " << x << " " << y << " " << z << " with pitch 0." << endl;
        }
        if (!kd.ik(position_cmds, x, y, z, -M_PI/2)) {
          cout << "Can't reach " << x << " " << y << " " << z << " with pitch -90 degrees." << endl;
        }
      }
    }
  }
  cout << "Reached everything in the workspace!" << endl;
}

