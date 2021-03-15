#ifndef DEVICE_FINDER_H
#define DEVICE_FINDER_H

#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include "shipbot_ros/track_device.h"

using namespace std;
using namespace Eigen;


class DeviceTracker {
  public:
    /*
     * constructor: sets the device type to WHEEL. Note that all thresholds should be
     * specified as [low, low, low, high, high, high]
     * ARGUMENTS
     * wheel_thresh: length-6 vector of threshold values for wheel valves
     * spigot_thresh: length-6 vector of threshold values for spigot valves
     * shuttlecock_thresh: length-6 vector of threshold values for shuttlecock valves
     * switch thresh: length-6 vector of threshold values for switches
     * wheel_thresh2: length-6 vector of threshold values for the fiducial on wheel valves
     * spigot_thresh2: length-6 vector of threshold values for the fiducial on spigot valves
     * fx, fy, cx, cy, k1, k2, p1, p2, k3: intrinsics
     */
    DeviceTracker(vector<int> &wheel_thresh,
                  vector<int> &spigot_thresh,
                  vector<int> &shuttlecock_thresh,
                  vector<int> &switch_thresh,
                  vector<int> &wheel_thresh2,
                  vector<int> &spigot_thresh2,
                  double fx, double fy, double cx, double cy,
                  double k1, double k2, double p1, double p2, double k3);

    /*
     * findDevice: estimates the pose of the device relative to the camera
     * ARGUMENTS
     * position: populated with the position estimate
     * orientation: populated with the orientation estimate
     * processed_image: populated with an image indicating the located device
     * image_ptr: pointer to image hypothetically containing the device
     * t: time stamp of the image
     */
    void findDevice(Vector3f &position, Quaternionf &orientation,
                    cv::Mat &processed_image,
                    shared_ptr<cv::Mat> image_ptr, double t);

    /*
     * setDevice: set which device findDevice looks for
     * ARGUMENTS
     * deviceType: device type to look for next time findDevice is called
     */
    void setDevice(int deviceType);

   private:
     // Not used now, but we might want to do some filtering later on
     Vector3f position;
     Quaternionf orientation;

     int deviceType;

     vector<int> device_thresh;
     vector<int> fid_thresh;

     const vector<int> wheel_thresh;
     const vector<int> spigot_thresh;
     const vector<int> shuttlecock_thresh;
     const vector<int> switch_thresh;

     const vector<int> wheel_thresh2;
     const vector<int> spigot_thresh2;

     const double wheel_radius = 0.047625;

     cv::Mat K;
     cv::Mat distortion;
};

#endif
