#ifndef DEVICE_FINDER_H
#define DEVICE_FINDER_H

#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include "shipbot_ros/find_device.h"

using namespace std;
using namespace Eigen;


class DeviceFinder {
  public:
    /*
     * constructor: sets the device type to NONE. Note that all thresholds should be
     * specified as [low, low, low, high, high, high]
     * ARGUMENTS
     * wheel_thresh: length-6 vector of threshold values for wheel valves
     * spigot_thresh: length-6 vector of threshold values for spigot valves
     * shuttlecock_thresh: length-6 vector of threshold values for shuttlecock valves
     * switch thresh: length-6 vector of threshold values for switches
     * wheel_thresh2: length-6 vector of threshold values for the fiducial on wheel valves
     * spigot_thresh2: length-6 vector of threshold values for the fiducial on spigot valves
     * shuttlecock_thresh2: length-6 vector of threshold values for the gray part of the shuttlecock
     * fx, fy, cx, cy, k1, k2, p1, p2, k3: intrinsics
     */
    DeviceFinder(vector<int> &wheel_thresh,
                  vector<int> &spigot_thresh,
                  vector<int> &shuttlecock_thresh,
                  vector<int> &switch_thresh,
                  vector<int> &wheel_thresh2,
                  vector<int> &spigot_thresh2,
                  vector<int> &shuttlecock_thresh2,
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
     double device_radius; // Only used for wheel and spigot

     const vector<int> wheel_thresh;
     const vector<int> spigot_thresh;
     const vector<int> shuttlecock_thresh;
     const vector<int> switch_thresh;

     const vector<int> wheel_thresh2;
     const vector<int> spigot_thresh2;
     const vector<int> shuttlecock_thresh2;

     // in meters
     const double wheel_radius = 0.047625;
     const double spigot_radius = 0.041275;
     const double shuttlecock_blue_length = 0.07;
     const double shuttlecock_blue_width = 0.0185;

     // Distance from center of bronze shuttlecock base to edge
     // of blue section, along the shuttlecock length axis
     const double shuttlecock_center_to_blue = 0.01824;

     // Depth from shuttlecock blue surface to center of
     // bronze base
     const double shuttlecock_depth = 0.01625;

     // Radius of circle connecting shuttlecock to center
     // of bronze base
     const double shuttlecock_center_rad = 0.008;

     const double switch_seph = 0.0381;
     const double switch_sepv = 0.0127;

     cv::Mat K;
     cv::Mat distortion;
 
     /*
      * findCircle: finds the pose of a circular (i.e. wheel or spigot) valve
      * ARGUMENTS
      * position: populated with the position estimate
      * orientation: populated with the orientation estimate
      * processed_image: populated with an image indicating the located device
      * image_ptr: pointer to image hypothetically containing the device
      * t: time stamp of the image
      */
     void findCircle(Vector3f &position, Quaternionf &orientation,
                     cv::Mat &processed_image,
                     shared_ptr<cv::Mat> image_ptr, double t);

     /*
      * findShuttlecock: finds the pose of a shuttlecock valve. The position
      * is set as the center of the bronze section holding the shuttlecock. The
      * shuttlecock's y-axis is along its length
      * ARGUMENTS
      * position: populated with the position estimate
      * orientation: populated with the orientation estimate
      * processed_image: populated with an image indicating the located device
      * image_ptr: pointer to image hypothetically containing the device
      * t: time stamp of the image
      */
     void findShuttlecock(Vector3f &position, Quaternionf &orientation,
                          cv::Mat &processed_image,
                          shared_ptr<cv::Mat> image_ptr, double t);

     /*
      * findSwitches: finds the pose of the leftmost switch in a breaker
      * ARGUMENTS
      * position: populated with the position estimate
      * orientation: populated with the orientation estimate
      * processed_image: populated with an image indicating the located device
      * image_ptr: pointer to image hypothetically containing the device
      * t: time stamp of the image
      */
     void findSwitches(Vector3f &position, Quaternionf &orientation,
                       cv::Mat &processed_image,
                       shared_ptr<cv::Mat> image_ptr, double t);
};

#endif
