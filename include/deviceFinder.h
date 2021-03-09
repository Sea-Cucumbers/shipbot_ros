#ifndef DEVICE_FINDER_H
#define DEVICE_FINDER_H

#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>

using namespace std;
using namespace Eigen;

// TODO: how do we handle multiple switches?
enum DeviceType {WHEEL, SPIGOT, SHUTTLECOCK, SWITCH};

class DeviceFinder {
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
     */
    DeviceFinder(vector<int> &wheel_thresh,
                 vector<int> &spigot_thresh,
                 vector<int> &shuttlecock_thresh,
                 vector<int> &switch_thresh,
                 vector<int> &wheel_thresh2,
                 vector<int> &spigot_thresh2);

    /*
     * findDevice: estimates the pose of the device relative to the camera
     * ARGUMENTS
     * position: populated with the position estimate
     * orientation: populated with the orientation estimate
     * processed_image: populated with an image indicating the device with a bounding box
     * image_ptr: pointer to image hypothetically containing the device
     * t: time stamp of the image
     */
    void findDevice(Vector3d &position, Quaterniond &orientation,
                    cv::Mat &processed_image,
                    shared_ptr<cv::Mat> image_ptr, double t);

    /*
     * setDevice: set which device findDevice looks for
     * ARGUMENTS
     * deviceType: device type to look for next time findDevice is called
     */
    void setDevice(DeviceType deviceType);

   private:
     // Not used now, but we might want to do some filtering later on
     Vector3d position;
     Quaterniond orientation;

     DeviceType deviceType;

     vector<int> wheel_thresh;
     vector<int> spigot_thresh;
     vector<int> shuttlecock_thresh;
     vector<int> switch_thresh;

     vector<int> wheel_thresh2;
     vector<int> spigot_thresh2;

     cv::SimpleBlobDetector::Params blobParams;
     cv::Ptr<cv::SimpleBlobDetector> detector;
};

#endif
