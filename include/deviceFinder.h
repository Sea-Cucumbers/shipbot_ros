#ifndef DEVICE_FINDER_H
#define DEVICE_FINDER_H

#include "shipbot_ros/WheelState.h"
#include "shipbot_ros/SpigotState.h"
#include "shipbot_ros/ShuttlecockState.h"
#include "shipbot_ros/SwitchState.h"
#include <librealsense2/rs.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include "util.hpp"

using namespace std;
using namespace cv;

enum DeviceType {NONE, WHEEL, SPIGOT, SHUTTLECOCK, BREAKERA, BREAKERB};

class DeviceFinder {
  public:
    /*
     * CONSTRUCTOR: sets the current device to NONE. All thresholds should be
     * [min1, min2, min3, max1, max2, max3] where 1, 2, 3 are whatever channels
     * we're using for thresholding
     * ARGUMENTS
     * wheel_thresh: threshold for wheel valve, hls
     * shuttlecock_thresh: threshold for shuttlecock valve, hls
     * spigot_thresh: threshold for spigot valve, hls
     * switch_thresh: threshold for switch valve, hls
     * black_thresh: additional breaker threshold
     * R_c_i, T_c_i: camera intrinsics
     * K, D: camera parameters
     */
    DeviceFinder(const vector<int> &wheel_thresh,
                 const vector<int> &spigot_thresh,
                 const vector<int> &shuttlecock_thresh,
                 const vector<int> &switch_thresh,
                 const vector<int> &black_thresh,
                 const Mat &R_c_i,
                 const Vec3d &T_c_i,
                 const Mat &K,
                 const Mat &D);

    /*
     * newFrames: stores new frames
     * ARGUMENTS
     * depth_frame: depth frame
     * color_frame: color frame
     */    
    void newFrames(const rs2::frame &depth_frame, const rs2::frame &color_frame);

    /*
     * findWheel: determines state of wheel valve, and sets current device to wheel
     * ARGUMENTS
     * state: populated with state
     */
    void findWheel(shipbot_ros::WheelState &state);

    /*
     * findSpigot: determines state of spigot valve, and sets current device to spigot
     * ARGUMENTS
     * state: populated with state
     */
    void findSpigot(shipbot_ros::SpigotState &state);

    /*
     * findShuttlecock: determines state of shuttlecock valve, and sets current device to shuttlecock
     * ARGUMENTS
     * state: populated with state
     */
    void findShuttlecock(shipbot_ros::ShuttlecockState &state);

    /*
     * findSwitches: determines state of switches on breaker, and sets current device to breaker
     * ARGUMENTS
     * state1: populated with state of leftmost switch
     * state2: populated with state of middle switch
     * state3: populated with state of rightmost switch
     * typeA: is this a type A breaker
     */
    void findSwitches(shipbot_ros::SwitchState &state1,
                      shipbot_ros::SwitchState &state2,
                      shipbot_ros::SwitchState &state3,
                      bool typeA);

    /*
     * getCurrentDevice: returns the device we are currently tracking
     * RETURN: current device being trackd
     */
    DeviceType getCurrentDevice();

    /*
     * getAnnotatedImage: get image annotated with device info from the last call
     * to any of the find functions. If we've never detected a device, this function
     * does nothing
     * ARGUMENTS
     * result: populated with result, if there is one
     * RETURN: true if we've ever detected a device, false if we haven't. If false,
     * result is untouched
     */
    bool getAnnotatedImage(Mat &result);

  private:
    rs2::frame depth_frame;
    rs2::frame color_frame;

    Mat rsimagec;
    Mat rsimage;
    rs2::pointcloud pc;

    Scalar wheel_thresh_min;
    Scalar wheel_thresh_max;
    Scalar shuttlecock_thresh_min;
    Scalar shuttlecock_thresh_max;
    Scalar spigot_thresh_min;
    Scalar spigot_thresh_max;
    Scalar switch_thresh_min;
    Scalar switch_thresh_max;
    Scalar black_thresh_min;
    Scalar black_thresh_max;

    DeviceType current_device;
    Scalar thresh_min;
    Scalar thresh_max;

    vector<Blob> blobs;

    Mat rsimagec_rgb;
    Mat rsimagec_hls;
    Mat rsimagec_segmented;
    Mat im_with_keypoints;
    Mat rsimagec_segmented_rgb;
    rs2::colorizer color_map;

    // For thresholding spigot
    Mat mask2;

    // Color frame dimensions
    int w1;
    int h1;

    // Camera parameters
    Mat R_c_i;
    Vec3d T_c_i;

    // RGB intrinsics
    Mat K;
    Mat D;

    SimpleBlobDetector::Params params;
    Ptr<SimpleBlobDetector> detector;

    /*
     * setCurrentDevice: sets current device and the associated thresholds
     */
    void setCurrentDevice(DeviceType device_type);

    /*
     * processFrames: convert rs frames to cv::Mat, threshold color frame,
     * detect blobs in color frame, transform point cloud into color frame.
     * Find closest 3d point in each blob and sort by distance using a pq.
     * If we're finding a breaker, the switches are the three closest blobs.
     * Otherwise, the device is the closest blob. Clears the blobs vector and
     * populates it with the necessary blobs. Draws keypoints on the binary
     * thresholded image, storing the result in im_with_keypoints.
     */
    void processFrames();

    /*
     * rethreshAndFitEllipse: should be called after processFrames. Rethresholds
     * the color image and fits an ellipse to the largest contour. Draws the ellipse
     * onto rsimagec_rgb
     * ARGUMENTS
     * ell: populated with fitted ellipse
     * r: we only consider contours within the bounding box that surrounds a
     * circle of radius r around the best blob
     * RETURN: true if we were able to fit an ellipse, false if not
     */
     bool rethreshAndFitEllipse(cv::RotatedRect &ell, int r);
};
#endif
