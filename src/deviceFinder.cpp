#include "deviceFinder.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>

DeviceFinder::DeviceFinder(vector<int> &wheel_thresh,
                           vector<int> &spigot_thresh,
                           vector<int> &shuttlecock_thresh,
                           vector<int> &switch_thresh,
                           vector<int> &wheel_thresh2,
                           vector<int> &spigot_thresh2) : wheel_thresh(wheel_thresh),
                                                          spigot_thresh(spigot_thresh),
                                                          shuttlecock_thresh(shuttlecock_thresh),
                                                          switch_thresh(switch_thresh),
                                                          wheel_thresh2(wheel_thresh2),
                                                          spigot_thresh2(spigot_thresh2),
                                                          position(0, 0, 0), orientation(1, 0, 0, 0) {
  setDevice(WHEEL);
}

void DeviceFinder::findDevice(Vector3d &position, Quaterniond &orientation,
                              cv::Mat &processed_image,
                              shared_ptr<cv::Mat> image_ptr, double t) {
  processed_image = *image_ptr;

  cv::Mat hsv;
  cvtColor(*image_ptr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat thresh1;
  cv::inRange(hsv, cv::Scalar(device_thresh[0], device_thresh[1], device_thresh[2]), cv::Scalar(device_thresh[3], device_thresh[4], device_thresh[5]), thresh1);

  vector<vector<cv::Point>> contours;
  findContours(thresh1, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  if (contours.size() == 0) {
    return;
  }
  double best_area = cv::contourArea(contours[0]);
  size_t best_idx = 0;
  for (size_t i = 1; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > best_area) {
      best_idx = i;
      best_area = area;
    }
  }

  cv::RotatedRect ell = fitEllipse(contours[best_idx]);
  ellipse(processed_image, ell, cv::Scalar(0, 255, 0), 2);
}

void DeviceFinder::setDevice(DeviceType deviceType) {
  this->deviceType = deviceType;
  switch (deviceType) {
    case WHEEL: {
      device_thresh = wheel_thresh;
      fid_thresh = wheel_thresh2;
      break;
    } case SPIGOT: {
      device_thresh = spigot_thresh;
      fid_thresh = spigot_thresh2;
      break;
    } case SHUTTLECOCK: {
      device_thresh = shuttlecock_thresh;
      break;
    } case SWITCH: {
      device_thresh = switch_thresh;
      break;
    }
  }
}
