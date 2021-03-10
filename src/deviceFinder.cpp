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
  blobParams.filterByColor = true;
  blobParams.blobColor = 255;
  blobParams.filterByArea = true;

  fid_blobParams.filterByColor = true;
  fid_blobParams.blobColor = 255;
  fid_blobParams.filterByArea = false;
  fid_blobParams.minArea = 25;
  fid_blobParams.maxArea = 1000;
  fid_blobParams.filterByCircularity = false;
  fid_blobParams.filterByConvexity = false;
  fid_blobParams.filterByInertia = false;
  fid_detector = cv::SimpleBlobDetector::create(fid_blobParams);

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

  vector<cv::KeyPoint> keypoints;
  detector->detect(thresh1, keypoints);

  if (keypoints.size() == 0) {
    return;
  }

  cv::KeyPoint best = keypoints[0];
  for (vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
    if (it->size > best.size) {
      best = *it;
    }
  }

  // Circle the device
  cv::circle(processed_image, best.pt, best.size/2, cv::Scalar(0, 255, 0), 5, cv::LINE_AA);

  // We've found the circle we're looking for. Search its rim for the fiducial
  cv::Mat mask = cv::Mat::zeros(image_ptr->rows, image_ptr->cols, CV_8UC1);
  circle(mask, best.pt, best.size/2, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
  cv::Mat mask2 = cv::Mat::zeros(image_ptr->rows, image_ptr->cols, CV_8UC1);
  circle(mask2, best.pt, best.size/3, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
  cv::bitwise_xor(mask, mask2, mask);
  
  cv::Mat thresh2;
  cv::inRange(hsv, cv::Scalar(fid_thresh[0], fid_thresh[1], fid_thresh[2]), cv::Scalar(fid_thresh[3], fid_thresh[4], fid_thresh[5]), thresh2);
  cv::bitwise_and(mask, thresh2, thresh2);
  cv::bitwise_not(thresh1, thresh1);
  cv::bitwise_and(thresh1, thresh2, thresh2);

  // Circle the fiducial
  fid_detector->detect(thresh2, keypoints);
  if (keypoints.size() == 0) {
    return;
  }
  best = keypoints[0];
  for (vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
    if (it->size > best.size) {
      best = *it;
    }
  }

  cv::circle(processed_image, best.pt, best.size/2, cv::Scalar(255, 0, 0), 5, cv::LINE_AA);
}

void DeviceFinder::setDevice(DeviceType deviceType) {
  this->deviceType = deviceType;

  // For circular types, look for circles (or ellipses)
  if (deviceType == WHEEL || deviceType == SPIGOT) {
    blobParams.filterByCircularity = true;
    blobParams.minCircularity = 0.1;

    blobParams.filterByConvexity = true;
    blobParams.minConvexity = 0.87;

    blobParams.filterByInertia = true;
    blobParams.minInertiaRatio = 0.5;

    blobParams.minArea = 400;
    blobParams.maxArea = 100000;

    if (deviceType == SPIGOT) {
      blobParams.minArea = 100;
    }
  } else {
    blobParams.filterByCircularity = false;
    if (deviceType == SHUTTLECOCK) {
      blobParams.filterByInertia = true;
      blobParams.minInertiaRatio = 0.01;
    } else {
      blobParams.filterByInertia = false;
    }
  }
  detector = cv::SimpleBlobDetector::create(blobParams);

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
