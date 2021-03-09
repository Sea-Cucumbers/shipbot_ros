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
  setDevice(WHEEL);
}

void DeviceFinder::findDevice(Vector3d &position, Quaterniond &orientation,
                              cv::Mat &processed_image,
                              shared_ptr<cv::Mat> image_ptr, double t) {
  cvtColor(*image_ptr, processed_image, cv::COLOR_BGR2HSV);

  vector<int> thresh;
  switch (deviceType) {
    case WHEEL: {
      thresh = wheel_thresh;
      break;
    } case SPIGOT: {
      thresh = spigot_thresh;
      break;
    } case SHUTTLECOCK: {
      thresh = shuttlecock_thresh;
      break;
    } case SWITCH: {
      thresh = switch_thresh;
      break;
    }
  }
  cv::inRange(processed_image, cv::Scalar(thresh[0], thresh[1], thresh[2]), cv::Scalar(thresh[3], thresh[4], thresh[5]), processed_image);

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(processed_image, keypoints);
  cout << keypoints.size() << endl;
  cv::cvtColor(processed_image, processed_image, cv::COLOR_GRAY2BGR);
  cv::drawKeypoints(processed_image, keypoints, processed_image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
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
}
