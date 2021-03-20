#include "deviceFinder.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>
#include "shipbot_ros/find_device.h"

DeviceFinder::DeviceFinder(vector<int> &wheel_thresh,
                             vector<int> &spigot_thresh,
                             vector<int> &shuttlecock_thresh,
                             vector<int> &switch_thresh,
                             vector<int> &wheel_thresh2,
                             vector<int> &spigot_thresh2,
                             vector<int> &shuttlecock_thresh2,
                             double fx, double fy, double cx, double cy,
                             double k1, double k2, double p1, double p2, double k3) : 
                                                            wheel_thresh(wheel_thresh),
                                                            spigot_thresh(spigot_thresh),
                                                            shuttlecock_thresh(shuttlecock_thresh),
                                                            switch_thresh(switch_thresh),
                                                            wheel_thresh2(wheel_thresh2),
                                                            spigot_thresh2(spigot_thresh2),
                                                            shuttlecock_thresh2(shuttlecock_thresh2),
                                                            position(0, 0, 0), 
                                                            orientation(1, 0, 0, 0) {
  K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 1) = fy;
  K.at<float>(1, 2) = cy;

  distortion = cv::Mat::zeros(1, 5, CV_32F);
  distortion.at<float>(0, 0) = k1;
  distortion.at<float>(0, 1) = k2;
  distortion.at<float>(0, 2) = p1;
  distortion.at<float>(0, 3) = p2;
  distortion.at<float>(0, 4) = k3;

  setDevice(shipbot_ros::find_device::Request::NONE);
}

void DeviceFinder::findDevice(Vector3f &position, Quaternionf &orientation,
                               cv::Mat &processed_image,
                               shared_ptr<cv::Mat> image_ptr, double t) {
  // TODO: most of the code in these functions is repeat. Figure out a better way to 
  // divide functionality
  if (deviceType == shipbot_ros::find_device::Request::WHEEL ||
      deviceType == shipbot_ros::find_device::Request::SPIGOT) {
    findCircle(position, orientation, processed_image, image_ptr, t);
  } else if (deviceType == shipbot_ros::find_device::Request::SHUTTLECOCK) {
    findShuttlecock(position, orientation, processed_image, image_ptr, t);
  } else if (deviceType == shipbot_ros::find_device::Request::SWITCH) {
    findSwitches(position, orientation, processed_image, image_ptr, t);
  } else {
    processed_image = image_ptr->clone();
  }
}

void DeviceFinder::setDevice(int deviceType) {
  this->deviceType = deviceType;
  switch (deviceType) {
    case shipbot_ros::find_device::Request::WHEEL: {
      device_thresh = wheel_thresh;
      fid_thresh = wheel_thresh2;
      device_radius = wheel_radius;
      break;
    } case shipbot_ros::find_device::Request::SPIGOT: {
      device_thresh = spigot_thresh;
      fid_thresh = spigot_thresh2;
      device_radius = spigot_radius;
      break;
    } case shipbot_ros::find_device::Request::SHUTTLECOCK: {
      device_thresh = shuttlecock_thresh;
      break;
    } case shipbot_ros::find_device::Request::SWITCH: {
      device_thresh = switch_thresh;
      break;
    }
  }
}

void DeviceFinder::findCircle(Vector3f &position, Quaternionf &orientation,
                               cv::Mat &processed_image,
                               shared_ptr<cv::Mat> image_ptr, double t) {
  processed_image = image_ptr->clone();

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

  if (contours[best_idx].size() < 5) {
    return;
  }

  cv::RotatedRect ell = fitEllipse(contours[best_idx]);
  ellipse(processed_image, ell, cv::Scalar(0, 255, 0), 2);

  vector<cv::Point3f> objectPoints;
  objectPoints.push_back(cv::Point3f(-device_radius, 0, 0));
  objectPoints.push_back(cv::Point3f(device_radius, 0, 0));
  objectPoints.push_back(cv::Point3f(0, -device_radius, 0));
  objectPoints.push_back(cv::Point3f(0, device_radius, 0));

  vector<cv::Point2f> imagePoints;
  double c = cos(ell.angle);
  double s = sin(ell.angle);
  cv::Point2f hvec(ell.size.height*c/2, ell.size.height*s/2);
  cv::Point2f wvec(-ell.size.width*s/2, ell.size.width*c/2);
  imagePoints.push_back(ell.center + hvec);
  imagePoints.push_back(ell.center - hvec);
  imagePoints.push_back(ell.center + wvec);
  imagePoints.push_back(ell.center - wvec);

  cv::Mat rvec;
  cv::Mat tvec;

  cv::solvePnP(objectPoints, imagePoints, K, distortion, rvec, tvec);

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  Matrix3f rmat_eigen;
  cv::cv2eigen(rmat, rmat_eigen);
  cv::cv2eigen(tvec, position);

  rmat_eigen.col(0) = -rmat_eigen.col(2).cross(Vector3f(0, 1, 0)).normalized();
  rmat_eigen.col(1) = rmat_eigen.col(2).cross(rmat_eigen.col(0)).normalized();

  //orientation = Quaternionf(rmat_eigen);

  cout << position/0.0254 << endl << endl;

  this->position = position;
  this->orientation = orientation;
}

void DeviceFinder::findShuttlecock(Vector3f &position, Quaternionf &orientation,
                                    cv::Mat &processed_image,
                                    shared_ptr<cv::Mat> image_ptr, double t) {
  processed_image = image_ptr->clone();

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

  if (contours[best_idx].size() < 5) {
    return;
  }

  cv::RotatedRect rect = cv::minAreaRect(contours[best_idx]);

  // Draw rectangle
  cv::Point2f rect_points[4];
  rect.points(rect_points);
  for (int j = 0; j < 4; j++)
  {
    cv::line(processed_image, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0));
  }

  vector<cv::Point3f> objectPoints;
  objectPoints.push_back(cv::Point3f(-shuttlecock_blue_width, 0, 0));
  objectPoints.push_back(cv::Point3f(shuttlecock_blue_width, 0, 0));
  objectPoints.push_back(cv::Point3f(0, -shuttlecock_blue_length, 0));
  objectPoints.push_back(cv::Point3f(0, shuttlecock_blue_length, 0));

  vector<cv::Point2f> imagePoints;
  double c = cos(rect.angle);
  double s = sin(rect.angle);
  double width;
  double length;
  if (rect.size.height > rect.size.width) {
    length = rect.size.height;
    width = rect.size.width;
  } else {
    length = rect.size.width;
    width = rect.size.height;
  }
  cv::Point2f hvec(length*c/2, length*s/2);
  cv::Point2f wvec(-width*s/2, width*c/2);
  imagePoints.push_back(rect.center + hvec);
  imagePoints.push_back(rect.center - hvec);
  imagePoints.push_back(rect.center + wvec);
  imagePoints.push_back(rect.center - wvec);

  cv::Mat rvec;
  cv::Mat tvec;

  cv::solvePnP(objectPoints, imagePoints, K, distortion, rvec, tvec);

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  Matrix3f rmat_eigen;
  cv::cv2eigen(rmat, rmat_eigen);
  cv::cv2eigen(tvec, position);

  rmat_eigen.col(0) = -rmat_eigen.col(2).cross(Vector3f(0, 1, 0)).normalized();
  rmat_eigen.col(1) = rmat_eigen.col(2).cross(rmat_eigen.col(0)).normalized();

  //orientation = Quaternionf(rmat_eigen);
  cout << position/0.0254 << endl << endl;

  this->position = position;
  this->orientation = orientation;
}

void DeviceFinder::findSwitches(Vector3f &position, Quaternionf &orientation,
                                 cv::Mat &processed_image,
                                 shared_ptr<cv::Mat> image_ptr, double t) {}
