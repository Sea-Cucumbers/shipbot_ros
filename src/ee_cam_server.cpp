#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include "deviceFinder.h"
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/eigen.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace std;

class find_device {
  private:
    shared_ptr<DeviceFinder> finder;
    shared_ptr<cv::Mat> image_ptr;

  public:
    /*
     * find_device: constructor
     * ARGUMENTS
     * finder: pointer to device finder
     * image_ptr: pointer to image we should use to find device
     */
     find_device(shared_ptr<DeviceFinder> _finder,
                 shared_ptr<cv::Mat> _image_ptr) : finder(_finder),
                                                   image_ptr(_image_ptr) {}

    /*
     * operator (): gets device location from finder
     * ARGUMENTS
     * req: request, contains device type
     * res: populated with 3d position estimate of device center
     */
    bool operator () (shipbot_ros::find_device::Request &req,
                      shipbot_ros::find_device::Response &res) {
      finder->setDevice(req.deviceType);
      Vector3f position;
      Quaternionf orientation;
      cv::Mat processed_image;
      finder->findDevice(position, orientation, processed_image, image_ptr, 0);
      res.position.x = position(0);
      res.position.y = position(1);
      res.position.z = position(2);
    }
};

static bool got_image;

/* 
 * handle_image: image message handler
 */
class handle_image {
  private:
    shared_ptr<cv::Mat> cv_ptr;
    shared_ptr<double> time_ptr;
    
  public:
    /*
     * handle_image: constructor
     * ARGUMENTS
     * _cv_ptr: the pointer we should populate with the received image
     */
    handle_image(shared_ptr<cv::Mat> _cv_ptr,
                 shared_ptr<double> _time_ptr) : cv_ptr(_cv_ptr),
                                                 time_ptr(_time_ptr) {}

    /*
     * operator (): populate our message pointer with the received message
     * ARGUMENTS
     * image_ptr: pointer to received message
     */
    void operator () (const sensor_msgs::ImageConstPtr &image_ptr) {
      cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(*image_ptr, sensor_msgs::image_encodings::BGR8);
      *cv_ptr = bridge_ptr->image;
      *time_ptr = image_ptr->header.stamp.toSec();
      got_image = true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ee_cam_server");
  ros::NodeHandle nh("~");

  vector<int> wheel_thresh;
  vector<int> spigot_thresh;
  vector<int> shuttlecock_thresh;
  vector<int> shuttlecock_thresh2;
  vector<int> switch_thresh;
  vector<int> wheel_thresh2;
  vector<int> spigot_thresh2;

  nh.getParam("/wheel_thresh", wheel_thresh);
  nh.getParam("/spigot_thresh", spigot_thresh);
  nh.getParam("/shuttlecock_thresh", shuttlecock_thresh);
  nh.getParam("/shuttlecock_thresh2", shuttlecock_thresh2);
  nh.getParam("/switch_thresh", switch_thresh);
  nh.getParam("/wheel_thresh2", wheel_thresh2);
  nh.getParam("/spigot_thresh2", spigot_thresh2);

  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  nh.getParam("/fx", fx);
  nh.getParam("/fy", fy);
  nh.getParam("/cx", cx);
  nh.getParam("/cy", cy);
  nh.getParam("/k1", k1);
  nh.getParam("/k2", k2);
  nh.getParam("/p1", p1);
  nh.getParam("/p2", p2);
  nh.getParam("/k3", k3);

  shared_ptr<DeviceFinder> finder = make_shared<DeviceFinder>(wheel_thresh,
                                                              spigot_thresh,
                                                              shuttlecock_thresh,
                                                              switch_thresh,
                                                              wheel_thresh2,
                                                              spigot_thresh2,
                                                              shuttlecock_thresh2,
                                                              fx, fy, cx, cx, k1, k2, p1, p2, k3);

  shared_ptr<cv::Mat> image_ptr = make_shared<cv::Mat>();
  shared_ptr<double> time_ptr = make_shared<double>();
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/openmv_cam/image/raw", 1, handle_image(image_ptr, time_ptr));

  ros::Rate r(10);
  while (!got_image && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }

  ros::ServiceServer find_device_service = nh.advertiseService<shipbot_ros::find_device::Request,
                                                               shipbot_ros::find_device::Response>("find_device",
                                                                                                   find_device(finder, image_ptr));

  ros::spin();
}

