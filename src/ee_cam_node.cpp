#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "deviceTracker.h"
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/eigen.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace std;
bool run_tracker = false;

class track_device {
  private:
    shared_ptr<DeviceTracker> tracker;

  public:
    /*
     * track_device: constructor
     * ARGUMENTS
     * tracker: pointer to device tracker whom we should tell
     * to track a new device
     */
     track_device(shared_ptr<DeviceTracker> _tracker) : tracker(_tracker) {}

    /*
     * operator (): tell the tracker to track the new device type
     * ARGUMENTS
     * req: request, telling us which device to track
     * res: technically supposed to be populated with the response, but
     * the response isn't used
     */
    bool operator () (shipbot_ros::track_device::Request &req,
                      shipbot_ros::track_device::Response &res) {
      tracker->setDevice(req.deviceType);
      run_tracker = req.deviceType != shipbot_ros::track_device::Request::NONE;
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
  ros::init(argc, argv, "ee_cam_node");
  ros::NodeHandle nh("~");

  vector<int> wheel_thresh;
  vector<int> spigot_thresh;
  vector<int> shuttlecock_thresh;
  vector<int> switch_thresh;
  vector<int> wheel_thresh2;
  vector<int> spigot_thresh2;

  nh.getParam("/wheel_thresh", wheel_thresh);
  nh.getParam("/spigot_thresh", spigot_thresh);
  nh.getParam("/shuttlecock_thresh", shuttlecock_thresh);
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

  tf2_ros::TransformBroadcaster pose_br;
  geometry_msgs::TransformStamped pose;
  pose.header.frame_id = "camera";
  pose.child_frame_id = "device pose";

  shared_ptr<DeviceTracker> tracker = make_shared<DeviceTracker>(wheel_thresh,
                                                                 spigot_thresh,
                                                                 shuttlecock_thresh,
                                                                 switch_thresh,
                                                                 wheel_thresh2,
                                                                 spigot_thresh2,
                                                                 fx, fy, cx, cx, k1, k2, p1, p2, k3);

  ros::ServiceServer track_device_service = nh.advertiseService<shipbot_ros::track_device::Request, shipbot_ros::track_device::Response>("track_device", track_device(tracker));

  shared_ptr<cv::Mat> image_ptr = make_shared<cv::Mat>();
  shared_ptr<double> time_ptr = make_shared<double>();
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/openmv_cam/image/raw", 1, handle_image(image_ptr, time_ptr));

  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/shipbot/ee_processed_image", 1);

  ros::Rate r(10);
  while (!got_image && ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }

  double start_t = ros::Time::now().toSec();
  Vector3f position(0, 0, 0);
  Quaternionf orientation(1, 0, 0, 0);
  while (ros::ok())
  {
    if (run_tracker) {
      cv_bridge::CvImagePtr processed_image_ptr = boost::make_shared<cv_bridge::CvImage>(); 
      tracker->findDevice(position, orientation, processed_image_ptr->image, image_ptr, *time_ptr);
      pose.transform.translation.x = position(0);
      pose.transform.translation.y = position(1);
      pose.transform.translation.z = position(2);
      pose.transform.rotation.x = orientation.x();
      pose.transform.rotation.y = orientation.y();
      pose.transform.rotation.z = orientation.z();
      pose.transform.rotation.w = orientation.w();

      pose.header.stamp = ros::Time::now();
      pose_br.sendTransform(pose);

      sensor_msgs::Image processed_image_msg;
      processed_image_ptr->toImageMsg(processed_image_msg);
      processed_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
      image_pub.publish(processed_image_msg);
    }

    r.sleep();
    ros::spinOnce();
  }
}

