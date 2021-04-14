#include "deviceFinder.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "shipbot_ros/QueryWheel.h"
#include "shipbot_ros/QuerySpigot.h"
#include "shipbot_ros/QueryBreaker.h"
#include "shipbot_ros/QueryShuttlecock.h"
#include <std_srvs/SetBool.h>
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;

class query_wheel {
  private:
    shared_ptr<DeviceFinder> finder;

  public:
    /*
     * query_wheel: constructor
     * ARGUMENTS
     * finder: pointer to device finder
     */
    query_wheel(shared_ptr<DeviceFinder> _finder) : finder(_finder) {}

    /*
     * operator (): tell the finder to find a wheel valve
     * ARGUMENTS
     * req: request
     * res: response containing device state
     */
    bool operator () (shipbot_ros::QueryWheel::Request &req,
                      shipbot_ros::QueryWheel::Response &res) {
      finder->findWheel(res.state);
      return true;
    }
};

class query_spigot {
  private:
    shared_ptr<DeviceFinder> finder;

  public:
    /*
     * query_spigot: constructor
     * ARGUMENTS
     * finder: pointer to device finder
     */
    query_spigot(shared_ptr<DeviceFinder> _finder) : finder(_finder) {}

    /*
     * operator (): tell the finder to find a spigot valve
     * ARGUMENTS
     * req: request
     * res: response containing device state
     */
    bool operator () (shipbot_ros::QuerySpigot::Request &req,
                      shipbot_ros::QuerySpigot::Response &res) {
      finder->findSpigot(res.state);
      return true;
    }
};

class query_shuttlecock {
  private:
    shared_ptr<DeviceFinder> finder;

  public:
    /*
     * QueryShuttlecock: constructor
     * ARGUMENTS
     * finder: pointer to device finder
     */
    query_shuttlecock(shared_ptr<DeviceFinder> _finder) : finder(_finder) {}

    /*
     * operator (): tell the finder to find a shuttlecock valve
     * ARGUMENTS
     * req: request
     * res: response containing device state
     */
    bool operator () (shipbot_ros::QueryShuttlecock::Request &req,
                      shipbot_ros::QueryShuttlecock::Response &res) {
      finder->findShuttlecock(res.state);
      return true;
    }
};

class query_breaker {
  private:
    shared_ptr<DeviceFinder> finder;

  public:
    /*
     * query_breaker: constructor
     * ARGUMENTS
     * finder: pointer to device finder
     */
    query_breaker(shared_ptr<DeviceFinder> _finder) : finder(_finder) {}

    /*
     * operator (): tell the finder to find breaker switches
     * ARGUMENTS
     * req: request
     * res: response containing device state
     */
    bool operator () (shipbot_ros::QueryBreaker::Request &req,
                      shipbot_ros::QueryBreaker::Response &res) {
      finder->findSwitches(res.state1, res.state2, res.state3, true);
      return true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense_node");
  ros::NodeHandle nh("~");

  rs2::pointcloud pc;
  rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
	rs2::pipeline_profile profile;
  int flag = 0;

	while (flag == 0){
		try {
			profile = pipe.start();
			flag = 1;
		} catch (const rs2::error & e) {
			std::cout << "Could not set Vars, will try again." << std::endl;
			std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		} catch (const std::exception & e) {
			std::cout << "Could not set Vars, will try again." << std::endl;
			std::cerr << e.what() << std::endl;
		}
	}
	std::cout << "RealSense found!" << std::endl;

  // Find camera parameters, print them out
  auto ir1_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::stream_profile>();
	rs2_extrinsics extrinsics = get_extrinsics(ir1_stream, color_stream);
	Mat R_c_i = get_rotation(extrinsics);
	Vec3d T_c_i = get_translation(extrinsics);
	std::cout << "R_c_i = " << R_c_i << std::endl;
	std::cout << "T_c_i = " << T_c_i << std::endl;

  // Try retrieving rgb intrinsics
	auto intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
  Mat K, D;
  double kd[9] = {615.084, 0.0, 323.046, 0.0, 615.019, 242.301, 0.0,0.0,1.0};
  double dd[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  K = Mat(3,3,CV_64F,kd);
  D = Mat(1,5,CV_64F,dd);
	std::cout << "K = " << K << std::endl;
	std::cout << "D = " << D << std::endl;

  // Read thresholds
  vector<int> wheel_thresh;
  vector<int> shuttlecock_thresh;
  vector<int> spigot_thresh;
  vector<int> switch_thresh;
  vector<int> black_thresh;
  nh.getParam("/wheel_thresh", wheel_thresh);
  nh.getParam("/spigot_thresh", spigot_thresh);
  nh.getParam("/shuttlecock_thresh", shuttlecock_thresh);
  nh.getParam("/switch_thresh", switch_thresh);
  nh.getParam("/black_thresh", black_thresh);

  // Initialize device services, and a service that toggles continuous publishing
  shared_ptr<DeviceFinder> finder = make_shared<DeviceFinder>(wheel_thresh, spigot_thresh, shuttlecock_thresh, switch_thresh, black_thresh, R_c_i, T_c_i, K, D);
  ros::ServiceServer query_wheel_service = nh.advertiseService<shipbot_ros::QueryWheel::Request, shipbot_ros::QueryWheel::Response>("query_wheel", query_wheel(finder));
  ros::ServiceServer query_spigot_service = nh.advertiseService<shipbot_ros::QuerySpigot::Request, shipbot_ros::QuerySpigot::Response>("query_spigot", query_spigot(finder));
  ros::ServiceServer query_shuttlecock_service = nh.advertiseService<shipbot_ros::QueryShuttlecock::Request, shipbot_ros::QueryShuttlecock::Response>("query_shuttlecock", query_shuttlecock(finder));
  ros::ServiceServer query_breaker_service = nh.advertiseService<shipbot_ros::QueryBreaker::Request, shipbot_ros::QueryBreaker::Response>("query_breaker", query_breaker(finder));

  // Messages we use for continuous publishing
  shipbot_ros::WheelState wheel_state;
  shipbot_ros::SpigotState spigot_state;
  shipbot_ros::ShuttlecockState shuttlecock_state;
  shipbot_ros::SwitchState switch1_state;
  shipbot_ros::SwitchState switch2_state;
  shipbot_ros::SwitchState switch3_state;

  // Continuous publishers
  ros::Publisher wheel_pub = nh.advertise<shipbot_ros::WheelState>("/shipbot/wheel_state", 1);
  ros::Publisher spigot_pub = nh.advertise<shipbot_ros::SpigotState>("/shipbot/spigot_state", 1);
  ros::Publisher shuttlecock_pub = nh.advertise<shipbot_ros::ShuttlecockState>("/shipbot/shuttlecock_state", 1);
  ros::Publisher switch1_pub = nh.advertise<shipbot_ros::SwitchState>("/shipbot/switch1_state", 1);
  ros::Publisher switch2_pub = nh.advertise<shipbot_ros::SwitchState>("/shipbot/switch2_state", 1);
  ros::Publisher switch3_pub = nh.advertise<shipbot_ros::SwitchState>("/shipbot/switch3_state", 1);

  // Publish image if desired
  bool pub_image = false;
  nh.getParam("pub_image", pub_image);
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/shipbot/rs_image", 1);
  cv_bridge::CvImagePtr rs_image_ptr = boost::make_shared<cv_bridge::CvImage>();
  rs_image_ptr->encoding = "bgr8";

  bool continuous_publishing = false;
  nh.getParam("continuous_publishing", continuous_publishing);

  ros::Rate r(10);
  while(ros::ok()) {
    // Wait for the next set of frames from RealSense
    auto frames = pipe.wait_for_frames();

    rs2::frame depth_frame = frames.get_depth_frame();
    rs2::frame color_frame = frames.get_color_frame();
    finder->newFrames(depth_frame, color_frame);

    if (continuous_publishing) {
      DeviceType current_device = finder->getCurrentDevice();
      if (current_device == WHEEL) {
        finder->findWheel(wheel_state);
        wheel_pub.publish(wheel_state);
      } else if (current_device == SPIGOT) {
        finder->findSpigot(spigot_state);
        spigot_pub.publish(spigot_state);
      } else if (current_device == SHUTTLECOCK) {
        finder->findShuttlecock(shuttlecock_state);
        shuttlecock_pub.publish(shuttlecock_state);
      } else if (current_device == BREAKERA) {
        finder->findSwitches(switch1_state, switch2_state, switch3_state, true);
        switch1_pub.publish(switch1_state);
        switch2_pub.publish(switch2_state);
        switch3_pub.publish(switch3_state);
      } else if (current_device == BREAKERB) {
        finder->findSwitches(switch1_state, switch2_state, switch3_state, false);
        switch1_pub.publish(switch1_state);
        switch2_pub.publish(switch2_state);
        switch3_pub.publish(switch3_state);
      }

      if (pub_image) {
        bool image_exists = finder->getAnnotatedImage(rs_image_ptr->image);
        if (image_exists) {
          image_pub.publish(rs_image_ptr->toImageMsg());
        }
      }
    }

    r.sleep();
    ros::spinOnce();
  }
  
  ros::spin();
}

