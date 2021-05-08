#include "deviceFinder.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "shipbot_ros/WheelState.h"
#include "shipbot_ros/SpigotState.h"
#include "shipbot_ros/SwitchState.h"
#include "shipbot_ros/BreakerState.h"
#include "shipbot_ros/ShuttlecockState.h"
#include "shipbot_ros/FindDevice.h"
#include <librealsense2/rs.hpp>
#include "util.hpp"             // Get extrinsics
#include <algorithm>            // std::min, std::max
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <iostream>

using namespace std;
using namespace cv;

class find_device {
  private:
    shared_ptr<int> device_type;

  public:
    /*
     * find_device: constructor
     * ARGUMENTS
     * _device_type: pointer to device type
     */
    find_device(shared_ptr<int> _device_type) : device_type(_device_type) {}

    /*
     * operator (): tell the node to find some device
     * ARGUMENTS
     * req: request containing device type
     * res: response, not used
     */
    bool operator () (shipbot_ros::FindDevice::Request &req,
                      shipbot_ros::FindDevice::Response &res) {
      *device_type = req.device_type;
      return true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense_node");
  ros::NodeHandle nh("~");

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

  Scalar blue_min(95, 21, 191);
  Scalar blue_max(110, 120,255);

  // blue, original, shuttlecock
  Scalar blue_min_2(98, 25, 160);
  Scalar blue_max_2(110, 178,255);

  // lime, original, spigot
  Scalar lime_min(40, 64, 25);
  Scalar lime_max(80, 204, 77);

  // orange, original
  Scalar orange_min(5,77,102);
  Scalar orange_max(15, 204, 255);

  // black, original, rgb
  Scalar black_min(0,0,0);
  Scalar black_max(40, 45, 60);

  Scalar thresh_min, thresh_max;


  // Initialize device services, and a service that toggles continuous publishing
  shared_ptr<int> device_type = make_shared<int>(shipbot_ros::FindDevice::Request::NONE);;
  ros::ServiceServer find_device_service = nh.advertiseService<shipbot_ros::FindDevice::Request, shipbot_ros::FindDevice::Response>("find_device", find_device(device_type));

  // Messages we use for continuous publishing
  shipbot_ros::WheelState wheel_state;
  shipbot_ros::SpigotState spigot_state;
  shipbot_ros::ShuttlecockState shuttlecock_state;
  shipbot_ros::BreakerState breaker_state;
  breaker_state.switches = vector<shipbot_ros::SwitchState>(3);

  // Continuous publishers
  ros::Publisher wheel_pub = nh.advertise<shipbot_ros::WheelState>("/shipbot/wheel_state", 1);
  ros::Publisher spigot_pub = nh.advertise<shipbot_ros::SpigotState>("/shipbot/spigot_state", 1);
  ros::Publisher shuttlecock_pub = nh.advertise<shipbot_ros::ShuttlecockState>("/shipbot/shuttlecock_state", 1);
  ros::Publisher breaker_pub = nh.advertise<shipbot_ros::BreakerState>("/shipbot/breaker_state", 1);

  // Publish image if desired
  bool pub_image = false;
  nh.getParam("pub_image", pub_image);
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/shipbot/rs_image", 1);
  cv_bridge::CvImagePtr rs_image_ptr = boost::make_shared<cv_bridge::CvImage>();
  rs_image_ptr->encoding = "bgr8";

  rs2::pointcloud pc;
  rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
	rs2::pipeline_profile profile;
	rs2::colorizer color_map;
	
	std::vector<Point3d> points_c;

	Mat rsimage_zoom;
	Mat rsimage_zoom_gray;
	Mat rsimagec_gray;
	Mat rsimagec_zoom;
  int frame_num = 0;
  char filename_rs[60];

  Mat rsimagec_blurred;
  Mat rsimagec_rgb;
  Mat rsimagec_hls;
  Mat rsimagec_segmented;
  Mat mask1, mask2;
  int flag  = 0;

  SimpleBlobDetector::Params params;
  init_bolb_detector_params(params);
  std::vector<Point3d> breaker_coords;
  Point3d valve_coord;
  int error;

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
	std::cout << "Device found!" << std::endl;

  // Find camera parameters, print them out
  auto ir1_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::stream_profile>();
	rs2_extrinsics extrinsics = get_extrinsics(ir1_stream, color_stream);
	Mat R_c_i = get_rotation(extrinsics);
	Vec3d T_c_i = get_translation(extrinsics);
	std::cout << "R_c_i = " << R_c_i << std::endl;
	std::cout << "T_c_i = " << T_c_i << std::endl;

  // try retrieving rgb intrinsics
	auto intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
  Mat K, D;
  double kd[9] = {615.084, 0.0, 323.046, 0.0, 615.019, 242.301, 0.0,0.0,1.0};
  double dd[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  K = Mat(3,3,CV_64F,kd);
  D = Mat(1,5,CV_64F,dd);
	std::cout << "K = " << K << std::endl;
	std::cout << "D = " << D << std::endl;

  ros::Rate r(10);
  while(ros::ok()) {
    r.sleep();
    ros::spinOnce();

    if (*device_type == shipbot_ros::FindDevice::Request::NONE) {
      continue;
    }

    if (*device_type == shipbot_ros::FindDevice::Request::BREAKERA ||
        *device_type == shipbot_ros::FindDevice::Request::BREAKERB) {
      thresh_min = orange_min;
      thresh_max = orange_max;
    }
    else if (*device_type == shipbot_ros::FindDevice::Request::SHUTTLECOCK) {
      thresh_min = blue_min_2;
      thresh_max = blue_max_2;
    }
    else {
      thresh_min = blue_min;
      thresh_max = blue_max;
    }

    error = 0;
    // ----------------Wait for the next set of frames from RealSense
    auto frames = pipe.wait_for_frames();

    rs2::frame depth_frame = frames.get_depth_frame();
    rs2::frame color_frame = frames.get_color_frame();
    // Query frame size (width and height)
    const int w = depth_frame.as<rs2::video_frame>().get_width();
    const int h = depth_frame.as<rs2::video_frame>().get_height();
    const int w1 = color_frame.as<rs2::video_frame>().get_width();
    const int h1 = color_frame.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
    Mat rsimagec(Size(w1, h1), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    Mat rsimage(Size(w, h), CV_8UC3, (void*)depth_frame.apply_filter(color_map).get_data(), Mat::AUTO_STEP);

    rs2::points points_i;
    pc.map_to(color_frame);
    points_i = pc.calculate(depth_frame);

    // Step 1: Find colored blobs in colored frame, draw
    //For segmenting the image in RGB format.
    cvtColor(rsimagec, rsimagec_rgb, COLOR_BGR2RGB);
    //gammaCorrection(rsimagec_rgb, rsimagec_rgb, 0.45); // adjust brightness
    cvtColor(rsimagec_rgb, rsimagec_hls, COLOR_BGR2HLS);

    // actual valves and breakers
    inRange(rsimagec_hls, thresh_min, thresh_max, rsimagec_segmented);
    if (*device_type == shipbot_ros::FindDevice::Request::SPIGOT) {
      inRange(rsimagec_hls, lime_min, lime_max, mask2);
      bitwise_or(rsimagec_segmented, mask2, rsimagec_segmented);
    }
    else if (*device_type == shipbot_ros::FindDevice::Request::BREAKERA ||
        *device_type == shipbot_ros::FindDevice::Request::BREAKERB){
      GaussianBlur(rsimagec_segmented, rsimagec_segmented, Size(5,5), 0.0);
      inRange(rsimagec_segmented, 1, 255, rsimagec_segmented);
    }
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    // Detect blobs.
    std::vector<KeyPoint> keypoints;
    detector->detect(rsimagec_segmented, keypoints);
    

    // Draw detected blobs as green circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints, rsimagec_segmented_rgb;
    cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
    drawKeypoints(rsimagec_segmented_rgb, keypoints, im_with_keypoints, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        


    // Step 2: Transform point could and project onto colored frame
    Mat R_i = Mat::eye(3,3,CV_64F);
    Vec3d T_i(0,0,0);
    // transform points from infra1 frame to color frame
    rs2::points points;
    points_c = transform_points(R_c_i, T_c_i, points_i);
    std::unordered_map<std::string, cv::Point3d> u;
    std::vector<Point2d> pixels_c;
    projectPoints0(points_c, R_i, T_i, K, D, pixels_c, u);
    std::cout << points_c.size() << ", " << pixels_c.size() << std::endl;

    // Step 3: Find the 3D coordinate corresponding to 
    // the blob in 2D; average that, print
    std::cout << "num keypoints: " << keypoints.size() << std::endl;

    if (keypoints.size() == 0) {
      wheel_state.visible = false;
      spigot_state.visible = false;
      shuttlecock_state.visible = false;
      for (int i = 0; i < 3; ++i) {
        breaker_state.switches[i].visible = false;
      }
      std::cout << "Bad frame: no keypoint" << std::endl;
      error = ERROR_BAD_FRAME;
      continue;
    }

    // we compute and store the L2 distance and z value of each keypoint
    KeyPoint best_keypoint;
    Point3d best_point;
    Point2d best_pixel;
    double min_z = 0;
    int best_point_idx = 0;
    std::priority_queue<Blob> blob_pq;
    for (int k = 0; k < keypoints.size(); k++) {
      KeyPoint curr_keypoint = keypoints.at(k);
      std::cout << "blob " << k << " size: " << curr_keypoint.size << std::endl;
      Point2d pixel = curr_keypoint.pt;
      int pixel_x = (int)pixel.x;
      int pixel_y = (int)pixel.y;
      size_t area = curr_keypoint.size;
      int r = (int)sqrt(area);
      double z = 0;
      cv::Point3d best_point_tmp;
      cv::Point2d best_pixel_tmp;
      double best_dist_tmp = 0;
      // check every pixel in the blob, find 3D coords
      // and get one 3D coord representing the blob
      for (int i = -r; i < r+1; i++) {
        int x = pixel_x + i;
        for (int j = -r; j < r+1; j++) {
          int y = pixel_y + j;
          std::string key = get_string_2(x, y);
          auto got = u.find(key);
          if (got != u.end()) {
            Point3d pgot = got->second;
            double dist_tmp = sqrt(pgot.x * pgot.x + pgot.y * pgot.y + pgot.z * pgot.z);
            if (best_dist_tmp == 0 || best_dist_tmp > dist_tmp){
              z = pgot.z;
              best_point_tmp = pgot;
              Point2d px((double)x, (double)y);
              best_pixel_tmp = px;
              best_dist_tmp = dist_tmp;
            }
          }
        }
      }
      // push the blob to pq
      Blob blob(best_dist_tmp, best_point_tmp, best_pixel_tmp, curr_keypoint);
      if (!(blob.point.x == 0 && blob.point.y == 0 && blob.point.z == 0))
        blob_pq.push(blob);
    }

    std::vector<Blob> breakers;
    Blob best_blob;
    if (blob_pq.empty()){
      wheel_state.visible = false;
      spigot_state.visible = false;
      shuttlecock_state.visible = false;
      for (int i = 0; i < 3; ++i) {
        breaker_state.switches[i].visible = false;
      }
      std::cout << "Bad frame: no good blob" << std::endl;
      error = ERROR_BAD_FRAME;
      continue;
    }
    // for valves, we find one best keypoint
    if (*device_type != shipbot_ros::FindDevice::Request::BREAKERA &&
        *device_type != shipbot_ros::FindDevice::Request::BREAKERB) {
      best_blob = blob_pq.top();
      blob_pq.pop();
      // store point into valve_coord
      valve_coord = best_blob.point;
      std::cout << "Pixel: " << best_blob.pixel.x << ", " << best_blob.pixel.y << "\tPoint: " << best_blob.point.x << ", " << best_blob.point.y << ", " << best_blob.point.z << std::endl;

      switch (*device_type) {
        case shipbot_ros::FindDevice::Request::WHEEL:
          wheel_state.position.x = best_blob.point.x;
          wheel_state.position.y = best_blob.point.y;
          wheel_state.position.z = best_blob.point.z;
          break;
        case shipbot_ros::FindDevice::Request::SPIGOT:
          spigot_state.position.x = best_blob.point.x;
          spigot_state.position.y = best_blob.point.y;
          spigot_state.position.z = best_blob.point.z;
          break;
        case shipbot_ros::FindDevice::Request::SHUTTLECOCK:
          shuttlecock_state.position.x = best_blob.point.x;
          shuttlecock_state.position.y = best_blob.point.y;
          shuttlecock_state.position.z = best_blob.point.z;
          break;
      }
    }
    // for breakers, we find 3 best keypoints from PQ
    // currently, these are unordered. They're ordered in Step 4.2
    else {
      if (blob_pq.size() < 3) {
        for (int i = 0; i < 3; ++i) {
          breaker_state.switches[i].visible = false;
        }
        continue;
      }
      for (int i = 0; i < 3; ++i) {
        breaker_state.switches[i].visible = true;
      }
      for (int i = 0; i < 3; i++) {
        best_blob = blob_pq.top();
        breakers.push_back(best_blob);
        blob_pq.pop();
        /*
        std::cout << "Breaker " << i << ": " <<std::endl;
        std::cout << "Pixel: " << best_blob.pixel.x << ", " << best_blob.pixel.y << "\tPoint: " << best_blob.point.x << ", " << best_blob.point.y << ", " << best_blob.point.z << std::endl;
        */
      }
      //std::cout << std::endl;
    }

    // Step 4: Determine states and configurations
    Mat rethresh_mask = Mat::zeros(Size(w1, h1), CV_8UC1);
    Mat rsimage_rethreshed;
    int r;
    // Step 4.1: if it's a valve:
    if (*device_type != shipbot_ros::FindDevice::Request::BREAKERA &&
        *device_type != shipbot_ros::FindDevice::Request::BREAKERB) {
      // if it's a shuttlecock valve, we just check its #pixels
      if (*device_type == shipbot_ros::FindDevice::Request::SHUTTLECOCK) {
        r = 100;
      }
      else if (*device_type == shipbot_ros::FindDevice::Request::SPIGOT) {
        r = 90;
      }
      else if (*device_type == shipbot_ros::FindDevice::Request::WHEEL) {
        r = 170;
      }
      // re-threshold the image. we only keep the valve of interest
      int xc, yc;
      xc = best_blob.pixel.x;
      yc = best_blob.pixel.y;
      cv::rectangle(rethresh_mask, Point(std::max(xc-r, 0), std::max(yc-r, 0)), Point(std::min(xc+r, w1), std::min(yc+r, h1)), 255, FILLED);
      bitwise_and(rethresh_mask, rsimagec_segmented, rsimage_rethreshed);

      // find its contours and fit an ellipse
      std::vector<std::vector<cv::Point>> contours;
      findContours(rsimage_rethreshed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      if (contours.size() == 0) {
        wheel_state.visible = false;
        spigot_state.visible = false;
        shuttlecock_state.visible = false;
        std::cout << "bad frame: No contour!" << std::endl;
        error = ERROR_BAD_FRAME;
        continue;
      }
      double best_area = cv::contourArea(contours[0]);
      size_t best_idx = 0;
      for (size_t i = 1; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) {
          best_idx = i;
          best_area = area;
        }
      }
      if (contours[best_idx].size() < 5) {
        wheel_state.visible = false;
        spigot_state.visible = false;
        shuttlecock_state.visible = false;
        std::cout << "bad frame: too few countours" << std::endl;
        error = ERROR_BAD_FRAME;
        continue;
      }
      cv::RotatedRect ell = fitEllipse(contours[best_idx]);
      cv::ellipse(rsimagec_rgb, ell, cv::Scalar(0, 255, 0), 5);

      // check the bounding box's shape
      float wid = ell.size.width;
      float hei = ell.size.height;
      float ratio = get_rect_ratio(wid, hei);
      std::cout << "Ratio: " << ratio << std::endl;
      if (*device_type != shipbot_ros::FindDevice::Request::SHUTTLECOCK) {
        spigot_state.vertical = ratio < 0.4;
        wheel_state.vertical = false;
      }
      else { // shuttlecock, we also determine device state
        Rect br = ell.boundingRect();
        std::cout << "br w, h: " << br.width << ", " << br.height << std::endl;
        // check br size
        if (br.width > 110){
          if (ratio < 0.13){
            shuttlecock_state.vertical = true;
            shuttlecock_state.open = true;
          }
          else{
            shuttlecock_state.vertical = false;
            shuttlecock_state.open = false;
          }
        }
        else {
          if (br.height > 110){
            shuttlecock_state.vertical = false;
            shuttlecock_state.open = true;
          } else {
            shuttlecock_state.vertical = true;
            shuttlecock_state.open = false;
          }
        }
      }
    }
    // Step 4.2: if the device is a breaker box
    else {
      //// Determine breaker state (up/down)
      // sort based on pixel x
      Blob breaker[3];
      double x_1, x_2, x_3;
      double min_x = -1;
      int b1_idx = 0;
      double max_x = -1;
      int b3_idx = 0;
      int b2_idx = 0;
      bool isb1b3[3] = {false, false, false};
      for (int i = 0; i < 3; i++) {
        double cur_x = breakers.at(i).pixel.x;
        if (min_x == -1 || cur_x < min_x) {
          min_x = cur_x;
          b1_idx = i;
        }
        if (max_x == -1 || cur_x > max_x) {
          max_x = cur_x;
          b3_idx = i;
        }
      }
      isb1b3[b1_idx] = true;
      isb1b3[b3_idx] = true;
      breaker[0] = breakers.at(b1_idx);
      breaker[2] = breakers.at(b3_idx);
      for (int i = 0; i < 3; i++) {
        if (isb1b3[i] == false) {
          b2_idx = i;
        }
      }
      breaker[1] = breakers.at(b2_idx);

      cout << breaker[0].point.x << " " << breaker[0].point.y << " " << breaker[0].point.z << endl;
      cout << breaker[1].point.x << " " << breaker[1].point.y << " " << breaker[1].point.z << endl;
      cout << breaker[2].point.x << " " << breaker[2].point.y << " " << breaker[2].point.z << endl;
      cout << endl;

      bool all_zero = false;
      for (int b = 0; b < 3; ++b) {
        if (breaker[b].point.x == 0 &&
            breaker[b].point.y == 0 &&
            breaker[b].point.z == 0) {
          all_zero = true;
          break;
        }
      }

      if (all_zero) {
        for (int b = 0; b < 3; ++b) {
          breaker_state.switches[b].visible = false;
        }
        continue;
      }

      Mat black_mask;
      inRange(rsimagec_rgb, black_min, black_max, black_mask);
      bitwise_or(black_mask, rsimagec_segmented, rsimagec_segmented);
      // check b1 and b3 position to find roi
      double dx = (breaker[2].pixel.x - breaker[0].pixel.x)/5;
      std::cout << dx << std::endl;
      // draw text on the breakders 
      for (int i = 0; i < 3; i++) {
        breaker_state.switches[i].position.x = breaker[i].point.x;
        breaker_state.switches[i].position.y = breaker[i].point.y;
        breaker_state.switches[i].position.z = breaker[i].point.z;

        // store 3d point in output vector
        breaker_coords.push_back(breaker[0].point);

        // determine breaker state
        Mat breaker_pic;
        Rect breaker_roi;
        Point up_left;
        Point bot_right;
        int x0, y0, x1, y1;

        x0 = (int)(breaker[i].pixel.x - dx/1.7);
        if (x0 < 0)
            x0 = 0;
        y0 = (int)(breaker[i].pixel.y - dx*2.7);
        if (y0 < 0) 
            y0 = 0;
        x1 = (int)(breaker[i].pixel.x + dx/1.7);
        if (x1 >= w1)
            x1 = w1-1;
        y1 = (int)(breaker[i].pixel.y + dx*3);
        if (y1 > h1-1)
            y1 = h1-1;

        up_left = Point2i(x0, y0);
        bot_right = Point2i(x1, y1);
        breaker_roi = Rect(up_left, bot_right);
        rectangle(im_with_keypoints, breaker_roi, Scalar(255, 0,0),2);
        breaker_pic = rsimagec_segmented(breaker_roi);

        // we look at the roi and check for the bound
        int white_idx_top, white_idx_bot;
        for (int g = 0; g < breaker_roi.height; g+= 3) {
          if (breaker_pic.at<uchar>(g, (int)breaker[i].pixel.x - breaker_roi.tl().x) > 0 && g >= 40) {
            white_idx_top = g;
            break;
          }
        }
        for (int g = breaker_roi.height-1; g >= 0; g-= 3) {
          if (breaker_pic.at<uchar>(g, (int)breaker[i].pixel.x - breaker_roi.tl().x) > 0) {
            white_idx_bot = g;
            break;
          }
        }
        int upper = white_idx_top + breaker_roi.tl().y;
        int lower = white_idx_bot + breaker_roi.tl().y;
        int left = breaker_roi.tl().x - 15;
        int right = breaker_roi.br().x + 15;
        rectangle(im_with_keypoints, Rect(Point2i(left, upper), Point2i(right, lower)), Scalar(0,0,255), 2);
        int dy_up = abs(upper - (int)breaker[i].keypoint.pt.y);
        int dy_low = abs(lower - (int)breaker[i].keypoint.pt.y);
        std::cout << (float)dy_up / dy_low << std::endl;
        if (*device_type == shipbot_ros::FindDevice::Request::BREAKERA) {
          breaker_state.switches[i].up = (float)dy_up / dy_low < 0.35;
        }
        else if (*device_type == shipbot_ros::FindDevice::Request::BREAKERB) {
          breaker_state.switches[i].up = (float)dy_low / dy_up >= 0.35;
        }
        std::string tx;
        if (breaker_state.switches[i].up) {
          tx = "U" + std::to_string(i+1);
        } else {
          tx = "D" + std::to_string(i+1);
        }
        putText(rsimagec_rgb, tx, breaker[i].pixel, FONT_HERSHEY_SIMPLEX, 2,Scalar(0,255,0), 3);
      }
    }

    wheel_state.visible = true;
    spigot_state.visible = true;
    shuttlecock_state.visible = true;
    for (int i = 0; i < 3; ++i) {
      breaker_state.switches[i].visible = true;
    }

    if (pub_image) {
      // Show blobs
      cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
      Mat imgarray[] = {rsimagec_rgb, im_with_keypoints, rsimagec_segmented_rgb};	
      hconcat(imgarray, 3, rs_image_ptr->image);
      image_pub.publish(rs_image_ptr->toImageMsg());
    }

    switch (*device_type) {
      case shipbot_ros::FindDevice::Request::WHEEL:
        wheel_pub.publish(wheel_state);
        break;
      case shipbot_ros::FindDevice::Request::SPIGOT:
        spigot_pub.publish(spigot_state);
        break;
      case shipbot_ros::FindDevice::Request::SHUTTLECOCK:
        shuttlecock_pub.publish(shuttlecock_state);
        break;
      case shipbot_ros::FindDevice::Request::BREAKERA:
        breaker_pub.publish(breaker_state);
        break;
      case shipbot_ros::FindDevice::Request::BREAKERB:
        breaker_pub.publish(breaker_state);
        break;
    }
  }
}

