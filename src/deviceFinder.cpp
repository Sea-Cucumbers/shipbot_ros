#include "deviceFinder.h"
#include <iostream>

DeviceFinder::DeviceFinder(const vector<int> &wheel_thresh,
                           const vector<int> &spigot_thresh,
                           const vector<int> &shuttlecock_thresh,
                           const vector<int> &switch_thresh,
                           const vector<int> &black_thresh,
                           const Mat &R_c_i,
                           const Vec3d &T_c_i,
                           const Mat &K,
                           const Mat &D) : current_device(NONE), R_c_i(R_c_i), T_c_i(T_c_i),
                                           K(K), D(D) {
  if (wheel_thresh.size() != 6) {
    cout << "Wheel threshold vector must be length 6!" << endl;
    exit(1);
  }
  if (shuttlecock_thresh.size() != 6) {
    cout << "Shuttlecock threshold vector must be length 6!" << endl;
    exit(1);
  }
  if (spigot_thresh.size() != 6) {
    cout << "Spigot threshold vector must be length 6!" << endl;
    exit(1);
  }
  if (switch_thresh.size() != 6) {
    cout << "Switch threshold vector must be length 6!" << endl;
    exit(1);
  }
  if (black_thresh.size() != 6) {
    cout << "Black threshold vector must be length 6!" << endl;
    exit(1);
  }

  wheel_thresh_min = cv::Scalar(wheel_thresh[0], wheel_thresh[1], wheel_thresh[2]);
  wheel_thresh_max = cv::Scalar(wheel_thresh[3], wheel_thresh[4], wheel_thresh[5]);

  shuttlecock_thresh_min = cv::Scalar(shuttlecock_thresh[0], shuttlecock_thresh[1], shuttlecock_thresh[2]);
  shuttlecock_thresh_max = cv::Scalar(shuttlecock_thresh[3], shuttlecock_thresh[4], shuttlecock_thresh[5]);

  spigot_thresh_min = cv::Scalar(spigot_thresh[0], spigot_thresh[1], spigot_thresh[2]);
  spigot_thresh_max = cv::Scalar(spigot_thresh[3], spigot_thresh[4], spigot_thresh[5]);

  switch_thresh_min = cv::Scalar(switch_thresh[0], switch_thresh[1], switch_thresh[2]);
  switch_thresh_max = cv::Scalar(switch_thresh[3], switch_thresh[4], switch_thresh[5]);

  black_thresh_min = cv::Scalar(black_thresh[0], black_thresh[1], black_thresh[2]);
  black_thresh_max = cv::Scalar(black_thresh[3], black_thresh[4], black_thresh[5]);

  init_bolb_detector_params(params);
  detector = SimpleBlobDetector::create(params);
}

void DeviceFinder::newFrames(const rs2::frame &depth_frame, const rs2::frame &color_frame) {
  this->depth_frame = depth_frame;
  this->color_frame = color_frame;
}

void DeviceFinder::findWheel(shipbot_ros::WheelState &state) {
  state.visible = false;
  setCurrentDevice(WHEEL);
  processFrames();
  int found = blobs.size();
  if (!found) {
    return;
  }
  state.position.x = blobs[0].point.x;
  state.position.y = blobs[0].point.y;
  state.position.z = blobs[0].point.z;

  cv::RotatedRect ell;
  state.visible = rethreshAndFitEllipse(ell, 170);
}

void DeviceFinder::findSpigot(shipbot_ros::SpigotState &state) {
  state.visible = false;
  setCurrentDevice(SPIGOT);
  processFrames();
  int found = blobs.size();
  if (!found) {
    return;
  }
  state.position.x = blobs[0].point.x;
  state.position.y = blobs[0].point.y;
  state.position.z = blobs[0].point.z;

  // Determine orientation
  cv::RotatedRect ell;
  if (!rethreshAndFitEllipse(ell, 90)) {
    return;
  }

  // Check the bounding box's shape
  float wid = ell.size.width;
  float hei = ell.size.height;
  float ratio = get_rect_ratio(wid, hei);
  state.vertical = ratio < 0.4;
  state.visible = true;
}

void DeviceFinder::findShuttlecock(shipbot_ros::ShuttlecockState &state) {
  state.visible = false;
  setCurrentDevice(SHUTTLECOCK);
  processFrames();
  int found = blobs.size();
  if (!found) {
    cout << "not found" << endl;
    return;
  }
  state.position.x = blobs[0].point.x;
  state.position.y = blobs[0].point.y;
  state.position.z = blobs[0].point.z;

  // Get orientation from number of pixels
  state.vertical = blobs[0].keypoint.size < 45;
  
  // Determine status
  cv::RotatedRect ell;
  if (!rethreshAndFitEllipse(ell, 100)) {
    return;
  }

  // Check the bounding box's shape
  float wid = ell.size.width;
  float hei = ell.size.height;
  float ratio = get_rect_ratio(wid, hei);
  if (!state.vertical) {
    // Check bounding rec
    Rect br = ell.boundingRect();
    state.open = br.width <= br.height;
  } else {
    // We count the #pixels
    state.open = blobs[0].keypoint.size > 30;
  }
  state.visible = true;
}

void DeviceFinder::findSwitches(shipbot_ros::SwitchState &state1,
                                shipbot_ros::SwitchState &state2,
                                shipbot_ros::SwitchState &state3,
                                bool typeA) {
  state1.visible = false;
  state2.visible = false;
  state3.visible = false;
  setCurrentDevice(typeA ? BREAKERA : BREAKERB);
  processFrames();
  int found = blobs.size();
  if (found < 3) {
    // TODO: handle partial visibility
    return;
  }

  // Determine status

  // Sort based on pixel x
  Blob breaker[3];
  double x_1, x_2, x_3;
  double min_x = -1;
  int b1_idx = 0;
  double max_x = -1;
  int b3_idx = 0;
  int b2_idx = 0;
  bool isb1b3[3] = {false, false, false};
  for (int i = 0; i < 3; i++) {
    double cur_x = blobs[i].pixel.x;
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
  breaker[0] = blobs[b1_idx];
  breaker[2] = blobs[b3_idx];
  for (int i = 0; i < 3; i++){
    if(isb1b3[i] == false) {
      b2_idx = i;
    }
  }
  breaker[1] = blobs[b2_idx];

  Mat black_mask;
  inRange(rsimagec_rgb, black_thresh_min, black_thresh_max, black_mask);
  bitwise_or(black_mask, rsimagec_segmented, rsimagec_segmented);
  // Check b1 and b3 position to find roi
  double dx = (breaker[2].pixel.x - breaker[0].pixel.x)/5;
  vector<Point3d> breaker_coords;
  int breaker_state[3];
  // Draw text on the breakers 
  for (int i = 0; i < 3; i++) {
    // Store 3d point in output vector
    breaker_coords.push_back(breaker[0].point);

    // determine breaker state
    Mat breaker_pic;
    Rect breaker_roi;
    Point up_left;
    Point bot_right;
    int x0, y0, x1, y1;

    x0 = (int)(breaker[i].pixel.x - dx/1.7);
    if (x0 < 0) {
      x0 = 0;
    }
    y0 = (int)(breaker[i].pixel.y - dx*2.7);
    if (y0 < 0) {
      y0 = 0;
    }
    x1 = (int)(breaker[i].pixel.x + dx/1.7);
    if (x1 >= w1) {
      x1 = w1-1;
    }
    y1 = (int)(breaker[i].pixel.y + dx*3);
    if (y1 > h1-1) {
      y1 = h1-1;
    }

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
    if (typeA) {
      if ((float)dy_up / dy_low < 0.35) {
          breaker_state[i] = BREAKER_UP;
      } else {
        breaker_state[i] = BREAKER_DOWN;
      }
    } else if (!typeA) {
      if ((float)dy_low / dy_up < 0.35) {
        breaker_state[i] = BREAKER_DOWN;
      } else {
        breaker_state[i] = BREAKER_UP;
      }
    }
    std::string tx;
    if (breaker_state[i] == BREAKER_UP) {
      tx = "U" + std::to_string(i+1);
    } else {
      tx = "D" + std::to_string(i+1);
    }
    putText(rsimagec_rgb, tx, breaker[i].pixel, FONT_HERSHEY_SIMPLEX, 2,Scalar(0,255,0), 3);
  }
  state1.up = breaker_state[0] == BREAKER_UP;
  state2.up = breaker_state[1] == BREAKER_UP;
  state3.up = breaker_state[2] == BREAKER_UP;
  state1.visible = true;
  state2.visible = true;
  state3.visible = true;
}

DeviceType DeviceFinder::getCurrentDevice() {
  return current_device;
}

void DeviceFinder::setCurrentDevice(DeviceType device_type) {
  current_device = device_type;
  switch (device_type) {
    case WHEEL:
      thresh_min = wheel_thresh_min;
      thresh_max = wheel_thresh_max;
      break;
    case SPIGOT:
      thresh_min = wheel_thresh_min;
      thresh_max = wheel_thresh_max;
      break;
    case SHUTTLECOCK:
      thresh_min = shuttlecock_thresh_min;
      thresh_max = shuttlecock_thresh_max;
      break;
    case BREAKERA:
      thresh_min = switch_thresh_min;
      thresh_max = switch_thresh_max;
      break;
    case BREAKERB:
      thresh_min = switch_thresh_min;
      thresh_max = switch_thresh_max;
      break;
  }
}

bool DeviceFinder::getAnnotatedImage(Mat &result) {
  if (!rsimagec_rgb.total() || !im_with_keypoints.total() || !rsimagec_segmented_rgb.total()) {
    return false;
  }
  cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
  Mat imgarray[] = {rsimagec_rgb, im_with_keypoints, rsimagec_segmented_rgb};
  hconcat(imgarray, 3, result);
  return true;
}

void DeviceFinder::processFrames() {
  blobs.clear();

  // Query frame size (width and height)
  int w = depth_frame.as<rs2::video_frame>().get_width();
  int h = depth_frame.as<rs2::video_frame>().get_height();
  w1 = color_frame.as<rs2::video_frame>().get_width();
  h1 = color_frame.as<rs2::video_frame>().get_height();

  // Create OpenCV matrix of size (w, h) from the colorized depth data
  rsimagec = cv::Mat(Size(w1, h1), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
  rsimage = cv::Mat(Size(w, h), CV_8UC3, (void*)depth_frame.apply_filter(color_map).get_data(), Mat::AUTO_STEP);

  rs2::points points_i;
  pc.map_to(color_frame);
  points_i = pc.calculate(depth_frame);

  // Step 1: Find colored blobs in colored frame, draw
  // For segmenting the image in RGB format.
  cvtColor(rsimagec, rsimagec_rgb, COLOR_BGR2RGB);
  //gammaCorrection(rsimagec_rgb, rsimagec_rgb, 0.45); // adjust brightness
  cvtColor(rsimagec_rgb, rsimagec_hls, COLOR_BGR2HLS);

  // Actual valves and breakers
  inRange(rsimagec_hls, thresh_min, thresh_max, rsimagec_segmented);
  if (current_device == SPIGOT) {
    inRange(rsimagec_hls, spigot_thresh_min, spigot_thresh_max, mask2);
    bitwise_or(rsimagec_segmented, mask2, rsimagec_segmented);
  }

  // Detect blobs.
  std::vector<KeyPoint> keypoints;
  detector->detect(rsimagec_segmented, keypoints);
  

  // Draw detected blobs as green circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
  drawKeypoints(rsimagec_segmented_rgb, keypoints, im_with_keypoints, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  


  // Step 2: Transform point cloud and project onto colored frame
  Mat R_i = Mat::eye(3,3,CV_64F);
  Vec3d T_i(0,0,0);
  // transform points from infra1 frame to color frame
  rs2::points points;
	vector<Point3d> points_c = transform_points(R_c_i, T_c_i, points_i);
  std::unordered_map<std::string, cv::Point3d> u;
  std::vector<Point2d> pixels_c;
  projectPoints0(points_c, R_i, T_i, K, D, pixels_c, u);

  // Step 3: Find the 3D coordinate corresponding to 
  // the blob in 2D; average that
  if (keypoints.size() == 0) {
    return;
  }

  // We compute and store the L2 distance and z value of each keypoint
  KeyPoint best_keypoint;
  Point3d best_point;
  Point2d best_pixel;
  double min_z = 0;
  int best_point_idx = 0;
  std::priority_queue<Blob> blob_pq;
  for (int k = 0; k < keypoints.size(); k++){
    KeyPoint curr_keypoint = keypoints.at(k);
    Point2d pixel = curr_keypoint.pt;
    int pixel_x = (int)pixel.x;
    int pixel_y = (int)pixel.y;
    size_t area = curr_keypoint.size;
    int r = (int)sqrt(area);
    double z = 0;
    cv::Point3d best_point_tmp;
    cv::Point2d best_pixel_tmp;
    double best_dist_tmp;
    // Check every pixel in the blob, find 3D coords
    // and get one 3D coord representing the blob
    for (int i = -r; i < r+1; i++) {
      int x = pixel_x + i;
      for (int j = -r; j < r+1; j++) {
        int y = pixel_y + j;
        std::string key = get_string_2(x, y);
        auto got = u.find(key);
        if (got != u.end()) {
          Point3d pgot = got->second;
          if (z == 0 || z > pgot.z) {
            z = pgot.z;
            best_point_tmp = pgot;
            Point2d px((double)x, (double)y);
            best_pixel_tmp = px;
            best_dist_tmp = sqrt(pgot.x * pgot.x + pgot.y * pgot.y + z * z);
          }
        }
      }
    }
    // Push the blob to pq
    Blob blob(best_dist_tmp, best_point_tmp, best_pixel_tmp, curr_keypoint);
    blob_pq.push(blob);
  }

  // For valves, we find one best keypoint
  int nblobs = (current_device == BREAKERA || current_device == BREAKERB) ? 3 : 1;
  for (int i = 0; i < nblobs && blob_pq.size() != 0; i++){
    blobs.push_back(blob_pq.top());
    blob_pq.pop();
  }
}

bool DeviceFinder::rethreshAndFitEllipse(cv::RotatedRect &ell, int r) {
  Mat rethresh_mask = Mat::zeros(Size(w1, h1), CV_8UC1);
  Mat rsimage_rethreshed;

  // Re-threshold the image, only keeping the valve of interest
  int xc, yc;
  xc = blobs[0].pixel.x;
  yc = blobs[0].pixel.y;
  cv::rectangle(rethresh_mask, Point(xc-r, yc-r), Point(xc+r, yc+r), 255, FILLED);
  bitwise_and(rethresh_mask, rsimagec_segmented, rsimage_rethreshed);

  // Find its contours and fit an ellipse
  std::vector<std::vector<cv::Point>> contours;
  findContours(rsimage_rethreshed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  if (contours.size() == 0) {
    cout << "no contours" << endl;
    return false;
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
    cout << "best contour has too few points" << endl;
    return false;
  }
  ell = fitEllipse(contours[best_idx]);
  cv::ellipse(rsimagec_rgb, ell, cv::Scalar(0, 255, 0), 5);
  return true;
}
