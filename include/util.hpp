#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <iomanip>
#include <map>
#include <utility>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <queue>

using namespace cv;

// device types
#define DEVICE_GATE 0 // V1
#define DEVICE_LARGE 1 // V2
#define DEVICE_SHUTTLECOCK 2 // V3
#define DEVICE_BREAKER  3 // B
// breakers A and B differs, so we have 2 cases
#define BREAKER_A 106
#define BREAKER_B 107

// valve configuration
#define DEVICE_VERTICAL 100 // facing up
#define DEVICE_HORIZONTAL 101// facing the front

// shuttlecock state
#define VALVE_OPEN 102 // in line with pipe
#define VALVE_CLOSED 103 // perpendicular to pipe 

// breaker state
#define BREAKER_UP 104
#define BREAKER_DOWN 105

// error code
#define ERROR_BAD_ARGCNT 200
#define ERROR_BAD_TYPE 201
#define ERROR_BAD_DISPLAY 202
#define ERROR_BAD_FRAME 203 // allows the pipeline to continue



// data structure to store each blob
class Blob {
public:
    double dist; // L2 distance
    cv::Point3d point;
    cv::Point2d pixel;
    cv::KeyPoint keypoint;
    Blob();
    Blob(double dist_, Point3d point_, Point2d pixel_, KeyPoint keypoint_);

    friend bool operator<(const Blob& b1, const Blob& b2) {
        if(b1.dist > b2.dist)
            return true;
        return false;
    }
};

std::string get_valve_string(int device_config, int device_state, int device_type);

std::string get_breaker_string(int a);

rs2_extrinsics get_extrinsics(const rs2::stream_profile& from_stream, 
                    const rs2::stream_profile& to_stream);

Vec3d get_translation(rs2_extrinsics extrinsics);

Mat get_rotation(rs2_extrinsics extrinsics);

Mat get_R_T(Mat R, Vec3d T);

Mat get_tf(Mat R_0_1, Vec3d T_0_1, Mat R_1_2, Vec3d T_1_2);

/* Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint */
cv::Point3d transform_point_to_point(Mat R, Vec3d T, rs2::vertex from_point);

std::vector<cv::Point3d> transform_points(Mat R, Vec3d T, rs2::points from_points);

std::string get_string_2(double x_, double y_);

void projectPoints0(std::vector<cv::Point3d> &points, Mat R, Vec3d T, Mat K, Mat D, std::vector<cv::Point2d> &pixels, std::unordered_map<std::string, cv::Point3d> &u);


/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
void project_point_to_pixel(double pixel[2], const struct rs2_intrinsics * intrin, const double point[3]);

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
void deproject_pixel_to_point(double point[3], const struct rs2_intrinsics * intrin, const double pixel[2], double depth);

void MyEllipse( Mat img, double angle );

void init_bolb_detector_params(SimpleBlobDetector::Params &params);

float get_rect_ratio(float wid, float hei);

void gammaCorrection(const Mat &img, Mat &img_corrected, const double gamma_);

#endif
