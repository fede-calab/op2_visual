// File: include/op2_visual/marker_detection.h
#ifndef MARKER_DETECTION_H
#define MARKER_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

// Detect ArUco markers in a frame
void detectMarkers(const cv::Mat&                          frame,
                   const cv::Ptr<cv::aruco::Dictionary>&   dictionary,
                   std::vector<int>&                       ids,
                   std::vector<std::vector<cv::Point2f>>& corners);

// Estimate pose for each detected marker
void estimateMarkerPoses(const std::vector<std::vector<cv::Point2f>>& corners,
                         float markerSize,
                         const cv::Mat& cameraMatrix,
                         const cv::Mat& distCoeffs,
                         std::vector<cv::Vec3d>& rvecs,
                         std::vector<cv::Vec3d>& tvecs);

// Compute 3D distance (in meters) between two translation vectors
float compute3DDistance(const cv::Vec3d& tvec0, 
                        const cv::Vec3d& tvec1);

// Utility to draw axes on markers
void drawMarkerAxes(cv::Mat& frame,
                    const cv::Mat& cameraMatrix,
                    const cv::Mat& distCoeffs,
                    const cv::Vec3d& rvec,
                    const cv::Vec3d& tvec,
                    float axisLength);

// Utility to get the center of a marker in 2D (image space)
cv::Point2f getMarkerCenter(const std::vector<cv::Point2f>& corners);

// Draw an offset dot between two markers
cv::Point2f drawOffsetDot(cv::Mat& frame,
                          const std::vector<cv::Point2f>& corners0,
                          const std::vector<cv::Point2f>& corners1,
                          float offsetRatio);

#endif // MARKER_DETECTION_H