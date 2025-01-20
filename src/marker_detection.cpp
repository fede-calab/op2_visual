// File: src/marker_detection.cpp
#include "op2_visual/marker_detection.h"
#include <opencv2/aruco.hpp>
#include <cmath>

void detectMarkers(const cv::Mat&                          frame,
                   const cv::Ptr<cv::aruco::Dictionary>&   dictionary,
                   std::vector<int>&                       ids,
                   std::vector<std::vector<cv::Point2f>>& corners)
{
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
}

void estimateMarkerPoses(const std::vector<std::vector<cv::Point2f>>& corners,
                         float markerSize,
                         const cv::Mat& cameraMatrix,
                         const cv::Mat& distCoeffs,
                         std::vector<cv::Vec3d>& rvecs,
                         std::vector<cv::Vec3d>& tvecs)
{
    cv::aruco::estimatePoseSingleMarkers(corners, markerSize,
                                         cameraMatrix, distCoeffs,
                                         rvecs, tvecs);
}

float compute3DDistance(const cv::Vec3d& tvec0, const cv::Vec3d& tvec1)
{
    cv::Vec3d diff = tvec0 - tvec1;
    return static_cast<float>(std::sqrt(diff[0]*diff[0] + 
                                        diff[1]*diff[1] + 
                                        diff[2]*diff[2]));
}

void drawMarkerAxes(cv::Mat& frame,
                    const cv::Mat& cameraMatrix,
                    const cv::Mat& distCoeffs,
                    const cv::Vec3d& rvec,
                    const cv::Vec3d& tvec,
                    float axisLength)
{
    cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, axisLength);
}

cv::Point2f getMarkerCenter(const std::vector<cv::Point2f>& corners)
{
    // Average of the 4 corners
    return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
}

cv::Point2f drawOffsetDot(cv::Mat& frame,
                          const std::vector<cv::Point2f>& corners0,
                          const std::vector<cv::Point2f>& corners1,
                          float offsetRatio)
{
    cv::Point2f c0 = getMarkerCenter(corners0);
    cv::Point2f c1 = getMarkerCenter(corners1);
    cv::Point2f offsetDot = c0 + offsetRatio * (c1 - c0);

    // Draw an orange-ish circle
    cv::circle(frame, offsetDot, 10, cv::Scalar(0, 165, 255), -1);
    return offsetDot;
}