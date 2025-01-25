// File: src/ball_detection.cpp
#include "op2_visual/ball_detection.h"
#include <opencv2/opencv.hpp>
#include <vector>

bool detectBallWithMask(const cv::Mat& frame,
                        cv::Mat&       ballMask,
                        cv::Point2f&   ballCenter,
                        float&         ballRadius)
{
    try {
        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Example green range
        cv::Scalar lowerBound(40, 100, 100);
        cv::Scalar upperBound(80, 255, 255);

        // Threshold to get ballMask
        cv::inRange(hsv, lowerBound, upperBound, ballMask);

        if (cv::countNonZero(ballMask) == 0) {
            return false;
        }

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(ballMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) {
            return false;
        }

        // Largest contour
        double maxArea = 0.0;
        int maxIdx = -1;
        for (int i = 0; i < (int)contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxIdx  = i;
            }
        }

        // Minimum contour area threshold
        if (maxIdx == -1 || maxArea < 100.0) {
            return false;
        }

        // Fit circle around the largest contour
        cv::minEnclosingCircle(contours[maxIdx], ballCenter, ballRadius);
        return true;
    }
    catch (...) {
        // Catch-all for safety
        return false;
    }
}