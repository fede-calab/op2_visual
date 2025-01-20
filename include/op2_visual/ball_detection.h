// File: include/op2_visual/ball_detection.h
#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <opencv2/opencv.hpp>

// Detect the ball in a given frame using color masking
bool detectBallWithMask(const cv::Mat& frame,
                        cv::Mat&       ballMask,
                        cv::Point2f&   ballCenter,
                        float&         ballRadius);

#endif // BALL_DETECTION_H