// File: src/ball_tracking.cpp
#include "op2_visual/ball_tracking.h"

// Include the helper modules
#include "op2_visual/ball_detection.h"
#include "op2_visual/marker_detection.h"
#include "op2_visual/display_utils.h"

#include <csignal>
#include <atomic>
#include <chrono>
#include <iostream>
#include <fstream>    // For file I/O
#include <iomanip>    // For std::fixed and std::setprecision
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ball_msgs/ball.h>

// Global control for graceful shutdown
static std::atomic_bool g_run(true);

// Signal handler for Ctrl+C
void sigintHandler(int)
{
    g_run.store(false);
}

int ball_tracking(ros::NodeHandle nh, ros::Publisher& ball_pub, bool no_display)
{
    // Register SIGINT for Ctrl+C
    std::signal(SIGINT, sigintHandler);

    // Open camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam.\n";
        return -1;
    }

    // Load camera calibration
    std::string yamlFilePath = ros::package::getPath("op2_visual") + "/calibration/logitech_c270_hd_webcam/logitech_c270_hd_webcam_params.yml";
    cv::FileStorage fs(yamlFilePath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open camera calibration file.\n";
        return -1;
    }
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    // Marker dictionary
    float markerSize = 0.098f; // meters
    // auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // Ball detection variables
    cv::Mat ballMask;
    cv::Point2f ballCenter;
    float ballRadius = 0.015f; // meters

    // Velocity tracking
    double previousPos = -1.0;
    auto previousTime  = std::chrono::high_resolution_clock::now();

    // ----------------------------
    // Initialize CSV Logging
    // ----------------------------
    std::ofstream csvFile;
    std::string csvFilePath = ros::package::getPath("op2_visual") + "/data/ball_tracking_data.csv";
    csvFile.open(csvFilePath, std::ios::out);
    if (!csvFile.is_open()) {
        std::cerr << "Error: Could not open CSV file for writing.\n";
        // Depending on requirements, you might choose to exit or continue without logging
        // return -1;
    } else {
        // Write CSV header
        csvFile << "Timestamp,DistanceCM,OffsetRatio,BallDetected,DistToMarker0,DistToMarker1,DistFromOffset,Velocity,FrameTimeSec,FrameRateHz\n";
    }

    int width = 0;
    int height = 0;

    int x = 0;
    int y = 0;

    bool frameFlag = 0;

    // Main loop
    while (g_run.load()) {
        // Grab frame
        cv::Mat originalFrame;
        cap >> originalFrame;
        if (originalFrame.empty()) {
            std::cerr << "ATTENTION: Could not read frame from webcam.\n";
            continue;
        }


        // Define the height of the central band
        // For example, central half of the frame
        int band_height = originalFrame.rows / 2;

        // Calculate the starting y-coordinate to center the band
        int y_start = (originalFrame.rows - band_height) / 2;

        // Ensure y_start is non-negative
        y_start = std::max(y_start, 0);

        // Define the ROI for the central band
        cv::Rect roi(0,                 // x-coordinate (left)
                    y_start,            // y-coordinate (top of the central band)
                    originalFrame.cols, // width (same as full frame)
                    band_height);       // height (central band height)

        // Crop the frame using the ROI
        cv::Mat frame = originalFrame(roi).clone();


        auto frameStart = std::chrono::high_resolution_clock::now();

        // 1) Detect ArUco markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detectMarkers(frame, dictionary, ids, corners);

        // 2) Detect ball
        bool ballDetected = detectBallWithMask(frame, ballMask, ballCenter, ballRadius);

        // Variables for display
        double distanceCM    = 0.0;
        double distToMarker0 = 0.0;
        double distToMarker1 = 0.0;
        double distFromOffset = 0.0;
        double velocity       = 0.0;
        float offsetRatio     = 0.0f;

        // Check for both markers #0 and #1
        bool haveMarker0 = (std::find(ids.begin(), ids.end(), 0) != ids.end());
        bool haveMarker1 = (std::find(ids.begin(), ids.end(), 1) != ids.end());

        if (haveMarker0 && haveMarker1) {
            // Estimate poses
            std::vector<cv::Vec3d> rvecs, tvecs;
            estimateMarkerPoses(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

            // Get index of marker 0 and marker 1
            int idx0 = (int)(std::find(ids.begin(), ids.end(), 0) - ids.begin());
            int idx1 = (int)(std::find(ids.begin(), ids.end(), 1) - ids.begin());

            // Compute distance (meters) -> convert to cm
            float distM = compute3DDistance(tvecs[idx0], tvecs[idx1]);
            distanceCM   = distM * 100.0f;

            // Read offset ratio from ROS parameter server
            nh.getParam("offset_ratio", offsetRatio);


            // If ball detected, compute distances
            if (ballDetected) {
                cv::Point2f center0 = getMarkerCenter(corners[idx0]);
                cv::Point2f center1 = getMarkerCenter(corners[idx1]);

                // (Optional) Draw ball & lines
                if (!no_display) {
                    cv::circle(frame, ballCenter, (int)ballRadius, cv::Scalar(0, 255, 0), 2);
                    cv::line(frame, ballCenter, center0, cv::Scalar(0, 0, 255), 2);
                    cv::line(frame, ballCenter, center1, cv::Scalar(0, 0, 255), 2);
                }

                double pxDistM0 = cv::norm(center0 - ballCenter);
                double pxDistM1 = cv::norm(center1 - ballCenter);
                if ((pxDistM0 + pxDistM1) > 1e-3) {
                    distToMarker0 = (pxDistM0 / (pxDistM0 + pxDistM1)) * distanceCM;
                    distToMarker1 = (pxDistM1 / (pxDistM0 + pxDistM1)) * distanceCM;
                }

                // Distance from offset
                distFromOffset = (distanceCM * offsetRatio) - distToMarker0;

                // Compute velocity
                double currentPos = distToMarker0;
                auto currentTime  = std::chrono::high_resolution_clock::now();
                if (previousPos >= 0.0) {
                    double traveled = currentPos - previousPos;
                    std::chrono::duration<double> dt = currentTime - previousTime;
                    if (dt.count() > 0.0) {
                        velocity = traveled / dt.count();  // cm/s
                    }
                }
                previousPos  = currentPos;
                previousTime = currentTime;

                // Publish on ROS topic
                ball_msgs::ball ball_msg;
                ball_msg.velocity = velocity;
                ball_msg.distance = distFromOffset;
                ball_pub.publish(ball_msg);

                ROS_INFO("Published ball info - Velocity: %.2f cm/s, Distance: %.2f cm",
                         ball_msg.velocity, ball_msg.distance);
            }

            // (Optional) Draw marker axes & offset dot
            if (!no_display) {
                drawMarkerAxes(frame, cameraMatrix, distCoeffs, rvecs[idx0], tvecs[idx0], 0.05f);
                drawMarkerAxes(frame, cameraMatrix, distCoeffs, rvecs[idx1], tvecs[idx1], 0.05f);
                drawOffsetDot(frame, corners[idx0], corners[idx1], offsetRatio);
            }
        }

        // Compute frame time
        auto frameEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = frameEnd - frameStart;
        double seconds = elapsed.count();
        double hz      = (seconds > 0.0) ? (1.0 / seconds) : 0.0;

        // Update console display
        updateDisplay(distanceCM, offsetRatio, ballDetected,
                      distToMarker0, distToMarker1, distFromOffset,
                      velocity, seconds, hz);


        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        // For more precision, include milliseconds
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // Format timestamp as "YYYY-MM-DD HH:MM:SS.mmm"
        std::stringstream timestamp_ss;
        timestamp_ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

        // Write data to CSV if the file is open
        if (csvFile.is_open()) {
            csvFile << "\"" << timestamp_ss.str() << "\","
                    << distanceCM << ","
                    << offsetRatio << ","
                    << ballDetected << ","
                    << distToMarker0 << ","
                    << distToMarker1 << ","
                    << distFromOffset << ","
                    << velocity << ","
                    << std::fixed << std::setprecision(6) << seconds << ","
                    << std::fixed << std::setprecision(2) << hz
                    << "\n";
        }


        // Show frames if requested
        if (!no_display) {
            cv::imshow("Processed Frame", frame);
            cv::imshow("Ball Mask", ballMask);
        }

        // Check for ESC key
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    // Cleanup
    cap.release();
    cv::destroyAllWindows();


    // Close CSV file if open
    if (csvFile.is_open()) {
        csvFile.close();
        std::cout << "CSV file saved at: " << csvFilePath << "\n";
    }


    return 0;
}
