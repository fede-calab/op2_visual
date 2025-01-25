// File: src/op2_visual_node.cpp
#include "op2_visual/ball_tracking.h"
#include <ros/ros.h>
#include <ball_msgs/ball.h>
#include <string>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ball_tracking_node");
    ros::NodeHandle nh;

    std::string cameraName = "logitech_c270_hd_webcam";
    bool no_gui = false;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--") {
            // Skip ROS argument separator
            continue;
        } else if (arg == "--no-gui") {
            no_gui = true;
        } else if (arg == "--default-camera") {
            // Uses default cameraName value as the name for the camera
            continue;
        } else if (arg.substr(0, 13) == "--camera-name") {
            if (i + 1 < argc) {
                cameraName = argv[++i];
            } else {
                std::cerr << "Error: --camera-name option requires a value." << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return 1;
        }
    }

    // Output parsed arguments for verification
    std::cout << "Using camera name: " << (cameraName.empty() ? "No camera name specified" : cameraName) << std::endl;
    std::cout << "No GUI mode: " << (no_gui ? "Enabled" : "Disabled") << std::endl;

    nh.setParam("no_gui", no_gui);
    nh.setParam("camera_name", cameraName);

    // Setting a scalar parameter
    nh.setParam("offset_ratio", 0.5);

    // Create a publisher on the "ball_topic"
    ros::Publisher ball_pub = nh.advertise<ball_msgs::ball>("ball_topic", 10);

    // Call the main tracking function
    ball_tracking(nh, ball_pub);

    return 0;
}