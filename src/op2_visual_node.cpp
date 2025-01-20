// File: src/op2_visual_node.cpp
#include "op2_visual/ball_tracking.h"
#include <ros/ros.h>
#include <ball_msgs/ball.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ball_tracking_node");
    ros::NodeHandle nh;

    bool no_gui = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--no-gui") {
            no_gui = true;
        }
    }

    // Setting a scalar parameter
    nh.setParam("offset_ratio", 0.5);

    // Create a publisher on the "ball_topic"
    ros::Publisher ball_pub = nh.advertise<ball_msgs::ball>("ball_topic", 10);

    // Call the main tracking function
    ball_tracking(nh, ball_pub, no_gui);

    return 0;
}