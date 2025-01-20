// File: include/op2_visual/ball_tracking.h
#ifndef BALL_TRACKING_H
#define BALL_TRACKING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ball_msgs/ball.h>

// The main tracking function to be called by your ROS node.
int ball_tracking(ros::NodeHandle nh, ros::Publisher& ball_pub, bool no_display);

#endif // BALL_TRACKING_H