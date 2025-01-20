// File: include/op2_visual/display_utils.h
#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include <iostream>

// ANSI escape codes for terminal control
#define CLEAR_SCREEN "\033[2J"
#define CURSOR_HOME  "\033[H"

// Round a float to nearest integer
float roundUp(float n);

// Update your console/terminal display with current tracking info
void updateDisplay(double markerDistance,
                   double offsetRatio,
                   bool   ballDetected,
                   double distToMarker0,
                   double distToMarker1,
                   double distFromOffset,
                   double velocity,
                   double frameTime,
                   double frameRate);

#endif // DISPLAY_UTILS_H