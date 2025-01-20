// File: src/display_utils.cpp
#include "op2_visual/display_utils.h"
#include <cmath>

float roundUp(float n)
{
    return std::round(n);
}

void updateDisplay(double markerDistance,
                   double offsetRatio,
                   bool   ballDetected,
                   double distToMarker0,
                   double distToMarker1,
                   double distFromOffset,
                   double velocity,
                   double frameTime,
                   double frameRate)
{
    // Clear screen and position cursor to top-left
    std::cout << CLEAR_SCREEN << CURSOR_HOME;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    std::cout << "                Ball Tracking Status                \n";
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

    bool markersDetected = (markerDistance > 0.0);
    std::cout << "Marker Status: " 
              << (markersDetected ? "Detected" : "Not Detected") << "\n";

    if (markersDetected) {
        std::cout << "Distance between markers: " << markerDistance << " cm\n";
        std::cout << "Offset (ratio):           " << offsetRatio << "\n";
        std::cout << "Offset (absolute):        " << offsetRatio * markerDistance << " cm\n";
    }

    std::cout << "\nBall Status: " 
              << (ballDetected ? "Detected" : "Not Detected") << "\n";

    if (ballDetected) {
        std::cout << "Distance to Marker 0:     " << distToMarker0 << " cm\n";
        std::cout << "Distance to Marker 1:     " << distToMarker1 << " cm\n";
        std::cout << "Distance from Offset:     " << distFromOffset << " cm\n";
        std::cout << "Ball Velocity:            " << velocity << " cm/s\n";
    }

    std::cout << "\nFrame Processing Time: " << frameTime 
              << " s (" << frameRate << " Hz)\n";

    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    std::cout << "Press Ctrl+C to exit\n";
    std::cout.flush();
}