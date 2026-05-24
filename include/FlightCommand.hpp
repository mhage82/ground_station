#pragma once

#include <string>

struct FlightCommand
{
    double roll = 0.0;
    double pitch = 0.0;
    double throttle = 0.0;
    double yaw = 0.0;

    bool armRequested = false;
    bool disarmRequested = false;
    bool takeoffRequested = false;
    bool landRequested = false;
    bool emergencyStopRequested = false;

    bool precisionMode = false;

    std::string controlMode = "simple";
};