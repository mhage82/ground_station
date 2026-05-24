#pragma once

#include "FlightCommand.hpp"
#include "RcChannels.hpp"

class FlightCommandToRcMapper
{
public:
    RcChannels map(const FlightCommand& command) const;

private:
    static int mapCenteredAxisToPwm(double value);
    static int mapThrottleToPwm(double value);
    static double clamp(double value, double minValue, double maxValue);
};