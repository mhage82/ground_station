#include "FlightCommandToRcMapper.hpp"

double FlightCommandToRcMapper::clamp(double value, double minValue, double maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

int FlightCommandToRcMapper::mapCenteredAxisToPwm(double value)
{
    value = clamp(value, -1.0, 1.0);

    return static_cast<int>(1500.0 + value * 500.0);
}

int FlightCommandToRcMapper::mapThrottleToPwm(double value)
{
    value = clamp(value, 0.0, 1.0);

    return static_cast<int>(1000.0 + value * 1000.0);
}

RcChannels FlightCommandToRcMapper::map(const FlightCommand& command) const
{
    RcChannels channels;

    channels.ch1Roll = mapCenteredAxisToPwm(command.roll);
    channels.ch2Pitch = mapCenteredAxisToPwm(command.pitch);
    channels.ch3Throttle = mapThrottleToPwm(command.throttle);
    channels.ch4Yaw = mapCenteredAxisToPwm(command.yaw);

    // Placeholder mode channel.
    // Later this can select Stabilize/AltHold/Loiter/etc. in ArduPilot.
    if (command.controlMode == "simple")
    {
        channels.ch5Mode = 1000;
    }
    else
    {
        channels.ch5Mode = 1500;
    }

    // Placeholder aux channel.
    // Use precision mode as a visible RC aux behavior for now.
    channels.ch6Aux = command.precisionMode ? 2000 : 1000;

    channels.ch7Aux = 0;
    channels.ch8Aux = 0;

    return channels;
}