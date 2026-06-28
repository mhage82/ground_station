#include "FlightCommandToRcMapper.hpp"

namespace
{
    constexpr int RC_OVERRIDE_IGNORE = 65535;
}

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

    // Do not override mode/aux channels. ArduPilot commonly uses ch5 as the
    // flight mode switch, which can fight SET_MODE if we keep forcing it.
    channels.ch5Mode = RC_OVERRIDE_IGNORE;
    channels.ch6Aux = RC_OVERRIDE_IGNORE;
    channels.ch7Aux = RC_OVERRIDE_IGNORE;
    channels.ch8Aux = RC_OVERRIDE_IGNORE;

    return channels;
}
