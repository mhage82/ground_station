#include "ControllerMapper.hpp"

#include <cmath>
#include <string>

namespace
{
    constexpr double DEAD_BAND = 0.08;
    constexpr double DOMINANT_AXIS_THRESHOLD = 0.35;
    constexpr double CROSS_AXIS_DEAD_BAND = 0.35;

    void applyCrossAxisDeadBand(double& firstAxis, double& secondAxis)
    {
        const double firstAbs = std::abs(firstAxis);
        const double secondAbs = std::abs(secondAxis);

        if (firstAbs >= DOMINANT_AXIS_THRESHOLD &&
            secondAbs <= CROSS_AXIS_DEAD_BAND)
        {
            secondAxis = 0.0;
        }

        if (secondAbs >= DOMINANT_AXIS_THRESHOLD &&
            firstAbs <= CROSS_AXIS_DEAD_BAND)
        {
            firstAxis = 0.0;
        }
    }
}

ControllerMapper::ControllerMapper()
{
    updateCommand();
}

bool ControllerMapper::handleEvent(const input_event& event)
{
    if (event.type == EV_ABS)
    {
        updateAxisState(event.code, event.value);
        return true;
    }

    if (event.type == EV_KEY)
    {
        std::string buttonName = buttonNameFromCode(event.code);

        if (buttonName.empty())
        {
            return false;
        }

        updateButtonState(buttonName, event.value);

        if (buttonName == "options" && event.value == 1)
        {
            toggleMode();
        }

        return true;
    }

    return false;
}

const ControllerState& ControllerMapper::getState() const
{
    return state;
}

double ControllerMapper::round3(double value)
{
    return std::round(value * 1000.0) / 1000.0;
}

double ControllerMapper::clamp(double value, double minValue, double maxValue)
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

double ControllerMapper::normalizeAxis(int value, bool invert)
{
    double normalized = (static_cast<double>(value) - 128.0) / 127.0;

    if (invert)
    {
        normalized = -normalized;
    }

    if (std::abs(normalized) < DEAD_BAND)
    {
        normalized = 0.0;
    }

    normalized = clamp(normalized, -1.0, 1.0);
    return round3(normalized);
}

double ControllerMapper::normalizeTrigger(int value)
{
    double normalized = static_cast<double>(value) / 255.0;

    if (normalized < DEAD_BAND)
    {
        normalized = 0.0;
    }

    normalized = clamp(normalized, 0.0, 1.0);
    return round3(normalized);
}

void ControllerMapper::updateCommand()
{
    const RawAxes& axes = state.rawAxes;

    double moveLeftRight = normalizeAxis(axes.absRx, true);
    double moveForwardBack = normalizeAxis(axes.absRy, true);
    double yaw = normalizeAxis(axes.absX, false);
    double altitude = normalizeAxis(axes.absY, true);

    applyCrossAxisDeadBand(moveLeftRight, moveForwardBack);
    applyCrossAxisDeadBand(yaw, altitude);

    state.command.moveLeftRight = moveLeftRight;
    state.command.moveForwardBack = moveForwardBack;
    state.command.yaw = yaw;
    state.command.altitude = altitude;

    state.command.l2 = normalizeTrigger(axes.absZ);
    state.command.r2 = normalizeTrigger(axes.absRz);

    state.command.dpadLeftRight = axes.hat0X;
    state.command.dpadUpDown = axes.hat0Y;
}

void ControllerMapper::updateAxisState(unsigned short code, int value)
{
    switch (code)
    {
        case ABS_X:
            state.rawAxes.absX = value;
            break;

        case ABS_Y:
            state.rawAxes.absY = value;
            break;

        case ABS_RX:
            state.rawAxes.absRx = value;
            break;

        case ABS_RY:
            state.rawAxes.absRy = value;
            break;

        case ABS_Z:
            state.rawAxes.absZ = value;
            break;

        case ABS_RZ:
            state.rawAxes.absRz = value;
            break;

        case ABS_HAT0X:
            state.rawAxes.hat0X = value;
            break;

        case ABS_HAT0Y:
            state.rawAxes.hat0Y = value;
            break;

        default:
            break;
    }

    updateCommand();
}

std::string ControllerMapper::buttonNameFromCode(unsigned short code)
{
    switch (code)
    {
        case 304: return "x";
        case 305: return "circle";

        // Verify these two based on your latest observed C++ output.
        case 307: return "triangle";
        case 308: return "square";

        case 310: return "l1";
        case 311: return "r1";

        case 312: return "l2_button";
        case 313: return "r2_button";

        case 314: return "wireless";
        case 315: return "options";
        case 316: return "home";

        default: return "";
    }
}

void ControllerMapper::updateButtonState(const std::string& buttonName, int value)
{
    if (value != 0 && value != 1)
    {
        return;
    }

    if (buttonName == "x")
    {
        state.buttons.x = value;
    }
    else if (buttonName == "circle")
    {
        state.buttons.circle = value;
    }
    else if (buttonName == "triangle")
    {
        state.buttons.triangle = value;
    }
    else if (buttonName == "square")
    {
        state.buttons.square = value;
    }
    else if (buttonName == "l1")
    {
        state.buttons.l1 = value;
    }
    else if (buttonName == "r1")
    {
        state.buttons.r1 = value;
    }
    else if (buttonName == "l2_button")
    {
        state.buttons.l2Button = value;
    }
    else if (buttonName == "r2_button")
    {
        state.buttons.r2Button = value;
    }
    else if (buttonName == "wireless")
    {
        state.buttons.wireless = value;
    }
    else if (buttonName == "options")
    {
        state.buttons.options = value;
    }
    else if (buttonName == "home")
    {
        state.buttons.home = value;
    }
}

void ControllerMapper::toggleMode()
{
    if (state.mode == "simple")
    {
        state.mode = "advanced";
    }
    else
    {
        state.mode = "simple";
    }
}
