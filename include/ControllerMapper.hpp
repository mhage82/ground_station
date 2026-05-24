#pragma once

#include <linux/input.h>

#include "ControllerTypes.hpp"

class ControllerMapper
{
public:
    ControllerMapper();

    bool handleEvent(const input_event& event);
    const ControllerState& getState() const;

private:
    ControllerState state;

    static double round3(double value);
    static double clamp(double value, double minValue, double maxValue);
    static double normalizeAxis(int value, bool invert);
    static double normalizeTrigger(int value);

    static std::string buttonNameFromCode(unsigned short code);

    void updateCommand();
    void updateAxisState(unsigned short code, int value);
    void updateButtonState(const std::string& buttonName, int value);
    void toggleMode();
};