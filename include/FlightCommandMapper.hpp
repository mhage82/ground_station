#pragma once

#include "FlightCommand.hpp"
#include "ControllerTypes.hpp"

class FlightCommandMapper
{
public:
    FlightCommand map(const ControllerState& controllerState);

private:
    bool previousArmCombo = false;
    bool previousDisarmCombo = false;
    bool previousTakeoffButton = false;
    bool previousLandButton = false;
    bool previousFlipCombo = false;
    bool previousEmergencyButton = false;

    bool previousPrecisionMode = false;
    double precisionBaseThrottle = 0.0;

    static bool risingEdge(bool previous, bool current);
};
