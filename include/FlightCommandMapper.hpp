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
    bool previousEmergencyButton = false;

    static bool risingEdge(bool previous, bool current);
};