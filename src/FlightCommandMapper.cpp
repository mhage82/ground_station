#include "FlightCommandMapper.hpp"

namespace
{
    constexpr double PRECISION_SCALE = 0.20;
    constexpr double PRECISION_TRIGGER_THRESHOLD = 0.30;
}

bool FlightCommandMapper::risingEdge(bool previous, bool current)
{
    return !previous && current;
}

FlightCommand FlightCommandMapper::map(const ControllerState& controllerState)
{
    FlightCommand command;

    command.controlMode = controllerState.mode;

    command.roll = controllerState.command.moveLeftRight;
    command.pitch = controllerState.command.moveForwardBack;
    command.yaw = controllerState.command.yaw;

    double rawThrottle = controllerState.command.altitude;
    command.throttle = rawThrottle;

    /*
        Safety mapping:

        X + R1      -> arm
        Circle + L1 -> disarm
        Triangle    -> takeoff
        Triangle+R1 -> flip
        Square      -> land
        Home/PS     -> emergency stop

        L2          -> precision mode modifier
    */

    bool armCombo =
        controllerState.buttons.x == 1 &&
        controllerState.buttons.r1 == 1;

    bool disarmCombo =
        controllerState.buttons.circle == 1 &&
        controllerState.buttons.l1 == 1;

    bool flipCombo =
        controllerState.buttons.triangle == 1 &&
        controllerState.buttons.r1 == 1;

    bool takeoffButton =
        controllerState.buttons.triangle == 1 &&
        !flipCombo;

    bool landButton =
        controllerState.buttons.square == 1;

    bool emergencyButton =
        controllerState.buttons.home == 1 ||
        controllerState.buttons.wireless == 1;

    command.armRequested = risingEdge(previousArmCombo, armCombo);
    command.disarmRequested = risingEdge(previousDisarmCombo, disarmCombo);
    command.takeoffRequested = risingEdge(previousTakeoffButton, takeoffButton);
    command.landRequested = risingEdge(previousLandButton, landButton);
    command.flipRequested = risingEdge(previousFlipCombo, flipCombo);
    command.emergencyStopRequested = emergencyButton;

    command.precisionMode =
        controllerState.buttons.l2Button == 1 ||
        controllerState.command.l2 >= PRECISION_TRIGGER_THRESHOLD;

    if (command.precisionMode)
    {
        if (!previousPrecisionMode)
        {
            precisionBaseThrottle = command.throttle;
        }

        command.roll *= PRECISION_SCALE;
        command.pitch *= PRECISION_SCALE;

        command.throttle =
            precisionBaseThrottle +
            (command.throttle - precisionBaseThrottle) * PRECISION_SCALE;

        command.yaw *= PRECISION_SCALE;
    }

    previousArmCombo = armCombo;
    previousDisarmCombo = disarmCombo;
    previousTakeoffButton = takeoffButton;
    previousLandButton = landButton;
    previousFlipCombo = flipCombo;
    previousEmergencyButton = emergencyButton;
    previousPrecisionMode = command.precisionMode;

    return command;
}
