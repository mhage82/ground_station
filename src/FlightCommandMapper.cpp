#include "FlightCommandMapper.hpp"

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

    /*
        Controller altitude is -1.0 to +1.0.

        For a flight-stack-style command, throttle is usually easier
        to represent as 0.0 to 1.0.

        altitude = -1.0 -> throttle = 0.0
        altitude =  0.0 -> throttle = 0.5
        altitude = +1.0 -> throttle = 1.0
    */
    command.throttle = controllerState.command.altitude;

    if (command.throttle < 0.0)
    {
        command.throttle = 0.0;
    }
    else if (command.throttle > 1.0)
    {
        command.throttle = 1.0;
    }

    /*
        Safety mapping:

        X + R1      -> arm
        Circle + L1 -> disarm
        Triangle    -> takeoff
        Square      -> land
        Home        -> emergency stop

        L2 or R2    -> precision mode modifier
    */

    bool armCombo =
        controllerState.buttons.x == 1 &&
        controllerState.buttons.r1 == 1;

    bool disarmCombo =
        controllerState.buttons.circle == 1 &&
        controllerState.buttons.l1 == 1;

    bool takeoffButton =
        controllerState.buttons.triangle == 1;

    bool landButton =
        controllerState.buttons.square == 1;

    bool emergencyButton =
        controllerState.buttons.home == 1;

    command.armRequested = risingEdge(previousArmCombo, armCombo);
    command.disarmRequested = risingEdge(previousDisarmCombo, disarmCombo);
    command.takeoffRequested = risingEdge(previousTakeoffButton, takeoffButton);
    command.landRequested = risingEdge(previousLandButton, landButton);
    command.emergencyStopRequested = risingEdge(previousEmergencyButton, emergencyButton);

    command.precisionMode =
        controllerState.buttons.l2Button == 1 ||
        controllerState.buttons.r2Button == 1;

    if (command.precisionMode)
    {
        command.roll *= 0.35;
        command.pitch *= 0.35;
        command.yaw *= 0.35;
    }

    previousArmCombo = armCombo;
    previousDisarmCombo = disarmCombo;
    previousTakeoffButton = takeoffButton;
    previousLandButton = landButton;
    previousEmergencyButton = emergencyButton;

    return command;
}