#include "FlightCommandPublisher.hpp"
#include "CommandPrinter.hpp"

#include <chrono>
#include <thread>

namespace
{
    constexpr uint32_t ARDUCOPTER_MODE_LOITER = 5;
    constexpr uint32_t ARDUCOPTER_MODE_LAND = 9;
    constexpr uint32_t ARDUCOPTER_MODE_FLIP = 14;
    constexpr float TAKEOFF_ALTITUDE_METERS = 2.0f;
    constexpr double HOVER_THROTTLE = 0.5;
    constexpr double HOVER_THROTTLE_RANGE = 0.5;

    double clamp(double value, double minValue, double maxValue)
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

    bool commandsAreEqual(const FlightCommand& a, const FlightCommand& b)
    {
        return
            a.roll == b.roll &&
            a.pitch == b.pitch &&
            a.throttle == b.throttle &&
            a.yaw == b.yaw &&
            a.armRequested == b.armRequested &&
            a.disarmRequested == b.disarmRequested &&
            a.takeoffRequested == b.takeoffRequested &&
            a.landRequested == b.landRequested &&
            a.flipRequested == b.flipRequested &&
            a.emergencyStopRequested == b.emergencyStopRequested &&
            a.precisionMode == b.precisionMode &&
            a.controlMode == b.controlMode;
    }
}

FlightCommandPublisher::FlightCommandPublisher(SharedFlightCommand& sharedCommandRef)
    : sharedCommand(sharedCommandRef),
      mavlinkSender("127.0.0.1", 5760, 255, 0)
{
}

bool FlightCommandPublisher::open()
{
     if (!mavlinkSender.open())
    {
        return false;
    }

    mavlinkSender.sendHeartbeat();
    mavlinkSender.requestDefaultTelemetryStreams();

    return true;
}

void FlightCommandPublisher::run(std::atomic<bool>& running, int publishRateHz)
{
    using namespace std::chrono;

    int heartbeatCounter = 0;

    if (publishRateHz <= 0)
    {
        publishRateHz = 20;
    }

    const milliseconds periodMs(1000 / publishRateHz);

    FlightCommand previousCommand;
    bool havePreviousCommand = false;

    bool lastArmRequested = false;
    bool lastDisarmRequested = false;
    bool lastTakeoffRequested = false;
    bool lastLandRequested = false;
    bool lastFlipRequested = false;

    while (running.load())
    {
        FlightCommand command = sharedCommand.get();

        if (mavlinkSender.vehicleIsInAir())
        {
            command.throttle =
                clamp(
                    HOVER_THROTTLE + command.throttle * HOVER_THROTTLE_RANGE,
                    0.0,
                    1.0);
        }
        else
        {
            command.throttle = clamp(command.throttle, 0.0, 1.0);
        }

        RcChannels channels = rcMapper.map(command);


        // Keep ArduPilot accepting this process as the active GCS.
        if (heartbeatCounter <= 0)
        {
            mavlinkSender.sendHeartbeat();
            heartbeatCounter = publishRateHz;
        }

        heartbeatCounter--;

        if (command.armRequested && !lastArmRequested)
        {
            mavlinkSender.sendSetMode(ARDUCOPTER_MODE_LOITER);
            mavlinkSender.sendArmDisarm(true);
        }
        lastArmRequested = command.armRequested;

        if (command.disarmRequested && !lastDisarmRequested)
        {
            mavlinkSender.sendArmDisarm(false);
        }
        lastDisarmRequested = command.disarmRequested;

        if (command.takeoffRequested && !lastTakeoffRequested)
        {
            mavlinkSender.sendTakeoff(TAKEOFF_ALTITUDE_METERS);
        }
        lastTakeoffRequested = command.takeoffRequested;

        if (command.landRequested && !lastLandRequested)
        {
            mavlinkSender.sendSetMode(ARDUCOPTER_MODE_LAND);
        }
        lastLandRequested = command.landRequested;

        if (command.flipRequested && !lastFlipRequested)
        {
            mavlinkSender.sendSetMode(ARDUCOPTER_MODE_FLIP);
        }
        lastFlipRequested = command.flipRequested;

        if (command.emergencyStopRequested)
        {
            mavlinkSender.sendEmergencyStop();
        }

        mavlinkSender.sendRcOverride(channels);
        // mavlinkSender.sendManualControl(command);

        mavlinkSender.receiveAndPrintTelemetryOnce();

        if (!havePreviousCommand || !commandsAreEqual(previousCommand, command))
        {
            CommandPrinter::printFlightCommand(command);
            CommandPrinter::printRcChannels(channels);

            previousCommand = command;
            havePreviousCommand = true;
        }
        std::this_thread::sleep_for(periodMs);
    }
}
