#include "FlightCommandPublisher.hpp"
#include "CommandPrinter.hpp"

#include <chrono>
#include <thread>

namespace
{
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

    while (running.load())
    {
        FlightCommand command = sharedCommand.get();
        RcChannels channels = rcMapper.map(command);


        // Send a heartbeat every 5 seconds
        if (heartbeatCounter <= 0)
        {
            mavlinkSender.sendHeartbeat();
            heartbeatCounter = publishRateHz;
        }

        heartbeatCounter--;

        // mavlinkSender.sendRcOverride(channels);
        mavlinkSender.sendManualControl(command);

        mavlinkSender.receiveAndPrintTelemetryOnce();

        if (!havePreviousCommand || !commandsAreEqual(previousCommand, command))
        {
            // CommandPrinter::printFlightCommand(command);
            // CommandPrinter::printRcChannels(channels);

            previousCommand = command;
            havePreviousCommand = true;
        }
        std::this_thread::sleep_for(periodMs);
    }
}