#pragma once

#include <atomic>

#include "SharedFlightCommand.hpp"
#include "FlightCommandToRcMapper.hpp"
// #include "MavlinkRcOverrideSender.hpp"
#include "MavlinkTcpSender.hpp"

class FlightCommandPublisher
{
public:
    explicit FlightCommandPublisher(SharedFlightCommand& sharedCommand);

    bool open();
    void run(std::atomic<bool>& running, int publishRateHz);

private:
    SharedFlightCommand& sharedCommand;
    FlightCommandToRcMapper rcMapper;
    // MavlinkRcOverrideSender mavlinkSender;
    MavlinkTcpSender mavlinkSender;
};