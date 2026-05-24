#pragma once

#include <mutex>
#include "FlightCommand.hpp"

class SharedFlightCommand
{
public:
    void set(const FlightCommand& command);
    FlightCommand get() const;

private:
    mutable std::mutex mutex;
    FlightCommand latestCommand;
};