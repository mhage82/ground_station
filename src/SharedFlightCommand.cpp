#include "SharedFlightCommand.hpp"

void SharedFlightCommand::set(const FlightCommand& command)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestCommand = command;
}

FlightCommand SharedFlightCommand::get() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return latestCommand;
}