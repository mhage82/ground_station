#pragma once

#include "ControllerTypes.hpp"
#include "FlightCommand.hpp"
#include "RcChannels.hpp"

class CommandPrinter
{
public:
    static void printControllerState(const ControllerState& state);
    static void printFlightCommand(const FlightCommand& command);
    static void printRcChannels(const RcChannels& channels);
};