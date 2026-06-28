#include "CommandPrinter.hpp"

#include <iomanip>
#include <iostream>

void CommandPrinter::printControllerState(const ControllerState& state)
{
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "{\n";
    std::cout << "  \"mode\": \"" << state.mode << "\",\n";

    std::cout << "  \"command\": {\n";
    std::cout << "    \"move_left_right\": " << state.command.moveLeftRight << ",\n";
    std::cout << "    \"move_forward_back\": " << state.command.moveForwardBack << ",\n";
    std::cout << "    \"yaw\": " << state.command.yaw << ",\n";
    std::cout << "    \"altitude\": " << state.command.altitude << ",\n";
    std::cout << "    \"l2\": " << state.command.l2 << ",\n";
    std::cout << "    \"r2\": " << state.command.r2 << ",\n";
    std::cout << "    \"dpad_left_right\": " << state.command.dpadLeftRight << ",\n";
    std::cout << "    \"dpad_up_down\": " << state.command.dpadUpDown << "\n";
    std::cout << "  },\n";

    std::cout << "  \"buttons\": {\n";
    std::cout << "    \"x\": " << state.buttons.x << ",\n";
    std::cout << "    \"circle\": " << state.buttons.circle << ",\n";
    std::cout << "    \"triangle\": " << state.buttons.triangle << ",\n";
    std::cout << "    \"square\": " << state.buttons.square << ",\n";
    std::cout << "    \"l1\": " << state.buttons.l1 << ",\n";
    std::cout << "    \"r1\": " << state.buttons.r1 << ",\n";
    std::cout << "    \"l2_button\": " << state.buttons.l2Button << ",\n";
    std::cout << "    \"r2_button\": " << state.buttons.r2Button << ",\n";
    std::cout << "    \"wireless\": " << state.buttons.wireless << ",\n";
    std::cout << "    \"options\": " << state.buttons.options << ",\n";
    std::cout << "    \"home\": " << state.buttons.home << "\n";
    std::cout << "  }\n";

    std::cout << "}\n\n";
}

void CommandPrinter::printFlightCommand(const FlightCommand& command)
{
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "FLIGHT_COMMAND "
              << "mode=" << command.controlMode
              << " roll=" << command.roll
              << " pitch=" << command.pitch
              << " throttle=" << command.throttle
              << " yaw=" << command.yaw
              << " precision=" << command.precisionMode
              << " arm=" << command.armRequested
              << " disarm=" << command.disarmRequested
              << " takeoff=" << command.takeoffRequested
              << " land=" << command.landRequested
              << " flip=" << command.flipRequested
              << " emergency=" << command.emergencyStopRequested
              << "\n";
}

void CommandPrinter::printRcChannels(const RcChannels& channels)
{
    std::cout << "RC_CHANNELS "
              << "ch1_roll=" << channels.ch1Roll
              << " ch2_pitch=" << channels.ch2Pitch
              << " ch3_throttle=" << channels.ch3Throttle
              << " ch4_yaw=" << channels.ch4Yaw
              << " ch5_mode=" << channels.ch5Mode
              << " ch6_aux=" << channels.ch6Aux
              << " ch7_aux=" << channels.ch7Aux
              << " ch8_aux=" << channels.ch8Aux
              << "\n";
}
