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

    std::cout << "{\n";
    std::cout << "  \"flight_command\": {\n";
    std::cout << "    \"control_mode\": \"" << command.controlMode << "\",\n";
    std::cout << "    \"roll\": " << command.roll << ",\n";
    std::cout << "    \"pitch\": " << command.pitch << ",\n";
    std::cout << "    \"throttle\": " << command.throttle << ",\n";
    std::cout << "    \"yaw\": " << command.yaw << ",\n";
    std::cout << "    \"precision_mode\": " << command.precisionMode << ",\n";
    std::cout << "    \"arm_requested\": " << command.armRequested << ",\n";
    std::cout << "    \"disarm_requested\": " << command.disarmRequested << ",\n";
    std::cout << "    \"takeoff_requested\": " << command.takeoffRequested << ",\n";
    std::cout << "    \"land_requested\": " << command.landRequested << ",\n";
    std::cout << "    \"emergency_stop_requested\": " << command.emergencyStopRequested << "\n";
    std::cout << "  }\n";
    std::cout << "}\n\n";
}

void CommandPrinter::printRcChannels(const RcChannels& channels)
{
    std::cout << "{\n";
    std::cout << "  \"rc_channels\": {\n";
    std::cout << "    \"ch1_roll\": " << channels.ch1Roll << ",\n";
    std::cout << "    \"ch2_pitch\": " << channels.ch2Pitch << ",\n";
    std::cout << "    \"ch3_throttle\": " << channels.ch3Throttle << ",\n";
    std::cout << "    \"ch4_yaw\": " << channels.ch4Yaw << ",\n";
    std::cout << "    \"ch5_mode\": " << channels.ch5Mode << ",\n";
    std::cout << "    \"ch6_aux\": " << channels.ch6Aux << ",\n";
    std::cout << "    \"ch7_aux\": " << channels.ch7Aux << ",\n";
    std::cout << "    \"ch8_aux\": " << channels.ch8Aux << "\n";
    std::cout << "  }\n";
    std::cout << "}\n\n";
}
