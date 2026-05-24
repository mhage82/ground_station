#include "MavlinkRcOverrideSender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <cstdint>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>

MavlinkRcOverrideSender::MavlinkRcOverrideSender(
    const std::string& targetIpValue,
    int targetPortValue,
    unsigned char systemIdValue,
    unsigned char componentIdValue
)
    : targetIp(targetIpValue),
      targetPort(targetPortValue),
      systemId(systemIdValue),
      componentId(componentIdValue),
      socketFd(-1),
      isOpen(false),
      targetAddress{}
{
}

MavlinkRcOverrideSender::~MavlinkRcOverrideSender()
{
    close();
}

bool MavlinkRcOverrideSender::open()
{
    if (isOpen)
    {
        return true;
    }

    socketFd = socket(AF_INET, SOCK_DGRAM, 0);

    if (socketFd < 0)
    {
        std::cerr << "Failed to create UDP socket.\n";
        return false;
    }

    std::memset(&targetAddress, 0, sizeof(targetAddress));

    targetAddress.sin_family = AF_INET;
    targetAddress.sin_port = htons(static_cast<uint16_t>(targetPort));

    if (inet_pton(AF_INET, targetIp.c_str(), &targetAddress.sin_addr) != 1)
    {
        std::cerr << "Invalid target IP address: " << targetIp << "\n";
        close();
        return false;
    }

    isOpen = true;

    std::cout << "MAVLink UDP sender opened to "
              << targetIp << ":" << targetPort << "\n";

    return true;
}

void MavlinkRcOverrideSender::close()
{
    if (socketFd >= 0)
    {
        ::close(socketFd);
        socketFd = -1;
    }

    isOpen = false;
}

bool MavlinkRcOverrideSender::sendRcOverride(const RcChannels& channels)
{
    if (!isOpen)
    {
        return false;
    }

    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;

    mavlink_msg_rc_channels_override_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        targetComponent,
        static_cast<uint16_t>(channels.ch1Roll),
        static_cast<uint16_t>(channels.ch2Pitch),
        static_cast<uint16_t>(channels.ch3Throttle),
        static_cast<uint16_t>(channels.ch4Yaw),
        static_cast<uint16_t>(channels.ch5Mode),
        static_cast<uint16_t>(channels.ch6Aux),
        static_cast<uint16_t>(channels.ch7Aux),
        static_cast<uint16_t>(channels.ch8Aux),
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX
    );

    const uint16_t length =
        mavlink_msg_to_send_buffer(buffer, &message);

    const ssize_t sentBytes = sendto(
        socketFd,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(&targetAddress),
        sizeof(targetAddress)
    );

    // std::cout << "Sent RC override: "
    //       << channels.ch1Roll << ", "
    //       << channels.ch2Pitch << ", "
    //       << channels.ch3Throttle << ", "
    //       << channels.ch4Yaw
    //       << " bytes=" << sentBytes << "\n";

    if (sentBytes != static_cast<ssize_t>(length))
    {
        std::cerr << "Failed to send MAVLink RC override packet.\n";
        return false;
    }

    return true;
}

bool MavlinkRcOverrideSender::sendHeartbeat()
{
    if (!isOpen)
    {
        return false;
    }

    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        systemId,
        componentId,
        &message,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        MAV_STATE_ACTIVE
    );

    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);

    const ssize_t sentBytes = sendto(
        socketFd,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(&targetAddress),
        sizeof(targetAddress)
    );

    return sentBytes == static_cast<ssize_t>(length);
}

bool MavlinkRcOverrideSender::sendManualControl(const FlightCommand& command)
{
    if (!isOpen)
    {
        return false;
    }

    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    constexpr uint8_t targetSystem = 1;

    auto clampDouble = [](double value, double minValue, double maxValue)
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
    };

    auto axisToManual = [&](double value)
    {
        value = clampDouble(value, -1.0, 1.0);
        return static_cast<int16_t>(value * 1000.0);
    };

    auto throttleToManual = [&](double value)
    {
        value = clampDouble(value, 0.0, 1.0);
        return static_cast<int16_t>(value * 1000.0);
    };

    /*
        MAVLink MANUAL_CONTROL:
        x = pitch forward/back
        y = roll left/right
        z = throttle 0..1000
        r = yaw left/right

        Our FlightCommand:
        roll     = -1.0..+1.0
        pitch    = -1.0..+1.0
        throttle =  0.0..+1.0
        yaw      = -1.0..+1.0
    */

    const int16_t x = axisToManual(command.pitch);
    const int16_t y = axisToManual(command.roll);
    const int16_t z = throttleToManual(command.throttle);
    const int16_t r = axisToManual(command.yaw);

    const uint16_t buttons = 0;

    mavlink_msg_manual_control_pack(
        systemId,
        componentId,
        &message,
        targetSystem,

        x,
        y,
        z,
        r,

        buttons,
        0,      // buttons2
        0,      // enabled_extensions

        0,      // s
        0,      // t
        0,      // aux1
        0,      // aux2
        0,      // aux3
        0,      // aux4
        0,      // aux5
        0       // aux6
    );

    const uint16_t length =
        mavlink_msg_to_send_buffer(buffer, &message);

    const ssize_t sentBytes = sendto(
        socketFd,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(&targetAddress),
        sizeof(targetAddress)
    );

    if (sentBytes < 0)
    {
        std::cerr << "sendto MANUAL_CONTROL failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    // std::cout << "Sent MANUAL_CONTROL: "
    //           << "x=" << x
    //           << " y=" << y
    //           << " z=" << z
    //           << " r=" << r
    //           << " bytes=" << sentBytes
    //           << "\n";

    return sentBytes == static_cast<ssize_t>(length);
}
