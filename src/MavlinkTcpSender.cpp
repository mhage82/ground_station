#include "MavlinkTcpSender.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <iostream>

MavlinkTcpSender::MavlinkTcpSender(
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
      landedState(MAV_LANDED_STATE_UNDEFINED)
{
}

MavlinkTcpSender::~MavlinkTcpSender()
{
    close();
}

bool MavlinkTcpSender::open()
{
    if (isOpen)
    {
        return true;
    }

    socketFd = socket(AF_INET, SOCK_STREAM, 0);

    if (socketFd < 0)
    {
        std::cerr << "Failed to create TCP socket: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    sockaddr_in serverAddress {};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(static_cast<uint16_t>(targetPort));

    if (inet_pton(AF_INET, targetIp.c_str(), &serverAddress.sin_addr) != 1)
    {
        std::cerr << "Invalid TCP target IP: " << targetIp << "\n";
        close();
        return false;
    }

    if (connect(
            socketFd,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) < 0)
    {
        std::cerr << "Failed to connect TCP MAVLink socket to "
                  << targetIp << ":" << targetPort
                  << " error: " << std::strerror(errno) << "\n";

        close();
        return false;
    }

    isOpen = true;
    std::cout << "MAVLink TCP sender connected to "
              << targetIp << ":" << targetPort << "\n";

    setReceiveNonBlocking(true);
    
    
    return true;
}

void MavlinkTcpSender::close()
{
    if (socketFd >= 0)
    {
        ::close(socketFd);
        socketFd = -1;
    }

    isOpen = false;
}

bool MavlinkTcpSender::sendMavlinkMessage(const mavlink_message_t& message)
{
    if (!isOpen)
    {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint16_t length =
        mavlink_msg_to_send_buffer(buffer, &message);

    ssize_t sentBytes = send(socketFd, buffer, length, 0);

    if (sentBytes < 0)
    {
        std::cerr << "TCP send failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    if (sentBytes != static_cast<ssize_t>(length))
    {
        std::cerr << "Partial TCP MAVLink send. Sent "
                  << sentBytes << " of " << length << " bytes.\n";
        return false;
    }

    return true;
}

bool MavlinkTcpSender::sendHeartbeat()
{
    mavlink_message_t message {};

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

    return sendMavlinkMessage(message);
}

bool MavlinkTcpSender::sendRcOverride(const RcChannels& channels)
{
    mavlink_message_t message {};

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

    bool result = sendMavlinkMessage(message);

    // std::cout << "Sent TCP RC override: "
    //           << channels.ch1Roll << ", "
    //           << channels.ch2Pitch << ", "
    //           << channels.ch3Throttle << ", "
    //           << channels.ch4Yaw << "\n";

    return result;
}

bool MavlinkTcpSender::sendManualControl(const FlightCommand& command)
{
    mavlink_message_t message {};

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

    const int16_t x = axisToManual(command.pitch);
    const int16_t y = axisToManual(command.roll);
    const int16_t z = throttleToManual(command.throttle);
    const int16_t r = axisToManual(command.yaw);

    constexpr uint16_t buttons = 0;

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
        0,
        0,

        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    );

    bool result = sendMavlinkMessage(message);

    // std::cout << "Sent TCP MANUAL_CONTROL: "
    //           << "x=" << x
    //           << " y=" << y
    //           << " z=" << z
    //           << " r=" << r
    //           << "\n";

    return result;
}

bool MavlinkTcpSender::sendSetMode(uint32_t customMode)
{
    mavlink_message_t message {};

    constexpr uint8_t targetSystem = 1;

    mavlink_msg_set_mode_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        customMode
    );

    std::cout << "Sending SET_MODE custom_mode=" << customMode << "\n";
    return sendMavlinkMessage(message);
}

bool MavlinkTcpSender::sendArmDisarm(bool arm)
{
    mavlink_message_t message {};

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;

    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        targetComponent,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm ? 1.0f : 0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    std::cout << "Sending " << (arm ? "ARM" : "DISARM") << " command\n";
    return sendMavlinkMessage(message);
}

bool MavlinkTcpSender::sendTakeoff(float altitudeMeters)
{
    mavlink_message_t message {};

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;

    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        targetComponent,
        MAV_CMD_NAV_TAKEOFF,
        0,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        altitudeMeters
    );

    std::cout << "Sending TAKEOFF command altitude_m=" << altitudeMeters << "\n";
    return sendMavlinkMessage(message);
}

bool MavlinkTcpSender::sendEmergencyStop()
{
    mavlink_message_t terminationMessage {};
    mavlink_message_t disarmMessage {};

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;
    constexpr float ardupilotForceDisarmMagic = 21196.0f;

    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &terminationMessage,
        targetSystem,
        targetComponent,
        MAV_CMD_DO_FLIGHTTERMINATION,
        0,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &disarmMessage,
        targetSystem,
        targetComponent,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0f,
        ardupilotForceDisarmMagic,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    std::cout << "Sending EMERGENCY STOP flight termination + force disarm\n";

    const bool terminationSent = sendMavlinkMessage(terminationMessage);
    const bool disarmSent = sendMavlinkMessage(disarmMessage);

    return terminationSent && disarmSent;
}

void MavlinkTcpSender::setReceiveNonBlocking(bool enabled)
{
    if (socketFd < 0)
    {
        return;
    }

    int flags = fcntl(socketFd, F_GETFL, 0);

    if (flags < 0)
    {
        return;
    }

    if (enabled)
    {
        flags |= O_NONBLOCK;
    }
    else
    {
        flags &= ~O_NONBLOCK;
    }

    fcntl(socketFd, F_SETFL, flags);
}

bool MavlinkTcpSender::receiveAndPrintTelemetryOnce()
{
    if (!isOpen)
    {
        return false;
    }

    uint8_t readBuffer[1024];
    bool receivedAnyMessage = false;

    while (true)
    {
        const ssize_t bytesRead =
            recv(socketFd, readBuffer, sizeof(readBuffer), 0);

        if (bytesRead < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }

            std::cerr << "TCP receive failed: "
                      << std::strerror(errno) << "\n";
            return false;
        }

        // std::cout << "TCP RX bytes=" << bytesRead << "\n";

        if (bytesRead == 0)
        {
            std::cerr << "TCP connection closed by ArduPilot SITL.\n";
            isOpen = false;
            return false;
        }

        for (ssize_t i = 0; i < bytesRead; ++i)
        {
            mavlink_message_t message;

            if (mavlink_parse_char(
                    MAVLINK_COMM_0,
                    readBuffer[i],
                    &message,
                    &receiveStatus))
            {
                handleReceivedMessage(message);
                receivedAnyMessage = true;
            }
        }
    }

    return receivedAnyMessage;
}

bool MavlinkTcpSender::vehicleIsInAir() const
{
    return landedState.load() == MAV_LANDED_STATE_IN_AIR;
}

void MavlinkTcpSender::handleReceivedMessage(const mavlink_message_t& message)
{
    switch (message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);

            // std::cout << "RX HEARTBEAT: "
            //           << "sys=" << static_cast<int>(message.sysid)
            //           << " comp=" << static_cast<int>(message.compid)
            //           << " type=" << static_cast<int>(heartbeat.type)
            //           << " autopilot=" << static_cast<int>(heartbeat.autopilot)
            //           << " base_mode=" << static_cast<int>(heartbeat.base_mode)
            //           << " system_status=" << static_cast<int>(heartbeat.system_status)
            //           << "\n";
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&message, &attitude);

            // std::cout << "RX ATTITUDE: "
            //           << "roll=" << attitude.roll
            //           << " pitch=" << attitude.pitch
            //           << " yaw=" << attitude.yaw
            //           << "\n";
            break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS:
        {
            mavlink_rc_channels_t rc;
            mavlink_msg_rc_channels_decode(&message, &rc);

            // std::cout << "RX RC_CHANNELS: "
            //           << "ch1=" << rc.chan1_raw
            //           << " ch2=" << rc.chan2_raw
            //           << " ch3=" << rc.chan3_raw
            //           << " ch4=" << rc.chan4_raw
            //           << " ch5=" << rc.chan5_raw
            //           << " ch6=" << rc.chan6_raw
            //           << " ch7=" << rc.chan7_raw
            //           << " ch8=" << rc.chan8_raw
            //           << "\n";
            break;
        }

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
            mavlink_servo_output_raw_t servo;
            mavlink_msg_servo_output_raw_decode(&message, &servo);

            // std::cout << "RX SERVO_OUTPUT_RAW: "
            //           << "s1=" << servo.servo1_raw
            //           << " s2=" << servo.servo2_raw
            //           << " s3=" << servo.servo3_raw
            //           << " s4=" << servo.servo4_raw
            //           << "\n";
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD:
        {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(&message, &hud);

            // std::cout << "RX VFR_HUD: "
            //           << "alt=" << hud.alt
            //           << " groundspeed=" << hud.groundspeed
            //           << " throttle=" << hud.throttle
            //           << " climb=" << hud.climb
            //           << "\n";
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(&message, &pos);

            // std::cout << "RX LOCAL_POSITION_NED: "
            //         << "x=" << pos.x
            //         << " y=" << pos.y
            //         << " z=" << pos.z
            //         << " vx=" << pos.vx
            //         << " vy=" << pos.vy
            //         << " vz=" << pos.vz
                    // << "\n";
            break;
        }

        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        {
            mavlink_extended_sys_state_t state;
            mavlink_msg_extended_sys_state_decode(&message, &state);

            const int previousLandedState = landedState.exchange(state.landed_state);

            if (previousLandedState != state.landed_state)
            {
                std::cout << "RX EXTENDED_SYS_STATE: "
                          << "landed_state=" << static_cast<int>(state.landed_state)
                          << "\n";
            }

            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&message, &sys);

            // std::cout << "RX SYS_STATUS: "
            //         << "battery_voltage_mv=" << sys.voltage_battery
            //         << " battery_remaining=" << static_cast<int>(sys.battery_remaining)
            //         << " load=" << sys.load
            //         << "\n";
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&message, &ack);

            std::cout << "RX COMMAND_ACK: "
                      << "command=" << ack.command
                      << " result=" << static_cast<int>(ack.result)
                      << "\n";
            break;
        }
        default:
        {
            // std::cout << "RX MSG ID: "
            //         << static_cast<int>(message.msgid)
            //         << " sys=" << static_cast<int>(message.sysid)
            //         << " comp=" << static_cast<int>(message.compid)
            //         << "\n";
            break;
        }
    }
}

bool MavlinkTcpSender::requestMessageInterval(uint32_t messageId, double rateHz)
{
    mavlink_message_t message {};

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;

    float intervalUs = -1.0f;

    if (rateHz > 0.0)
    {
        intervalUs = static_cast<float>(1000000.0 / rateHz);
    }

    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        targetComponent,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,

        static_cast<float>(messageId),  // param1: MAVLink message ID
        intervalUs,                     // param2: interval in microseconds
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    std::cout << "Requesting message interval: msg_id="
              << messageId
              << " rate_hz=" << rateHz
              << " interval_us=" << intervalUs
              << "\n";

    return sendMavlinkMessage(message);
}

bool MavlinkTcpSender::requestDefaultTelemetryStreams()
{
    bool ok = true;

    ok = requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 10.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, 5.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_VFR_HUD, 5.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS, 2.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5.0) && ok;
    ok = requestMessageInterval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5.0) && ok;
    ok = requestDataStream(MAV_DATA_STREAM_ALL, 10, true) && ok;

    return ok;
}

bool MavlinkTcpSender::requestDataStream(uint8_t streamId, uint16_t rateHz, bool start)
{
    mavlink_message_t message {};

    constexpr uint8_t targetSystem = 1;
    constexpr uint8_t targetComponent = 1;

    mavlink_msg_request_data_stream_pack(
        systemId,
        componentId,
        &message,
        targetSystem,
        targetComponent,
        streamId,
        rateHz,
        start ? 1 : 0
    );

    return sendMavlinkMessage(message);
}
