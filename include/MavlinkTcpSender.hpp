#pragma once

#include <atomic>
#include <string>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include "FlightCommand.hpp"
#include "RcChannels.hpp"

class MavlinkTcpSender
{
public:
    MavlinkTcpSender(
        const std::string& targetIp,
        int targetPort,
        unsigned char systemId,
        unsigned char componentId
    );

    ~MavlinkTcpSender();

    bool open();
    void close();

    bool sendHeartbeat();
    bool sendRcOverride(const RcChannels& channels);
    bool sendManualControl(const FlightCommand& command);
    bool sendSetMode(uint32_t customMode);
    bool sendArmDisarm(bool arm);
    bool sendTakeoff(float altitudeMeters);
    bool sendEmergencyStop();
    bool receiveAndPrintTelemetryOnce();
    bool vehicleIsInAir() const;
    void setReceiveNonBlocking(bool enabled);
    bool requestMessageInterval(uint32_t messageId, double rateHz);
    bool requestDefaultTelemetryStreams();
    bool requestDataStream(uint8_t streamId, uint16_t rateHz, bool start);

private:
    std::string targetIp;
    int targetPort;
    unsigned char systemId;
    unsigned char componentId;

    int socketFd;
    bool isOpen;

    mavlink_status_t receiveStatus {};
    std::atomic<int> landedState;

    bool sendMavlinkMessage(const mavlink_message_t& message);
    void handleReceivedMessage(const mavlink_message_t& message);
};
