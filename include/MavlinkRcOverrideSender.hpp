#pragma once

#include <string>
#include <netinet/in.h>
#include "RcChannels.hpp"
#include "FlightCommand.hpp"

class MavlinkRcOverrideSender
{
public:
    MavlinkRcOverrideSender(
        const std::string& targetIp,
        int targetPort,
        unsigned char systemId,
        unsigned char componentId
    );

    ~MavlinkRcOverrideSender();

    bool open();
    void close();
    bool sendRcOverride(const RcChannels& channels);
    bool sendHeartbeat();
    bool sendManualControl(const FlightCommand& command);
    
private:
    std::string targetIp;
    int targetPort;
    unsigned char systemId;
    unsigned char componentId;

    int socketFd;
    bool isOpen;

    sockaddr_in targetAddress;
};