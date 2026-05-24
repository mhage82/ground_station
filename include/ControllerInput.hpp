#pragma once

#include <linux/input.h>

#include <optional>
#include <string>

class ControllerInput
{
public:
    ControllerInput();
    ~ControllerInput();

    bool openController();
    bool readEvent(input_event& event);
    void closeController();

    std::string getDevicePath() const;

private:
    int fileDescriptor;
    std::string devicePath;

    static std::string getDeviceName(const std::string& path);
    static bool isMainControllerDevice(const std::string& name);
    static std::optional<std::string> findControllerDevice();
};