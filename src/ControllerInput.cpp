#include "ControllerInput.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

ControllerInput::ControllerInput()
    : fileDescriptor(-1)
{
}

ControllerInput::~ControllerInput()
{
    closeController();
}

bool ControllerInput::openController()
{
    std::optional<std::string> foundDevice = findControllerDevice();

    if (!foundDevice.has_value())
    {
        std::cerr << "Could not find main controller device.\n";
        return false;
    }

    devicePath = foundDevice.value();

    fileDescriptor = open(devicePath.c_str(), O_RDONLY);

    if (fileDescriptor < 0)
    {
        std::cerr << "Failed to open " << devicePath
                  << ": " << std::strerror(errno) << "\n";
        std::cerr << "Try running with sudo.\n";
        return false;
    }

    std::cout << "Using controller device: " << devicePath << "\n";
    return true;
}

bool ControllerInput::readEvent(input_event& event)
{
    if (fileDescriptor < 0)
    {
        return false;
    }

    ssize_t bytesRead = read(fileDescriptor, &event, sizeof(event));

    if (bytesRead == static_cast<ssize_t>(sizeof(event)))
    {
        return true;
    }

    if (bytesRead < 0)
    {
        std::cerr << "Read error: " << std::strerror(errno) << "\n";
    }

    return false;
}

void ControllerInput::closeController()
{
    if (fileDescriptor >= 0)
    {
        close(fileDescriptor);
        fileDescriptor = -1;
    }
}

std::string ControllerInput::getDevicePath() const
{
    return devicePath;
}

std::string ControllerInput::getDeviceName(const std::string& path)
{
    int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);

    if (fd < 0)
    {
        return "";
    }

    char name[256] = {};

    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0)
    {
        close(fd);
        return "";
    }

    close(fd);
    return std::string(name);
}

bool ControllerInput::isMainControllerDevice(const std::string& name)
{
    bool hasControllerName =
        name.find("DualSense Wireless Controller") != std::string::npos ||
        name.find("Wireless Controller") != std::string::npos;

    bool isMotionSensor =
        name.find("Motion Sensors") != std::string::npos;

    bool isTouchpad =
        name.find("Touchpad") != std::string::npos;

    return hasControllerName && !isMotionSensor && !isTouchpad;
}

std::optional<std::string> ControllerInput::findControllerDevice()
{
    const std::string inputDir = "/dev/input";

    for (const auto& entry : fs::directory_iterator(inputDir))
    {
        const std::string path = entry.path().string();
        const std::string filename = entry.path().filename().string();

        if (filename.rfind("event", 0) != 0)
        {
            continue;
        }

        std::string name = getDeviceName(path);

        if (name.empty())
        {
            continue;
        }

        std::cout << "Found input device: " << path << " - " << name << "\n";

        if (isMainControllerDevice(name))
        {
            return path;
        }
    }

    return std::nullopt;
}