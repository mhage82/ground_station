#pragma once

#include <string>

struct RawAxes
{
    int absX = 128;
    int absY = 128;
    int absRx = 128;
    int absRy = 128;
    int absZ = 0;
    int absRz = 0;
    int hat0X = 0;
    int hat0Y = 0;
};

struct Buttons
{
    int x = 0;
    int circle = 0;
    int triangle = 0;
    int square = 0;

    int l1 = 0;
    int r1 = 0;

    int l2Button = 0;
    int r2Button = 0;

    int wireless = 0;
    int options = 0;
    int home = 0;
};

struct Command
{
    double moveLeftRight = 0.0;
    double moveForwardBack = 0.0;
    double yaw = 0.0;
    double altitude = 0.0;

    double l2 = 0.0;
    double r2 = 0.0;

    int dpadLeftRight = 0;
    int dpadUpDown = 0;
};

struct ControllerState
{
    std::string mode = "simple";
    RawAxes rawAxes;
    Buttons buttons;
    Command command;
};
