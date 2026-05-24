#include <linux/input.h>

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

#include "ControllerInput.hpp"
#include "ControllerMapper.hpp"
#include "FlightCommandMapper.hpp"
#include "SharedFlightCommand.hpp"
#include "FlightCommandPublisher.hpp"

namespace
{
    std::atomic<bool> gRunning(true);

    void signalHandler(int)
    {
        gRunning.store(false);
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "Searching for controller...\n";

    ControllerInput controllerInput;

    if (!controllerInput.openController())
    {
        return 1;
    }

    ControllerMapper controllerMapper;
    FlightCommandMapper flightCommandMapper;
    SharedFlightCommand sharedFlightCommand;

    FlightCommand initialCommand =
        flightCommandMapper.map(controllerMapper.getState());

    sharedFlightCommand.set(initialCommand);

    FlightCommandPublisher publisher(sharedFlightCommand);

    if (!publisher.open())
    {
        std::cerr << "Failed to open MAVLink publisher.\n";
        return 1;
    }

    std::thread publisherThread(
        [&publisher]()
        {
            publisher.run(gRunning, 20);
        }
    );

    std::cout << "Reading controller events. Press Ctrl+C to stop.\n\n";

    while (gRunning.load())
    {
        input_event event {};

        if (controllerInput.readEvent(event))
        {
            bool stateChanged = controllerMapper.handleEvent(event);

            if (stateChanged)
            {
                const ControllerState& controllerState =
                    controllerMapper.getState();

                FlightCommand flightCommand =
                    flightCommandMapper.map(controllerState);

                sharedFlightCommand.set(flightCommand);
            }
        }
    }

    if (publisherThread.joinable())
    {
        publisherThread.join();
    }

    std::cout << "Stopped.\n";

    return 0;
}