cmake_minimum_required(VERSION 3.16)

project(quad_ground_station LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(quad_ground_station
    src/main.cpp
    src/ControllerInput.cpp
    src/ControllerMapper.cpp
    src/CommandPrinter.cpp
    src/FlightCommandMapper.cpp
    src/SharedFlightCommand.cpp
    src/FlightCommandPublisher.cpp
    src/FlightCommandToRcMapper.cpp
    src/MavlinkTcpSender.cpp
)

target_include_directories(quad_ground_station PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    $ENV{HOME}/workspace/ardupilot/build/sitl/libraries/GCS_MAVLink/include

)