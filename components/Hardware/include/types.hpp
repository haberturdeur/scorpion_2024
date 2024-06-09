#pragma once

#include "RBControl_angle.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>

namespace HW {
using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
using rb::Angle;

struct SensorData {
    TimePoint timestamp = {};
    bool danger = false;

    struct Position {
        int x = 0;
        int y = 0;
        Angle heading = Angle::rad(0);
    } position;

    struct Encoders {
        std::int32_t left = 0;
        std::int32_t right;
    } encoders;

    struct Buttons {
        bool sw1 = false;
        bool sw2 = false;
        bool sw3 = false;
    } buttons;

    struct Compass {
        float heading = 0;
    } compass;
};

struct OutputData {
    struct Motors {
        int left;
        int right;
    } motors;

    struct BatteryPicker {
        Angle heading;
        Angle pitch;
        bool hold;
    } batteryPicker;

    struct Arm {
        Angle pitch;
        bool closed;
    } arm;

    struct Leds {
        bool red;
        bool yellow;
        bool green;
        bool blue;
    } leds;
};

static inline void dump(const SensorData& data) {
    std::cout << "SensorData: "
              << "timestamp=" << std::chrono::duration_cast<std::chrono::milliseconds>(data.timestamp.time_since_epoch()).count() << "ms"
              << ", danger=" << data.danger
              << ", position=(" << data.position.x << ", " << data.position.y << ", " << data.position.heading << ")"
              << ", encoders=(" << data.encoders.left << ", " << data.encoders.right << ")"
              << ", buttons=(" << data.buttons.sw1 << ", " << data.buttons.sw2 << ", " << data.buttons.sw3 << ")"
              << ", compass=(" << data.compass.heading << ")"
              << std::endl;
}

static inline void dump(const OutputData& data) {
    std::cout << "OutputData: "
              << "motors=(" << data.motors.left << ", " << data.motors.right << ")"
              << ", batteryPicker=(" << data.batteryPicker.heading << ", " << data.batteryPicker.pitch << ", " << data.batteryPicker.hold << ")"
              << ", arm=(" << data.arm.pitch << ", " << data.arm.closed << ")"
              << ", leds=(" << data.leds.red << ", " << data.leds.yellow << ", " << data.leds.green << ", " << data.leds.blue << ")"
              << std::endl;
}

} // namespace HW
