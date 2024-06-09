#pragma once

#include "OutputBuilder.hpp"

#include "types.hpp"

#include "Compass.hpp"
#include "RBControl_angle.hpp"
#include "RBControl_manager.hpp"
#include "RBControl_pinout.hpp"
#include "Regulator.hpp"

#include "iot_servo.h"

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <chrono>
#include <cstdint>

namespace HW {

namespace detail {

void initHoldServo(gpio_num_t pin) {
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                pin,
            },
            .ch = {
                LEDC_CHANNEL_1,
            },
        },
        .channel_number = 1,
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

void setHoldServo(float angle) {
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, angle);
}

} // namespace detail

static constexpr int wheelRadius = 34;
static constexpr int wheelDistance = 225;
static constexpr int encoderTicksPerRevolution = 2000;

static constexpr int mmToTicks(float mm) {
    return mm * encoderTicksPerRevolution / (2 * M_PI * wheelRadius);
}

static constexpr float ticksToMm(std::int32_t ticks) {
    return ticks * 2.f * M_PI * wheelRadius / encoderTicksPerRevolution;
}

static constexpr int angleToTicks(float degrees) {
    return degrees * wheelDistance / (2 * wheelRadius);
}

class Hardware {
public:
    static constexpr rb::MotorId leftMotor = rb::MotorId::M6;
    static constexpr rb::MotorId rightMotor = rb::MotorId::M4;

    static constexpr int batteryPickerHeadingServo = 0;
    static constexpr int batteryPickerPitchServo = 1;

    static constexpr int armServo = 2;
    static constexpr int handServo = 3;

    static constexpr gpio_num_t holdServoPin = rb::ENC5B;

    static constexpr OutputData defaultOutput = {
        .motors = { 0, 0 },
        .batteryPicker = { Angle::deg(120), Angle::deg(80), true },
        .arm = { Angle::deg(80), false },
        .leds = { false, false, false, false },
    };

private:
    rb::Manager& m_man;
    Compass m_compass;

    SensorData::Position m_lastPosition {};
    SensorData::Encoders m_lastEncoders {};

    SensorData::Position updatePosition(SensorData::Encoders newEncoders) {
        using namespace lx16a;
        Angle heading = m_lastPosition.heading;

        float leftDelta = ticksToMm(newEncoders.left - m_lastEncoders.left);
        float rightDelta = ticksToMm(newEncoders.right - m_lastEncoders.right);

        Angle headingChange = Angle::rad((rightDelta - leftDelta) / (wheelDistance));
        float distance = (leftDelta + rightDelta);

        SensorData::Position out;

        heading += headingChange;

        int x = distance * cos(heading);
        int y = distance * sin(heading);

        out = {
            .x = m_lastPosition.x + x,
            .y = m_lastPosition.y + y,
            .heading = heading,
        };

        m_lastPosition = out;
        m_lastEncoders = newEncoders;

        return out;
    }

public:
    Hardware(rb::Manager& man)
        : m_man(man)
        , m_compass(I2C::getBusHandle(I2C_NUM_0).value()) {
        m_compass.setOperationMode(Compass::OpMode::Continuous);

        using rb::operator""_deg;

        auto& bus = m_man.servoBus();
        bus.limit(batteryPickerHeadingServo, 25_deg, 200_deg);
        bus.limit(batteryPickerPitchServo, 0_deg, 80_deg);
        bus.limit(armServo, 0_deg, 80_deg);
        bus.limit(handServo, 0_deg, 33_deg);

        // Init stupid servo
        detail::initHoldServo(holdServoPin);

        reset();
    }

    void reset() {
        apply(defaultOutput);
    }

    void applyMotors(const OutputData::Motors& output) {
        m_man.setMotors()
            .power(leftMotor, std::clamp(output.left, -100, 100))
            .power(rightMotor, -std::clamp(output.left, -100, 100))
            .set();
    }

    void applyBatteryPicker(const OutputData::BatteryPicker& output) {
        m_man.servoBus().set(batteryPickerHeadingServo, output.heading);
        m_man.servoBus().set(batteryPickerPitchServo, output.pitch);
        detail::setHoldServo(output.hold ? 163 : 70);
    }

    void applyArm(const OutputData::Arm& output) {
        auto& bus = m_man.servoBus();

        bus.set(armServo, output.pitch);
        bus.set(handServo, output.closed ? rb::Angle::deg(0) : rb::Angle::deg(33));
    }

    void applyLeds(const OutputData::Leds& output) {
        auto& leds = m_man.leds();
        leds.red(output.red);
        leds.yellow(output.yellow);
        leds.green(output.green);
        leds.blue(output.blue);
    }

    void apply(const OutputData& output) {
        // dump(output);
        applyMotors(output.motors);
        // applyBatteryPicker(output.batteryPicker);
        // applyArm(output.arm);
        applyLeds(output.leds);
    }

    SensorData read() {
        auto now = std::chrono::steady_clock::now();
        SensorData::Encoders encoders {
            .left = m_man.motor(leftMotor).encoder()->value(),
            .right = -m_man.motor(rightMotor).encoder()->value(),
        };

        return SensorData {
            .timestamp = now,
            .position = updatePosition(encoders),
            .encoders = encoders,
            .buttons = {
                .sw1 = m_man.expander().digitalRead(rb::EB0) == 0,
                .sw2 = m_man.expander().digitalRead(rb::EB1) == 0,
                .sw3 = m_man.expander().digitalRead(rb::EB2) == 0,
            },
            .compass = {
                .heading = m_compass.readHeading(),
            },
        };
    }
};

} // namespace HW