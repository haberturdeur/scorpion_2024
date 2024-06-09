#pragma once

#include "types.hpp"
#include "Hardware.hpp"
#include "Regulator.hpp"
#include <cstdint>

namespace HW {
static constexpr int mmToTicks(float mm);

class OutputBuilder {
private:
    OutputData m_output;
    const OutputData m_initial;

    Regulator<int, std::int32_t> m_left;
    Regulator<int, std::int32_t> m_right;

    OutputBuilder(const OutputBuilder&) = default;
    OutputBuilder& operator=(const OutputBuilder&) = default;

public:
    OutputBuilder(OutputData initial = {})
        : m_output(initial)
        , m_initial(initial) {}

    OutputBuilder(OutputBuilder&&) = default;
    OutputBuilder& operator=(OutputBuilder&&) = default;

    ~OutputBuilder() = default;

    OutputBuilder copy() const { return *this; }

    OutputData build(TimePoint now, SensorData::Encoders encoders) {
        m_output.motors.left = m_left.update(now, encoders.left);
        m_output.motors.right = m_right.update(now, encoders.right);
        return m_output;
    }

    void reset() {
        m_output = m_initial;
        m_left.reset();
        m_right.reset();
    }

    void motorsSpeed(int left, int right) {
        m_left.setSpeed(left);
        m_right.setSpeed(right);
    }

    void motorsPosition(std::int32_t left, std::int32_t right) {
        m_left.setPosition(left);
        m_right.setPosition(right);
    }

    void motorsPower(int left, int right) {
        m_left.setPower(left);
        m_right.setPower(right);
    }

    void motorsStop() {
        m_left.stop();
        m_right.stop();
    }

    void armClosed(bool closed) {
        m_output.arm.closed = closed;
    }

    void armPitch(Angle angle) {
        m_output.arm.pitch = angle;
    }

    void batteryPickerHeading(Angle angle) {
        m_output.batteryPicker.heading = angle;
    }

    void batteryPickerPitch(Angle angle) {
        m_output.batteryPicker.pitch = angle;
    }

    void holdBatteryPicker(bool hold) {
        m_output.batteryPicker.hold = hold;
    }

    void ledRed(bool on) {
        m_output.leds.red = on;
    }

    void ledYellow(bool on) {
        m_output.leds.yellow = on;
    }

    void ledGreen(bool on) {
        m_output.leds.green = on;
    }

    void ledBlue(bool on) {
        m_output.leds.blue = on;
    }

    void leds(bool on) {
        m_output.leds.red = on;
        m_output.leds.yellow = on;
        m_output.leds.green = on;
        m_output.leds.blue = on;
    }
};
} // namespace HW
