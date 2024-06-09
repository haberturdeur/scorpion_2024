#include "./I2CRegs.hpp"

#include "./I2C.hpp"

#include "driver/gpio.h"
#include "esp_err.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>
#include <iostream>

template <typename T>
using Vector3 = std::array<T, 3>;

template <typename Ta, typename Tb, typename Tc = decltype(Ta() * Tb())>
Vector3<Tc> crossProduct(const Vector3<Ta>& a, const Vector3<Tb>& b) {
    auto [ax, ay, az] = a;
    auto [bx, by, bz] = b;
    return {
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx
    };
}

template <typename Ta, typename Tb, typename Tc = decltype(Ta() * Tb())>
Tc dotProduct(const Vector3<Ta>& a, const Vector3<Tb>& b) {
    auto [ax, ay, az] = a;
    auto [bx, by, bz] = b;
    return ax * bx + ay * by + az * bz;
}

template <typename In>
Vector3<float> normalize(const Vector3<In>& a) {
    auto [x, y, z] = a;
    float length = std::sqrt(x * x + y * y + z * z);
    return { x / length, y / length, z / length };
}

class Compass {
public:
    struct Pins {
        gpio_num_t int1;
        gpio_num_t int2;
        gpio_num_t drdy;
    };

    static constexpr std::uint8_t address = 0x1e;

private:
    I2C::Device m_device;

    std::optional<Pins> m_pins;

    bool m_calibrationRunning = false;

    Vector3<std::int16_t> m_min = {
        std::numeric_limits<std::int16_t>::max(),
        std::numeric_limits<std::int16_t>::max(),
        std::numeric_limits<std::int16_t>::max()
    };

    Vector3<std::int16_t> m_max = {
        std::numeric_limits<std::int16_t>::min(),
        std::numeric_limits<std::int16_t>::min(),
        std::numeric_limits<std::int16_t>::min()
    };

    Vector3<std::int16_t> m_offset = { 0, 0, 0 };
    Vector3<float> m_scale = { 1.f, 1.f, 1.f };

    void initPins() {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << m_pins.value().int1) | (1ULL << m_pins.value().int2) | (1ULL << m_pins.value().drdy),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    void deinitPins() {
        gpio_reset_pin(m_pins.value().int1);
        gpio_reset_pin(m_pins.value().int2);
        gpio_reset_pin(m_pins.value().drdy);
    }

public:
    Compass(I2C::BusHandle bus, std::optional<Pins> pins = std::nullopt)
        : m_device(bus, address)
        , m_pins(pins) {

        if (m_pins.has_value())
            initPins();
    }

    ~Compass() {
        if (m_pins.has_value())
            deinitPins();
    }

    I2C::Device& device() {
        return m_device;
    }

    enum class OpMode {
        Continuous = 0b00,
        Single = 0b01,
        Sleep = 0b11,
    };

    void setOperationMode(OpMode mode) {
        m_device.write(static_cast<std::uint8_t>(MagRegs::MR_REG_M), static_cast<std::uint8_t>(mode));
    }

    Vector3<std::int16_t> readRaw() {
        auto data = m_device.read(static_cast<std::uint8_t>(MagRegs::OUT_X_H_M), 6);

        std::int16_t x = (data[0] << 8) | data[1];
        std::int16_t z = (data[2] << 8) | data[3];
        std::int16_t y = (data[4] << 8) | data[5];

        if (m_calibrationRunning) {
            m_min[0] = std::min(m_min[0], x);
            m_min[1] = std::min(m_min[1], y);
            m_min[2] = std::min(m_min[2], z);

            m_max[0] = std::max(m_max[0], x);
            m_max[1] = std::max(m_max[1], y);
            m_max[2] = std::max(m_max[2], z);
        }

        return { x, y, z };
    }

    Vector3<float> read() {
        auto raw = readRaw();
        Vector3<float> out;

        for (int i = 0; i < 3; ++i)
            out[i] = (raw[i] - m_offset[i]) * m_scale[i];

        return out;
    }

    // Calculate heading in degrees from magnetometer data around the Z axis
    float readHeading() {
        auto data = read();

        float heading = std::atan2(data[1], data[0]) * 180 / M_PI;
        if (heading < 0)
            heading += 360;

        return heading;
    }

    void startCalibration() {
        m_calibrationRunning = true;

        for (int i = 0; i < 3; ++i)
            m_min[i] = std::numeric_limits<std::int16_t>::max();

        for (int i = 0; i < 3; ++i)
            m_max[i] = std::numeric_limits<std::int16_t>::min();
    }

    void dumpCalibration() {
        std::cout << "Min: " << m_min[0] << ", " << m_min[1] << ", " << m_min[2] << std::endl;
        std::cout << "Max: " << m_max[0] << ", " << m_max[1] << ", " << m_max[2] << std::endl;
        std::cout << "Offset: " << m_offset[0] << ", " << m_offset[1] << ", " << m_offset[2] << std::endl;
        std::cout << "Scale: " << m_scale[0] << ", " << m_scale[1] << ", " << m_scale[2] << std::endl;
    }

    void stopCalibration() {
        m_calibrationRunning = false;

        setCalibrationData(m_min, m_max);
    }

    void setCalibrationData(Vector3<std::int16_t> min, Vector3<std::int16_t> max) {
        for (int i = 0; i < 3; ++i)
            m_offset[i] = static_cast<std::int16_t>((max[i] + min[i]) / 2);

        float avg_delta = 0;

        for (int i = 0; i < 3; ++i)
            avg_delta += max[i] - min[i];

        avg_delta /= 3;

        for (int i = 0; i < 3; ++i)
            m_scale[i] = avg_delta / (max[i] - min[i]);
    }
};
