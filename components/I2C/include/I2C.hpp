#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>
#include <optional>

namespace I2C {
using BusHandle = std::shared_ptr<i2c_master_bus_t>;

BusHandle initBus(i2c_port_num_t port, gpio_num_t scl, gpio_num_t sda);

void scanDevices(BusHandle& bus);

std::optional<BusHandle> getBusHandle(i2c_port_num_t port);

class Device {
private:
    i2c_master_dev_handle_t m_dev_handle = nullptr;

    BusHandle m_busHandle;

public:
    Device(BusHandle bus, std::uint8_t addr, std::uint32_t speed = 100000);
    ~Device();

    std::vector<std::uint8_t> read(std::uint8_t reg, std::size_t len);
    std::uint8_t read(std::uint8_t reg);

    void write(std::uint8_t reg, std::span<const std::uint8_t> data);
    void write(std::uint8_t reg, std::uint8_t data);
};

} // namespace I2C
