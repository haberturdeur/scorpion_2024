#include "I2C.hpp"

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>
#include <optional>

namespace I2C {

static std::array<std::optional<BusHandle>, I2C_NUM_MAX> g_busHandles = {};

BusHandle initBus(i2c_port_num_t port, gpio_num_t scl, gpio_num_t sda) {
    assert(port < I2C_NUM_MAX);
    assert(!g_busHandles[port].has_value());

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    g_busHandles[port] = std::shared_ptr<i2c_master_bus_t>(bus_handle, i2c_del_master_bus);
    return g_busHandles[port].value();
}

void scanDevices(BusHandle& bus) {
    for (std::uint8_t addr = 0; addr < 128; ++addr) {
        if (i2c_master_probe(bus.get(), addr, 50) == ESP_OK)
            printf("Found device at address 0x%02x\n", addr);
    }
}

std::optional<BusHandle> getBusHandle(i2c_port_num_t port) {
    if (port >= I2C_NUM_MAX)
        return std::nullopt;

    return g_busHandles[port];
}

Device::Device(BusHandle bus, std::uint8_t addr, std::uint32_t speed)
    : m_busHandle(bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = speed,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(m_busHandle.get(), &dev_cfg, &m_dev_handle));
}

Device::~Device() {
    if (m_dev_handle)
        i2c_master_bus_rm_device(m_dev_handle);
}

std::vector<std::uint8_t> Device::read(std::uint8_t reg, std::size_t len) {
    std::vector<std::uint8_t> data(len);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(m_dev_handle, &reg, 1, data.data(), len, -1));
    return data;
}

std::uint8_t Device::read(std::uint8_t reg) {
    std::uint8_t data;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(m_dev_handle, &reg, 1, &data, 1, -1));
    return data;
}

void Device::write(std::uint8_t reg, std::span<const std::uint8_t> data) {
    std::vector<std::uint8_t> buf(data.size() + 1);
    buf[0] = reg;
    std::copy(data.begin(), data.end(), buf.begin() + 1);
    ESP_ERROR_CHECK(i2c_master_transmit(m_dev_handle, buf.data(), buf.size(), -1));
}

void Device::write(std::uint8_t reg, std::uint8_t data) {
    std::uint8_t buf[2] = { reg, data };
    ESP_ERROR_CHECK(i2c_master_transmit(m_dev_handle, buf, 2, -1));
}

} // namespace I2C
