#pragma once

#include "Hardware.hpp"
#include "OutputBuilder.hpp"
#include "types.hpp"
#include "shards.hpp"

class Program {
private:
    HW::Hardware& m_hardware;
    HW::OutputBuilder m_builder;

    Shards::Shard m_current;

public:
    Program(HW::Hardware& hardware, Shards::Shard start = Shards::halt, HW::OutputData initial = HW::Hardware::defaultOutput)
        : m_hardware(hardware)
        , m_builder(initial)
        , m_current(start) {
        m_hardware.apply(initial);
    }

    void tick() {
        auto sensorData = m_hardware.read();
        HW::dump(sensorData);
        m_current = m_current(sensorData, m_builder);
        m_hardware.apply(m_builder.build(sensorData.timestamp, sensorData.encoders));
    }
};
