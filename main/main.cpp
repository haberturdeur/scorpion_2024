#include "OutputBuilder.hpp"
#include "Program.hpp"

#include "Hardware.hpp"
#include "shards.hpp"
#include "shards/utils.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

static constexpr rb::MotorId left = rb::MotorId::M6;
static constexpr rb::MotorId right = rb::MotorId::M4;

using rb::operator""_deg;

Shards::Shard mainProgram = Shards::AwaitButton{1, Shards::MoveForward{500, Shards::Rotate{90_deg, Shards::halt}}};

extern "C" void app_main(void) {
    auto& man = rb::Manager::get();
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE | rb::ManagerInstallFlags::MAN_DISABLE_BATTERY_MANAGEMENT);
    auto& servoBus = man.initSmartServoBus(4);

    HW::Hardware hardware(man);


    // hardware.applyMotors({100, 100});

    // Regulator<int, float> m_left;
    // Regulator<int, float> m_right;

    // m_left.setSpeed(100);
    // m_right.setSpeed(100);

    // HW::OutputBuilder builder(HW::Hardware::defaultOutput);
    // builder.motorsSpeed(50, 50);

    // while (true) {
    //     auto sensors = hardware.read();
    //     HW::dump(sensors);

    //     hardware.apply(builder.build(sensors.timestamp, sensors.encoders));
    //     std::this_thread::sleep_for(10ms);
    // }

    Program program(hardware, mainProgram);

    while (true) {
        // std::cout << "Tick" << std::endl;
        program.tick();
        std::this_thread::sleep_for(10ms);
    }
}

// Shards::Shard mainProgram;