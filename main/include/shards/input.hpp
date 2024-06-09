#pragma once

#include "shards/base.hpp"
#include "shards/utils.hpp"

namespace Shards {

struct AwaitButton {
    int button;
    Shard next = halt;

    Shard operator()(HW::SensorData sensorData, Output builder) {
        bool btn;
        switch (button) {
            case 1:
                btn = sensorData.buttons.sw1;
                break;
            case 2:
                btn = sensorData.buttons.sw2;
                break;
            case 3:
                btn = sensorData.buttons.sw3;
                break;
            default:
                btn = false;
        }

        if (btn) {
            return next;
        } else {
            return *this;
        }
    }
};

} // namespace Shards
