#pragma once

#include "Hardware.hpp"
#include "math.hpp"
#include "shards/base.hpp"
#include "types.hpp"

#include <iostream>
#include <optional>

namespace Shards {

Shard halt(HW::SensorData, Output builder) {
    ESP_LOGI("Shards", "Halt");
    builder.motorsStop();
    builder.leds(false);
    builder.ledGreen(true);
    return halt;
}

Shard error(HW::SensorData, Output builder) {
    ESP_LOGI("Shards", "Error");
    builder.motorsStop();
    builder.leds(false);
    builder.ledRed(true);
    return error;
}

struct MoveForward {
    int distance;
    Shard next;
    std::optional<Point> target = std::nullopt;

    Shard operator()(HW::SensorData sensorData, Output builder) {
        builder.leds(false);
        builder.ledYellow(true);
        if (!target.has_value()) {
            std::cout << "MoveForward: target not set" << std::endl;
            target = {
                sensorData.position.x + HW::mmToTicks(distance) * cos(sensorData.position.heading),
                sensorData.position.y + HW::mmToTicks(distance) * sin(sensorData.position.heading),
            };
        }
        float error = distance2(sensorData.position, target.value());

        Point projection = {
            sensorData.position.x + 1 * cos(sensorData.position.heading),
            sensorData.position.y + 1 * sin(sensorData.position.heading),
        };

        float error2 = distance2(projection, target.value());
        if (error2 >= error)
            error = -error;

        if (error > 0) {
            builder.motorsPosition(sensorData.encoders.left + error, sensorData.encoders.right + error);
            // builder.motorsSpeed(50, 50);
            return *this;
        } else {
            builder.motorsStop();
            return next;
        }
    }
};

struct Rotate {
    rb::Angle angle;
    Shard next;

    Shard operator()(HW::SensorData sensorData, Output builder) {
        builder.leds(false);
        builder.ledBlue(true);
        rb::Angle error = angle - sensorData.position.heading;
        if (error.deg() < 1 && error.deg() < -1) {
            builder.motorsStop();
            return next;
        }

        builder.motorsSpeed(error.deg(), -error.deg());
        return *this;
    }
};

} // namespace Shards
