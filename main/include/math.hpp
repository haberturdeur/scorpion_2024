#pragma once

#include "angle.hpp"

#include <cmath>

struct Point {
    float x = 0.f;
    float y = 0.f;
};

auto distance2(auto a, auto b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

auto movePoint(auto point, lx16a::Angle angle, auto distance) {
    return decltype(point) {
        point.x + std::cos(angle.rad()) * distance,
        point.y + std::sin(angle.rad()) * distance
    };
}
