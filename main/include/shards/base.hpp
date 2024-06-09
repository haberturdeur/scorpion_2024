#pragma once

#include "Hardware.hpp"

#include <functional>

namespace Shards {

namespace detail {

template <typename... Args>
struct RecursiveHelper {
    typedef std::function<RecursiveHelper(Args...)> Type;
    RecursiveHelper(Type f)
        : func(f) {}
    operator Type() { return func; }
    Type func;
};

} // namespace detail

using Output = HW::OutputBuilder&;
using Shard = detail::RecursiveHelper<HW::SensorData, Output>::Type;

} // namespace Shards
