#pragma once

#include "RBControl_manager.hpp"
#include "RBControl_pinout.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include <array>
#include <cassert>
#include <chrono>
#include <mutex>

static inline std::int64_t getTime() {
    return esp_timer_get_time() / 1000;
}

template <typename _Output, typename _Input, typename _Float = float, typename _TimePoint = std::chrono::time_point<std::chrono::steady_clock>>
class PositionRegulator {
public:
    using Output = _Output;
    using Input = _Input;
    using Float = _Float;
    using TimePoint = _TimePoint;

private:
    Float m_P;
    Float m_I;
    Float m_D;

    Output m_integral = 0;
    Output m_integralCap = 1000;

    Float m_derivative = 0;
    Input m_lastError = 0;

    TimePoint m_lastTime;

    Output m_target;

public:
    PositionRegulator(Float P, Float I, Float D)
        : m_P(P)
        , m_I(I)
        , m_D(D)
        , m_lastTime(std::chrono::steady_clock::now()) {
    }

    void setConstants(Float P, Float I, Float D) {
        m_P = P;
        m_I = I;
        m_D = D;
    }

    void setTarget(Output target) {
        m_target = target;
    }

    Output update(TimePoint now, Input value) {
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTime).count();
        if (timeDiff == 0) {
            return 0;
        }
        m_lastTime = now;

        Input error = m_target - value;

        m_integral += error * timeDiff;
        if (m_integral > m_integralCap) {
            m_integral = m_integralCap;
        } else if (m_integral < -m_integralCap) {
            m_integral = -m_integralCap;
        }

        m_derivative = (error - m_lastError) / timeDiff;
        m_lastError = error;

        Float p = m_P * error;
        Float i = m_I * m_integral;
        Float d = m_D * m_derivative;

        std::cout << "Value: " << value << " Target: " << m_target << " Error: " << error << " P: " << p << " I: " << i << " D: " << d << "\n";

        return p + i + d;
    }

    void reset() {
        m_integral = 0;
        m_derivative = 0;
        m_lastError = 0;
        m_lastTime = std::chrono::steady_clock::now();
        m_target = 0;
    }

    TimePoint lastUpdate() const {
        return m_lastTime;
    }

    Output target() const {
        return m_target;
    }

    void setIntegralCap(Output cap) {
        m_integralCap = cap;
    }
};

template <typename _Output, typename _Input, typename _Float = float, typename _TimePoint = std::chrono::time_point<std::chrono::steady_clock>>
class SpeedRegulator {
public:
    using Output = _Output;
    using Input = _Input;
    using Float = _Float;
    using TimePoint = _TimePoint;

private:
    enum class State {
        Stopped,
        Accelerating,
        Decelerating,
        Holding,
    };

    PositionRegulator<Output, Input, Float, TimePoint> m_regulator;

    Float m_acceleration = 2;

    Float m_targetSpeed = 0;
    Float m_realSpeed = 0;
    Input m_lastPosition = 0;
    State m_state = State::Stopped;

    auto sinceLastUpdate(TimePoint now) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_regulator.lastUpdate()).count();
    }

    Float calculateSpeed(TimePoint now, Input position) {
        Float speed = (position - m_lastPosition) / sinceLastUpdate(now);
        m_lastPosition = position;
        return speed;
    }

public:
    SpeedRegulator(Float P, Float I, Float D, Float acceleration)
        : m_regulator(P, I, D)
        , m_acceleration(acceleration) {
    }

    void setConstants(Float P, Float I, Float D) {
        m_regulator.setConstants(P, I, D);
    }

    void setAcceleration(Float acceleration) {
        m_acceleration = acceleration;
    }

    void setTarget(Float target) {
        if (target > m_targetSpeed) {
            m_state = State::Accelerating;
        } else if (target < m_targetSpeed) {
            m_state = State::Decelerating;
        }
        m_targetSpeed = target;
    }

    Output update(TimePoint now, Input position) {
        switch (m_state) {
        case State::Stopped:
            break;
        case State::Holding:
            m_regulator.setTarget(m_regulator.target() + m_realSpeed);
            break;
        case State::Accelerating:
            m_realSpeed += m_acceleration * sinceLastUpdate(now);
            if (m_realSpeed >= m_targetSpeed) {
                m_state = State::Holding;
            }
            m_regulator.setTarget(m_regulator.target() + m_realSpeed);
            break;

        case State::Decelerating:
            m_realSpeed -= m_acceleration * sinceLastUpdate(now);
            if (m_realSpeed <= m_targetSpeed) {
                m_state = State::Holding;
            }
            m_regulator.setTarget(m_regulator.target() + m_realSpeed);
            break;
        }
        return m_regulator.update(now, position);
    }

    void reset() {
        m_regulator.reset();
        m_targetSpeed = 0;
        m_realSpeed = 0;
        m_lastPosition = 0;
        m_state = State::Stopped;
    }
};

template <typename _Output, typename _Input, typename _Float = float, typename _TimePoint = std::chrono::time_point<std::chrono::steady_clock>>
class Regulator {
public:
    using Output = _Output;
    using Input = _Input;
    using Float = _Float;
    using TimePoint = _TimePoint;
    using PosReg = PositionRegulator<Output, Input, Float, TimePoint>;
    using SpeedReg = SpeedRegulator<Output, Input, Float, TimePoint>;

private:
    enum class Mode {
        Power,
        Position,
        Speed,
    };

    Mode m_mode = Mode::Power;

    PosReg m_posReg { 0.5, 0.001, 0.1 };
    SpeedReg m_speedReg { 0.5, 0.001, 0.1, 0.1 };
    int m_power = 0;
    Input m_lastPosition = 0;

public:
    void setSpeed(float speed) {
        if (m_mode != Mode::Speed) {
            m_mode = Mode::Speed;
            m_speedReg.reset();
        }
        m_speedReg.setTarget(speed);
    }

    void setPosition(int position) {
        if (m_mode != Mode::Position) {
            m_mode = Mode::Position;
            m_posReg.reset();
        }
        m_posReg.setTarget(position);
    }

    void movePosition(std::int32_t position) {
        if (m_mode != Mode::Position) {
            m_mode = Mode::Position;
            m_posReg.reset();
        }
        m_posReg.setTarget(m_lastPosition + position);
    }

    void setPower(int power) {
        m_mode = Mode::Power;
        m_power = power;
    }

    void stop() {
        m_mode = Mode::Power;
        m_power = 0;
    }

    Output update(TimePoint now, Input pos) {
        m_lastPosition = pos;
        switch (m_mode) {
            case Mode::Power:
                return m_power;
            
            case Mode::Position:
                return m_posReg.update(now, pos);

            case Mode::Speed:
                return m_speedReg.update(now, pos);

        };

        return 0;
    }

    void reset() {
        m_posReg.reset();
        m_speedReg.reset();
        m_power = 0;
        m_lastPosition = 0;
    }
};