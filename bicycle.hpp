#ifndef BICYCLE_HPP_
#define BICYCLE_HPP_

namespace bicycle {

class Bicycle
{
public:
    struct State
    {
        float x;
        float y;
        float vel;
        float yaw;
        float yaw_rate;
        float steer_angle;
    }; // struct State

    struct Config
    {
        float wheel_base;
        float gc_to_back_axle;  // gc: gravity center
        float max_steer;
        float min_steer;
    }; // struct Config

    Bicycle(const State& state, const Config& cfg)
        : state_(state)
        , cfg_(cfg) {}
    void act(float steer_spd, float accel, float dt);
    inline const State& state() const { return state_; }
private:
    State state_;
    const Config cfg_;
}; // class Bicycle

} // namespace bicycle

#endif // BICYCLE_HPP_


// #define BICYCLE_IMPLEMENTATION  // delete me


#ifdef BICYCLE_IMPLEMENTATION
#ifndef BICYCLE_CPP_
#define BICYCLE_CPP_

#include <cmath>

namespace bicycle {

template<typename T>
T max(T a, T b) { return a > b ? a : b; }

template<typename T>
T min(T a, T b) { return a < b ? a : b; }

#define BICYCLE_EPSf 1.0e-6f

void Bicycle::act(float steer_spd, float accel, float dt)
{
    const auto calc_steer_angle{state_.steer_angle + steer_spd * dt};
    state_.steer_angle = max(min(calc_steer_angle, cfg_.max_steer), cfg_.min_steer);
    state_.vel += accel * dt;

    const auto slip_angle{std::atan2(cfg_.gc_to_back_axle * std::tan(state_.steer_angle), cfg_.wheel_base)};
    const auto vel_angle{slip_angle + state_.yaw};
    const auto dy{state_.vel * std::sin(vel_angle)};
    const auto dx{state_.vel * std::cos(vel_angle)};
    const auto r{cfg_.wheel_base / (std::tan(state_.steer_angle) * std::cos(slip_angle) + BICYCLE_EPSf)};
    state_.x += dx * dt;
    state_.y += dy * dt;
    state_.yaw_rate = state_.vel / r;
    state_.yaw += state_.yaw_rate * dt;
}

} // namespace bicycle

#endif // BICYCLE_CPP_
#endif // BICYCLE_IMPLEMENTATION
