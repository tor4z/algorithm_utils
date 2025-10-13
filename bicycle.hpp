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

    Bicycle(const State& state, float wheel_base, float max_steer, float min_steer)
        : state_(state)
        , wheel_base_(wheel_base)
        , max_steer_(max_steer)
        , min_steer_(min_steer) {}
    void act(float steer_spd, float accel, float dt);
    inline const State& state() const { return state_; }
private:
    const float wheel_base_;
    const float max_steer_;
    const float min_steer_;
    State state_;
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

#define EPSf 1.0e-6f

void Bicycle::act(float steer_spd, float accel, float dt)
{
    const auto calc_steer_angle{state_.steer_angle + steer_spd * dt};
    state_.steer_angle = max(min(calc_steer_angle, max_steer_), min_steer_);
    state_.vel += accel * dt;

    const auto vel_angle{state_.steer_angle + state_.yaw};
    const auto dy{state_.vel * std::sin(vel_angle)};
    const auto dx{state_.vel * std::cos(vel_angle)};
    state_.x += dx * dt;
    state_.y += dy * dt;
    const auto r{wheel_base_ / (std::tan(state_.steer_angle) + EPSf)};
    state_.yaw_rate = state_.vel / r;
    state_.yaw += state_.yaw_rate * dt;
}

} // namespace bicycle

#endif // BICYCLE_CPP_
#endif // BICYCLE_IMPLEMENTATION
