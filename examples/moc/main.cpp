#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>
#include <chrono>

#define MOC_IMPLEMENTATION
#include "games/moc.hpp"


const float car_accel{2.0f};
const moc::Action actions[]{moc::ACT_DEACCEL, moc::ACT_ACCEL, moc::ACT_MAINTAIN};

inline const std::string& stringify(moc::Action act)
{
    static const std::string invalid{"invalid"};
    static const std::string accel{"accel"};
    static const std::string deaccel{"deaccel"};
    static const std::string maintain{"maintain"};

    switch (act) {
    case moc::ACT_ACCEL:    return accel;
    case moc::ACT_DEACCEL:  return deaccel;
    case moc::ACT_MAINTAIN: return maintain;
    default:                return invalid;
    }
}

struct Profiler
{
    Profiler() { reset(); }

    void tic()
    {
        const auto now{std::chrono::steady_clock::now()};
        tic_ = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    }

    void toc()
    {
        const auto now{std::chrono::steady_clock::now()};
        const auto time_cost{std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch() - tic_)};
        const auto time_cost_micro{static_cast<unsigned int>(time_cost.count())};
        max_ = std::max(max_, time_cost_micro);
        min_ = std::min(min_, time_cost_micro);
        avg_ = double(avg_ * n_) / double(n_ + 1) + (double)time_cost_micro / double(n_ + 1);
        ++n_;
    }

    void report(int n)
    {
        auto to_ms{[](unsigned int micro) -> unsigned int { return micro / 1000; }};

        if (n_ % n == 0) {
            std::cout << "[Profiler] min: " << to_ms(min_)
                      << "ms, max: " << to_ms(max_)
                      << "ms, avg: " << to_ms(avg_) << "ms\n"; 
            reset();
        }
    }
private:
    std::chrono::microseconds tic_;
    unsigned int max_;
    unsigned int min_;
    unsigned int avg_;
    int n_;

    void reset()
    {
        max_ = 0;
        min_ = 0xffffffff;
        avg_ = 0;
        n_ = 0;
    }
}; // struct Profiler

struct POMDPSolver
{
    explicit POMDPSolver(float dt) : root_({}), dt_(dt) {}

    struct StateNode;

    struct ActionNode
    {
        std::vector<StateNode> children;
        moc::Action act;
        float value;
    }; // struct ActionNode

    struct StateNode
    {
        std::vector<moc::Obstacle> obsts;
        std::vector<ActionNode> children;
        moc::Vec2f ego_position;
        moc::Vec2f ego_coll_shape;
        float ego_heading;
        float ego_speed;
        int depth;
    }; // struct StateNode

    moc::Action solve(const StateNode& state);
private:
    struct ActionSimResult
    {
        StateNode state;
        float reward;
    };

    StateNode root_;
    float dt_;

    float simulate(StateNode& state, moc::Action act);
    ActionSimResult simulate_action(const StateNode& state, moc::Action act);
}; // struct POMDPSolver

moc::Action POMDPSolver::solve(const POMDPSolver::StateNode& state)
{
    static Profiler profiler{};

    root_ = state;
    root_.depth = 0;
    profiler.tic();
    for (auto act: actions) {
        simulate(root_, act);
    }
    profiler.toc();
    profiler.report(10);

    {
        // take best action
        auto best_value{-std::numeric_limits<float>::infinity()};
        moc::Action result{moc::ACT_MAINTAIN};
        for (const auto& an: root_.children) {
            if (an.value > best_value) {
                best_value = an.value;
                result = an.act;
            }
            std::cout << stringify(an.act) << ": " << an.value << "\n";
        }
        std::cout << "best: " << stringify(result) << "\n";
        std::cout << "====\n";
        return result;
    }
}

float POMDPSolver::simulate(StateNode& state, moc::Action act)
{
    constexpr auto gamma{0.003f};
    if (state.depth > 5) return 0.0f;

    ActionNode an{.act=act, .value=0};
    {
        const auto sa{simulate_action(state, act)};
        an.children.push_back(sa.state);
        auto& new_state{an.children.back()};
        for (auto act: actions) {
            an.value += sa.reward * gamma + simulate(new_state, act);
        }
    }

    state.children.push_back(an);
    return an.value;
}

POMDPSolver::ActionSimResult POMDPSolver::simulate_action(const StateNode& state, moc::Action act)
{
    ActionSimResult asr{};
    float accel{};

    switch (act) {
    case moc::ACT_ACCEL:
        asr.reward = 2.0f;
        accel = car_accel;
        break;
    case moc::ACT_DEACCEL:
        asr.reward = 0.0f;
        accel = -car_accel;
        break;
    case moc::ACT_MAINTAIN:
        asr.reward = 1.0f;
        accel = 0;
        break;
    }

    asr.state.ego_heading = state.ego_heading;
    asr.state.ego_speed = state.ego_speed + dt_ * accel;
    const auto mov_dist{asr.state.ego_speed * dt_};
    // std::cout << "mov_dist: " << mov_dist << ", " << accel << "\n";
    asr.state.ego_position.x = state.ego_position.x + mov_dist * std::cos(asr.state.ego_heading);
    asr.state.ego_position.y = state.ego_position.y + mov_dist * std::sin(asr.state.ego_heading);
    asr.state.ego_coll_shape = state.ego_coll_shape;
    asr.state.depth = state.depth + 1;
    asr.state.obsts = state.obsts;

    const moc::Obb ego_obb{.center  = asr.state.ego_position,
                           .shape   = asr.state.ego_coll_shape,
                           .heading = asr.state.ego_heading};

    if (accel >= 0) {
        for (const auto& obst: asr.state.obsts) {
            if (moc::point_in_obb(obst.center, ego_obb)) {
                asr.reward -= 10.0f;
                break;
            }
        }
    }

    return asr;
}

int main()
{
    auto settings{moc::EnvSettings::from_default()};
    settings.num_persons = 500;
    settings.car_accel = car_accel;
    auto env{moc::Env(settings)};
    bool terminated{false};

    POMDPSolver solver{0.2f};
    POMDPSolver::StateNode state{};
    bool first_step{true};
    while (!terminated) {
        const auto act{first_step ? moc::ACT_ACCEL : solver.solve(state)};
        const auto sr{env.step(act)};
        state.children.clear();
        state.ego_coll_shape = sr.state.coll_shape;
        state.ego_position = sr.state.position;
        state.ego_heading = sr.state.heading;
        state.ego_speed = sr.state.speed;
        state.obsts = sr.obs.obstacles;
        if (sr.terminated) terminated = true;
        first_step = false;
    }

    return 0;
}
