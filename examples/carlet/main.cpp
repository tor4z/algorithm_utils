#include <cstdlib>
#include <ctime>
#include <iostream>

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


constexpr double target_spd{carlet::kmph_to_mps(120.0)};


void plan(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    carlet::Veh* lead{nullptr};
    for (const auto veh: obs.vehs) {
        if (!veh) continue;
        if (veh->state().x < state.x) continue;
        if (veh->state().x - state.x > 60.0f) continue;
        if (carlet::abs(veh->state().y - state.y) > 2.0f) continue;
        if (carlet::abs(veh->state().x - state.x) < 2.0f) continue; // ego
        if (lead && veh->state().x > lead->state().x) continue;
        lead = veh;
    }

    const auto cruise_error{target_spd - state.vel};
    if (!lead) {
        ctrl.accel = cruise_error * 0.1f;
    } else {
        const auto target_ht{2.5};
        const auto x_diff{lead->state().x - state.x};
        const auto ht{x_diff / state.vel};
        const auto desire_x{target_ht * state.vel};
        const auto dist_error{x_diff - desire_x};
        const auto vel_error{lead->state().vel - state.vel};
        ctrl.accel = dist_error * 0.2 + vel_error * 0.4;
    }

    ctrl.accel = carlet::min(ctrl.accel, 2.0f);
    ctrl.steer = 0.0f;
}

float calc_stop_dist(float from_v, float to_v, float max_accel)
{
    const auto min_t{(to_v - from_v) / max_accel};
    std::cout << "t: " << min_t << "\n";
    return from_v * min_t + 0.5f * max_accel * carlet::pow2(min_t) - min_t * to_v;
}

int main()
{
    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=5000.0f, .y=0.0f, .z=0.0f},
        3, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla);
    sim->gen_random_vehs(150,
        carlet::kmph_to_mps(40.0),
        carlet::kmph_to_mps(120.0));

    carlet::Veh* v{};
    carlet::Veh::Obs full_obs{sim->full_obs()};
    carlet::Veh::Control ctrl{};

    while (sim->is_running()) {
        if ((v = sim->get_ctrl_veh()) != nullptr) {
            plan(full_obs, v->state(), ctrl);
            v->act(ctrl);
        }
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}
