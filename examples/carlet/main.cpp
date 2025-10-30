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
        if (veh->state().x - state.x > 100.0f) continue;
        if (carlet::abs(veh->state().y - state.y) > 2.0f) continue;
        if (carlet::abs(veh->state().x - state.x) < 2.0f) continue; // ego
        if (lead && veh->state().x > lead->state().x) continue;

        lead = veh;
    }

    const auto cruise_error{target_spd - state.vel};
    if (!lead) {
        std::cout << "cruise\n";
        ctrl.accel = cruise_error * 0.2f;
    } else {
        const auto x_diff{lead->state().x - state.x};
        const auto ht{x_diff / state.vel};
        const auto follow_error{carlet::min(lead->state().vel, target_spd) - state.vel};
        const auto accel_cruise{cruise_error * 0.2};
        const auto accel_follow{follow_error * 0.2};
        const auto target_ht{2.5};
        ctrl.accel = ht < target_ht ? carlet::min(accel_cruise, follow_error) : carlet::max(accel_cruise, follow_error);
        std::cout << "lead vel: " << carlet::mps_to_kmph(lead->state().vel)
                  << ", ego vel: " << carlet::mps_to_kmph(state.vel)
                  << ", x_diff: " << x_diff
                  << ", ht: " << ht << "\n";
    }

    ctrl.steer = 0.0f;
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
    sim->gen_random_vehs(80);

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
