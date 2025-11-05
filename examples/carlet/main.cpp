#include <ctime>
#include <cstdlib>

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


constexpr double target_spd{carlet::kmph_to_mps(120.0)};

int get_lane_idx(const carlet::Veh::Obs& obs, const carlet::Veh::State& state)
{
    int target_idx{-1};
    float min_dist{2.0f};
    const Vector3 ego_pose{
        .x=static_cast<float>(state.x),
        .y=static_cast<float>(state.y),
        .z=static_cast<float>(state.z)
    };
    for (const auto& road: obs.map.road_net) {
        for (size_t i = 0; i < road.lanes.size(); ++i) {
            const auto& lane{road.lanes.at(i)};
            for (const auto& waypoint : lane) {
                const auto this_dist{Vector3Distance(ego_pose, waypoint.c)};
                if (this_dist < min_dist) {
                    min_dist = this_dist;
                    target_idx = i;
                }
            }
        }
    }

    return target_idx;
}

void lka(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    int curr_waypoint_idx{-1};
    float min_dist{5.0f};
    const auto preview_length{20.0f};
    const auto target_lane_idx{get_lane_idx(obs, state)};
    const Vector3 ego_preview{
        .x=static_cast<float>(state.x + preview_length * std::cos(state.yaw)),
        .y=static_cast<float>(state.y + preview_length * std::sin(state.yaw)),
        .z=static_cast<float>(state.z)
    };

    if (target_lane_idx >= 0) {
        const auto lane{obs.map.road_net.at(0).lanes.at(target_lane_idx)};
        for (size_t i = 0; i < lane.size(); ++i) {
            const auto& waypoint{lane.at(i)};
            const auto this_dist{Vector3Distance(ego_preview, waypoint.c)};
            if (this_dist < min_dist) {
                min_dist = this_dist;
                curr_waypoint_idx = i;
            }
        }
    }

    ctrl.steer = 0.0f;
    if (curr_waypoint_idx >= 0) {
        const auto lane{obs.map.road_net.at(0).lanes.at(target_lane_idx)};
        const auto& curr_waypoint{lane.at(curr_waypoint_idx)};
        const auto error{ego_preview.y - curr_waypoint.c.y};
        ctrl.steer = -error * 0.01;
    }
}

void acc(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
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
}

void ch_lane(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl, int target_lane_idx)
{
    if (target_lane_idx < 0) {
        return;
    }

    const carlet::Road::LaneSample* curr_waypoint{nullptr};
    const auto preview_length{30.0f};
    float min_dist{5.0f};
    const auto& lane{obs.map.road_net.at(0).lanes.at(target_lane_idx)};
    const Vector3 ego_preview{
        .x=static_cast<float>(state.x + preview_length * std::cos(state.yaw)),
        .y=static_cast<float>(state.y + preview_length * std::sin(state.yaw)),
        .z=static_cast<float>(state.z)
    };
    for (const auto& waypoint : lane) {
        const auto this_dist{Vector3Distance(ego_preview, waypoint.c)};
        if (this_dist < min_dist) {
            min_dist = this_dist;
            curr_waypoint = &waypoint;
        }
    }

    ctrl.steer = 0.0f;
    if (curr_waypoint) {
        const auto error{ego_preview.y - curr_waypoint->c.y};
        ctrl.steer = -error * 0.01;
    }
}

enum class Behavior
{
    NOMINAL = 0,
    CH_LEFT,
    CH_RIGHT
}; // enum class Behavior

Behavior behavior_plan(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    if (state.x > 20.0f) {
        return Behavior::CH_LEFT;
    }
    return Behavior::NOMINAL;
}

void plan(const carlet::Veh::Obs& obs, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    acc(obs, state, ctrl);
    const auto behavior{behavior_plan(obs, state, ctrl)};
    if (behavior == Behavior::CH_LEFT) {
        ch_lane(obs, state, ctrl, 0);
    } else if (behavior == Behavior::CH_RIGHT) {
        ch_lane(obs, state, ctrl, 0);
    } else if (behavior == Behavior::NOMINAL) {
        lka(obs, state, ctrl);
    }
}

int main()
{
    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=5000.0f, .y=0.0f, .z=0.0f},
        2, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
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
