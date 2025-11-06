#include <cstddef>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <raylib.h>
#include <raymath.h>
#include <vector>

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


constexpr double target_spd{carlet::kmph_to_mps(120.0)};

template<typename T>
inline constexpr T avg2(const T& a, const T& b) { return (a + b) / 2; }

template<>
inline constexpr Vector3 avg2<Vector3>(const Vector3& a, const Vector3& b)
{
    return Vector3{
        .x=(a.x + b.x) / 2,
        .y=(a.y + b.y) / 2,
        .z=(a.z + b.z) / 2,
    };
}

template<>
inline constexpr Vector2 avg2<Vector2>(const Vector2& a, const Vector2& b)
{
    return Vector2{
        .x=(a.x + b.x) / 2,
        .y=(a.y + b.y) / 2,
    };
}

void lka(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    if (sensor_data.lanelets.empty()) {
        return;
    }

    constexpr auto preview_length{30.0f};
    const Vector3 ego_preview{
        .x=static_cast<float>(preview_length * std::cos(state.yaw)),
        .y=static_cast<float>(preview_length * std::sin(state.yaw)),
        .z=0.0f
    };

    std::vector<Vector3> lane_center{};
    const auto& left_lanelet{sensor_data.lanelets.at(1)};
    const auto& right_lanelet{sensor_data.lanelets.at(-1)};
    if (left_lanelet.empty() || right_lanelet.empty()) return;

    const auto num_samples{carlet::min(left_lanelet.size(), right_lanelet.size())};
    lane_center.reserve(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        lane_center.push_back(avg2(left_lanelet.at(i), right_lanelet.at(i)));
    }

    float min_dist{5.0f};
    int waypoint_idx{-1};
    for (size_t i = 0; i < lane_center.size(); ++i) {
        const auto this_dist{Vector3Distance(ego_preview, lane_center.at(i))};
        if (this_dist < min_dist) {
            waypoint_idx = i;
            min_dist = this_dist;
        }
    }

    ctrl.steer = 0.0f;
    if (waypoint_idx >= 0) {
        const auto& curr_waypoint{lane_center.at(waypoint_idx)};
        const auto error{ego_preview.y - curr_waypoint.y};
        ctrl.steer = -error * 0.01;
    }
}

void acc(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    const carlet::Veh::Obstacle* lead{nullptr};
    for (const auto obst: sensor_data.obsts) {
        if (obst.center.x < 0) continue;
        if (obst.center.x > 60.0f) continue;
        if (carlet::abs(obst.center.y - state.y) > 2.0f) continue;
        if (lead && obst.center.x > lead->center.x) continue;
        lead = &obst;
    }

    const auto cruise_error{target_spd - state.vel};
    if (!lead) {
        ctrl.accel = cruise_error * 0.1f;
    } else {
        const auto target_ht{2.5};
        const auto x_diff{lead->center.x - state.x};
        const auto ht{x_diff / state.vel};
        const auto desire_x{target_ht * state.vel};
        const auto dist_error{x_diff - desire_x};
        const auto vel_error{lead->vel - state.vel};
        ctrl.accel = dist_error * 0.2 + vel_error * 0.4;
    }

    ctrl.accel = carlet::min(ctrl.accel, 2.0f);
}

void ch_lane(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& state, carlet::Veh::Control& ctrl, int ch_lane_id)
{
    if (sensor_data.lanelets.empty()) {
        return;
    }

    std::vector<Vector3> target_lane_center{};
    const std::vector<Vector3>* left_lanelet{};
    const std::vector<Vector3>* right_lanelet{};

    if (ch_lane_id == -1) {
        // change lane right
        if (sensor_data.lanelets.at(-2).empty()) {
            std::cerr << "No available right lane\n";
            return;
        }

        right_lanelet = &sensor_data.lanelets.at(-2);
        left_lanelet = &sensor_data.lanelets.at(-1);
    } else if (ch_lane_id == 1) {
        // change lane left
        if (sensor_data.lanelets.at(2).empty()) {
            std::cerr << "No available left lane\n";
            return;
        }
        right_lanelet = &sensor_data.lanelets.at(2);
        left_lanelet = &sensor_data.lanelets.at(1);
    } else {
        std::cerr << "Unknown change lane id";
        return;
    }

    const auto num_samples{carlet::min(right_lanelet->size(), left_lanelet->size())};
    target_lane_center.reserve(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        target_lane_center.push_back(avg2(left_lanelet->at(i), right_lanelet->at(i)));
    }

    const Vector3* curr_waypoint{nullptr};
    const auto preview_length{30.0f};
    float min_dist{5.0f};
    const Vector3 ego_preview{
        .x=static_cast<float>(preview_length * std::cos(state.yaw)),
        .y=static_cast<float>(preview_length * std::sin(state.yaw)),
        .z=0.0f
    };
    for (const auto& waypoint : target_lane_center) {
        const auto this_dist{Vector3Distance(ego_preview, waypoint)};
        if (this_dist < min_dist) {
            min_dist = this_dist;
            curr_waypoint = &waypoint;
        }
    }

    ctrl.steer = 0.0f;
    if (curr_waypoint) {
        const auto error{ego_preview.y - curr_waypoint->y};
        ctrl.steer = -error * 0.01;
    }
}

enum class Behavior
{
    NOMINAL = 0,
    CH_LEFT,
    CH_RIGHT
}; // enum class Behavior

Behavior behavior_plan(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    if (state.x > 20.0f && !sensor_data.lanelets.empty() && !sensor_data.lanelets.at(2).empty()) {
        return Behavior::CH_LEFT;
    }
    return Behavior::NOMINAL;
}

void plan(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& state, carlet::Veh::Control& ctrl)
{
    acc(sensor_data, state, ctrl);
    const auto behavior{behavior_plan(sensor_data, state, ctrl)};
    if (behavior == Behavior::CH_LEFT) {
        ch_lane(sensor_data, state, ctrl, 1);
    } else if (behavior == Behavior::CH_RIGHT) {
        ch_lane(sensor_data, state, ctrl, -1);
    } else if (behavior == Behavior::NOMINAL) {
        lka(sensor_data, state, ctrl);
    }
}

int main()
{
    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=500.0f, .y=0.0f, .z=0.0f},
        2, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
    sim->gen_random_vehs(40,
        carlet::kmph_to_mps(40.0),
        carlet::kmph_to_mps(120.0));

    carlet::Veh* v{};
    carlet::Veh::Control ctrl{};

    while (sim->is_running()) {
        if ((v = sim->get_ctrl_veh()) != nullptr) {
            plan(v->sensor_data(), v->state(), ctrl);
            v->act(ctrl);
        }
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}
