#include "common.hpp"
#include "settings.hpp"

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"

enum class Action
{
    NOMINAL = 0,
    CH_LEFT,
    CH_RIGHT
}; // enum class Action

struct State
{
    const Scene* scene;
    Action last_act;
    int lane_id;
}; // struct State

struct ActResult
{
    State state;
    float reward;
}; // struct ActResult

static Action actions[]{
    Action::NOMINAL,
    Action::CH_LEFT,
    Action::CH_RIGHT,
};

void lka(const Scene& scene, carlet::Veh::AngleControl& ctrl)
{
    constexpr auto preview_length{30.0f};

    ctrl.eps_angle = 0.0f;
    if (scene.lanes.at(0).samples.empty()) {
        return;
    }

    const auto& ego{scene.ego};
    const auto& curr_lane{scene.lanes.at(0)};
    const Vector3 ego_preview{
        .x=static_cast<float>(preview_length * std::cos(ego.yaw)),
        .y=static_cast<float>(preview_length * std::sin(ego.yaw)),
        .z=0.0f
    };

    const auto waypoint_idx{find_waypoint(ego_preview, curr_lane)};
    if (waypoint_idx >= 0) {
        const auto& curr_waypoint{curr_lane.samples.at(waypoint_idx).center};
        const auto error{ego_preview.y - curr_waypoint.y};
        ctrl.eps_angle = -error * 0.01;
    }
}

float follow_accel(const carlet::Veh::State& ego, const ObstacleInfo& lead)
{
    const auto x_diff{lead.center.x};
    const auto ht{x_diff / ego.vel};
    const auto desire_x{target_ht * ego.vel};
    const auto dist_error{x_diff - desire_x};
    const auto vel_error{lead.vel - ego.vel};
    return dist_error * 0.05 + vel_error * 0.2;
}

void acc(const Scene& scene, int target_lane_id, carlet::Veh::AngleControl& ctrl)
{
    ctrl.accel = 0.0f;
    const auto cruise_error{target_spd - scene.ego.vel};
    int lead_idx{target_lane_id == 0
        ? scene.lead_idx
        : target_lane_id == -1
            ? scene.right_lead_idx
            : scene.left_lead_idx};

    if (lead_idx < 0) {
        ctrl.accel = cruise_error * 0.1f;
    } else {
        const auto& lead{scene.obsts.at(lead_idx)};
        ctrl.accel = follow_accel(scene.ego, lead);
    }
}

void ch_lane(const Scene& scene, carlet::Veh::AngleControl& ctrl, int ch_lane_id)
{
    const auto preview_length{30.0f};
    ctrl.eps_angle = 0.0f;

    if (ch_lane_id != -1 && ch_lane_id != 1) return;
    const auto& target_lane{scene.lanes.at(ch_lane_id)};
    if (target_lane.samples.empty()) return;

    const auto& ego{scene.ego};
    const Vector3 ego_preview{
        .x=static_cast<float>(preview_length * std::cos(ego.yaw)),
        .y=static_cast<float>(preview_length * std::sin(ego.yaw)),
        .z=0.0f
    };

    const int waypoint_idx{find_waypoint(ego_preview, target_lane)};
    if (waypoint_idx >= 0) {
        const auto& preview_waypoint{target_lane.samples.at(waypoint_idx)};
        const auto error{ego_preview.y - preview_waypoint.center.y};
        ctrl.eps_angle = -error * 0.01;
    }
}

struct Agent
{
    virtual ActResult act(Action a, State s)
    {
        int target_lane_id{0};
        int lead_idx{-1};
        bool target_lane_valid{true};

        switch (a) {
        case Action::NOMINAL:
            target_lane_id = 0;
            lead_idx = s.scene->lead_idx;
            break;
        case Action::CH_LEFT:
            target_lane_id = 1;
            lead_idx = s.scene->left_lead_idx;
            break;
        case Action::CH_RIGHT:
            target_lane_id = -1;
            lead_idx = s.scene->right_lead_idx;
            break;
        }

        ActResult ar;
        ar.state.lane_id = target_lane_id;
        if (s.scene->lanes.at(target_lane_id).samples.empty()) {
            target_lane_valid = false;
        }

        if (lead_idx == -1) {
            ar.reward = 1.0f;
        } else {
            const auto& lead{s.scene->obsts.at(lead_idx)};
            const auto target_accel{follow_accel(s.scene->ego, lead)};
            if (target_accel >= 1.0f) {
                ar.reward += lead.vel - target_spd;
            } else {
                ar.reward += target_accel;
            }
        }

        int nearest_back_obst_idx{-1};
        float max_back_obst_x{-100.0f /* max back obstacle dist we care */};
        const auto& obsts{s.scene->lanes.at(target_lane_id).obsts};
        for (size_t i = 0; i < obsts.size(); ++i) {
            const auto obst_idx{obsts.at(i)};
            const auto& obst{s.scene->obsts.at(obst_idx)};
            if (obst.center.x > 0.0f) continue;
            if (obst.center.x > max_back_obst_x) {
                max_back_obst_x = obst.center.x;
                nearest_back_obst_idx = obst_idx;
            }
        }

        if (a == Action::NOMINAL) {
            ar.reward += 0.5f;
        } else {
            // for change lane action
            if (nearest_back_obst_idx >= 0) {
                const auto& obst{s.scene->obsts.at(nearest_back_obst_idx)};
                if (obst.center.x > -10.0f) ar.reward += -10000.0f;
            }
        }

        if (s.last_act != a) {
            ar.reward -= 0.5f;
        }

        if (!target_lane_valid) {
            ar.reward = -10000.0f;
        }
        return ar;
    }
}; // struct Agent

Action behavior_plan(const Scene& scene, carlet::Veh::AngleControl& ctrl)
{
    static Action last_act{};
    Agent agent{};

    State state{.scene=&scene, .last_act=last_act};
    Action best_act{Action::NOMINAL};
    float best_reward{-10000.0f};

    for (auto act: actions) {
        const auto& this_ar{agent.act(act, state)};
        if (this_ar.reward > best_reward) {
            best_reward = this_ar.reward;
            best_act = act;
        }
    }

    last_act = best_act;
    return best_act;
}

void plan(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& ego_state, carlet::Veh::AngleControl& ctrl)
{
    Scene scene{sensor_data, ego_state};

    int target_lane_id{0};
    const auto behavior{behavior_plan(scene, ctrl)};

    switch (behavior) {
    case Action::CH_LEFT:
        ch_lane(scene, ctrl, 1);
        target_lane_id = 1;
        break;
    case Action::CH_RIGHT:
        ch_lane(scene, ctrl, -1);
        target_lane_id = -1;
        break;
    case Action::NOMINAL:
    default:
        lka(scene, ctrl);
        break;
    }

    acc(scene, target_lane_id, ctrl);

    // std::cout << "behavior: " << static_cast<int>(behavior)
    //           << ", steer: " << ctrl.steer
    //           << ", spd: " << carlet::mps_to_kmph(ego_state.vel)
    //           << ", accel: " << ctrl.accel << "\n";
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=5000.0f, .y=0.0f, .z=0.0f},
        4, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
    sim->gen_random_vehs(20,
        carlet::kmph_to_mps(40.0f),
        carlet::kmph_to_mps(150.0f));

    carlet::Veh* v{};
    carlet::Veh::AngleControl ctrl{};

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
