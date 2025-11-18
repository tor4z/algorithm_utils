// #include "common.hpp"
// #include "settings.hpp"

// #define CARLET_IMPLEMENTATION
// #include "carlet.hpp"


// void lka(const Scene& scene, carlet::Veh::Control& ctrl)
// {
//     constexpr auto preview_length{30.0f};

//     ctrl.steer = 0.0f;
//     if (scene.lanes.at(0).samples.empty()) {
//         return;
//     }

//     const auto& ego{scene.ego};
//     const auto& curr_lane{scene.lanes.at(0)};
//     const Vector3 ego_preview{
//         .x=static_cast<float>(preview_length * std::cos(ego.yaw)),
//         .y=static_cast<float>(preview_length * std::sin(ego.yaw)),
//         .z=0.0f
//     };

//     const auto waypoint_idx{find_waypoint(ego_preview, curr_lane)};
//     if (waypoint_idx >= 0) {
//         const auto& curr_waypoint{curr_lane.samples.at(waypoint_idx).center};
//         const auto error{ego_preview.y - curr_waypoint.y};
//         ctrl.steer = -error * 0.01;
//     }
// }

// void acc(const Scene& scene, int target_lane_id, carlet::Veh::Control& ctrl)
// {
//     const auto cruise_error{target_spd - scene.ego.vel};
//     int lead_idx{target_lane_id == 0
//         ? scene.lead_idx
//         : target_lane_id == -1
//             ? scene.right_lead_idx
//             : scene.left_lead_idx};

//     if (lead_idx < 0) {
//         ctrl.accel = cruise_error * 0.1f;
//     } else {
//         const auto& lead{scene.obsts.at(lead_idx)};
//         const auto& ego{scene.ego};

//         const auto x_diff{lead.center.x};
//         const auto ht{x_diff / ego.vel};
//         const auto desire_x{target_ht * ego.vel};
//         const auto dist_error{x_diff - desire_x};
//         const auto vel_error{lead.vel - ego.vel};
//         // std::cout << "x_diff: " << x_diff << "dist_error: " << dist_error
//         //     << ", vel_error: " << vel_error << '\n';
//         ctrl.accel = dist_error * 0.05 + vel_error * 0.2;
//     }

//     ctrl.accel = carlet::min(ctrl.accel, 2.0f);
// }

// void ch_lane(const Scene& scene, carlet::Veh::Control& ctrl, int ch_lane_id)
// {
//     const auto preview_length{30.0f};
//     ctrl.steer = 0.0f;

//     if (ch_lane_id != -1 && ch_lane_id != 1) return;
//     const auto& target_lane{scene.lanes.at(ch_lane_id)};
//     if (target_lane.samples.empty()) return;

//     const auto& ego{scene.ego};
//     const Vector3 ego_preview{
//         .x=static_cast<float>(preview_length * std::cos(ego.yaw)),
//         .y=static_cast<float>(preview_length * std::sin(ego.yaw)),
//         .z=0.0f
//     };

//     const int waypoint_idx{find_waypoint(ego_preview, target_lane)};
//     if (waypoint_idx >= 0) {
//         const auto& preview_waypoint{target_lane.samples.at(waypoint_idx)};
//         const auto error{ego_preview.y - preview_waypoint.center.y};
//         ctrl.steer = -error * 0.01;
//     }
// }

// enum class Behavior
// {
//     NOMINAL = 0,
//     CH_LEFT,
//     CH_RIGHT
// }; // enum class Behavior

// bool is_ch_lane_cmd(Behavior b)
// {
//     return b == Behavior::CH_RIGHT ||
//         b == Behavior::CH_LEFT;
// }

// Behavior behavior_plan(const Scene& scene, carlet::Veh::Control& ctrl)
// {
//     constexpr float max_spd{1000.0f};
//     constexpr float max_lead_s{1000.0f};
//     constexpr float exp_spd_gain{1.0f};

//     const auto has_lead{scene.lead_idx >= 0};
//     const auto has_left_lead{scene.left_lead_idx >= 0};
//     const auto has_right_lead{scene.right_lead_idx >= 0};

//     const auto has_left_lane{!scene.lanes.at(1).samples.empty()};
//     const auto has_right_lane{!scene.lanes.at(-1).samples.empty()};

//     const auto lead_vel{has_lead ? scene.obsts.at(scene.lead_idx).vel : max_spd};
//     const auto left_lead_vel{has_left_lead
//         ? scene.obsts.at(scene.left_lead_idx).vel
//         : has_left_lane ? max_spd : -1.0f};
//     const auto right_lead_vel{has_right_lead
//         ? scene.obsts.at(scene.right_lead_idx).vel
//         : has_right_lane ? max_spd : -1.0f};

//     const auto left_lead_ht{has_left_lead
//         ? scene.obsts.at(scene.left_lead_idx).center.x / scene.ego.vel
//         : max_lead_s};
//     const auto right_lead_ht{has_right_lead
//         ? scene.obsts.at(scene.right_lead_idx).center.x / scene.ego.vel
//         : max_lead_s};

//     static Behavior last_behavior{Behavior::NOMINAL};
//     static float last_lane_offset{0.0f};
//     Behavior behavior{Behavior::NOMINAL};

//     const auto& lane0{scene.lanes.at(0)};
//     if (lane0.samples.empty()) return behavior;

//     const auto& sample{lane0.samples.at(0)};
//     const auto lane_offset{sample.left.y + sample.right.y};
//     const auto lane_changed{carlet::abs(last_lane_offset - lane_offset) > 1.0f};

//     int fast_lane{0};
//     if (left_lead_vel > right_lead_vel) {
//         if (left_lead_vel > (lead_vel + exp_spd_gain) && left_lead_ht > target_ht) fast_lane = 1;
//     } else {
//         if (right_lead_vel > (lead_vel + exp_spd_gain)  && right_lead_ht > target_ht) fast_lane = -1;
//     }

//     if (fast_lane == 1) {
//         behavior = Behavior::CH_LEFT;
//     } else if (fast_lane == -1) {
//         behavior = Behavior::CH_RIGHT;
//     }

//     if (is_ch_lane_cmd(last_behavior) && !lane_changed) {
//         behavior = last_behavior;
//     }

//     last_lane_offset = lane_offset;
//     last_behavior = behavior;
//     return behavior;
// }

// void plan(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& ego_state, carlet::Veh::Control& ctrl)
// {
//     Scene scene{sensor_data, ego_state};

//     int target_lane_id{0};
//     const auto behavior{behavior_plan(scene, ctrl)};

//     switch (behavior) {
//     case Behavior::CH_LEFT:
//         ch_lane(scene, ctrl, 1);
//         target_lane_id = 1;
//         break;
//     case Behavior::CH_RIGHT:
//         ch_lane(scene, ctrl, -1);
//         target_lane_id = -1;
//         break;
//     case Behavior::NOMINAL:
//     default:
//         lka(scene, ctrl);
//         break;
//     }

//     acc(scene, target_lane_id, ctrl);

//     // std::cout << "behavior: " << static_cast<int>(behavior)
//     //     << ", steer: " << ctrl.steer
//     //     << ", spd: " << carlet::mps_to_kmph(ego_state.vel)
//     //     << ", accel: " << ctrl.accel << "\n";
// }

// int main(int argc, char** argv)
// {
//     (void)argc;
//     (void)argv;

//     srand(time(NULL));

//     const auto straight_road{carlet::Road::gen_straight(
//         Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
//         Vector3{.x=2000.0f, .y=0.0f, .z=0.0f},
//         2, 3.7f)};

//     auto sim{carlet::Simulator::instance()};
//     sim->map().road_net.push_back(straight_road);
//     sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
//     sim->gen_random_vehs(20,
//         carlet::kmph_to_mps(40.0),
//         carlet::kmph_to_mps(80.0));

//     carlet::Veh* v{};
//     carlet::Veh::Control ctrl{};

//     while (sim->is_running()) {
//         if ((v = sim->get_ctrl_veh()) != nullptr) {
//             plan(v->sensor_data(), v->state(), ctrl);
//             v->act(ctrl);
//         }
//         sim->step(0.02f);
//         sim->render();
//     }
//     return 0;
// }

int main() {}
