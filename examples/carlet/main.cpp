#include <ctime>
#include <iostream>

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


constexpr double target_spd{carlet::kmph_to_mps(150.0)};

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

struct ObstacleInfo
{
    Vector3 center;
    Vector3 shape;
    float heading;
    float vel;
    int lane_id;
    int lane_sample_idx;
}; // struct ObstacleInfo

struct LaneInfo
{
    explicit LaneInfo(int id) : id(id) {}

    struct Sample {
        Vector3 left;
        Vector3 center;
        Vector3 right;
    }; // struct Sample

    std::vector<Sample> samples;
    std::vector<int> obsts;
    int id;
}; // struct LaneInfo

struct Scene
{
    Scene(const carlet::Veh::SensorData& sensor_data,
        const carlet::Veh::State& ego_state);

    std::unordered_map<int, LaneInfo> lanes;
    std::vector<ObstacleInfo> obsts;
    carlet::Veh::State ego;
    int lead_idx;
private:
    int find_lead_idx();
    int project_obst_lane(const carlet::Veh::Obstacle& obst, const LaneInfo& lane);
    void setup_obst(const carlet::Veh::Obstacle& obst);
    void setup_lane(const std::vector<Vector3>& left,
        const std::vector<Vector3>& right, LaneInfo& lane);
}; // struct Scene

Scene::Scene(const carlet::Veh::SensorData& sensor_data,
    const carlet::Veh::State& ego_state)
    : ego(ego_state)
{
    obsts.reserve(sensor_data.obsts.size());

    lanes.insert(std::make_pair(-1, LaneInfo{-1}));
    lanes.insert(std::make_pair(0, LaneInfo{0}));
    lanes.insert(std::make_pair(1, LaneInfo{1}));

    auto lanelet_valid{[&sensor_data] (int lanelet_id) -> bool {
        return sensor_data.lanelets.find(lanelet_id) != sensor_data.lanelets.end() &&
               !sensor_data.lanelets.at(lanelet_id).empty();
    }};

    if (lanelet_valid(-2) && lanelet_valid(-1)) {
        // setup lane -1
        setup_lane(sensor_data.lanelets.at(-1), sensor_data.lanelets.at(-2), lanes.at(-1));
    }

    if (lanelet_valid(-1) && lanelet_valid(1)) {
        // setup lane 0
        setup_lane(sensor_data.lanelets.at(1), sensor_data.lanelets.at(-1), lanes.at(0));
    }

    if (lanelet_valid(1) && lanelet_valid(2)) {
        // setup lane 1
        setup_lane(sensor_data.lanelets.at(2), sensor_data.lanelets.at(1), lanes.at(1));
    }

    for (const auto& obst : sensor_data.obsts) {
        setup_obst(obst);
    }

    lead_idx = find_lead_idx();
}

void Scene::setup_obst(const carlet::Veh::Obstacle& obst)
{
    ObstacleInfo obst_info;
    bool obst_valid{false};

    for (const auto& it: lanes) {
        if (it.first < -1 || it.first > 1) continue;
        if (it.second.samples.empty()) continue;

        const auto sample_idx{project_obst_lane(obst, it.second)};
        if (sample_idx < 0) continue;

        const auto& project_sample{it.second.samples.at(sample_idx)};
        const auto on_left{obst.center.y > project_sample.left.y};
        const auto on_right{obst.center.y < project_sample.right.y};
        const auto on_this_lane{!(on_left || on_right)};

        if (on_this_lane) {
            obst_info.lane_id = it.first;
            obst_info.lane_sample_idx = sample_idx;
            obst_valid = true;
            break;
        }
    }

    if (!obst_valid) return;

    obst_info.center = obst.center;
    obst_info.shape = obst.shape;
    obst_info.heading = obst.heading;
    obst_info.vel = obst.vel;
    obsts.push_back(obst_info);
    lanes.at(obst_info.lane_id).obsts.push_back(obsts.size() - 1);
}

void Scene::setup_lane(const std::vector<Vector3>& left,
    const std::vector<Vector3>& right, LaneInfo& lane)
{
    if (left.empty()) return;
    if (right.empty()) return;

    const auto num_samples{carlet::min(right.size(), left.size())};
    lane.samples.resize(num_samples);

    for (size_t i = 0; i < num_samples; ++i) {
        auto& sample{lane.samples.at(i)};
        sample.center = avg2(left.at(i), right.at(i));
        sample.left = left.at(i);
        sample.right = right.at(i);
    }
}

int Scene::project_obst_lane(const carlet::Veh::Obstacle& obst, const LaneInfo& lane)
{
    if (lane.samples.empty()) return -1;

    int low{0};
    int high{static_cast<int>(lane.samples.size()) - 1};
    float dist_low{Vector3DistanceSqr(obst.center, lane.samples.at(low).center)};
    float dist_high{Vector3DistanceSqr(obst.center, lane.samples.at(high).center)};

    while (low < high) {
        const auto mid{(low + high) / 2};
        if (dist_low < dist_high) {
            high = mid - 1;
            dist_high = Vector3DistanceSqr(obst.center, lane.samples.at(high).center);
        } else {
            low = mid + 1;
            dist_low = Vector3DistanceSqr(obst.center, lane.samples.at(low).center);
        }
    }

    return (low + high) / 2;
}

int Scene::find_lead_idx()
{
    int lead_idx{-1};
    float lead_center_x{1000.0f /* Big enough x by default*/};
    if (lanes.find(0) != lanes.end()) {
        for (auto obst_idx: lanes.at(0).obsts) {
            const auto& obst{obsts.at(obst_idx)};
            if (obst.center.x < 0.0f) continue;
            if (obst.center.x < lead_center_x) {
                lead_center_x = obst.center.x;
                lead_idx = obst_idx;
            }
        }
    }
    return lead_idx;
}

int find_waypoint(const Vector3& point, const LaneInfo& lane)
{
    float min_dist{10.0f /* Big enough default dist */};
    int waypoint_idx{-1};
    for (size_t i = 0; i < lane.samples.size(); ++i) {
        const auto this_dist{Vector3Distance(point, lane.samples.at(i).center)};
        if (this_dist < min_dist) {
            waypoint_idx = i;
            min_dist = this_dist;
        }
    }

    return waypoint_idx;
}

void lka(const Scene& scene, carlet::Veh::Control& ctrl)
{
    constexpr auto preview_length{30.0f};

    ctrl.steer = 0.0f;
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
        ctrl.steer = -error * 0.01;
    }
}

void acc(const Scene& scene, carlet::Veh::Control& ctrl)
{
    const auto cruise_error{target_spd - scene.ego.vel};

    if (scene.lead_idx < 0) {
        ctrl.accel = cruise_error * 0.1f;
    } else {
        const auto& lead{scene.obsts.at(scene.lead_idx)};
        const auto& ego{scene.ego};

        const auto target_ht{2.5};
        const auto x_diff{lead.center.x};
        const auto ht{x_diff / ego.vel};
        const auto desire_x{target_ht * ego.vel};
        const auto dist_error{x_diff - desire_x};
        const auto vel_error{lead.vel - ego.vel};
        ctrl.accel = dist_error * 0.2 + vel_error * 0.4;
    }

    ctrl.accel = carlet::min(ctrl.accel, 2.0f);
}

void ch_lane(const Scene& scene, carlet::Veh::Control& ctrl, int ch_lane_id)
{
    const auto preview_length{30.0f};
    ctrl.steer = 0.0f;

    if (ch_lane_id != -1 || ch_lane_id != 1) return;
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
        ctrl.steer = -error * 0.01;
    }
}

enum class Behavior
{
    NOMINAL = 0,
    CH_LEFT,
    CH_RIGHT
}; // enum class Behavior

Behavior behavior_plan(const Scene& scene, carlet::Veh::Control& ctrl)
{
    const auto has_lead{scene.lead_idx >= 0};
    const auto lead_vel{has_lead ? scene.obsts.at(scene.lead_idx).vel : -1.0f};
    const auto has_left_lane{!scene.lanes.at(1).samples.empty()};
    const auto has_right_lane{!scene.lanes.at(-1).samples.empty()};
    const auto should_ch_lane{has_lead && (lead_vel < target_spd)};

    if (should_ch_lane) {
        return has_left_lane
            ? Behavior::CH_LEFT
            : has_right_lane
                ? Behavior::CH_RIGHT
                : Behavior::NOMINAL;
    }

    return Behavior::NOMINAL;
}

void plan(const carlet::Veh::SensorData& sensor_data, const carlet::Veh::State& ego_state, carlet::Veh::Control& ctrl)
{
    Scene scene{sensor_data, ego_state};

    acc(scene, ctrl);
    const auto behavior{behavior_plan(scene, ctrl)};

    switch (behavior) {
    case Behavior::CH_LEFT:
        ch_lane(scene, ctrl, 1);
        break;
    case Behavior::CH_RIGHT:
        ch_lane(scene, ctrl, -1);
        break;
    case Behavior::NOMINAL:
    default:
        lka(scene, ctrl);
        break;
    }

    std::cout << "behavior: " << static_cast<int>(behavior) << ", steer: " << ctrl.steer << "\n";
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=2000.0f, .y=0.0f, .z=0.0f},
        2, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
    sim->gen_random_vehs(80,
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
