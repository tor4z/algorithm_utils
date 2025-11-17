#include "common.hpp"
#include <raymath.h>

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

    lead_idx = find_lead_idx(lanes.find(0)->second);
    left_lead_idx = find_lead_idx(lanes.find(1)->second);
    right_lead_idx = find_lead_idx(lanes.find(-1)->second);
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

    const int last_idx{static_cast<int>(lane.samples.size()) - 1};
    int low{0};
    int high{last_idx};
    float dist_low{Vector3DistanceSqr(obst.center, lane.samples.at(low).center)};
    float dist_high{Vector3DistanceSqr(obst.center, lane.samples.at(high).center)};

    while (low < high) {
        const auto mid{(low + high) / 2};
        if (dist_low < dist_high) {
            high = carlet::max(mid - 1, 0);
            dist_high = Vector3DistanceSqr(obst.center, lane.samples.at(high).center);
        } else {
            low = carlet::min(mid + 1, last_idx);
            dist_low = Vector3DistanceSqr(obst.center, lane.samples.at(low).center);
        }
    }

    return (low + high) / 2;
}

int Scene::find_lead_idx(const LaneInfo& lane)
{
    int lead_idx{-1};
    float lead_center_x{1000.0f /* Big enough x by default*/};
    for (auto obst_idx: lane.obsts) {
        const auto& obst{obsts.at(obst_idx)};
        if (obst.center.x < 0.0f) continue;
        if (obst.center.x < lead_center_x) {
            lead_center_x = obst.center.x;
            lead_idx = obst_idx;
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
