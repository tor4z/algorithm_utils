#ifndef COMMON_HPP_
#define COMMON_HPP_

#include "carlet.hpp"

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
    int left_lead_idx;
    int right_lead_idx;
private:
    int find_lead_idx(const LaneInfo& lane);
    int project_obst_lane(const carlet::Veh::Obstacle& obst, const LaneInfo& lane);
    void setup_obst(const carlet::Veh::Obstacle& obst);
    void setup_lane(const std::vector<Vector3>& left,
        const std::vector<Vector3>& right, LaneInfo& lane);
}; // struct Scene


int find_waypoint(const Vector3& point, const LaneInfo& lane);

#endif // COMMON_HPP_
