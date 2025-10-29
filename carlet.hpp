#ifndef CARLET_HPP_
#define CARLET_HPP_

#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <vector>
#include <raylib.h>

#define CARLET_DEF_SINGLETON(classname)                           \
    public:                                                       \
        static inline classname* instance()                       \
        {                                                         \
            static classname *instance_ = nullptr;                \
            static std::once_flag flag;                           \
            if (!instance_) {                                     \
                std::call_once(flag, [&](){                       \
                    instance_ = new (std::nothrow) classname();   \
                });                                               \
            }                                                     \
            return instance_;                                     \
        }                                                         \
    private:                                                      \
        classname(const classname&) = delete;                     \
        classname& operator=(const classname&) = delete;          \
        classname(const classname&&) = delete;                    \
        classname& operator=(const classname&&) = delete;


namespace carlet {

struct Road
{
    struct Strip {
        Vector3 l;  // left line edge
        Vector3 r;  // right line edge
    }; // struct Strip

    struct LaneSample {
        Vector3 c;  // lane center sample point
        float width;
    }; // struct LaneSample

    using Lane = std::vector<LaneSample>;
    using Lanelet = std::vector<Strip>;
    using RoadEdge = std::vector<Strip>;

    std::vector<Lane> lanes;
    std::vector<Lanelet> lanelets;
    RoadEdge left_edge;
    RoadEdge right_edge;

    static Road gen_straight(const Vector3& start_position, float length, int num_lane, float lane_width);
}; // struct Road

struct RoadNet
{
    std::vector<Road> roads;
}; // struct RoadNet

struct Map
{
    RoadNet road_net;
    Mesh road_mesh;
    Model road_model;
    Mesh edge_mesh;
    Model edge_model;
    Mesh lanelet_mesh;
    Model lanelet_model;
}; // struct Map

struct VehModel
{
    float wheel_base;
    float gc_to_back_axle;  // gc: gravity center
    float max_steer;
    float min_steer;
}; // struct VehModel

class BicycleModel
{
public:
    struct State
    {
        float x;
        float y;
        float z;
        float vel;
        float accel;
        float jerk;
        float yaw;
        float yaw_rate;
        float steer_angle;

        static State init_with(float init_x, float init_y, float init_vel);
    }; // struct State

    BicycleModel(const State& state, const VehModel& vm)
        : state_(state)
        , vm_(vm) {}
    void act(float steer, float accel, float dt);
    inline const State& state() const { return state_; }
private:
    State state_;
    const VehModel vm_;
}; // class BicycleModel

struct Object
{
    Model model;
    Color color;
    virtual bool step(float dt) { return true; }
}; // struct Object

struct Veh: public Object
{
    Veh(float init_x, float init_y, float init_vel, const VehModel& model)
        : steer(0.0f)
        , accel(0.0f)
        , dynamic(BicycleModel::State::init_with(init_x, init_y, init_vel), model)
        {}

    inline const BicycleModel::State& state() const { return dynamic.state(); }
    float steer;
    float accel;
    BicycleModel dynamic;
}; // struct Veh

struct ControllableVeh : public Veh
{
    ControllableVeh(float init_x, float init_y, float init_vel, const VehModel& model)
        : Veh(init_x, init_y, init_vel, model) {}
    void act(float steer, float accel);
    virtual bool step(float dt) override;
}; // struct ControllableVehicle

struct SelfDrivingVeh : public Veh
{
    virtual bool step(float dt) override { return true; }
}; // struct ControllableVehicle

class Simulator
{
    CARLET_DEF_SINGLETON(Simulator)
public:
    ~Simulator() = default;
    bool is_running();
    bool step(float dt);
    void render();

    int create_ctrl_veh(const VehModel& model);
    ControllableVeh& get_ctrl_veh(int id);
    void gen_random_vehs(int n);
    inline Map& map() { return map_; }
private:
    Simulator();
    void map_to_mesh_model();
    void update_camera();

    Map map_;
    // std::vector<SelfDrivingVeh> sd_vehs_;
    std::vector<ControllableVeh> ctrl_vehs_;
    Camera3D camera_;
}; // class Simulator

namespace veh_model {

extern const VehModel tesla;

}; // namespace veh_model

} // namespace carlet

#endif // CARLET_HPP_


#define CARLET_IMPLEMENTATION delete me


#ifdef CARLET_IMPLEMENTATION
#ifndef CARLET_CPP_
#define CARLET_CPP_

#include <cassert>
#include <rlgl.h>
#include <raymath.h>


#ifndef CARLET_EPSf
#   define CARLET_EPSf 1.0e-6f
#endif // CARLET_EPSf

#ifndef CARLET_TARGET_FPS
#   define CARLET_TARGET_FPS        60
#endif // CARLET_TARGET_FPS

#ifndef CARLET_WIN_WIDTH
#   define CARLET_WIN_WIDTH         800
#endif // CARLET_WIN_WIDTH

#ifndef CARLET_WIN_HEIGHT
#   define CARLET_WIN_HEIGHT        600
#endif // CARLET_WIN_HEIGHT

#ifndef CARLET_WIN_TITLE
#   define CARLET_WIN_TITLE         "Carlet Simulator"
#endif // CARLET_WIN_TITLE

#ifndef CARLET_ROAD_EDGE_WIDTH
#   define CARLET_ROAD_EDGE_WIDTH   0.1f   // meter, which is 1cm
#endif // CARLET_ROAD_EDGE_WIDTH

#ifndef CARLET_LANELET_WIDTH
#   define CARLET_LANELET_WIDTH     0.1f   // meter, which is 1cm
#endif // CARLET_LANELET_WIDTH

#ifndef CARLET_DEF_VEH_LEN
#   define CARLET_DEF_VEH_LEN       4.2f
#endif // CARLET_DEF_VEH_LEN

#ifndef CARLET_DEF_VEH_WIDTH
#   define CARLET_DEF_VEH_WIDTH     1.98f
#endif // CARLET_DEF_VEH_WIDTH

#ifndef CARLET_DEF_VEH_HEIGHT
#   define CARLET_DEF_VEH_HEIGHT     1.6f
#endif // CARLET_DEF_VEH_HEIGHT


namespace carlet {

template<typename T>
inline constexpr T min(T a, T b) { return a > b ? b : a; }

template<typename T>
inline constexpr T max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline constexpr T clamp(T v, T low, T high) { return v > high ? high : v < low ? low : v; }

template<typename T>
inline constexpr T mps_to_kmph(T mps) { return mps * static_cast<T>(3.6); }

template<typename T>
inline constexpr T kmph_to_mps(T kmph) { return kmph / static_cast<T>(3.6); }

template<typename T>
inline constexpr T rad_to_deg(T rad) { return rad / M_PI * 180.0; }

template<typename T>
inline constexpr T deg_to_rad(T deg) { return deg / 180.0 * M_PI; }

namespace veh_model {

const VehModel tesla {
    .wheel_base = 2.8f,
    .gc_to_back_axle = 1.5f,
    .max_steer = deg_to_rad(25.0f),
    .min_steer = deg_to_rad(-25.0f),
}; // tesla

}; // namespace veh_model

static const Color veh_colors[] {
    YELLOW,  GOLD,    ORANGE,     PINK,    RED,   MAROON,
    GREEN,   LIME,    DARKGREEN,  SKYBLUE, BLUE,  DARKBLUE,
    PURPLE,  VIOLET,  DARKPURPLE, BEIGE,   BROWN, DARKBROWN,
    MAGENTA, RAYWHITE
};

#ifdef _GLIBCXX_OSTREAM
inline std::ostream& operator<<(std::ostream& os, const Vector2& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Vector3& vec)
{
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const BicycleModel::State& state)
{
    os << std::fixed << std::setprecision(2) << "State {"
       << "x: " << state.x
       << ", y: " << state.y
       << ", z: " << state.z
       << ", vel: " << state.vel
       << ", accel: " << state.accel
       << ", jerk: " << state.jerk
       << ", yaw: " << state.yaw
       << ", yaw_rate: " << state.yaw_rate
       << ", steer_angle: " << state.steer_angle
       << "}";
    return os;
}
#endif // _GLIBCXX_OSTREAM

inline const Color& random_color()
{
    constexpr auto num_colors{sizeof(veh_colors) / sizeof(veh_colors[0])};
    return veh_colors[rand() % num_colors];
}

inline void to_raylib_mesh3(float* vec)
{
    const auto x{vec[0]};
    const auto y{vec[1]};
    const auto z{vec[2]};
    vec[0] = y;
    vec[1] = z;
    vec[2] = -x;
}

inline Vector3 to_raylib(const Vector3& vec)
{
    return Vector3{.x = -vec.y, .y = vec.x, .z = vec.z};
}

BicycleModel::State BicycleModel::State::init_with(float init_x, float init_y, float init_vel)
{
    return State{
        .x = init_x,
        .y = init_y,
        .z = CARLET_DEF_VEH_HEIGHT / 2.0f,
        .vel = init_vel,
        .accel = 0.0f,
        .jerk = 0.0f,
        .yaw = 0.0f,
        .yaw_rate = 0.0f,
        .steer_angle = 0.0f,
    };
}

Road Road::gen_straight(const Vector3& start_position, float length, int num_lane, float lane_width)
{
    constexpr auto min_lane_width{2.0f};    // meter
    constexpr auto sample_length{1.0f};     // meter

    assert(length > 0.0f && "Bad road length");
    assert(num_lane > 0 && "Bad number of lane");
    assert(lane_width > min_lane_width && "Bad lane width");

    const auto num_samples{max(static_cast<int>(length / sample_length), 1)};
    const auto road_width{num_lane * lane_width};

    Road road;
    road.left_edge.resize(num_samples);
    road.right_edge.resize(num_samples);
    if (num_lane >= 2) {
        road.lanelets.resize(num_lane - 1);
        for (auto& lanelet : road.lanelets) {
            lanelet.resize(num_samples);
        }
    }

    road.lanes.resize(road.lanelets.size() + 1);
    for (auto& lane: road.lanes) {
        lane.resize(num_samples);
    }

    constexpr auto half_road_edge_width{CARLET_ROAD_EDGE_WIDTH / 2.0f};
    const auto road_left_edge_y_r{start_position.y + road_width / 2.0f - half_road_edge_width};
    const auto road_left_edge_y_l{start_position.y + road_width / 2.0f + half_road_edge_width};
    const auto road_right_edge_y_r{start_position.y - road_width / 2.0f - half_road_edge_width};
    const auto road_right_edge_y_l{start_position.y - road_width / 2.0f + half_road_edge_width};

    for (int i = 0; i < num_samples; ++i) {
        const auto sample_x{sample_length * i};
        road.left_edge.at(i).l.x = sample_x;
        road.left_edge.at(i).l.y = road_left_edge_y_l;
        road.left_edge.at(i).l.z = 0;
        road.left_edge.at(i).r.x = sample_x;
        road.left_edge.at(i).r.y = road_left_edge_y_r;
        road.left_edge.at(i).r.z = 0;

        road.right_edge.at(i).l.x = sample_x;
        road.right_edge.at(i).l.y = road_right_edge_y_l;
        road.right_edge.at(i).l.z = 0;
        road.right_edge.at(i).r.x = sample_x;
        road.right_edge.at(i).r.y = road_right_edge_y_r;
        road.right_edge.at(i).r.z = 0;

        if (!road.lanelets.empty()) {
            for (size_t j = 0; j < road.lanelets.size(); ++j) {
                auto& lanelet{road.lanelets.at(j)};
                const auto lane_offset{(j + 1) * lane_width};
                lanelet.at(i).l.x = sample_x;
                lanelet.at(i).l.y = road_left_edge_y_l - lane_offset;
                lanelet.at(i).l.z = 0;
                lanelet.at(i).r.x = sample_x;
                lanelet.at(i).r.y = lanelet.at(i).l.y - CARLET_LANELET_WIDTH;
                lanelet.at(i).r.z = 0;
            }
            for (int j = 0; j <= static_cast<int>(road.lanelets.size()); ++j) {
                const auto left_lanelet_idx{j - 1};
                const auto right_lanelet_idx{j};

                const auto left_edge{left_lanelet_idx < 0
                    ? road_left_edge_y_l
                    : road.lanelets.at(left_lanelet_idx).at(i).l.y};
                const auto right_edge{right_lanelet_idx == road.lanelets.size()
                    ? road_right_edge_y_r
                    : road.lanelets.at(right_lanelet_idx).at(i).r.y};

                road.lanes.at(j).at(i).width = lane_width;
                road.lanes.at(j).at(i).c.x = sample_x;
                road.lanes.at(j).at(i).c.y = (left_edge + right_edge) / 2.0f;
                road.lanes.at(j).at(i).c.z = 0;
            }
        } else {
            // single lane road
            road.lanes.at(0).at(i).width = lane_width;
            road.lanes.at(0).at(i).c.x = sample_x;
            road.lanes.at(0).at(i).c.y = (road_right_edge_y_l + road_left_edge_y_r) / 2.0f;
            road.lanes.at(0).at(i).c.z = 0;
        }
    }

    return road;
}

bool Simulator::is_running()
{
    return !WindowShouldClose();
}

void Simulator::map_to_mesh_model()
{
    static bool mesh_generated{false};
    if (mesh_generated) return;
    mesh_generated = true;

    int num_lanelets{0};
    map_.road_mesh.vertexCount = 0;
    map_.edge_mesh.vertexCount = 0;
    map_.lanelet_mesh.vertexCount = 0;
    for (const auto& road : map_.road_net.roads) {
        map_.edge_mesh.vertexCount += road.left_edge.size() * 2;
        map_.edge_mesh.vertexCount += road.right_edge.size() * 2;
        map_.road_mesh.vertexCount += max(road.right_edge.size(), road.left_edge.size()) * 2; // road

        for (const auto& lanelet : road.lanelets) {
            map_.lanelet_mesh.vertexCount += lanelet.size() * 2;
            ++num_lanelets;
        }
    }

    map_.road_mesh.triangleCount = map_.road_mesh.vertexCount - 2;
    map_.road_mesh.vertices = (float*)RL_CALLOC(3 * map_.road_mesh.vertexCount, sizeof(float)); // Vertex positions
    map_.road_mesh.normals = (float*)RL_CALLOC(3 * map_.road_mesh.vertexCount, sizeof(float)); // Normals
    map_.road_mesh.indices = (unsigned short*)RL_CALLOC(map_.road_mesh.triangleCount * 3, sizeof(unsigned short)); // Index data
    
    map_.edge_mesh.triangleCount = map_.edge_mesh.vertexCount - (2 * 2);
    map_.edge_mesh.vertices = (float*)RL_CALLOC(3 * map_.edge_mesh.vertexCount, sizeof(float)); // Vertex positions
    map_.edge_mesh.normals = (float*)RL_CALLOC(3 * map_.edge_mesh.vertexCount, sizeof(float)); // Normals
    map_.edge_mesh.indices = (unsigned short*)RL_CALLOC(map_.edge_mesh.triangleCount * 3, sizeof(unsigned short)); // Index data
    
    map_.lanelet_mesh.triangleCount = map_.lanelet_mesh.vertexCount - (2 * num_lanelets);
    map_.lanelet_mesh.vertices = (float*)RL_CALLOC(3 * map_.lanelet_mesh.vertexCount, sizeof(float)); // Vertex positions
    map_.lanelet_mesh.normals = (float*)RL_CALLOC(3 * map_.lanelet_mesh.vertexCount, sizeof(float)); // Normals
    map_.lanelet_mesh.indices = (unsigned short*)RL_CALLOC(map_.lanelet_mesh.triangleCount * 3, sizeof(unsigned short)); // Index data

    int v_idx{0};
    int t_idx{0};
    int num_v_last_line{0};
    for (const auto& road : map_.road_net.roads) {
        // road
        int num_v_this_line{0};
        const size_t num_strip{min(road.left_edge.size(), road.right_edge.size())};
        for (size_t i = 0; i < num_strip; ++i) {
            const auto& left_strip{road.left_edge.at(i)};
            const auto& right_strip{road.right_edge.at(i)};
            map_.road_mesh.vertices[v_idx * 6 + 0] = left_strip.l.x;
            map_.road_mesh.vertices[v_idx * 6 + 1] = left_strip.l.y;
            map_.road_mesh.vertices[v_idx * 6 + 2] = left_strip.l.z;
            to_raylib_mesh3(map_.road_mesh.vertices + v_idx * 6);
            map_.road_mesh.vertices[v_idx * 6 + 3] = right_strip.r.x;
            map_.road_mesh.vertices[v_idx * 6 + 4] = right_strip.r.y;
            map_.road_mesh.vertices[v_idx * 6 + 5] = right_strip.r.z;
            to_raylib_mesh3(map_.road_mesh.vertices + v_idx * 6 + 3);
            map_.road_mesh.normals[v_idx * 6 + 0] = 0.0f;
            map_.road_mesh.normals[v_idx * 6 + 1] = 1.0f;
            map_.road_mesh.normals[v_idx * 6 + 2] = 0.0f;
            map_.road_mesh.normals[v_idx * 6 + 3] = 0.0f;
            map_.road_mesh.normals[v_idx * 6 + 4] = 1.0f;
            map_.road_mesh.normals[v_idx * 6 + 5] = 0.0f;
            num_v_this_line += 2;
            ++v_idx;
        }

        for (size_t i = 0; i < num_v_this_line - 2; i += 2) {
            map_.road_mesh.indices[t_idx * 6 + 0] = i;
            map_.road_mesh.indices[t_idx * 6 + 1] = i + 2;
            map_.road_mesh.indices[t_idx * 6 + 2] = i + 1;
            map_.road_mesh.indices[t_idx * 6 + 3] = i + 1;
            map_.road_mesh.indices[t_idx * 6 + 4] = i + 2;
            map_.road_mesh.indices[t_idx * 6 + 5] = i + 3;
            ++t_idx;
        }

        // left road edge
        num_v_this_line = 0;
        num_v_last_line = 0;
        v_idx = 0;
        t_idx = 0;
        for (const auto& strip: road.left_edge) {
            map_.edge_mesh.vertices[v_idx * 6 + 0] = strip.l.x;
            map_.edge_mesh.vertices[v_idx * 6 + 1] = strip.l.y;
            map_.edge_mesh.vertices[v_idx * 6 + 2] = strip.l.z;
            to_raylib_mesh3(map_.edge_mesh.vertices + v_idx * 6);
            map_.edge_mesh.vertices[v_idx * 6 + 3] = strip.r.x;
            map_.edge_mesh.vertices[v_idx * 6 + 4] = strip.r.y;
            map_.edge_mesh.vertices[v_idx * 6 + 5] = strip.r.z;
            to_raylib_mesh3(map_.edge_mesh.vertices + v_idx * 6 + 3);
            map_.edge_mesh.normals[v_idx * 6 + 0] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 1] = 1.0f;
            map_.edge_mesh.normals[v_idx * 6 + 2] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 3] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 4] = 1.0f;
            map_.edge_mesh.normals[v_idx * 6 + 5] = 0.0f;
            num_v_this_line += 2;
            ++v_idx;
        }

        for (size_t i = 0; i < num_v_this_line - 2; i += 2) {
            map_.edge_mesh.indices[t_idx * 6 + 0] = num_v_last_line + i;
            map_.edge_mesh.indices[t_idx * 6 + 1] = num_v_last_line + i + 2;
            map_.edge_mesh.indices[t_idx * 6 + 2] = num_v_last_line + i + 1;
            map_.edge_mesh.indices[t_idx * 6 + 3] = num_v_last_line + i + 1;
            map_.edge_mesh.indices[t_idx * 6 + 4] = num_v_last_line + i + 2;
            map_.edge_mesh.indices[t_idx * 6 + 5] = num_v_last_line + i + 3;
            ++t_idx;
        }

        // right road edge
        num_v_this_line = 0;
        num_v_last_line = v_idx * 2;
        for (const auto& strip: road.right_edge) {
            map_.edge_mesh.vertices[v_idx * 6 + 0] = strip.l.x;
            map_.edge_mesh.vertices[v_idx * 6 + 1] = strip.l.y;
            map_.edge_mesh.vertices[v_idx * 6 + 2] = strip.l.z;
            to_raylib_mesh3(map_.edge_mesh.vertices + v_idx * 6);
            map_.edge_mesh.vertices[v_idx * 6 + 3] = strip.r.x;
            map_.edge_mesh.vertices[v_idx * 6 + 4] = strip.r.y;
            map_.edge_mesh.vertices[v_idx * 6 + 5] = strip.r.z;
            to_raylib_mesh3(map_.edge_mesh.vertices + v_idx * 6 + 3);
            map_.edge_mesh.normals[v_idx * 6 + 0] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 1] = 1.0f;
            map_.edge_mesh.normals[v_idx * 6 + 2] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 3] = 0.0f;
            map_.edge_mesh.normals[v_idx * 6 + 4] = 1.0f;
            map_.edge_mesh.normals[v_idx * 6 + 5] = 0.0f;
            ++v_idx;
            num_v_this_line += 2;
        }

        for (size_t i = 0; i < num_v_this_line - 2; i += 2) {
            map_.edge_mesh.indices[t_idx * 6 + 0] = num_v_last_line + i;
            map_.edge_mesh.indices[t_idx * 6 + 1] = num_v_last_line + i + 2;
            map_.edge_mesh.indices[t_idx * 6 + 2] = num_v_last_line + i + 1;
            map_.edge_mesh.indices[t_idx * 6 + 3] = num_v_last_line + i + 1;
            map_.edge_mesh.indices[t_idx * 6 + 4] = num_v_last_line + i + 2;
            map_.edge_mesh.indices[t_idx * 6 + 5] = num_v_last_line + i + 3;
            ++t_idx;
        }

        v_idx = 0;
        t_idx = 0;
        for (const auto& lanelet : road.lanelets) {
            num_v_this_line = 0;
            num_v_last_line = v_idx * 2;
            for (const auto& strip: lanelet) {
                map_.lanelet_mesh.vertices[v_idx * 6 + 0] = strip.l.x;
                map_.lanelet_mesh.vertices[v_idx * 6 + 1] = strip.l.y;
                map_.lanelet_mesh.vertices[v_idx * 6 + 2] = strip.l.z;
                to_raylib_mesh3(map_.lanelet_mesh.vertices + v_idx * 6);
                map_.lanelet_mesh.vertices[v_idx * 6 + 3] = strip.r.x;
                map_.lanelet_mesh.vertices[v_idx * 6 + 4] = strip.r.y;
                map_.lanelet_mesh.vertices[v_idx * 6 + 5] = strip.r.z;
                to_raylib_mesh3(map_.lanelet_mesh.vertices + v_idx * 6 + 3);
                map_.lanelet_mesh.normals[v_idx * 6 + 0] = 0.0f;
                map_.lanelet_mesh.normals[v_idx * 6 + 1] = 1.0f;
                map_.lanelet_mesh.normals[v_idx * 6 + 2] = 0.0f;
                map_.lanelet_mesh.normals[v_idx * 6 + 3] = 0.0f;
                map_.lanelet_mesh.normals[v_idx * 6 + 4] = 1.0f;
                map_.lanelet_mesh.normals[v_idx * 6 + 5] = 0.0f;
                ++v_idx;
                num_v_this_line += 2;
            }

            for (size_t i = 0; i < num_v_this_line - 2; i += 2) {
                map_.lanelet_mesh.indices[t_idx * 6 + 0] = num_v_last_line + i;
                map_.lanelet_mesh.indices[t_idx * 6 + 1] = num_v_last_line + i + 2;
                map_.lanelet_mesh.indices[t_idx * 6 + 2] = num_v_last_line + i + 1;
                map_.lanelet_mesh.indices[t_idx * 6 + 3] = num_v_last_line + i + 1;
                map_.lanelet_mesh.indices[t_idx * 6 + 4] = num_v_last_line + i + 2;
                map_.lanelet_mesh.indices[t_idx * 6 + 5] = num_v_last_line + i + 3;
                ++t_idx;
            }
        }
    }

    UploadMesh(&map_.road_mesh, false);
    UploadMesh(&map_.edge_mesh, false);
    UploadMesh(&map_.lanelet_mesh, false);
    map_.road_model = LoadModelFromMesh(map_.road_mesh);
    map_.edge_model = LoadModelFromMesh(map_.edge_mesh);
    map_.lanelet_model = LoadModelFromMesh(map_.lanelet_mesh);
}

Simulator::Simulator()
{
    InitWindow(CARLET_WIN_WIDTH, CARLET_WIN_HEIGHT, CARLET_WIN_TITLE);
    SetTargetFPS(CARLET_TARGET_FPS);

    camera_.position    = {0.0f, 5.0f, 0.0f};
    camera_.target      = {0.0f, 0.0f, 0.0f};
    camera_.up          = {0.0f, 1.0f, 0.0f};
    camera_.fovy        = 45.0f;
    camera_.projection  = CAMERA_PERSPECTIVE;
}

void Simulator::update_camera()
{
    camera_.fovy += GetMouseWheelMove() * -5;
    camera_.fovy = clamp(camera_.fovy, 0.0f, 170.0f);

    static int last_left_down_x{-1};
    static int last_left_down_y{-1};
    static int last_right_down_x{-1};
    static int last_right_down_y{-1};

    static Vector3 camera_pos_offset{camera_.position};
    static Vector3 camera_target_offset{camera_.target};

    if (!ctrl_vehs_.empty()) {
        // camera follow this first controllable car    
        const auto veh{ctrl_vehs_.at(0)};
        const Vector3 veh_position{.x = veh.state().x, .y = veh.state().y, .z = veh.state().z};
        camera_.target.z = camera_target_offset.z + -veh_position.x - 80;
        camera_.target.x = camera_target_offset.x;
        camera_.target.y = camera_target_offset.y;
        camera_.position.z = camera_pos_offset.z - veh_position.y + 20;
        camera_.position.x = camera_pos_offset.x;
        camera_.position.y = camera_pos_offset.y;
    } else {
        camera_.target.z = camera_target_offset.z;
        camera_.target.x = camera_target_offset.x;
        camera_.target.y = camera_target_offset.y;
        camera_.position.z = camera_pos_offset.z;
        camera_.position.x = camera_pos_offset.x;
        camera_.position.y = camera_pos_offset.y;
    }

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        last_left_down_x = -1;
        last_left_down_y = -1;
    } else if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        int this_left_down_x{GetMouseX()};
        int this_left_down_y{GetMouseY()};
        if (last_left_down_x > 0 || last_left_down_y > 0) {
            const auto delta_x{this_left_down_x - last_left_down_x};
            const auto delta_y{this_left_down_y - last_left_down_y};
            camera_pos_offset.x -= delta_x * 0.03f;
            camera_pos_offset.z -= delta_y * 0.03;
            camera_target_offset.x -= delta_x * 0.03f;
            camera_target_offset.z -= delta_y * 0.03;
        }
        last_left_down_x = this_left_down_x;
        last_left_down_y = this_left_down_y;
    } else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        last_right_down_x = -1;
        last_right_down_y = -1;
    } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        int this_right_down_x{GetMouseX()};
        int this_right_down_y{GetMouseY()};
        if (last_right_down_x > 0 || last_right_down_y > 0) {
            const auto delta_x{this_right_down_x - last_right_down_x};
            const auto delta_y{this_right_down_y - last_right_down_y};
            camera_target_offset.y += delta_y * 0.5f;
            camera_target_offset.x -= delta_x * 0.1f;
        }
        last_right_down_x = this_right_down_x;
        last_right_down_y = this_right_down_y;
    }
    UpdateCamera(&camera_, CAMERA_CUSTOM);
}

void Simulator::render()
{
    update_camera();

    static const Vector3 zero_vec{};
    BeginDrawing();
        ClearBackground(GRAY);
        BeginMode3D(camera_);
            {
                // draw map
                map_to_mesh_model();
                DrawModel(map_.road_model, zero_vec, 1.0f, DARKGRAY);
                DrawModel(map_.edge_model, zero_vec, 1.0f, BLACK);
                DrawModel(map_.lanelet_model, zero_vec, 1.0f, YELLOW);
            }
            {
                // draw vehicle
                for (const auto& veh: ctrl_vehs_) {
                    DrawModel(veh.model, zero_vec, 1.0f, veh.color);
                    DrawModelWires(veh.model, zero_vec, 1.0f, WHITE);
                }
            }
            // DrawGrid(10, 1.0f);
        EndMode3D();

        // Draw in 2d space
        {
            // Show veh speed
            if (!ctrl_vehs_.empty()) {
                const auto& veh{ctrl_vehs_.at(0)};
                constexpr auto font_size{40};
                constexpr auto buf_size{32};
                constexpr auto str_len{8};   // I guess
                constexpr auto str_position_x{CARLET_WIN_WIDTH / 2 - (str_len / 2) * font_size / 2};
                char buf[buf_size];
                snprintf(buf, buf_size, "%.1f KM/H", mps_to_kmph(veh.state().vel));
                DrawText(buf, str_position_x, 10, font_size, RED);
            }
        }
    EndDrawing();
}

int Simulator::create_ctrl_veh(const VehModel& model)
{
    ControllableVeh veh{0.0f, 0.0f, 0.0f, model};
    veh.color = random_color();
    auto mesh{GenMeshCube(CARLET_DEF_VEH_WIDTH, CARLET_DEF_VEH_HEIGHT, CARLET_DEF_VEH_LEN)};
    UploadMesh(&mesh, true);
    veh.model = LoadModelFromMesh(mesh);

    ctrl_vehs_.push_back(veh);
    return ctrl_vehs_.size() - 1;
}

ControllableVeh& Simulator::get_ctrl_veh(int id)
{
    assert(id >= 0 || id < ctrl_vehs_.size());
    return ctrl_vehs_.at(id);
}

void Simulator::gen_random_vehs(int n)
{
}

bool Simulator::step(float dt)
{
    for (auto& veh: ctrl_vehs_) {
        veh.step(dt);
    }
    return true;
}

void ControllableVeh::act(float steer, float accel)
{
    this->steer = steer;
    this->accel = accel;
}

bool ControllableVeh::step(float dt)
{
    dynamic.act(steer, accel, dt);
    // setup render model
    model.transform = MatrixRotateY(state().yaw);
    model.transform.m12 = -state().y;
    model.transform.m13 = state().z;
    model.transform.m14 = -state().x;
    return true;
}

void BicycleModel::act(float steer, float accel, float dt)
{
    state_.steer_angle = max(min(steer, vm_.max_steer), vm_.min_steer);
    state_.vel += accel * dt;

    const auto slip_angle{std::atan2(vm_.gc_to_back_axle * std::tan(state_.steer_angle), vm_.wheel_base)};
    const auto vel_angle{slip_angle + state_.yaw};
    const auto dy{state_.vel * std::sin(vel_angle)};
    const auto dx{state_.vel * std::cos(vel_angle)};
    const auto r{vm_.wheel_base / (std::tan(state_.steer_angle) * std::cos(slip_angle) + CARLET_EPSf)};
    state_.x += dx * dt;
    state_.y += dy * dt;
    state_.yaw_rate = state_.vel / r;
    state_.yaw += state_.yaw_rate * dt;
}

} // namespace carlet

#endif // CARLET_CPP_
#endif // CARLET_IMPLEMENTATION
