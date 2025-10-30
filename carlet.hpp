#ifndef CARLET_HPP_
#define CARLET_HPP_

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

class Simulator;

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

    static Road gen_straight(const Vector3& start_position, const Vector3& end_position, int num_lane, float lane_width);
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
        double x;
        double y;
        double z;
        double vel;
        double accel;
        double jerk;
        double yaw;
        double yaw_rate;
        double steer_angle;

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
    Object();

    const int id;
    Vector3 shape;
    Model model;
    Color color;
private:
    friend class Simulator;
    virtual bool step(float dt) { return true; }
}; // struct Object

struct Veh: public Object
{
    Veh(float init_x, float init_y, float init_vel, const VehModel& model)
        : Object()
        , steer(0.0f)
        , accel(0.0f)
        , dynamic(BicycleModel::State::init_with(init_x, init_y, init_vel), model)
        {}

    inline const BicycleModel::State& state() const { return dynamic.state(); }

    float steer;
    float accel;
    BicycleModel dynamic;
private:
    friend class Simulator;
    virtual bool step(float dt) override;
}; // struct Veh

struct ControllableVeh : public Veh
{
    ControllableVeh(float init_x, float init_y, float init_vel, const VehModel& model)
        : Veh(init_x, init_y, init_vel, model) {}
    void act(float steer, float accel);
}; // struct ControllableVehicle

struct IDMVeh : public Veh
{
    IDMVeh(float init_x, float init_y, float init_vel, const VehModel& model)
        : Veh(init_x, init_y, init_vel, model) {}
}; // struct IDMVeh

class Simulator
{
    CARLET_DEF_SINGLETON(Simulator)
public:
    ~Simulator() = default;
    bool is_running();
    bool step(float dt);
    void render();

    int create_ctrl_veh(const VehModel& model);
    ControllableVeh& get_ctrl_veh(int idx);
    void gen_random_vehs(int n);
    inline Map& map() { return map_; }
private:
    Simulator();
    void map_to_mesh_model();
    void update_camera();
    bool collision_with_any_veh(const Veh& veh);

    Map map_;
    std::vector<IDMVeh> idm_vehs_;
    std::vector<ControllableVeh> ctrl_vehs_;
    Camera3D camera_;
}; // class Simulator

namespace veh_model {

extern const VehModel tesla;

}; // namespace veh_model

} // namespace carlet

#endif // CARLET_HPP_


#define CARLET_IMPLEMENTATION // delete me


#ifdef CARLET_IMPLEMENTATION
#ifndef CARLET_CPP_
#define CARLET_CPP_

#include <cassert>
#include <rlgl.h>
#include <raymath.h>

#ifdef _GLIBCXX_OSTREAM
#   include <iomanip>
#endif // _GLIBCXX_OSTREAM


#ifndef CARLET_EPSf
#   include <limits>
#   define CARLET_EPSf              std::numeric_limits<float>::epsilon()
#endif // CARLET_EPSf

#ifndef CARLET_EPS
#   include <limits>
#   define CARLET_EPS               std::numeric_limits<double>::epsilon()
#endif // CARLET_EPS

#ifndef CARLET_MAX_ID
#   define CARLET_MAX_ID            100000
#endif // CARLET_MAX_ID

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
#   define CARLET_ROAD_EDGE_WIDTH   0.1f   // meter, which is 10cm
#endif // CARLET_ROAD_EDGE_WIDTH

#ifndef CARLET_LANELET_WIDTH
#   define CARLET_LANELET_WIDTH     0.1f   // meter, which is 10cm
#endif // CARLET_LANELET_WIDTH

#ifndef CARLET_DEF_VEH_LEN
#   define CARLET_DEF_VEH_LEN       4.2f    // meter
#endif // CARLET_DEF_VEH_LEN

#ifndef CARLET_DEF_VEH_WIDTH
#   define CARLET_DEF_VEH_WIDTH     1.98f   // meter
#endif // CARLET_DEF_VEH_WIDTH

#ifndef CARLET_DEF_VEH_HEIGHT
#   define CARLET_DEF_VEH_HEIGHT    1.6f    // meter
#endif // CARLET_DEF_VEH_HEIGHT

#define CARLET_ARR_LEN(arr)         (sizeof(arr) / sizeof((arr)[0]))


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

namespace carlet {

template<typename T>
inline constexpr T min(T a, T b) { return a > b ? b : a; }

template<typename T>
inline constexpr T max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline constexpr T clamp(T v, T low, T high) { return v > high ? high : (v < low ? low : v); }

template<typename T>
inline constexpr T mps_to_kmph(T mps) { return mps * static_cast<T>(3.6); }

template<typename T>
inline constexpr T kmph_to_mps(T kmph) { return kmph / static_cast<T>(3.6); }

template<typename T>
inline constexpr T rad_to_deg(T rad) { return rad / M_PI * 180.0; }

template<typename T>
inline constexpr T deg_to_rad(T deg) { return deg / 180.0 * M_PI; }

bool check_veh_collision(const Veh& a, const Veh& b);

template<typename T>
inline T rand_ab(T a, T b)
{
    assert(b > a);
    const auto rv01{static_cast<float>(rand()) / static_cast<float>(RAND_MAX)};
    return rv01 * (b - a) + a;
}

namespace veh_model {

const VehModel tesla {
    .wheel_base = 2.8f,
    .gc_to_back_axle = 1.5f,
    .max_steer = deg_to_rad(25.0f),
    .min_steer = deg_to_rad(-25.0f),
}; // tesla

static const VehModel all_veh_models[] {
    tesla
};

inline const VehModel& random() {
    constexpr auto num_models{CARLET_ARR_LEN(all_veh_models)};
    return all_veh_models[rand() % num_models];
}

}; // namespace veh_model

static const Color veh_colors[] {
    YELLOW,  GOLD,    ORANGE,     PINK,    RED,   MAROON,
    GREEN,   LIME,    DARKGREEN,  SKYBLUE, BLUE,  DARKBLUE,
    PURPLE,  VIOLET,  DARKPURPLE, BEIGE,   BROWN, DARKBROWN,
    MAGENTA, RAYWHITE
};

inline const Color& random_color()
{
    constexpr auto num_colors{CARLET_ARR_LEN(veh_colors)};
    return veh_colors[rand() % num_colors];
}

inline const Color& next_color()
{
    constexpr auto num_colors{CARLET_ARR_LEN(veh_colors)};

    static int i{0};
    static std::mutex lock{};
    std::lock_guard<std::mutex> guard{lock};
    return veh_colors[i++ % num_colors];
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

inline int gen_id()
{
    static int cnt{0};
    static std::mutex lock{};

    std::lock_guard<std::mutex> guard{lock};
    int result{cnt};
    cnt = (cnt + 1) % CARLET_MAX_ID;
    return result;
}

Object::Object() : id(gen_id()) {}

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

Road Road::gen_straight(const Vector3& start_position, const Vector3& end_position, int num_lane, float lane_width)
{
    constexpr auto min_lane_width{2.0f};    // meter
    constexpr auto sample_length{1.0f};     // meter
    constexpr auto half_road_edge_width{CARLET_ROAD_EDGE_WIDTH / 2.0f};

    const auto length{Vector3Distance(start_position, end_position)};
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

    auto calc_center_line{[&start_position, &end_position, length] (float s, Vector3& center_point) -> void {
        center_point.x = (end_position.x - start_position.x) * s / length + start_position.x;
        center_point.y = (end_position.y - start_position.y) * s / length + start_position.y;
        center_point.z = (end_position.z - start_position.z) * s / length + start_position.z;
    }};

    Vector3 sample_center_point{};
    const auto road_heading{std::atan2(end_position.y - start_position.y, end_position.x - start_position.x)};
    const auto cos_road_heading{std::cos(road_heading)};
    const auto sin_road_heading{std::sin(road_heading)};

    for (int i = 0; i < num_samples; ++i) {
        const auto sample_s{sample_length * i};
        calc_center_line(sample_s, sample_center_point);

        const auto left_edge_offset_r{road_width / 2.0f - half_road_edge_width};
        const auto left_edge_offset_l{road_width / 2.0f + half_road_edge_width};
        const auto right_edge_offset_r{-road_width / 2.0f - half_road_edge_width};
        const auto right_edge_offset_l{-road_width / 2.0f + half_road_edge_width};

        road.left_edge.at(i).l.x = sample_center_point.x + left_edge_offset_l * sin_road_heading;
        road.left_edge.at(i).l.y = sample_center_point.y + left_edge_offset_l * cos_road_heading;
        road.left_edge.at(i).l.z = sample_center_point.z;
        road.left_edge.at(i).r.x = sample_center_point.x + left_edge_offset_r * sin_road_heading;
        road.left_edge.at(i).r.y = sample_center_point.y + left_edge_offset_r * cos_road_heading;
        road.left_edge.at(i).r.z = sample_center_point.z;

        road.right_edge.at(i).l.x = sample_center_point.x + right_edge_offset_l * sin_road_heading;
        road.right_edge.at(i).l.y = sample_center_point.y + right_edge_offset_l * cos_road_heading;
        road.right_edge.at(i).l.z = sample_center_point.z;
        road.right_edge.at(i).r.x = sample_center_point.x + right_edge_offset_r * sin_road_heading;
        road.right_edge.at(i).r.y = sample_center_point.y + right_edge_offset_r * cos_road_heading;
        road.right_edge.at(i).r.z = sample_center_point.z;

        if (!road.lanelets.empty()) {
            for (size_t j = 0; j < road.lanelets.size(); ++j) {
                auto& lanelet{road.lanelets.at(j)};
                const auto left_lanelet_offset{left_edge_offset_l - (j + 1) * lane_width};
                lanelet.at(i).l.x = sample_center_point.x + left_lanelet_offset * sin_road_heading;
                lanelet.at(i).l.y = sample_center_point.y + left_lanelet_offset * cos_road_heading;
                lanelet.at(i).l.z = sample_center_point.z;
                const auto right_lanelet_offset{lanelet.at(i).l.y - CARLET_LANELET_WIDTH};
                lanelet.at(i).r.x = sample_center_point.x + right_lanelet_offset * sin_road_heading;
                lanelet.at(i).r.y = sample_center_point.y + right_lanelet_offset * cos_road_heading;
                lanelet.at(i).r.z = sample_center_point.z;
            }

            for (int j = 0; j <= static_cast<int>(road.lanelets.size()); ++j) {
                const auto left_lanelet_idx{j - 1};
                const auto right_lanelet_idx{j};

                const auto& left_edge_point{left_lanelet_idx < 0
                    ? road.left_edge.at(i).l
                    : road.lanelets.at(left_lanelet_idx).at(i).l};
                const auto& right_edge_point{right_lanelet_idx == road.lanelets.size()
                    ? road.right_edge.at(i).r
                    : road.lanelets.at(right_lanelet_idx).at(i).r};

                road.lanes.at(j).at(i).width = lane_width;
                road.lanes.at(j).at(i).c.x = (left_edge_point.x + right_edge_point.x) / 2.0f;
                road.lanes.at(j).at(i).c.y = (left_edge_point.y + right_edge_point.y) / 2.0f;
                road.lanes.at(j).at(i).c.z = sample_center_point.z;
            }
        } else {
            // single lane road
            road.lanes.at(0).at(i).width = lane_width;
            road.lanes.at(0).at(i).c.x = sample_center_point.x;
            road.lanes.at(0).at(i).c.y = sample_center_point.y;
            road.lanes.at(0).at(i).c.z = sample_center_point.z;
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

    camera_.position    = {.x=0.0f, .y=5.0f, .z=0.0f};
    camera_.target      = {.x=0.0f, .y=0.0f, .z=0.0f};
    camera_.up          = {.x=0.0f, .y=1.0f, .z=0.0f};
    camera_.fovy        = 45.0f;
    camera_.projection  = CAMERA_PERSPECTIVE;
}

void Simulator::update_camera()
{
    camera_.fovy += GetMouseWheelMove() * -5.0f;
    camera_.fovy = clamp(camera_.fovy, 0.0f, 170.0f);

    static int last_left_down_x{-1};
    static int last_left_down_y{-1};
    static int last_right_down_x{-1};
    static int last_right_down_y{-1};

    static Vector3 camera_pos_offset{camera_.position};
    static Vector3 camera_target_offset{camera_.target};

    if (!ctrl_vehs_.empty()) {
        // camera follow this first controllable car    
        const auto& veh{ctrl_vehs_.at(0)};
        camera_.target.z = camera_target_offset.z - veh.state().x - 80;
        camera_.target.x = camera_target_offset.x;
        camera_.target.y = camera_target_offset.y;
        camera_.position.z = camera_pos_offset.z - veh.state().x + 20;
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

                for (const auto& veh: idm_vehs_) {
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
    veh.shape.x = CARLET_DEF_VEH_LEN;
    veh.shape.y = CARLET_DEF_VEH_WIDTH;
    veh.shape.z = CARLET_DEF_VEH_HEIGHT;
    auto mesh{GenMeshCube(CARLET_DEF_VEH_WIDTH, CARLET_DEF_VEH_HEIGHT, CARLET_DEF_VEH_LEN)};
    UploadMesh(&mesh, true);
    veh.model = LoadModelFromMesh(mesh);
    veh.color = next_color();

    ctrl_vehs_.push_back(veh);
    return ctrl_vehs_.size() - 1;
}

ControllableVeh& Simulator::get_ctrl_veh(int idx)
{
    assert(idx >= 0 || idx < ctrl_vehs_.size());
    return ctrl_vehs_.at(idx);
}

bool Simulator::collision_with_any_veh(const Veh& veh)
{
    for (const auto& other_veh: idm_vehs_) {
        if (check_veh_collision(veh, other_veh)) {
            return true;
        }
    }
    for (const auto& other_veh: ctrl_vehs_) {
        if (check_veh_collision(veh, other_veh)) {
            return true;
        }
    }
    return false;
}

void Simulator::gen_random_vehs(int n)
{
    constexpr auto min_spd{kmph_to_mps(20.0f /* kmph */)};  // mps
    constexpr auto max_spd{kmph_to_mps(80.0f /* kmph */)};  // mps

    const auto& roads{map_.road_net.roads};
    const auto num_roads{roads.size()};
    const auto num_veh_each_road{n / num_roads};

    for (const auto& road: roads) {
        const auto num_lane{road.lanes.size()};
        for (int i = 0; i < num_veh_each_road;) {
            const auto lane_idx{rand_ab(0ul, num_lane)};
            const auto& target_lane{road.lanes.at(lane_idx)};
            const auto sample_idx{rand_ab(0ul, target_lane.size())};
            const auto& waypoing{target_lane.at(sample_idx)};
            const auto veh_vel{rand_ab(min_spd, max_spd)};

            IDMVeh veh{waypoing.c.x, waypoing.c.y, veh_vel, veh_model::random()};
            veh.shape.x = CARLET_DEF_VEH_LEN;
            veh.shape.y = CARLET_DEF_VEH_WIDTH;
            veh.shape.z = CARLET_DEF_VEH_HEIGHT;

            // collision check
            if (collision_with_any_veh(veh)) continue;

            // properties for rendering
            auto mesh{GenMeshCube(CARLET_DEF_VEH_WIDTH, CARLET_DEF_VEH_HEIGHT, CARLET_DEF_VEH_LEN)};
            UploadMesh(&mesh, true);
            veh.color = next_color();
            veh.model = LoadModelFromMesh(mesh);
            idm_vehs_.push_back(veh);
            ++i;
        }
    }
}

bool Simulator::step(float dt)
{
    for (auto& veh: ctrl_vehs_) {
        veh.step(dt);
    }
    for (auto& veh: idm_vehs_) {
        veh.step(dt);
    }
    return true;
}

void ControllableVeh::act(float steer, float accel)
{
    this->steer = steer;
    this->accel = accel;
}

bool Veh::step(float dt)
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
    const auto r{vm_.wheel_base / (std::tan(state_.steer_angle) * std::cos(slip_angle) + CARLET_EPS)};
    state_.x += dx * dt;
    state_.y += dy * dt;
    state_.yaw_rate = state_.vel / r;
    state_.yaw += state_.yaw_rate * dt;
}

bool check_veh_collision(const Veh& a, const Veh& b)
{
    constexpr float min_veh_space{0.5f}; // meter

    if (a.id == b.id) return false;

    const auto a_yaw{a.state().yaw};
    const auto a_hw{a.shape.y / 2.0f}; // half width
    const auto a_hl{a.shape.x / 2.0f}; // half length
    const Vector2 a_center{.x=static_cast<float>(a.state().x), .y=static_cast<float>(a.state().y)};

    const auto b_yaw{b.state().yaw};
    const auto b_hw{b.shape.y / 2.0f}; // half width
    const auto b_hl{b.shape.x / 2.0f}; // half length
    const Vector2 b_center{.x=static_cast<float>(b.state().x), .y=static_cast<float>(b.state().y)};

    // quick collision check (naive solution)
    const auto ab_dist{Vector2Distance(a_center, b_center)};
    if (ab_dist > a_hl + b_hl + min_veh_space) {
        return false;
    }
    if (ab_dist < a_hw + b_hw + min_veh_space) {
        return true;
    }

    // 2d AABB check
    const auto a_sin_yaw{std::sin(a_yaw)};
    const auto a_cos_yaw{std::cos(a_yaw)};
    const auto a_aabb_tl_x{a_hl * a_cos_yaw + a_hw * a_sin_yaw + a_center.x};
    const auto a_aabb_tl_y{a_hw * a_cos_yaw + a_hl * a_sin_yaw + a_center.y};
    const auto a_aabb_tr_x{a_hl * a_cos_yaw - a_hw * a_sin_yaw + a_center.x};
    const auto a_aabb_tr_y{-a_hw * a_cos_yaw + a_hl * a_sin_yaw + a_center.y};
    const auto a_aabb_bl_x{-a_hl * a_cos_yaw + a_hw * a_sin_yaw + a_center.x};
    const auto a_aabb_bl_y{a_hw * a_cos_yaw - a_hl * a_sin_yaw + a_center.y};
    const auto a_aabb_br_x{-a_hl * a_cos_yaw - a_hw * a_sin_yaw + a_center.x};
    const auto a_aabb_br_y{-a_hw * a_cos_yaw - a_hl * a_sin_yaw + a_center.y};

    const auto b_sin_yaw{std::sin(b_yaw)};
    const auto b_cos_yaw{std::cos(b_yaw)};
    const auto b_aabb_tl_x{b_hl * b_cos_yaw + b_hw * b_sin_yaw + b_center.x};
    const auto b_aabb_tl_y{b_hw * b_cos_yaw + b_hl * b_sin_yaw + b_center.y};
    const auto b_aabb_tr_x{b_hl * b_cos_yaw - b_hw * b_sin_yaw + b_center.x};
    const auto b_aabb_tr_y{-b_hw * b_cos_yaw + b_hl * b_sin_yaw + b_center.y};
    const auto b_aabb_bl_x{-b_hl * b_cos_yaw + b_hw * b_sin_yaw + b_center.x};
    const auto b_aabb_bl_y{b_hw * b_cos_yaw - b_hl * b_sin_yaw + b_center.y};
    const auto b_aabb_br_x{-b_hl * b_cos_yaw - b_hw * b_sin_yaw + b_center.x};
    const auto b_aabb_br_y{-b_hw * b_cos_yaw - b_hl * b_sin_yaw + b_center.y};

    const auto a_max_x{max(max(a_aabb_tl_x, a_aabb_tr_x), max(a_aabb_bl_x, a_aabb_br_x))};
    const auto b_max_x{max(max(b_aabb_tl_x, b_aabb_tr_x), max(b_aabb_bl_x, b_aabb_br_x))};
    const auto a_min_x{min(min(a_aabb_tl_x, a_aabb_tr_x), min(a_aabb_bl_x, a_aabb_br_x))};
    const auto b_min_x{min(min(b_aabb_tl_x, b_aabb_tr_x), min(b_aabb_bl_x, b_aabb_br_x))};

    const auto a_max_y{max(max(a_aabb_tl_y, a_aabb_tr_y), max(a_aabb_bl_y, a_aabb_br_y))};
    const auto b_max_y{max(max(b_aabb_tl_y, b_aabb_tr_y), max(b_aabb_bl_y, b_aabb_br_y))};
    const auto a_min_y{min(min(a_aabb_tl_y, a_aabb_tr_y), min(a_aabb_bl_y, a_aabb_br_y))};
    const auto b_min_y{min(min(b_aabb_tl_y, b_aabb_tr_y), min(b_aabb_bl_y, b_aabb_br_y))};

    if (min(a_max_x, a_max_x) > max(a_min_x, a_min_x) &&
        min(a_max_y, a_max_y) > max(a_min_y, a_min_y)) {
        return true;
    }

    return false;
}

} // namespace carlet

#endif // CARLET_CPP_
#endif // CARLET_IMPLEMENTATION
