#ifndef CARLET_HPP_
#define CARLET_HPP_

#include <mutex>
#include <vector>
#include <raylib.h>


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
        Vector3 l;
        Vector3 r;
    }; // struct Strip
    using Lanelet = std::vector<Strip>;
    using RoadEdge = std::vector<Strip>;

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

struct Object
{
    Model model;
    Color color;
    Matrix transform;
    virtual bool step(float dt) { return true; }
}; // struct Object

struct Veh: public Object
{
    float spd;
    float accel;
    float jerk;
    float yaw_rate;
}; // struct Veh

struct ControllableVeh : public Veh
{
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

    int create_ctrl_veh();
    ControllableVeh& get_ctrl_veh(int id);
    void create_random_vehs(int n);
    inline Map& map() { return map_; }
private:
    Simulator();
    void map_to_mesh_model();

    Map map_;
    // std::vector<SelfDrivingVeh> sd_vehs_;
    std::vector<ControllableVeh> ctrl_vehs_;
    Camera3D camera_;
}; // class Simulator

} // namespace carlet

#endif // CARLET_HPP_


// #define CARLET_IMPLEMENTATION delete me

#ifdef CARLET_IMPLEMENTATION
#ifndef CARLET_CPP_
#define CARLET_CPP_

#include <cassert>
#include <rlgl.h>
#include <raymath.h>

namespace carlet {

static const Color veh_colors[] {
    YELLOW,  GOLD,    ORANGE,     PINK,    RED,   MAROON,
    GREEN,   LIME,    DARKGREEN,  SKYBLUE, BLUE,  DARKBLUE,
    PURPLE,  VIOLET,  DARKPURPLE, BEIGE,   BROWN, DARKBROWN,
    MAGENTA, RAYWHITE
};

template<typename T>
inline T min(T a, T b) { return a > b ? b : a; }

template<typename T>
inline T max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline T clamp(T v, T low, T high) { return v > high ? high : v < low ? low : v; }

template<typename T>
inline T mps_to_kmph(T mps) { return mps * static_cast<T>(3.6); }

template<typename T>
inline T kmph_to_mps(T kmph) { return kmph / static_cast<T>(3.6); }

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

    constexpr auto half_road_edge_width{CARLET_ROAD_EDGE_WIDTH / 2.0f};
    const auto road_left_edge_y_e{start_position.y + road_width / 2.0f - half_road_edge_width};
    const auto road_left_edge_y_s{start_position.y + road_width / 2.0f + half_road_edge_width};
    const auto road_right_edge_y_e{start_position.y - road_width / 2.0f - half_road_edge_width};
    const auto road_right_edge_y_s{start_position.y - road_width / 2.0f + half_road_edge_width};
    for (int i = 0; i < num_samples; ++i) {
        const auto sample_x{sample_length * i};
        road.left_edge.at(i).l.x = sample_x;
        road.left_edge.at(i).l.y = road_left_edge_y_s;
        road.left_edge.at(i).l.z = 0;
        road.left_edge.at(i).r.x = sample_x;
        road.left_edge.at(i).r.y = road_left_edge_y_e;
        road.left_edge.at(i).r.z = 0;

        road.right_edge.at(i).l.x = sample_x;
        road.right_edge.at(i).l.y = road_right_edge_y_s;
        road.right_edge.at(i).l.z = 0;
        road.right_edge.at(i).r.x = sample_x;
        road.right_edge.at(i).r.y = road_right_edge_y_e;
        road.right_edge.at(i).r.z = 0;

        if (!road.lanelets.empty()) {
            for (size_t j = 0; j < road.lanelets.size(); ++j) {
                auto& lanelet{road.lanelets.at(j)};
                const auto lane_offset{(j + 1) * lane_width};
                lanelet.at(i).l.x = sample_x;
                lanelet.at(i).l.y = road_left_edge_y_e - lane_offset;
                lanelet.at(i).l.z = 0;
                lanelet.at(i).r.x = sample_x;
                lanelet.at(i).r.y = road_left_edge_y_e - lane_offset - CARLET_LANELET_WIDTH;
                lanelet.at(i).r.z = 0;
            }
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

void Simulator::render()
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
        camera_.target.z = camera_target_offset.z + -veh.transform.m12 - 80;
        camera_.target.x = camera_target_offset.x;
        camera_.target.y = camera_target_offset.y;
        camera_.position.z = camera_pos_offset.z - veh.transform.m12 + 20;
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
                snprintf(buf, buf_size, "%.1f KM/H", mps_to_kmph(veh.spd));
                DrawText(buf, str_position_x, 10, font_size, RED);
            }
        }
    EndDrawing();
}

int Simulator::create_ctrl_veh()
{
    ControllableVeh veh{};
    veh.color = random_color();
    auto mesh{GenMeshCube(CARLET_DEF_VEH_WIDTH, CARLET_DEF_VEH_HEIGHT, CARLET_DEF_VEH_LEN)};
    UploadMesh(&mesh, true);
    veh.model = LoadModelFromMesh(mesh);
    veh.transform.m12 = 0.0f;
    veh.transform.m13 = 0.0f;
    veh.transform.m14 = CARLET_DEF_VEH_HEIGHT / 2.0f;

    ctrl_vehs_.push_back(veh);
    return ctrl_vehs_.size() - 1;
}

ControllableVeh& Simulator::get_ctrl_veh(int id)
{
    assert(id >= 0 || id < ctrl_vehs_.size());
    return ctrl_vehs_.at(id);
}

void Simulator::create_random_vehs(int n)
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
    this->accel = accel;
}

bool ControllableVeh::step(float dt)
{
    spd += accel * dt;
    transform.m12 += spd * dt;

    // setup model
    model.transform.m12 = -transform.m13;
    model.transform.m13 = transform.m14;
    model.transform.m14 = -transform.m12;
    return true;
}

#ifdef _GLIBCXX_OSTREAM
std::ostream& operator<<(std::ostream& os, const Vector2& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Vector3& vec)
{
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}
#endif // _GLIBCXX_OSTREAM

} // namespace carlet

#endif // CARLET_CPP_
#endif // CARLET_IMPLEMENTATION