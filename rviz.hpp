#if 0 // examples:

// 01-draw_pose.cpp

// should link with `raylib`

#define RVIZ_IMPLEMENTATION
#include "rviz.hpp"


int main()
{
    auto rviz{rviz::Viz::instance()};

    const rviz::Pose base{};
    rviz->draw_pose("base", base);

    rviz::Pose cam{};
    cam.x = 5.0f;
    cam.pitch = 3.14f;
    rviz->draw_pose("cam", cam);


    while (!rviz->closed()) {
        rviz->render();
    }

    return 0;
}
#endif // examples


#ifndef RVIZ_HPP_
#define RVIZ_HPP_

#include <vector>
#include <mutex>
#include <unordered_map>
#include <raylib.h>

#define RVIZ_DEF_SINGLETON(classname)                         \
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

namespace rviz {

using Point2f = Vector2;
using Point3f = Vector3;
using Point4f = Vector4;

struct PointI
{
    float x;
    float y;
    float z;
    float i;
}; // struct PointI

struct PointRGB
{
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
}; // struct PointRGB

struct Pose
{
    float x;
    float y;
    float z;
    // rotation by z-y-x
    float yaw;
    float roll;
    float pitch;
}; // struct Pose

struct GridMap2d
{
    float range_x[2];
    float range_y[2];
    int row;
    int col;
    std::vector<int> occupied_grid;
}; // struct GridMap2d

struct VehState2d
{
    float x;
    float y;
    float heading;
    float steer_angle;
    // vehicle parameter
    float wheel_base;
    float wheel_track;
    float wheel_radius;
    float wheel_width;
}; // struct VehState2d

using PointICloud = std::vector<PointI>;
using PointRGBCloud = std::vector<PointRGB>;


class Viz
{
    RVIZ_DEF_SINGLETON(Viz)

    enum DrawableType
    {
        DT_UNKNOWN = 0,
        DT_MESH_MODEL,
        DT_POSE,
        DT_MAP,
        DT_VEHICLE,
        DT_TRJ,
    }; // enum DrawableType

    struct DrawablePose
    {
        Vector3 start;
        Vector3 x_end;
        Vector3 y_end;
        Vector3 z_end;
    }; // struct DrawablePose

    struct DrawableMap
    {
        float range_x[2];
        float range_y[2];
        int row;
        int col;
        std::vector<int> occupied_grid;
    }; // struct DrawableMap

    struct DrawablePointcloudModel
    {
        Mesh mesh;
        Model model;
    }; // struct DrawablePointcloudModel

    struct DrawableTrj2d
    {
        std::vector<Vector2> points;
    }; // struct DrawableTrj2d

    struct DrawableVehicleModel {
        Vector3 position;
        Model front_axle;
        Model rear_axle;
        Model chassis;
        Model fr_wheel;
        Model fl_wheel;
        Model rr_wheel;
        Model rl_wheel;
    }; // struct VehicleModel

    struct Drawable
    {
        union {
            DrawablePointcloudModel* pointcloud_model;
            DrawableVehicleModel* vehicle_model;
            DrawablePose* pose;
            DrawableMap* map;
            DrawableTrj2d* trj2d;
        };
        DrawableType type;
        std::mutex lock;
    }; // struct Drawable
public:
    ~Viz();
    bool draw_pointcloud(const std::string& topic, const PointICloud& pc);
    bool draw_pointcloud(const std::string& topic, const PointRGBCloud& pc);
    bool draw_gridmap2d(const std::string& topic, const GridMap2d& map);
    bool draw_pose(const std::string& topic, const Pose& pose);
    bool draw_vehicle2d(const std::string& topic, const VehState2d& state);
    bool draw_trj2d(const std::string& topic, const std::vector<float>& xs, const std::vector<float>& ys);
    bool draw_trj2d(const std::string& topic, const std::vector<Point2f>& trj);
    bool draw_trj2d_point(const std::string& topic, const Point2f& point);
    bool draw_trj2d_point(const std::string& topic, const float x, const float y);
    bool draw_image();
    void render();
    bool closed();
private:
    Viz();
    Drawable* get_drawable(const std::string& topic);

    Camera camera_;
    Vector3 model_center_;
    std::unordered_map<std::string, Drawable*> drawables_;
}; // struct Viz

} // namespace rviz

#endif // RVIZ_HPP_


// #define RVIZ_IMPLEMENTATION // delete me


#ifdef RVIZ_IMPLEMENTATION
#ifndef RVIZ_CPP_
#define RVIZ_CPP_

#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <raymath.h>

#ifndef RVIZ_WIN_WIDTH
#   define RVIZ_WIN_WIDTH 800
#endif // RVIZ_WIN_WIDTH

#ifndef RVIZ_WIN_HEIGHT
#   define RVIZ_WIN_HEIGHT 600
#endif // RVIZ_WIN_HEIGHT

#ifndef RVIZ_WIN_NAME
#   define RVIZ_WIN_NAME "RVIZ"
#endif // RVIZ_WIN_NAME

#ifndef RVIZ_TARGET_FPS
#   define RVIZ_TARGET_FPS 20
#endif // RVIZ_TARGET_FPS

namespace rviz {

void heatmap(float intensity, Color& color)
{
    static Color palette[] {
        {29, 72, 119, 255},
        {27, 138, 90, 255},
        {251, 176, 33, 255},
        {246, 136, 56, 255},
        {238, 62, 50, 255}
    };
    static const auto palette_size{sizeof(palette) / sizeof(palette[0])};

    const auto palette_index{palette_size * intensity};
    const auto palette_index_floor{static_cast<int>(std::floor(palette_index))};
    const auto palette_index_ceil{static_cast<int>(std::ceil(palette_index))};

    const auto inter{palette_index - static_cast<float>(palette_index_floor)};
    const auto one_minus_inter{1.0f - inter};

    color.a = 255;
    color.r = one_minus_inter * palette[palette_index_floor].r + inter * palette[palette_index_ceil].r;
    color.g = one_minus_inter * palette[palette_index_floor].g + inter * palette[palette_index_ceil].g;
    color.b = one_minus_inter * palette[palette_index_floor].b + inter * palette[palette_index_ceil].b;
}

void set_pointcloud_mesh_buffer(size_t num_point, Mesh& mesh)
{
    const auto vertices_buf_size{num_point * 3};
    const auto color_buf_size{num_point * 4};

    if (mesh.vertices == nullptr || (mesh.vertexCount * 3) < vertices_buf_size) {
        delete [] mesh.vertices;
        mesh.vertices = new float[vertices_buf_size];
    }
    if (mesh.colors == nullptr || (mesh.vertexCount * 4) < color_buf_size) {
        delete [] mesh.colors;
        mesh.colors = new unsigned char[color_buf_size];
    }
    mesh.vertexCount = static_cast<int>(num_point);
    mesh.triangleCount = 1;
}

Viz::Viz()
{
    InitWindow(RVIZ_WIN_WIDTH, RVIZ_WIN_HEIGHT, RVIZ_WIN_NAME);    
    SetTargetFPS(RVIZ_TARGET_FPS);

    camera_.position   = {0.0f, 0.0f, 10.0f};
    camera_.target     = {0.0f, 0.0f, 0.0f};
    camera_.up         = {0.0f, 1.0f, 0.0f};
    camera_.fovy       = 45.0f;
    camera_.projection = CAMERA_PERSPECTIVE;

    model_center_.x = 0.0f;
    model_center_.y = 0.0f;
    model_center_.z = 0.0f;
}

Viz::~Viz()
{
    for (auto& it: drawables_) {
        switch (it.second->type) 
        {
        case DT_MESH_MODEL:
            delete [] it.second->pointcloud_model->mesh.colors;
            delete [] it.second->pointcloud_model->mesh.vertices;
            UnloadModel(it.second->pointcloud_model->model);
            delete it.second->pointcloud_model;
            break;
        case DT_VEHICLE:
            delete it.second->vehicle_model;
            break;
        case DT_POSE:
            delete it.second->pose;
            break;
        case DT_MAP:
            delete it.second->map;
            break;
        case DT_TRJ:
            delete it.second->trj2d;
            break;
        default:
            assert(false && "Unknown drawable type");
            break;
        }
        delete it.second;
    }
    CloseWindow();
}

Viz::Drawable* Viz::get_drawable(const std::string& topic)
{
    if (drawables_.find(topic) == drawables_.end()) {
        auto drawable{new Drawable};
        drawable->map = nullptr; // not a bug, we are using union
        drawables_.insert(std::make_pair(topic, drawable));
    }
    return drawables_.at(topic);
}

bool Viz::closed()
{
    return WindowShouldClose();
}

void Viz::render()
{
    camera_.fovy += GetMouseWheelMove() * -5;
    camera_.fovy = std::clamp(camera_.fovy, 0.0f, 170.0f);

    static int last_left_down_x{-1};
    static int last_left_down_y{-1};
    static int last_right_down_x{-1};
    static int last_right_down_y{-1};

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        last_left_down_x = -1;
        last_left_down_y = -1;
    } else if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        int this_left_down_x{GetMouseX()};
        int this_left_down_y{GetMouseY()};
        if (last_left_down_x > 0 || last_left_down_y > 0) {
            const auto delta_x{this_left_down_x - last_left_down_x};
            const auto delta_y{this_left_down_y - last_left_down_y};
            camera_.position.x -= delta_x * 0.03f;
            camera_.position.y += delta_y * 0.03f;
            camera_.target.x -= delta_x * 0.03f;
            camera_.target.y += delta_y * 0.03f;
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
            camera_.target.z += std::min(delta_y, delta_x) * 0.1f;
            camera_.target.y += delta_y * 0.1f;
            camera_.target.x -= delta_x * 0.1f;
        }
        last_right_down_x = this_right_down_x;
        last_right_down_y = this_right_down_y;
    }

    UpdateCamera(&camera_, CAMERA_CUSTOM);

    /*
     *    ^
     *    |
     *    |
     *    Y
     *    |
     *    |
     *    ------X------>
     * 
     */

    BeginDrawing();
        ClearBackground(BLACK);
        BeginMode3D(camera_);
            for (auto it : drawables_) {
                switch (it.second->type) {
                case DT_MESH_MODEL:
                    {
                        DrawModelPoints(it.second->pointcloud_model->model, model_center_, 1.0f, WHITE);
                    } break;
                case DT_POSE:
                    {
                        DrawLine3D(it.second->pose->start, it.second->pose->x_end, RED);
                        DrawLine3D(it.second->pose->start, it.second->pose->y_end, GREEN);
                        DrawLine3D(it.second->pose->start, it.second->pose->z_end, BLUE);
                    } break;
                case DT_MAP:
                    {
                        // Draw game grid
                        const auto map{it.second->map};
                        const auto res_x{(map->range_x[1] - map->range_x[0]) / map->col};
                        const auto res_y{(map->range_y[1] - map->range_y[0]) / map->row};
                        const auto half_cell_x{res_x / 2.0f};
                        const auto half_cell_y{res_y / 2.0f};
                        for (float y = map->range_y[0]; y <= map->range_y[1]; y += res_y) {
                            DrawLine3D((Vector3){map->range_x[0], y, 0}, (Vector3){map->range_x[1], y, 0}, GRAY);
                        }
                        for (float x = map->range_x[0]; x <= map->range_x[1]; x += res_x) {
                            DrawLine3D((Vector3){x, map->range_y[0], 0}, (Vector3){x, map->range_y[1], 0}, GRAY);
                        }

                        for (auto idx : map->occupied_grid) {
                            const float x{map->range_x[0] + (idx % map->col) * res_x};
                            const float y{map->range_y[1] - static_cast<int>(idx / map->row) * res_y};
                            DrawCube({x + half_cell_x, y - half_cell_y, -0.01}, res_x, res_y, 0.001f, GRAY);
                        }
                    } break;
                case DT_VEHICLE:
                    {
                        const auto vehicle{it.second->vehicle_model};
                        DrawModel(vehicle->front_axle, vehicle->position, 1.0f, GRAY);
                        DrawModel(vehicle->rear_axle, vehicle->position, 1.0f, GRAY);
                        DrawModel(vehicle->chassis, vehicle->position, 1.0f, GREEN);
                        DrawModel(vehicle->fr_wheel, vehicle->position, 1.0f, RED);
                        DrawModel(vehicle->fl_wheel, vehicle->position, 1.0f, RED);
                        DrawModel(vehicle->rr_wheel, vehicle->position, 1.0f, RED);
                        DrawModel(vehicle->rl_wheel, vehicle->position, 1.0f, RED);
                    } break;
                case
                DT_TRJ:
                    {
                        const auto trj{it.second->trj2d};
                        DrawLineStrip(trj->points.data(), trj->points.size(), WHITE);
                    } break;
                default:
                    assert(false && "Unknown drawable type");
                }
            }
            {
                Vector3 start{.x=0.0f, .y=0.0f, .z=0.0f};
                Vector3 x_end{.x=0.0f, .y=1.0f, .z=0.0f};
                Vector3 y_end{.x=-1.0f, .y=0.0f, .z=0.0f};
                Vector3 z_end{.x=0.0f, .y=0.0f, .z=1.0f};
                DrawLine3D(start, x_end, RED); 
                DrawLine3D(start, y_end, GREEN);
                DrawLine3D(start, z_end, BLUE);
            }
        EndMode3D();
    EndDrawing();
}

#define RVIZ_MAX_COLOR_VAL 255

bool Viz::draw_pointcloud(const std::string& topic, const PointRGBCloud& pc)
{
    auto drawable{get_drawable(topic)};
    if (!drawable->pointcloud_model) drawable->pointcloud_model = new DrawablePointcloudModel;
    set_pointcloud_mesh_buffer(pc.size(), drawable->pointcloud_model->mesh);
    for (size_t i = 0; i < pc.size(); ++i) {
        const auto& point{pc.at(i)};
        drawable->pointcloud_model->mesh.vertices[i * 3 + 0] = point.y;
        drawable->pointcloud_model->mesh.vertices[i * 3 + 1] = point.x;
        drawable->pointcloud_model->mesh.vertices[i * 3 + 2] = point.z;
        drawable->pointcloud_model->mesh.colors[i * 4 + 0] = static_cast<uint8_t>(std::clamp(point.r, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VAL);
        drawable->pointcloud_model->mesh.colors[i * 4 + 1] = static_cast<uint8_t>(std::clamp(point.g, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VAL);
        drawable->pointcloud_model->mesh.colors[i * 4 + 2] = static_cast<uint8_t>(std::clamp(point.b, 0.0f, 1.0f) * RVIZ_MAX_COLOR_VAL);
        drawable->pointcloud_model->mesh.colors[i * 4 + 3] = RVIZ_MAX_COLOR_VAL;
    }

    UploadMesh(&drawable->pointcloud_model->mesh, true);
    drawable->pointcloud_model->model = LoadModelFromMesh(drawable->pointcloud_model->mesh);
    drawable->type = DT_MESH_MODEL;
    return true;
}

bool Viz::draw_pointcloud(const std::string& topic, const PointICloud& pc)
{
    auto drawable{get_drawable(topic)};
    if (!drawable->pointcloud_model) drawable->pointcloud_model = new DrawablePointcloudModel;
    set_pointcloud_mesh_buffer(pc.size(), drawable->pointcloud_model->mesh);
    Color color;
    for (size_t i = 0; i < pc.size(); ++i) {
        const auto& point{pc.at(i)};
        heatmap(point.i, color);
        drawable->pointcloud_model->mesh.vertices[i * 3 + 0] = point.y;
        drawable->pointcloud_model->mesh.vertices[i * 3 + 1] = point.x;
        drawable->pointcloud_model->mesh.vertices[i * 3 + 2] = point.z;
        drawable->pointcloud_model->mesh.colors[i * 4 + 0] = color.r;
        drawable->pointcloud_model->mesh.colors[i * 4 + 1] = color.g;
        drawable->pointcloud_model->mesh.colors[i * 4 + 2] = color.b;
        drawable->pointcloud_model->mesh.colors[i * 4 + 3] = color.a;
    }

    UploadMesh(&drawable->pointcloud_model->mesh, true);
    drawable->pointcloud_model->model = LoadModelFromMesh(drawable->pointcloud_model->mesh);
    drawable->type = DT_MESH_MODEL;
    return true;
}

bool Viz::draw_gridmap2d(const std::string& topic, const GridMap2d& map)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};
    drawable->type = DT_MAP;
    if (!drawable->map) {
        drawable->map = new DrawableMap;
    }

    if(map.range_x[0] >= map.range_x[1] || map.range_y[0] >= map.range_y[1]) {
        std::cerr << "Bad map range\n";
        return false;
    }
    if(map.col <= 0 || map.row <= 0) {
        std::cerr << "Bad map size\n";
        return false;
    }

    auto drawable_map{drawable->map};
    drawable_map->range_x[0] = -map.range_y[1];
    drawable_map->range_x[1] = -map.range_y[0];
    drawable_map->range_y[0] = map.range_x[0];
    drawable_map->range_y[1] = map.range_x[1];
    drawable_map->row = map.row;
    drawable_map->col = map.col;

    const auto num_grids{map.row * map.col};
    for (const auto& ocp : map.occupied_grid) {
        if (ocp >= num_grids) continue;
        drawable_map->occupied_grid.push_back(ocp);
    }
    return true;
}

bool Viz::draw_pose(const std::string& topic, const Pose& pose)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};
    if (!drawable->pose) drawable->pose = new DrawablePose;
    drawable->pose->start.x = pose.y;
    drawable->pose->start.y = pose.x;
    drawable->pose->start.z = pose.z;

    const auto cos_yaw{std::cos(pose.yaw)};
    const auto cos_roll{std::cos(pose.roll)};
    const auto cos_pitch{std::cos(pose.pitch)};
    const auto sin_yaw{std::sin(pose.yaw)};
    const auto sin_roll{std::sin(pose.roll)};
    const auto sin_pitch{std::sin(pose.pitch)};

    drawable->pose->x_end.x = -(pose.y + cos_pitch * sin_yaw);
    drawable->pose->x_end.y = pose.x + cos_pitch * cos_yaw;
    drawable->pose->x_end.z = pose.z + -sin_pitch;

    drawable->pose->y_end.x = (pose.y + sin_roll * sin_pitch * sin_yaw - cos_roll * cos_yaw);
    drawable->pose->y_end.y = pose.x + sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    drawable->pose->y_end.z = pose.z + sin_roll * cos_pitch;

    drawable->pose->z_end.x = -(pose.y + cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw);
    drawable->pose->z_end.y = pose.x + cos_roll * sin_pitch * cos_yaw - sin_roll * sin_yaw;
    drawable->pose->z_end.z = pose.z + cos_roll * cos_pitch;
    drawable->type = DT_POSE;
    return true;
}

bool Viz::draw_vehicle2d(const std::string& topic, const VehState2d& state)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};

    float steer_angle{state.steer_angle};
    float heading{state.heading};
    const float wheel_base{state.wheel_base};
    const float wheel_track{state.wheel_track};
    const float chassis_width{1.0f};
    const float chassis_height{wheel_base};
    const float chassis_length{0.2f};
    const float axle_radius{0.2f};
    const float axle_length{wheel_track};
    const float wheel_radius{state.wheel_radius};
    const float wheel_length{state.wheel_width};
    const auto half_axle_length{axle_length / 2.0f};
    const auto half_wheel_radius{wheel_radius / 2.0f};
    const auto half_wheel_track{wheel_track / 2.0f};
    const auto wheel_to_center{half_wheel_track - wheel_length / 2.0f};
    const auto half_wheel_base{wheel_base / 2.0f};

    if (!drawable->vehicle_model) {
        drawable->vehicle_model = new DrawableVehicleModel;
        drawable->vehicle_model->front_axle = LoadModelFromMesh(GenMeshCylinder(axle_radius, axle_length, 32));
        drawable->vehicle_model->rear_axle = LoadModelFromMesh(GenMeshCylinder(axle_radius, axle_length, 32));
        drawable->vehicle_model->chassis = LoadModelFromMesh(GenMeshCube(chassis_width, chassis_height, chassis_length));
        drawable->vehicle_model->fr_wheel = LoadModelFromMesh(GenMeshCylinder(wheel_radius, wheel_length, 32));
        drawable->vehicle_model->fl_wheel = LoadModelFromMesh(GenMeshCylinder(wheel_radius, wheel_length, 32));
        drawable->vehicle_model->rr_wheel = LoadModelFromMesh(GenMeshCylinder(wheel_radius, wheel_length, 32));
        drawable->vehicle_model->rl_wheel = LoadModelFromMesh(GenMeshCylinder(wheel_radius, wheel_length, 32));
    }

    drawable->vehicle_model->position.y = state.x;
    drawable->vehicle_model->position.x = -state.y;
    drawable->vehicle_model->position.z = half_wheel_radius;

    const Vector3 front_axle_angle{.x = 0, .y = 0, .z = M_PI_2f + heading};
    const Vector3 rear_axle_angle{.x = 0, .y = 0, .z = M_PI_2f + heading};
    const Vector3 chassis_angle{.x = 0, .y = 0, .z = heading};
    const Vector3 fr_wheel_angle{.x = 0, .y = 0, .z = -M_PI_2f + steer_angle + heading};
    const Vector3 fl_wheel_angle{.x = 0, .y = 0, .z = M_PI_2f + steer_angle + heading};
    const Vector3 rr_wheel_angle{.x = 0, .y = 0, .z = -M_PI_2f + heading};
    const Vector3 rl_wheel_angle{.x = 0, .y = 0, .z = M_PI_2f + heading};

    const auto sin_heading{std::sin(heading)};
    const auto cos_heading{std::cos(heading)};
    drawable->vehicle_model->front_axle.transform = MatrixRotateXYZ(front_axle_angle);
    drawable->vehicle_model->front_axle.transform.m12 = cos_heading * half_axle_length - wheel_base * sin_heading;
    drawable->vehicle_model->front_axle.transform.m13 = sin_heading * half_axle_length + wheel_base * cos_heading;
    drawable->vehicle_model->front_axle.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->rear_axle.transform = MatrixRotateXYZ(rear_axle_angle);
    drawable->vehicle_model->rear_axle.transform.m12 = cos_heading * half_axle_length;
    drawable->vehicle_model->rear_axle.transform.m13 = sin_heading * half_axle_length;
    drawable->vehicle_model->rear_axle.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->chassis.transform = MatrixRotateXYZ(chassis_angle);
    drawable->vehicle_model->chassis.transform.m12 = -sin_heading * half_wheel_base;
    drawable->vehicle_model->chassis.transform.m13 = cos_heading * half_wheel_base;
    drawable->vehicle_model->chassis.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->fr_wheel.transform = MatrixRotateXYZ(fr_wheel_angle);
    drawable->vehicle_model->fr_wheel.transform.m12 = cos_heading * wheel_to_center - wheel_base * sin_heading;
    drawable->vehicle_model->fr_wheel.transform.m13 = sin_heading * wheel_to_center + wheel_base * cos_heading;
    drawable->vehicle_model->fr_wheel.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->fl_wheel.transform = MatrixRotateXYZ(fl_wheel_angle);
    drawable->vehicle_model->fl_wheel.transform.m12 = -cos_heading * wheel_to_center - wheel_base * sin_heading;
    drawable->vehicle_model->fl_wheel.transform.m13 = -sin_heading * wheel_to_center + wheel_base * cos_heading;
    drawable->vehicle_model->fl_wheel.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->rr_wheel.transform = MatrixRotateXYZ(rr_wheel_angle);
    drawable->vehicle_model->rr_wheel.transform.m12 = cos_heading * wheel_to_center;
    drawable->vehicle_model->rr_wheel.transform.m13 = sin_heading * wheel_to_center;
    drawable->vehicle_model->rr_wheel.transform.m14 = half_wheel_radius;
    drawable->vehicle_model->rl_wheel.transform = MatrixRotateXYZ(rl_wheel_angle);
    drawable->vehicle_model->rl_wheel.transform.m12 = -cos_heading * wheel_to_center;
    drawable->vehicle_model->rl_wheel.transform.m13 = -sin_heading * wheel_to_center;
    drawable->vehicle_model->rl_wheel.transform.m14 = half_wheel_radius;
    drawable->type = DT_VEHICLE;
    return true;
}

bool Viz::draw_trj2d(const std::string& topic, const std::vector<float>& xs, const std::vector<float>& ys)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};
    drawable->type = DT_TRJ;
    if (!drawable->trj2d) {
        drawable->trj2d = new DrawableTrj2d;
    }

    if (xs.empty() || ys.empty()) {
        return false;
    }
    if (xs.size() != ys.size()) {
        return false;
    }

    drawable->trj2d->points.resize(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
        drawable->trj2d->points.at(i).x = -ys.at(i);
        drawable->trj2d->points.at(i).y = xs.at(i);
    }
    return true;
}

bool Viz::draw_trj2d(const std::string& topic, const std::vector<Point2f>& trj)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};
    drawable->type = DT_TRJ;
    if (!drawable->trj2d) {
        drawable->trj2d = new DrawableTrj2d;
    }
    drawable->trj2d->points.resize(trj.size());
    for (size_t i = 0; i < trj.size(); ++i) {
        drawable->trj2d->points.at(i).x = -trj.at(i).y;
        drawable->trj2d->points.at(i).y = trj.at(i).x;
    }
    return true;
}

bool Viz::draw_trj2d_point(const std::string& topic, const Point2f& point)
{
    auto drawable{get_drawable(topic)};
    std::lock_guard<std::mutex> guard{drawable->lock};
    drawable->type = DT_TRJ;
    if (!drawable->trj2d) {
        drawable->trj2d = new DrawableTrj2d;
    }
    drawable->trj2d->points.push_back((Vector2){.x=-point.y, .y=point.x});
    return true;
}

bool Viz::draw_trj2d_point(const std::string& topic, const float x, const float y)
{
    return draw_trj2d_point(topic, (Vector2){.x=x, .y=y});
}

bool Viz::draw_image()
{
    return true;
}

} // namespace rviz

#endif // RVIZ_CPP_
#endif // RVIZ_IMPLEMENTATION
