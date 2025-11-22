#ifndef MOC_HPP_
#define MOC_HPP_

#include <vector>
#include <random>
#include <raylib.h>

namespace moc {

using Vec2f = Vector2;

struct Env;

struct State
{
    Vec2f position;
    Vec2f coll_shape;
    float heading;
    float speed;
}; // struct State

struct Obstacle
{
    Vec2f center;
    float radius;
}; // struct Obstacle

struct Observation
{
    std::vector<Obstacle> obstacles;
}; // struct Observation

struct Obb
{
    Vec2f center;
    Vec2f shape;
    float heading;
}; // struct Obb

enum Action
{
    ACT_ACCEL = 0,
    ACT_DEACCEL,
    ACT_MAINTAIN,
}; // enum Action

struct StepReturn
{
    State state;
    Observation obs;
    float reward;
    bool terminated;
}; // struct StepReturn

struct EnvSettings
{
    static EnvSettings from_default();

    unsigned int random_seed;
    int sim_fps;
    int num_persons;
    float max_person_spd;
    float max_person_accel;
    float person_accel_prob;
    float person_turn_prob;
    float person_turn_std;
    float person_preview_time;
    float map_length;
    float map_width;
    float car_length;
    float car_width;
    float car_accel;
    float car_deaccel;
    float car_max_accel;
    float car_sensing_dist;
}; // struct EnvSettings

struct Person
{
    Vec2f position;
    float heading;
    float spd;
    float accel;
    float target_spd;
    int id;

    void random_move(Env* env);
    void stop();
    void move(float accel, float heading, float dt);
}; // struct Person

struct Car
{
    Vec2f position;
    Vec2f shape;
    Vec2f coll_shape;
    float heading;
    float spd;
    Obb obb() const;
    Obb preview_obb(float dist) const;
    Vec2f to_car_system(const Vec2f& p);
    float to_car_system(float heading);
}; // struct Car

struct Env
{
    Env();
    explicit Env(const EnvSettings& settings);
    StepReturn step(Action act);
    inline float dt() const { return 1.0f / setup_.sim_fps; }
private:
    friend Person;

    std::vector<Person> persons_;
    const EnvSettings setup_;
    std::mt19937 rand_gen_;
    Car ego_;

    void render(const std::vector<int>& sensing_obsts);
    void reset_car();
    void spawn_persons();
    void step_persons();
    void step_car(Action act);
    bool coll_with_person(const Car& car);
    bool coll_with_car(const Person& p);
}; // struct Env

} // namespace moc

#endif // MOC_HPP_


#ifdef MOC_IMPLEMENTATION
#ifndef MOC_CPP_
#define MOC_CPP_

#define MOC_PI              3.14159265358979323846
#define MOC_PI_2            (MOC_PI / 2)
#define MOC_2_PI            (2 * MOC_PI)

#ifndef MOC_WIN_HEIGHT
#   define MOC_WIN_HEIGHT   600
#endif // MOC_WIN_HEIGHT

#ifndef MOC_WIN_WIDTH
#   define MOC_WIN_WIDTH    800
#endif // MOC_WIN_WIDTH

#ifndef MOC_SCALE
#   define MOC_SCALE        20.0f
#endif // MOC_SCALE

#ifndef MOC_PERSON_RADIUS
#   define MOC_PERSON_RADIUS 0.25f // meter
#endif // MOC_PERSON_RADIUS

#ifndef MOC_EPSf
#   include <limits>
#   define MOC_EPSf    std::numeric_limits<float>::epsilon()
#endif // MOC_EPSf

#include <cmath>
#include <cassert>
#include <cstdlib>
#include <raylib.h>

namespace moc {

const EnvSettings default_settings{
    .random_seed            = 0,
    .sim_fps                = 30,
    .num_persons            = 100,
    .max_person_spd         = 3.0f,     // m/s
    .max_person_accel       = 1.5f,     // m/s^2
    .person_accel_prob      = 0.2f,
    .person_turn_prob       = 0.1f,
    .person_turn_std        = 1.0f,
    .person_preview_time    = 1.0f,     // sec.
    .map_length             = 100.0f,   // meter
    .map_width              = 20.0f,    // meter
    .car_length             = 4.7f,     // meter
    .car_width              = 1.98f,    // meter
    .car_accel              = 1.0f,     // m/s^2
    .car_deaccel            = -2.0f,    // m/s^2
    .car_max_accel          = 8.0f,     // m/s^2, for preview calculation
    .car_sensing_dist       = 5.0f,     // m/s
};

template<typename T>
inline T rand_ab(T a, T b)
{
    assert(b > a);
    const auto rv01{static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) + 1.0f)};
    return rv01 * (b - a) + a;
}

template<typename T>
inline T norm_heading(T rad)
{
    while (rad > MOC_PI) { rad -= MOC_2_PI; }
    while (rad < MOC_PI) { rad += MOC_2_PI; }
    return rad;
}

template<typename T>
inline T rad_to_deg(T rad) { return rad / MOC_PI * 180.0; }

template<typename T>
inline T deg_to_rad(T deg) { return deg / 180.0 * MOC_PI; }

template<typename T>
inline T abs(T v) { return v < 0 ? -v : v; }

template<typename T>
inline T sign(T v) { return v < 0 ? -1 : 1; }

template<typename T>
inline T max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline T min(T a, T b) { return a < b ? a : b; }

template<typename T>
inline T pow2(T v) { return v * v; }

template<typename T>
inline T pow3(T v) { return v * v * v; }

inline float distance(const Vec2f& a, const Vec2f& b)
{
    return std::sqrt(pow2(a.x - b.x) + pow2(a.y - b.y));
}

/**
 * @brief cross project of oa X ob
 * 
 * @param o point
 * @param a point
 * @param b point
 * @return float 
 */
inline float cross(const Vec2f& o, const Vec2f& a, const Vec2f& b)
{
    Vec2f oa{.x=a.x - o.x, .y=a.y - o.y};
    Vec2f ob{.x=b.x - o.x, .y=b.y - o.y};
    return oa.x * ob.y - oa.y * ob.x;
}

bool point_in_obb(const Vec2f& p, const Obb& obb)
{
    const auto cos_heading{std::cos(obb.heading)};
    const auto sin_heading{std::sin(obb.heading)};

    const Vec2f rel_tr{.x=obb.shape.x / 2, .y=-obb.shape.y / 2};
    const Vec2f rel_tl{.x=obb.shape.x / 2, .y=obb.shape.y / 2};
    const Vec2f rel_br{.x=-obb.shape.x / 2, .y=-obb.shape.y / 2};
    const Vec2f rel_bl{.x=-obb.shape.x / 2, .y=obb.shape.y / 2};

    const Vec2f tr{.x = rel_tr.x * cos_heading + rel_tr.y * sin_heading + obb.center.x,
                   .y = rel_tr.y * cos_heading + rel_tr.x * sin_heading + obb.center.y};
    const Vec2f tl{.x = rel_tl.x * cos_heading + rel_tl.y * sin_heading + obb.center.x,
                   .y = rel_tl.y * cos_heading + rel_tl.x * sin_heading + obb.center.y};
    const Vec2f br{.x = rel_br.x * cos_heading + rel_br.y * sin_heading + obb.center.x,
                   .y = rel_br.y * cos_heading + rel_br.x * sin_heading + obb.center.y};
    const Vec2f bl{.x = rel_bl.x * cos_heading + rel_bl.y * sin_heading + obb.center.x,
                   .y = rel_bl.y * cos_heading + rel_bl.x * sin_heading + obb.center.y};

    return cross(bl, p, tl) * cross(br, tr, p) >= 0 &&
           cross(br, p, bl) * cross(tr, tl, p) >= 0;
}

inline Vec2f phy_to_raylib_shape(const Vec2f& p)
{
    return {.x=p.y * MOC_SCALE, .y=p.x * MOC_SCALE};
}

inline Vec2f phy_to_raylib(const Vec2f& p)
{
    return {
        .x = GetScreenWidth() / 2.0f - p.y * MOC_SCALE,
        .y = GetScreenHeight() - p.x * MOC_SCALE
    };
}

EnvSettings EnvSettings::from_default() { return default_settings; }

void Person::random_move(Env* env)
{
    // step heading
    if (rand_ab(0.0f, 1.0f) < env->setup_.person_turn_prob) {
        std::normal_distribution<> norm_dist{0.0f, 1.0f};
        heading += norm_dist(env->rand_gen_);
        heading = norm_heading(heading);
    }

    // step speed
    if (rand_ab(0.0f, 1.0f) < env->setup_.person_accel_prob) {
        target_spd = rand_ab(0.0f, env->setup_.max_person_spd);
        accel = rand_ab(0.0f, env->setup_.max_person_accel);
    }

    if (abs(spd - target_spd) > MOC_EPSf) {
        const auto accel_sign(target_spd - spd);
        const auto accel_spd{spd + env->dt() * accel * accel_sign};
        if (accel_sign > 0) spd = min(target_spd, accel_spd);
        else                spd = max(target_spd, accel_spd);
    }

    // step position
    const auto mov_dist{spd * env->dt()};
    position.x += mov_dist * std::cos(heading);
    position.y += mov_dist * std::sin(heading);
}

void Person::stop()
{
    spd = 0.0f;         // stop immediate
    target_spd = 0.0f;
}

void Person::move(float accel, float heading, float dt)
{
    this->heading = heading;
    const auto accel_spd{spd + dt * accel};
    spd = 2.0f;

    // step position
    const auto mov_dist{spd * dt};
    position.x += mov_dist * std::cos(this->heading);
    position.y += mov_dist * std::sin(this->heading);
}

Obb Car::obb() const
{
    return {.center=position, .shape=coll_shape, .heading=heading};
}

Obb Car::preview_obb(float dist) const
{
    return {.center  = {.x = position.x + dist / 2 * std::cos(heading),
                        .y = position.y + dist / 2 * std::sin(heading)},
            .shape   = {.x = coll_shape.x + dist, .y = coll_shape.y},
            .heading = heading};
}

Vec2f Car::to_car_system(const Vec2f& p)
{
    const auto dist{distance(p, position)};
    const auto angle{std::atan2(p.y - position.y, p.x - position.x)};

    return {.x = dist * std::cos(angle),
            .y = dist * std::sin(angle)};
}

float Car::to_car_system(float heading)
{
    return norm_heading(heading - this->heading);
}

Env::Env() : Env(default_settings) {}

Env::Env(const EnvSettings& settings)
    : setup_(settings)
{
    assert(settings.sim_fps > 0);
    assert(settings.num_persons >= 0);
    assert(settings.map_length > 0.0f);
    assert(settings.map_width > 0.0f);
    assert(settings.car_length > 0.0f);
    assert(settings.car_width > 0.0f);
    assert(settings.car_accel > 0.0f);
    assert(settings.car_max_accel > settings.car_accel);
    assert(settings.max_person_spd >= 0.0f);
    assert(settings.max_person_accel > 0.0f);
    assert(settings.person_accel_prob <= 1.0f && settings.person_accel_prob >= 0.0f);
    assert(settings.person_turn_prob <= 1.0f && settings.person_turn_prob >= 0.0f);

    rand_gen_ = std::mt19937(settings.random_seed);
    srand(settings.random_seed);

    InitWindow(MOC_WIN_WIDTH, MOC_WIN_HEIGHT, "Moving over Crowd");
    SetTargetFPS(setup_.sim_fps);
    reset_car();
    spawn_persons();
}

void Env::spawn_persons()
{
    persons_.resize(setup_.num_persons);
    for (int i = 0; i < setup_.num_persons;) {
        auto& person{persons_.at(i)};
        person.spd = rand_ab(0.0f, setup_.max_person_spd);
        person.target_spd = person.spd;
        person.heading = rand_ab(-MOC_PI, MOC_PI);
        person.position.x = rand_ab(0.0f, setup_.map_length);
        person.position.y = rand_ab(-setup_.map_width / 2, setup_.map_width / 2);
        person.id = i;
        if (coll_with_car(person)) { continue; /* retry */ }
        ++i;
    }
}

void Env::step_persons()
{
    const auto dt{this->dt()};
    const auto stop_time{ego_.spd / setup_.car_max_accel};
    auto mov_dist{stop_time * ego_.spd - 0.5f * setup_.car_max_accel * pow2(stop_time)};
    mov_dist = max(mov_dist, 1.0f);
    const auto ego_preview{ego_.preview_obb(mov_dist)};
    const auto ego_obb{ego_.obb()};

    for (auto& p : persons_) {
        const auto p_mov_dist{setup_.person_preview_time * p.spd};
        const Vec2f p_preview{.x = p.position.x + p_mov_dist * std::cos(p.heading),
                              .y = p.position.y + p_mov_dist * std::sin(p.heading)};

        if (point_in_obb(p.position, ego_preview) || point_in_obb(p_preview, ego_preview)) {
            const auto p_in_ego{ego_.to_car_system(p.position)};
            const auto avoid_dir{p_in_ego.y > 0 ? 1 : -1};
            const auto heading{norm_heading(ego_.heading + avoid_dir * MOC_PI_2)};
            const auto accel{setup_.max_person_accel};
            p.move(accel, heading, dt);
        } else if (point_in_obb(p_preview, ego_obb)) {
            p.stop();
        } else {
            p.random_move(this);
        }
    }
}

void Env::step_car(Action act)
{
    const auto dt{this->dt()};

    switch (act) {
    case ACT_ACCEL:
        ego_.spd += dt * setup_.car_accel;
        break;
    case ACT_DEACCEL:
        ego_.spd = max(ego_.spd + dt * setup_.car_deaccel, 0.0f);
        break;
    case ACT_MAINTAIN:
        break;
    }

    const auto mov_dist{ego_.spd * dt};
    ego_.position.x += mov_dist * std::cos(ego_.heading);
    ego_.position.y += mov_dist * std::sin(ego_.heading);
}

void Env::reset_car()
{
    const auto coll_padding{MOC_PERSON_RADIUS * 2};

    ego_.heading    = 0.0f;
    ego_.spd        = 0.0f;
    ego_.shape      = {.x=setup_.car_length, .y=setup_.car_width};
    ego_.coll_shape = {.x=ego_.shape.x + coll_padding, .y=ego_.shape.y + coll_padding};
    ego_.position   = {.x=setup_.car_length / 2, .y=0.0f};
}

bool Env::coll_with_person(const Car& car)
{
    const Obb ego_obb{car.obb()};
    for (const auto& p : persons_) {
        if (point_in_obb(p.position, ego_obb)) {
            return true;
        }
    }
    return false;
}

bool Env::coll_with_car(const Person& p)
{
    return point_in_obb(p.position, ego_.obb());
}

void Env::render(const std::vector<int>& sensing_obsts)
{
    int screen_half_height{GetScreenHeight() / 2};
    int pos_offset{0};

    auto offset_position{[&pos_offset] (const Vec2f& p) -> Vec2f {
        return {.x=p.x, .y=p.y + pos_offset};
    }};

    auto offset_rec{[&pos_offset] (const Rectangle& r) -> Rectangle {
        return {.x=r.x, .y=r.y + pos_offset, .width=r.width, .height=r.height};
    }};

    {
        // calculate position offset
        const auto ego_pos{phy_to_raylib(ego_.position)};
        if (ego_pos.y < screen_half_height) {
            pos_offset = screen_half_height - ego_pos.y;
        }
    }

    BeginDrawing();
        ClearBackground(DARKGRAY);
        {
            // draw grid
            Vec2f line_start{};
            Vec2f line_end{};
            for (float x = 0; x < setup_.map_length; x += 1) {
                line_start.x = x;
                line_start.y = -setup_.map_width / 2;
                line_end.x = x;
                line_end.y = setup_.map_width / 2;
                DrawLineV(offset_position(phy_to_raylib(line_start)), offset_position(phy_to_raylib(line_end)), BLACK);
            }
            for (float y = -setup_.map_width / 2; y < setup_.map_width / 2; y += 1) {   
                line_start.x = 0;
                line_start.y = y;
                line_end.x = setup_.map_length;
                line_end.y = y;
                DrawLineV(offset_position(phy_to_raylib(line_start)), offset_position(phy_to_raylib(line_end)), BLACK);
            }
        }

        // render car
        {
            const Vec2f raylib_shape{phy_to_raylib_shape(ego_.shape)};
            const Vec2f origin{.x=raylib_shape.x / 2, .y=raylib_shape.y / 2};
            const auto raylib_pos{phy_to_raylib(ego_.position)};

            const Rectangle rec{
                .x      = raylib_pos.x,
                .y      = raylib_pos.y,
                .width  = raylib_shape.x,
                .height = raylib_shape.y};

            const auto ego_rotation{rad_to_deg(-ego_.heading)};
            DrawRectanglePro(offset_rec(rec), origin, ego_rotation, BLUE);
            // draw simple sector to indicate ego heading
            const auto start_angle{45.0f + ego_rotation};
            const auto end_angle{135.0f + ego_rotation};
            DrawCircleSector(offset_position(raylib_pos), 10.0f,
                start_angle, end_angle, 16, YELLOW);
        }

        // render persons
        for (const auto& p: persons_) {
            bool in_sensing_list{false};
            for (auto it : sensing_obsts) {
                if (it == p.id) {
                    in_sensing_list = true;
                    break;
                }
            }
            DrawCircleV(offset_position(phy_to_raylib(p.position)), 5, in_sensing_list ? RED : GREEN);
        }
    EndDrawing();
}

StepReturn Env::step(Action act)
{
    StepReturn sr{};
    step_car(act);
    step_persons();

    sr.terminated = false;
    sr.reward = 0.0f;
    sr.state.coll_shape = ego_.coll_shape;
    sr.state.position = ego_.position;
    sr.state.speed = ego_.spd;
    sr.state.heading = ego_.heading;
    sr.obs.obstacles.clear();

    std::vector<int> sensing_obsts{};
    const auto ego_sensing_obb{ego_.preview_obb(setup_.car_sensing_dist)};
    for (const auto& p: persons_) {
        if (point_in_obb(p.position, ego_sensing_obb)) {
            sensing_obsts.push_back(p.id);
            sr.obs.obstacles.push_back({
                .center = p.position,
                .radius = MOC_PERSON_RADIUS
            });
        }
    }

    if (act == ACT_ACCEL || act == ACT_DEACCEL) {
        sr.reward -= 0.1f;
    }

    if (ego_.position.x > setup_.map_length) {
        sr.terminated = true;
        sr.reward += 100.0f;
    } else if (coll_with_person(ego_)) {
        sr.terminated = true;
        sr.reward -= 100.0f;
    }

    render(sensing_obsts);
    return sr;
}

} // namespace moc

#endif // MOC_CPP_
#endif // MOC_IMPLEMENTATION
