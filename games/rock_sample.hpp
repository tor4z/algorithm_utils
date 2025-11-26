/**
 * The rock sample game based on [1].
 * 
 * [1] T. Smith, R. Simmons, "Heuristic Search Value Iteration for POMDPs,"
 *     in Association for Uncertainty in Artificial Intelligence (UAI), 2004
 */

// #define ROCK_SAMPLE_IMPLEMENTATION // delete me

#ifndef ROCK_SAMPLE_HPP_
#define ROCK_SAMPLE_HPP_

#include <cstddef>
#include <vector>
#include <raylib.h>

namespace rs {

using Vec2f = Vector2;

struct Vec2i
{
    int x;
    int y;
}; // struct Vec2i

enum ActionType
{
    ACT_UP = 0,
    ACT_DOWN,
    ACT_LEFT,
    ACT_RIGHT,
    ACT_SAMPLE,
    ACT_CHECK,
}; // enum ActionType

struct Action
{
    ActionType type;
    int rock_idx;
}; // struct Action

enum RockType
{
    RT_GOOD = 0,
    RT_BAD,
}; // enum RockType

struct State
{
    std::vector<Vec2i> rocks;
    Vec2i ego;
}; // struct State

struct Observation
{
    float type_prob;
    int rock_idx;
    RockType rock_type;
}; // struct Observation

struct StepReturn
{
    State state;
    Observation obs;
    float reward;
    bool terminated;
}; // struct StepReturn

struct EnvSettings
{
    size_t random_seed;
    size_t map_size;
    size_t num_rocks;
    size_t num_good_rocks;
    float eta;
}; // struct EnvSettings

struct Env
{
    Env();
    explicit Env(const EnvSettings& settings);
    StepReturn step(Action act);

    const EnvSettings setup;
private:
    std::vector<RockType> rocks_type_;
    State state_;

    int find_rock_idx(const Vec2i& pos);
    void move_ego(Action act);
    void reset_ego();
    void gen_rocks();
    void render();
}; // struct Env

} // namespace rs

#endif // ROCK_SAMPLE_HPP_


#ifdef ROCK_SAMPLE_IMPLEMENTATION
#ifndef ROCK_SAMPLE_CPP_
#define ROCK_SAMPLE_CPP_

#include <cassert>
#include <cmath>
#include <cstdlib>

#ifndef ROCK_SAMPLE_WIN_WIDTH
#   define ROCK_SAMPLE_WIN_WIDTH 800
#endif // ROCK_SAMPLE_WIN_WIDTH

#ifndef ROCK_SAMPLE_WIN_HEIGHT
#   define ROCK_SAMPLE_WIN_HEIGHT 600
#endif // ROCK_SAMPLE_WIN_HEIGHT

#ifndef ROCK_SAMPLE_GRID_SIZE
#   define ROCK_SAMPLE_GRID_SIZE 60
#endif // ROCK_SAMPLE_GRID_SIZE

#ifndef ROCK_SAMPLE_EPSf
#   include <limits>
#   define ROCK_SAMPLE_EPSf std::numeric_limits<float>::epsilon();
#endif // ROCK_SAMPLE_EPSf

namespace rs {

const EnvSettings default_settings{
    .random_seed = 13,
    .map_size = 8,
    .num_rocks = 7,
    .num_good_rocks = 4,
    .eta = 0.1f
};

template<typename T>
inline constexpr T rand_ab(T a, T b)
{
    assert(b > a);
    const auto rv01{static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) + 1.0f)};
    return rv01 * (b - a) + a;
}

template<typename T>
inline constexpr T max(T a, T b) { return a > b ? a : b; }

template<typename T>
inline constexpr T min(T a, T b) { return a < b ? a : b; }

template<typename T>
inline constexpr T abs(T v) { return v > 0 ? v : -v; }

template<typename T>
inline constexpr T pow2(T v) { return v * v; }

inline bool near(float a, float b) { return abs(a - b) < ROCK_SAMPLE_EPSf }

template<typename T>
inline constexpr float distance(const T& a, const T& b)
{
    return std::sqrt(static_cast<float>(pow2(a.x - b.x) + pow2(a.y - b.y)));
}


Env::Env() : Env(default_settings) {}

Env::Env(const EnvSettings& settings) : setup(settings)
{
    srand(setup.random_seed);

    reset_ego();
    gen_rocks();
    InitWindow(ROCK_SAMPLE_WIN_WIDTH, ROCK_SAMPLE_WIN_HEIGHT, "Rock Sample");
    SetTargetFPS(60);
}

void Env::reset_ego()
{
    state_.ego.x = 0;
    state_.ego.y = 0;
}

void Env::gen_rocks()
{
    assert(setup.num_good_rocks <= setup.num_rocks);
    state_.rocks.resize(setup.num_rocks);
    rocks_type_.resize(setup.num_rocks);

    int cnt{0};
    while (cnt < setup.num_rocks) {
        const auto rand_x{rand_ab(0ul, setup.map_size)};
        const auto rand_y{rand_ab(0ul, setup.map_size)};
        bool this_pos_valid{true};
        for (int i = 0; i < cnt; ++i) {
            const auto& test_rock{state_.rocks.at(i)};
            if (test_rock.x == rand_x && test_rock.y == rand_y) {
                this_pos_valid = false;
            }
            if (state_.ego.x == rand_x && state_.ego.y == rand_y) {
                this_pos_valid = false;
            }
        }

        if (!this_pos_valid) continue;
        state_.rocks.at(cnt).x = rand_x;
        state_.rocks.at(cnt).y = rand_y;

        RockType rt{RT_BAD};
        if (cnt < setup.num_good_rocks) {
            rt = RT_GOOD;
        }
        rocks_type_.at(cnt) = rt;
        cnt++;
    }
}

StepReturn Env::step(Action act)
{
    StepReturn sr{};
    sr.terminated = false;
    sr.reward = 0.0f;
    sr.obs.rock_idx = -1;
    sr.obs.type_prob = 0.0f;

    switch (act.type) {
    case ACT_CHECK: {
        if (act.rock_idx >= 0 && act.rock_idx < state_.rocks.size()) {
            sr.obs.rock_idx = act.rock_idx;
            const auto& rock{state_.rocks.at(act.rock_idx)};
            if (near(setup.eta, 1.0f)) {
                sr.obs.type_prob = 1.0f;
            } else if (near(setup.eta, 0.0f)) {
                sr.obs.type_prob = 0.5f;
            } else {
                const auto dist{distance(state_.ego, rock)};
                sr.obs.type_prob = std::pow(setup.eta, dist);
            }
            const auto true_type{rocks_type_.at(act.rock_idx)};
            const auto wrong_type{true_type == RT_GOOD ? RT_BAD : RT_GOOD};
            sr.obs.rock_type = rand_ab(0.0f, 1.0f) > sr.obs.type_prob ? wrong_type : true_type;
        }
    } break;
    case ACT_SAMPLE: {
        const auto rock_idx{find_rock_idx(state_.ego)};
        if (rock_idx >= 0) {
            if (rocks_type_.at(rock_idx) == RT_GOOD) {
                sr.reward = 10.0f;
            } else {
                sr.reward = -10.0f;
            }
        }
    } break;
    default:
        move_ego(act);
        if (state_.ego.x == setup.map_size) {
            sr.reward = 10.0f;
            sr.terminated = true;
        }
    }

    render();

    sr.state = state_;
    return sr;
}

int Env::find_rock_idx(const Vec2i& pos)
{
    int idx{-1};
    for (size_t i = 0; i < state_.rocks.size(); ++i) {
        const auto& rock{state_.rocks.at(i)};
        if (pos.x == rock.x && pos.y == rock.y) {
            idx = i;
            break;
        }
    }

    return idx;
}

void Env::move_ego(Action act)
{
    switch (act.type) {
    case ACT_UP:
        state_.ego.y = max(0, state_.ego.y - 1);
        break;
    case ACT_DOWN:
        state_.ego.y = min(static_cast<int>(setup.map_size - 1), state_.ego.y + 1);
        break;
    case ACT_LEFT:
        state_.ego.x = max(0, state_.ego.x - 1);
        break;
    case ACT_RIGHT:
        state_.ego.x = min(static_cast<int>(setup.map_size), state_.ego.x + 1);
        break;
    default:
        break;
    }
}

void Env::render()
{
    BeginDrawing();
        ClearBackground(DARKGRAY);

        {
            // draw map
            const auto map_size_pix{setup.map_size * ROCK_SAMPLE_GRID_SIZE};

            for (int i = 0; i < setup.map_size + 1; ++i) {
                const auto grid_i{i * ROCK_SAMPLE_GRID_SIZE};
                DrawLine(0, grid_i, map_size_pix, grid_i, BLACK);
                DrawLine(grid_i, 0, grid_i, map_size_pix, BLACK);
            }

            // exit area
            DrawRectangle(setup.map_size * ROCK_SAMPLE_GRID_SIZE, 0,
                ROCK_SAMPLE_GRID_SIZE, map_size_pix, RED);
        }

        {
            // draw ego
            const auto ego_x{state_.ego.x * ROCK_SAMPLE_GRID_SIZE + ROCK_SAMPLE_GRID_SIZE / 2};
            const auto ego_y{state_.ego.y * ROCK_SAMPLE_GRID_SIZE + ROCK_SAMPLE_GRID_SIZE / 2};
            DrawCircle(ego_x, ego_y, ROCK_SAMPLE_GRID_SIZE / 3.0f, BLUE);
        }

        {
            // draw rocks
            for (const auto& rock: state_.rocks) {
                const auto x{rock.x * ROCK_SAMPLE_GRID_SIZE + ROCK_SAMPLE_GRID_SIZE / 2};
                const auto y{rock.y * ROCK_SAMPLE_GRID_SIZE + ROCK_SAMPLE_GRID_SIZE / 2};
                DrawCircle(x, y, ROCK_SAMPLE_GRID_SIZE / 4.0f, BROWN);
            }
        }
    EndDrawing();
}

} // namespace rs

#endif // ROCK_SAMPLE_CPP_
#endif // ROCK_SAMPLE_IMPLEMENTATION
