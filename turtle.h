#ifdef __cplusplus
extern "C" {
#endif

#ifndef TURTLE_H_
#define TURTLE_H_

#ifndef TUR_STEP_SIZE
#   define TUR_STEP_SIZE 0.2f
#endif // TUR_STEP_SIZE

struct tur_point {
    float x;
    float y;
};

struct tur_trj {
    struct tur_point* items;
    int size;
    int cap;
};

typedef struct {
    struct tur_trj trj;
    float x;
    float y;
    float heading;
} Turtle;

void turtle_forward(Turtle* turtle, float s);
void turtle_arc(Turtle* turtle, float r, float rad);

#endif // TURTLE_H_


// #define TURTLE_IMPLEMENTATION // delete me.

#ifdef TURTLE_IMPLEMENTATION
#ifndef TURTLE_C_
#define TURTLE_C_

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>

#ifndef TUR_DA_INIT_DA_CAP
#   define TUR_DA_INIT_DA_CAP 16
#endif // TUR_DA_INIT_DA_CAP

#define tur_da_append(da, item)                                                         \
    do {                                                                                \
        if (!(da)->items) {                                                             \
            (da)->items = malloc(TUR_DA_INIT_DA_CAP * sizeof((da)->items[0]));          \
            (da)->cap = TUR_DA_INIT_DA_CAP;                                             \
        } else if ((da)->size >= (da)->cap) {                                           \
            (da)->items = realloc((da)->items, (da)->cap * 2 * sizeof((da)->items[0])); \
            (da)->cap *= 2;                                                             \
        }                                                                               \
        (da)->items[(da)->size++] = (item);                                             \
    } while (0)

#define tur_da_clear(da)                                                                \
    do {                                                                                \
        (da)->size = 0;                                                                 \
    } while (0)

#define tur_da_free(da)                                                                 \
    do {                                                                                \
        if ((da)->items) {                                                              \
            free((da)->items);                                                          \
            (da)->items = NULL;                                                         \
        }                                                                               \
        (da)->size = 0;                                                                 \
        (da)->cap = 0;                                                                  \
    } while (0)

static inline float tur_absf(float x)
{
    return x > 0.0f ? x : -x;
}

static inline float tur_norm_2pif(float x)
{
    static const float db_pif = 2.0f * M_PI;
    while (x > M_PI) { x -= db_pif; }
    while (x < -M_PI) { x += db_pif; }
    return x;
}

void turtle_forward(Turtle* turtle, float s)
{
    assert(turtle != NULL);
    if (s == 0) return;

    const float sin_h = sin(turtle->heading);
    const float cos_h = cos(turtle->heading);
    const int num_segs = (int)(s / TUR_STEP_SIZE);
    float x = turtle->x;
    float y = turtle->y;

    struct tur_point p;
    for (int i = 0; i < num_segs; ++i) {
        x += cos_h * TUR_STEP_SIZE;
        y += sin_h * TUR_STEP_SIZE;
        p.x = x;
        p.y = y;
        tur_da_append(&(turtle->trj), p);
    }

    float last_seg;
    if ((last_seg = s - num_segs * TUR_STEP_SIZE) > 0) {
        x += cos_h * last_seg;
        y += sin_h * last_seg;
        p.x = x;
        p.y = y;
        tur_da_append(&(turtle->trj), p);
    }
    turtle->x = x;
    turtle->y = y;
}

void turtle_arc(Turtle* turtle, float r, float rad)
{
    assert(turtle != NULL);
    if (r <= 0) return;

    const float rad_sign = rad > 0.0f ? 1.0f : -1.0f;
    const float c_x = turtle->x - r * sin(turtle->heading);
    const float c_y = turtle->y + r * cos(turtle->heading);
    const float rad_step_size = rad_sign * TUR_STEP_SIZE / r;
    const int num_steps = tur_absf(rad / rad_step_size);
    const float last_step_size = rad - num_steps * rad_step_size;

    float x = turtle->x;
    float y = turtle->y;
    struct tur_point p;
    const float x_rel = x - c_x;
    const float y_rel = y - c_y;
    float angle = atan2(y_rel, x_rel);

    for (int i = 0; i < num_steps; ++i) {
        angle += rad_step_size;
        x = r * cos(angle);
        y = r * sin(angle);
        p.x = x + c_x;
        p.y = y + c_y;
        tur_da_append(&(turtle->trj), p);
    }

    if (tur_absf(last_step_size) > 0) {
        angle += last_step_size;
        x = r * cos(angle);
        y = r * sin(angle);
        p.x = x + c_x;
        p.y = y + c_y;
        tur_da_append(&(turtle->trj), p);
    }
    turtle->x = x + c_x;
    turtle->y = y + c_y;
    turtle->heading = tur_norm_2pif(turtle->heading + rad);
}

#endif // TURTLE_C_
#endif // TURTLE_IMPLEMENTATION

#ifdef __cplusplus
}
#endif
