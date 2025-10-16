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

#endif // TURTLE_H_


#define TURTLE_IMPLEMENTATION // delete me.

#ifdef TURTLE_IMPLEMENTATION
#ifndef TURTLE_C_
#define TURTLE_C_

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

    float laste_seg;
    if ((laste_seg = s - num_segs * TUR_STEP_SIZE) > 0) {
        x += cos_h * laste_seg;
        y += sin_h * laste_seg;
        p.x = x;
        p.y = y;
        tur_da_append(&(turtle->trj), p);
    }
    turtle->x = x;
    turtle->y = y;
}

#endif // TURTLE_C_
#endif // TURTLE_IMPLEMENTATION
