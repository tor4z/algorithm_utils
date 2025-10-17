#ifdef __cplusplus
extern "C" {
#endif

#ifndef RS_PATH_H_
#define RS_PATH_H_

#include <stdbool.h>

/*
 * Reeds, J.A., & Shepp, L.A. (1990). OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS.
 * Pacific Journal of Mathematics, 145, 367-393.
 *
 */

#define RS_NUM_PATTERNS 5

typedef enum {
    SP_L = 0,
    SP_R,
    SP_S
} RsSegPattern;

typedef struct {
    int num_segs;
    float length;
    float pattern_val[RS_NUM_PATTERNS];
    RsSegPattern patterns[RS_NUM_PATTERNS];
} RsPath;

// pp 190. eq. 8.1
bool rs_path_lpsplp(float x, float y, float phi, RsPath* path);
// pp 190. eq. 8.2
bool rs_path_lpsprp(float x, float y, float phi, RsPath* path);
// pp 190. eq. 8.3
bool rs_path_lprmlp(float x, float y, float phi, RsPath* path);

#endif // RS_PATH_H_


// #define RS_PATH_IMPLEMENTATION // delete me.

#ifdef RS_PATH_IMPLEMENTATION
#ifndef RS_PATH_C_
#define RS_PATH_C_

#include <math.h>

static inline float rs_pow2f(float x) { return x * x; }
static inline float rs_pow3f(float x) { return x * x * x; }
static inline float rs_absf(float x) { return x > 0.0f ? x : -x; }

static inline float rs_norm_2pif(float x)
{
    static const float db_pif = 2.0f * M_PI;

    while (x > M_PI) { x -= db_pif; }
    while (x < -M_PI) { x += db_pif; }
    return x;
}

static inline bool rs_to_polar(float x, float y, float* r, float* theta)
{
    if (!r || !theta) return false;
    *r = sqrtf(rs_pow2f(x) + rs_pow2f(y));
    *theta = atan2f(y, x);
    return true;
}

bool rs_path_lpsplp(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u;
    float t;
    rs_to_polar(x - sinf(phi), y - 1.0f + cosf(phi) , &u, &t);
    const float v = rs_norm_2pif(phi - t);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->patterns[0] = SP_L;
    path->patterns[1] = SP_S;
    path->patterns[2] = SP_L;
    path->pattern_val[0] = t;
    path->pattern_val[1] = u;
    path->pattern_val[2] = v;
    return true;
}

bool rs_path_lpsprp(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float t1;
    rs_to_polar(x + sinf(phi), y - 1.0f - cosf(phi) , &u1, &t1);
    const float u_2 = rs_pow2f(u1) - 4.0f;
    if (u_2 < 0.0f) { return false; }
    const float u = sqrtf(u_2);
    const float t = rs_norm_2pif(t1 + atan2f(u, 2.0f));
    const float v = rs_norm_2pif(t - phi);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->patterns[0] = SP_L;
    path->patterns[1] = SP_S;
    path->patterns[2] = SP_R;
    path->pattern_val[0] = t;
    path->pattern_val[1] = u;
    path->pattern_val[2] = v;
    return true;
}

bool rs_path_lprmlp(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x - sinf(phi);
    const float eta = y - 1.0f + cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 > 4.0f) return false;
    const float a = acosf(u1 / 4.0f);
    const float t = rs_norm_2pif(a + theta + M_PI_2);
    const float u = rs_norm_2pif(M_PI - 2.0f * a);
    const float v = rs_norm_2pif(phi - t - u);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->patterns[0] = SP_L;
    path->patterns[1] = SP_R;
    path->patterns[2] = SP_L;
    path->pattern_val[0] = t;
    path->pattern_val[1] = u;
    path->pattern_val[2] = v;
    return true;
}

#endif // RS_PATH_C_
#endif // RS_PATH_IMPLEMENTATION

#ifdef __cplusplus
}
#endif
