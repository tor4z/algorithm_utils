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
 * We stolen code from:
 * - http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
 *
 */

#define RS_NUM_PATTERNS 5

typedef enum {
    RSD_L = 0,  // left
    RSD_R,      // right
    RSD_S,      // straight
} RsDirection;

typedef enum {
    RSG_F = 0,  // forward
    RSG_B       // backward
} RsGear;

typedef struct
{
    float val;
    RsDirection dir;
    RsGear gear;
} RsPathPrimitive;

typedef struct {
    int num_segs;
    float length;
    RsPathPrimitive segs[RS_NUM_PATTERNS];
} RsPath;

// pp 190. eq. 8.1
bool rs_path_csc_1(float x, float y, float phi, RsPath* path);
// pp 190. eq. 8.2
bool rs_path_csc_2(float x, float y, float phi, RsPath* path);
// pp 190. eq. 8.3
bool rs_path_c_c_c(float x, float y, float phi, RsPath* path);
// pp 190. eq. 8.4
bool rs_path_c_cc(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.7
bool rs_path_ccu_cuc(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.8
bool rs_path_c_cucu_c(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.9
bool rs_path_c_c2sc_1(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.9
bool rs_path_c_c2sc_2(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.10
bool rs_path_csc2_c_1(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.10
bool rs_path_csc2_c_2(float x, float y, float phi, RsPath* path);
// pp 191. eq. 8.11
bool rs_path_c_c2sc2_c(float x, float y, float phi, RsPath* path);

bool rs_find_from_all_path(float x, float y, float phi, RsPath* path);
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

bool rs_path_csc_1(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u;
    float t;
    rs_to_polar(x - sinf(phi), y - 1.0f + cosf(phi) , &u, &t);
    const float v = rs_norm_2pif(phi - t);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_S;
    path->segs[1].gear = RSG_F;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = v;
    return true;
}

bool rs_path_csc_2(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float t1;
    rs_to_polar(x + sinf(phi), y - 1.0f - cosf(phi) , &u1, &t1);
    const float u_2 = rs_pow2f(u1) - 4.0f;
    if (u_2 < 0.0f) { return false; }
    const float u = sqrtf(u_2);
    const float t = rs_norm_2pif(t1 + atan2f(2.0f, u));
    const float v = rs_norm_2pif(t - phi);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_S;
    path->segs[1].gear = RSG_F;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_R;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = v;
    return true;
}

bool rs_path_c_c_c(float x, float y, float phi, RsPath* path)
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
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = v;
    return true;
}

bool rs_path_c_cc(float x, float y, float phi, RsPath* path)
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
    const float v = rs_norm_2pif(t + u - phi);

    path->length = rs_absf(u) + rs_absf(t) + rs_absf(v);
    path->num_segs = 3;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = v;
    return true;
}

bool rs_path_ccu_cuc(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x + sinf(phi);
    const float eta = y - 1.0f - cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 > 4.0f) return false;

    float t;
    float u;
    float v;
    if (u1 > 2.0f) {
        const float a = acosf((u1 / 2.0f - 1.0f) / 2.0f);
        t = rs_norm_2pif(theta + M_PI_2 - a);
        u = rs_norm_2pif(M_PI - a);
        v = rs_norm_2pif(phi - t + 2.0f * u);
    } else {
        const float a = acosf((u1 / 2.0f + 1.0f) / 2.0f);
        t = rs_norm_2pif(a + theta + M_PI_2);
        u = rs_norm_2pif(a);
        v = rs_norm_2pif(phi - t + 2.0f * u);
    }

    path->length = rs_absf(u) * 2.0f + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = u;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_c_cucu_c(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x + sinf(phi);
    const float eta = y - 1.0f - cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 > 6.0f) return false;

    const float val1 = (5.0f * - rs_pow2f(u1) / 4.0f) / 4.0f;
    if (val1 < 0.0f || val1 > 1.0f) return false;
    const float u = acosf(val1);
    const float val2 = sinf(u);
    const float a = asinf(2.0f * val2 / u1);
    const float t = rs_norm_2pif(M_PI_2 + theta + a);
    const float v = rs_norm_2pif(t - phi);

    path->length = rs_absf(u) * 2.0f + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = u;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_c_c2sc_1(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x - sinf(phi);
    const float eta = y - 1.0f + cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 < 2.0f) return false;

    const float u = sqrtf(rs_pow2f(u1) - 4.0f) - 2.0f;
    if (u < 0.0f) return false;

    const float a = atan2f(2.0f, u + 2.0f);
    const float t = rs_norm_2pif(M_PI_2 + theta + a);
    const float v = rs_norm_2pif(t + M_PI_2 - phi);

    path->length = rs_absf(u) + M_PI_2 + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = M_PI_2;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = u;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_c_c2sc_2(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x + sinf(phi);
    const float eta = y - 1.0f - cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 < 2.0f) return false;

    const float t = rs_norm_2pif(M_PI_2 + theta);
    const float u = u1 - 2.0f;
    const float v = rs_norm_2pif(phi - t - M_PI_2);

    path->length = rs_absf(u) + M_PI_2 + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = M_PI_2;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = u;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_csc2_c_1(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x - sinf(phi);
    const float eta = y - 1.0f + cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 < 2.0f) return false;

    const float u = sqrtf(rs_pow2f(u1) - 4.0f) - 2.0f;
    if (u < 0.0f) return false;
    const float a = atan2f(u + 2.0f, 2.0f);
    const float t = rs_norm_2pif(M_PI_2 + theta - a);
    const float v = rs_norm_2pif(t - M_PI_2 - phi);

    path->length = rs_absf(u) + M_PI_2 + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = M_PI_2;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_csc2_c_2(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x + sinf(phi);
    const float eta = y - 1.0f - cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 < 2.0f) return false;

    const float t = rs_norm_2pif(theta);
    const float u = u1 - 2.0f;
    const float v = rs_norm_2pif(phi - t - M_PI_2);

    path->length = rs_absf(u) + M_PI_2 + rs_absf(t) + rs_absf(v);
    path->num_segs = 4;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = u;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = M_PI_2;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = v;
    return true;
}

bool rs_path_c_c2sc2_c(float x, float y, float phi, RsPath* path)
{
    if (!path) return false;

    float u1;
    float theta;
    const float xi = x - sinf(phi);
    const float eta = y - 1.0f - cosf(phi);
    rs_to_polar(xi, eta, &u1, &theta);
    if (u1 < 4.0f) return false;

    const float u = sqrtf(rs_pow2f(u1) - 4.0f) - 4.0f;
    if (u < 0.0f) return false;
    const float a = atan2f(2.0f, u + 4.0f);
    const float t = rs_norm_2pif(M_PI_2 + theta + a);
    const float v = rs_norm_2pif(t - phi);

    path->length = rs_absf(u) + M_PI_2 + M_PI_2 + rs_absf(t) + rs_absf(v);
    path->num_segs = 5;
    path->segs[0].dir = RSD_L;
    path->segs[0].gear = RSG_F;
    path->segs[0].val = t;
    path->segs[1].dir = RSD_R;
    path->segs[1].gear = RSG_B;
    path->segs[1].val = M_PI_2;
    path->segs[2].dir = RSD_L;
    path->segs[2].gear = RSG_F;
    path->segs[2].val = u;
    path->segs[3].dir = RSD_L;
    path->segs[3].gear = RSG_F;
    path->segs[3].val = M_PI_2;
    path->segs[4].dir = RSD_L;
    path->segs[4].gear = RSG_F;
    path->segs[4].val = v;
    return true;
}

bool rs_find_from_all_path(float x, float y, float phi, RsPath* path)
{
    float min_length = 1.0e6f;
    RsPath tmp_path = {0};
    if (rs_path_csc_1(x, y, phi, &tmp_path)) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_csc_2(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_c_c(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_cc(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_ccu_cuc(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_cucu_c(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_c2sc_1(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_c2sc_2(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_csc2_c_1(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_csc2_c_2(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }
    if (rs_path_c_c2sc2_c(x, y, phi, &tmp_path) && tmp_path.length < min_length) {
        min_length = tmp_path.length;
        *path = tmp_path;
    }

    if (tmp_path.num_segs == 0) {
        return false;
    }
    return true;
}

#endif // RS_PATH_C_
#endif // RS_PATH_IMPLEMENTATION

#ifdef __cplusplus
}
#endif
