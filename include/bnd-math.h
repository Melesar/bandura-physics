#ifndef BND_MATH_H
#define BND_MATH_H

#include "bandura.h"
#include <math.h>

static inline bnd_v3 bnd_v3_cross(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};
}

static inline float bnd_v3_dot(bnd_v3 x, bnd_v3 y) {
  return x.x * y.x + x.y * y.y + x.z * y.z;
}

static inline bnd_v3 bnd_v3_add(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.x + y.x, x.y + y.y, x.z + y.z};
}

static inline bnd_v3 bnd_v3_scale(bnd_v3 x, float y) {
  return (bnd_v3){x.x * y, x.y * y, x.z * y};
}

static inline float bnd_v3_len(bnd_v3 x) {
  return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);
}

static inline bnd_v3 bnd_v3_normalize(bnd_v3 x) {
  float l = bnd_v3_len(x);
  if (l < 0.000001f) {
    return x;
  }

  float t = 1.0f / l;
  return (bnd_v3){ x.x * t, x.y * t, x.z * t };
}

static inline bnd_v3 bnd_v3_sub(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.x - y.x, x.y - y.y, x.z - y.z};
}

static inline float bnd_v3_lensqr(bnd_v3 x) {
  return x.x * x.x + x.y * x.y + x.z * x.z;
}

static inline float bnd_v3_distancesqr(bnd_v3 x, bnd_v3 y) {
  float dx = x.x - y.x;
  float dy = x.y - y.y;
  float dz = x.z - y.z;

  return dx * dx + dy * dy + dz * dz;
}

static inline bnd_v3 bnd_v3_zero(void) {
  return (bnd_v3){0, 0, 0};
}

static inline bnd_v3 bnd_v3_one(void) {
  return (bnd_v3){1, 1, 1};
}

static inline bnd_v3 bnd_v3_up(void) {
  return (bnd_v3){0, 1, 0};
}

static inline bnd_v3 bnd_v3_right(void) {
  return (bnd_v3){1, 0, 0};
}

static inline bnd_v3 bnd_v3_forward(void) {
  return (bnd_v3){0, 0, 1};
}

static inline bnd_v3 bnd_v3_rotate(bnd_v3 v, bnd_quat q) {
  bnd_v3 result;
  result.x =
    v.x * (q.x * q.x + q.w * q.w - q.y * q.y - q.z * q.z) +
    v.y * (2 * q.x * q.y - 2 * q.w * q.z) +
    v.z * (2 * q.x * q.z + 2 * q.w * q.y);

  result.y =
    v.x * (2 * q.w * q.z + 2 * q.x * q.y) +
    v.y * (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) +
    v.z * (-2 * q.w * q.x + 2 * q.y * q.z);

  result.z =
    v.x * (-2 * q.w * q.y + 2 * q.x * q.z) +
    v.y * (2 * q.w * q.x + 2 * q.y * q.z) +
    v.z * (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);

  return result;
}

static inline bnd_v3 bnd_v3_negate(bnd_v3 x) {
  return (bnd_v3){-x.x, -x.y, -x.z};
}

static inline bnd_v3 bnd_v3_barycentric(bnd_v3 p, bnd_v3 a, bnd_v3 b, bnd_v3 c) {
  bnd_v3 v0 = bnd_v3_sub(b, a);
  bnd_v3 v1 = bnd_v3_sub(c, a);
  bnd_v3 v2 = bnd_v3_sub(p, a);

  float d00 = bnd_v3_dot(v0, v0);
  float d01 = bnd_v3_dot(v0, v1);
  float d11 = bnd_v3_dot(v1, v1);
  float d20 = bnd_v3_dot(v2, v0);
  float d21 = bnd_v3_dot(v2, v1);

  float denom = d00 * d11 - d01 * d01;

  float y = (d11 * d20 - d01 * d21) / denom;
  float z = (d00 * d21 - d01 * d20) / denom;
  float x = 1.0f - z - y;

  return (bnd_v3){x, y, z};
}

static inline bnd_v3 bnd_v3_min(bnd_v3 a, bnd_v3 b) {
  return (bnd_v3){fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)};
}

static inline bnd_v3 bnd_v3_max(bnd_v3 a, bnd_v3 b) {
  return (bnd_v3){fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)};
}

// static inline bnd_quat bnd_quat_add(bnd_quat x, bnd_quat y) {
//   return (bnd_quat){x.x + y.x, x.y + y.y, x.z + y.z, x.w + y.w};
// }

static inline bnd_quat bnd_quat_scale(bnd_quat x, float y) {
  return (bnd_quat){x.x * y, x.y * y, x.z * y, x.w * y};
}

static inline bnd_quat bnd_quat_mul(bnd_quat x, bnd_quat y) {
  float qax = x.x, qay = x.y, qaz = x.z, qaw = x.w;
  float qbx = y.x, qby = y.y, qbz = y.z, qbw = y.w;

  bnd_quat result;
  result.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
  result.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
  result.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
  result.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

  return result;
}

static inline bnd_quat bnd_quat_normalize(bnd_quat x) {
  float length = sqrtf(x.x * x.x + x.y * x.y + x.z * x.z + x.w * x.w);
  if (length == 0.0f) {
    length = 1.0f;
  }
  float ilength = 1.0f / length;

  return bnd_quat_scale(x, ilength);
}

static inline bnd_quat bnd_quat_invert(bnd_quat x) {
  bnd_quat result = x;
  float lengthSq = x.x * x.x + x.y * x.y + x.z * x.z + x.w * x.w;

  if (lengthSq != 0.0f) {
    float invLength = 1.0f / lengthSq;

    result.x *= -invLength;
    result.y *= -invLength;
    result.z *= -invLength;
    result.w *= invLength;
  }

  return result;
}

static inline bnd_quat bnd_quat_identity(void) {
  return (bnd_quat){0, 0, 0, 1};
}

bnd_m3 bnd_m3_identity(void);
bnd_m3 bnd_m3_transpose(bnd_m3 m);
bnd_m3 bnd_m3_inverse(bnd_m3 m);
bnd_m3 bnd_m3_add(bnd_m3 a, bnd_m3 b);
bnd_m3 bnd_m3_multiply(bnd_m3 a, bnd_m3 b);
bnd_m3 bnd_m3_scale(bnd_m3 m, float s);
bnd_m3 bnd_m3_negate(bnd_m3 m);
bnd_v3 bnd_m3_rotate(bnd_v3 v, bnd_m3 m);
#endif
