#include "bandura.h"
#include <float.h>
#include <math.h>

v3 cross(v3 x, v3 y) {
  return vec3(x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x);
}

float dot(v3 x, v3 y) {
  return x.x * y.x + x.y * y.y + x.z * y.z;
}

v3 add(v3 x, v3 y) {
  return vec3(x.x + y.x, x.y + y.y, x.z + y.z);
}

v3 scale(v3 x, float y) {
  return vec3(x.x * y, x.y * y, x.z * y);
}

v3 normalize(v3 x) {
  float l = len(x);
  return l > EPSILON ? scale(x, 1.0 / l) : x;
}

v3 sub(v3 x, v3 y) {
  return vec3(x.x - y.x, x.y - y.y, x.z - y.z);
}

float len(v3 x) {
  return sqrtf(lensq(x));
}

float lensq(v3 x) {
  return x.x * x.x + x.y * x.y + x.z * x.z;
}

float distance(v3 x, v3 y) {
  return len(sub(x, y));
}

float distancesqr(v3 x, v3 y) {
  return lensq(sub(x, y));
}

v3 vec3(float x, float y, float z) {
  return (v3){ x, y, z };
}

v3 zero() {
  return vec3(0, 0, 0);
}

v3 one() {
  return vec3(1, 1, 1);
}

v3 up() {
  return vec3(0, 1, 0);
}

v3 right() {
  return vec3(1, 0, 0);
}

v3 forward() {
  return vec3(0, 0, 1);
}

v3 rotate(v3 v, quat q) {
  v3 result;
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

v3 negate(v3 x) {
  return vec3(-x.x, -x.y, -x.z);
}

v3 barycentric(v3 p, v3 a, v3 b, v3 c) {
  v3 v0 = sub(b, a);
  v3 v1 = sub(c, a);
  v3 v2 = sub(p, a);

  float d00 = dot(v0, v0);
  float d01 = dot(v0, v1);
  float d11 = dot(v1, v1);
  float d20 = dot(v2, v0);
  float d21 = dot(v2, v1);

  float denom = d00 * d11 - d01 * d01;

  float y = (d11 * d20 - d01 * d21) / denom;
  float z = (d00 * d21 - d01 * d20) / denom;
  float x = 1.0f - z - y;

  return vec3(x, y, z);
}

quat qadd(quat x, quat y) {
  return (quat){ x.x + y.x, x.y + y.y, x.z + y.z, x.w + y.w };
}

quat qscale(quat x, float y) {
  return (quat){ x.x * y, x.y * y, x.z * y, x.w * y };
}

quat qmul(quat x, quat y) {
  float qax = x.x, qay = x.y, qaz = x.z, qaw = x.w;
  float qbx = y.x, qby = y.y, qbz = y.z, qbw = y.w;

  quat result;
  result.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
  result.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
  result.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
  result.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

  return result;
}

quat qnormalize(quat x) {
  float length = sqrtf(x.x * x.x + x.y * x.y + x.z * x.z + x.w * x.w);
  if (length == 0.0f) {
    length = 1.0f;
  }
  float ilength = 1.0f / length;

  return qscale(x, ilength);
}

quat qinvert(quat x) {
  quat result = x;
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

quat qidentity() {
  return (quat){ 0, 0, 0, 1 };
}

m3 matrix_identity() {
  return (m3){ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
}

v3 matrix_rotate(v3 v, m3 m) {
  return (v3){ dot(*(v3 *)m.m0, v), dot(*(v3 *)m.m1, v), dot(*(v3 *)m.m2, v) };
}

m3 matrix_transpose(m3 m) {
  return (m3){ { m.m0[0], m.m1[0], m.m2[0] }, { m.m0[1], m.m1[1], m.m2[1] }, { m.m0[2], m.m1[2], m.m2[2] } };
}

m3 matrix_multiply(m3 a, m3 b) {
  m3 result;

  result.m0[0] = a.m0[0] * b.m0[0] + a.m0[1] * b.m1[0] + a.m0[2] * b.m2[0];
  result.m0[1] = a.m0[0] * b.m0[1] + a.m0[1] * b.m1[1] + a.m0[2] * b.m2[1];
  result.m0[2] = a.m0[0] * b.m0[2] + a.m0[1] * b.m1[2] + a.m0[2] * b.m2[2];

  result.m1[0] = a.m1[0] * b.m0[0] + a.m1[1] * b.m1[0] + a.m1[2] * b.m2[0];
  result.m1[1] = a.m1[0] * b.m0[1] + a.m1[1] * b.m1[1] + a.m1[2] * b.m2[1];
  result.m1[2] = a.m1[0] * b.m0[2] + a.m1[1] * b.m1[2] + a.m1[2] * b.m2[2];

  result.m2[0] = a.m2[0] * b.m0[0] + a.m2[1] * b.m1[0] + a.m2[2] * b.m2[0];
  result.m2[1] = a.m2[0] * b.m0[1] + a.m2[1] * b.m1[1] + a.m2[2] * b.m2[1];
  result.m2[2] = a.m2[0] * b.m0[2] + a.m2[1] * b.m1[2] + a.m2[2] * b.m2[2];

  return result;
}

v3 matrix_rotate_inverse(v3 v, m3 m) {
  return matrix_rotate(v, matrix_transpose(m));
}

m3 matrix_from_basis(v3 x, v3 y, v3 z) {
  return (m3){ { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } };
}

m3 matrix_negate(m3 m) {
  return (m3){ { -m.m0[0], -m.m0[1], -m.m0[2] }, { -m.m1[0], -m.m1[1], -m.m1[2] }, { -m.m2[0], -m.m2[1], -m.m2[2] } };
}

m3 matrix_inverse(m3 m) {
  float t4 = m.m0[0] * m.m1[1];
  float t6 = m.m0[0] * m.m1[2];
  float t8 = m.m0[1] * m.m1[0];
  float t10 = m.m0[2] * m.m1[0];
  float t12 = m.m0[1] * m.m2[0];
  float t14 = m.m0[2] * m.m2[0];

  // Calculate the determinant
  float t16 = (t4 * m.m2[2] - t6 * m.m2[1] - t8 * m.m2[2] + t10 * m.m2[1] + t12 * m.m1[2] - t14 * m.m1[1]);

  // Make sure the determinant is non-zero.
  if (t16 == (float)0.0f) {
    return m;
  }

  float t17 = 1 / t16;

  m3 result;
  result.m0[0] = (m.m1[1] * m.m2[2] - m.m1[2] * m.m2[1]) * t17;
  result.m0[1] = -(m.m0[1] * m.m2[2] - m.m0[2] * m.m2[1]) * t17;
  result.m0[2] = (m.m0[1] * m.m1[2] - m.m0[2] * m.m1[1]) * t17;
  result.m1[0] = -(m.m1[0] * m.m2[2] - m.m1[2] * m.m2[0]) * t17;
  result.m1[1] = (m.m0[0] * m.m2[2] - t14) * t17;
  result.m1[2] = -(t6 - t10) * t17;
  result.m2[0] = (m.m1[0] * m.m2[1] - m.m1[1] * m.m2[0]) * t17;
  result.m2[1] = -(m.m0[0] * m.m2[1] - t12) * t17;
  result.m2[2] = (t4 - t8) * t17;

  return result;
}

m3 matrix_skew_symmetric(v3 v) {
  return (m3){ { 0, -v.z, v.y }, { v.z, 0, -v.x }, { -v.y, v.x, 0 } };
}

m3 matrix_add(m3 a, m3 b) {
  return (m3){
    { a.m0[0] + b.m0[0], a.m0[1] + b.m0[1], a.m0[2] + b.m0[2] },
    { a.m1[0] + b.m1[0], a.m1[1] + b.m1[1], a.m1[2] + b.m1[2] },
    { a.m2[0] + b.m2[0], a.m2[1] + b.m2[1], a.m2[2] + b.m2[2] }
  };
}

m3 matrix_scale(m3 m, float s) {
  return (m3){
    { m.m0[0] * s, m.m0[1] * s, m.m0[2] * s },
    { m.m1[0] * s, m.m1[1] * s, m.m1[2] * s },
    { m.m2[0] * s, m.m2[1] * s, m.m2[2] * s },
  };
}

m3 matrix_sub(m3 a, m3 b) {
  return (m3){
    { a.m0[0] - b.m0[0], a.m0[1] - b.m0[1], a.m0[2] - b.m0[2] },
    { a.m1[0] - b.m1[0], a.m1[1] - b.m1[1], a.m1[2] - b.m1[2] },
    { a.m2[0] - b.m2[0], a.m2[1] - b.m2[1], a.m2[2] - b.m2[2] }
  };
}

m3 matrix_initial_inertia(v3 inertia) {
  return (m3){ { inertia.x, 0, 0 }, { 0, inertia.y, 0 }, { 0, 0, inertia.z } };
}

m3 matrix_inertia(m3 initial_inertia, quat q) {
  float a2 = q.x * q.x;
  float b2 = q.y * q.y;
  float c2 = q.z * q.z;
  float ac = q.x * q.z;
  float ab = q.x * q.y;
  float bc = q.y * q.z;
  float ad = q.w * q.x;
  float bd = q.w * q.y;
  float cd = q.w * q.z;

  m3 rotation;

  rotation.m0[0] = 1 - 2 * (b2 + c2);
  rotation.m0[1] = 2 * (ab - cd);
  rotation.m0[2] = 2 * (ac + bd);

  rotation.m1[0] = 2 * (ab + cd);
  rotation.m1[1] = 1 - 2 * (a2 + c2);
  rotation.m1[2] = 2 * (bc - ad);

  rotation.m2[0] = 2 * (ac - bd);
  rotation.m2[1] = 2 * (bc + ad);
  rotation.m2[2] = 1 - 2 * (a2 + b2);

  return matrix_multiply(matrix_multiply(rotation, initial_inertia), matrix_transpose(rotation));
}

m3 matrix_displacement_inertia(m3 i0, v3 offset, float mass) {
  float r = dot(offset, offset);

  m3 a = { 0 };
  a.m0[0] = r;
  a.m1[1] = r;
  a.m2[2] = r;

  m3 b;
  b.m0[0] = offset.x * offset.x;
  b.m0[1] = offset.x * offset.y;
  b.m0[2] = offset.x * offset.z;

  b.m1[0] = b.m0[1];
  b.m1[1] = offset.y * offset.y;
  b.m1[2] = offset.y * offset.z;

  b.m2[0] = b.m0[2];
  b.m2[1] = b.m1[2];
  b.m2[2] = offset.z * offset.z;

  return matrix_add(i0, matrix_scale(matrix_sub(a, b), mass));
}

quat integrate_rotation_midpoint(quat rotation, v3 angular_momentum, m3 base_inv_inertia, float dt) {
  m3 inv_inertia = matrix_inertia(base_inv_inertia, rotation);
  v3 omega = matrix_rotate(angular_momentum, inv_inertia);

  const float qdt = 0.25f * dt;
  const float hdt = 0.5f * dt;

  float half_angle = len(omega) * qdt;
  quat half_step;
  if (half_angle < 1e-6f) {
    half_step = (quat){ omega.x * qdt, omega.y * qdt, omega.z * qdt, 1.0f };
    half_step = qnormalize(half_step);
  } else {
    float scale_factor = sinf(half_angle) / len(omega);
    half_step = (quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(half_angle) };
  }

  quat mid_rotation = qnormalize(qmul(half_step, rotation));

  inv_inertia = matrix_inertia(base_inv_inertia, mid_rotation);
  omega = matrix_rotate(angular_momentum, inv_inertia);

  float angle = len(omega) * hdt;
  if (angle < 1e-6f) {
    quat step = (quat){ omega.x * hdt, omega.y * hdt, omega.z * hdt, 1.0f };
    step = qnormalize(step);
    return qnormalize(qmul(step, rotation));
  }

  float scale_factor = sinf(angle) / len(omega);
  quat step = (quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(angle) };

  return qnormalize(qmul(step, rotation));
}

float distance_to_line_segment(v3 from, v3 a, v3 b, v3 *closest) {
  v3 d = sub(b, a);
  v3 ao = sub(a, from);

  float t = -1.0 * dot(ao, d);
  t /= lensq(d);

  if (t <= 0) {
    *closest = a;
    return distancesqr(a, from);
  } else if (t >= 1) {
    *closest = b;
    return distancesqr(b, from);
  } else {
    *closest = add(a, scale(d, t));

    return distancesqr(*closest, from);
  }
}

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c, v3 *closest) {
  v3 d1 = sub(b, a);
  v3 d2 = sub(c, a);
  v3 ao = sub(a, from);

  float v = dot(d1, d1);
  float w = dot(d2, d2);
  float p = dot(ao, d1);
  float q = dot(ao, d2);
  float r = dot(d1, d2);

  float s, t;
  float distance = 0;
  float d = w * v - r * r;
  if (fabsf(d) < FLT_EPSILON) {
    s = t = -1;
  } else {
    s = (q * r - w * p) / d;
    t = (-s * r - q) / w;
  }

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1 && t + s <= 1) {
    d1 = scale(d1, s);
    d2 = scale(d2, t);

    *closest = add(a, add(d1, d2));
    distance = distancesqr(*closest, from);
  } else {
    distance = distance_to_line_segment(from, a, b, closest);

    v3 closest_2;
    float distance2 = distance_to_line_segment(from, a, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }

    distance2 = distance_to_line_segment(from, b, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }
  }

  return distance;
}
