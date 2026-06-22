#include "bnd-math.h"
#include <float.h>

bnd_m3 bnd_m3_identity() {
  return (bnd_m3){ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
}

bnd_v3 bnd_m3_rotate(bnd_v3 v, bnd_m3 m) {
  return (bnd_v3){ bnd_v3_dot(*(bnd_v3 *)m.m0, v), bnd_v3_dot(*(bnd_v3 *)m.m1, v), bnd_v3_dot(*(bnd_v3 *)m.m2, v) };
}

bnd_m3 bnd_m3_transpose(bnd_m3 m) {
  return (bnd_m3){ { m.m0[0], m.m1[0], m.m2[0] }, { m.m0[1], m.m1[1], m.m2[1] }, { m.m0[2], m.m1[2], m.m2[2] } };
}

bnd_m3 bnd_m3_multiply(bnd_m3 a, bnd_m3 b) {
  bnd_m3 result;

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

bnd_v3 bnd_m3_rotate_inverse(bnd_v3 v, bnd_m3 m) {
  return bnd_m3_rotate(v, bnd_m3_transpose(m));
}

bnd_m3 bnd_m3_from_basis(bnd_v3 x, bnd_v3 y, bnd_v3 z) {
  return (bnd_m3){ { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } };
}

bnd_m3 bnd_m3_negate(bnd_m3 m) {
  return (bnd_m3){ { -m.m0[0], -m.m0[1], -m.m0[2] }, { -m.m1[0], -m.m1[1], -m.m1[2] }, { -m.m2[0], -m.m2[1], -m.m2[2] } };
}

bnd_m3 bnd_m3_inverse(bnd_m3 m) {
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

  bnd_m3 result;
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

bnd_m3 bnd_m3_skew_symmetric(bnd_v3 v) {
  return (bnd_m3){ { 0, -v.z, v.y }, { v.z, 0, -v.x }, { -v.y, v.x, 0 } };
}

bnd_m3 bnd_m3_add(bnd_m3 a, bnd_m3 b) {
  return (bnd_m3){
    { a.m0[0] + b.m0[0], a.m0[1] + b.m0[1], a.m0[2] + b.m0[2] },
    { a.m1[0] + b.m1[0], a.m1[1] + b.m1[1], a.m1[2] + b.m1[2] },
    { a.m2[0] + b.m2[0], a.m2[1] + b.m2[1], a.m2[2] + b.m2[2] }
  };
}

bnd_m3 bnd_m3_scale(bnd_m3 m, float s) {
  return (bnd_m3){
    { m.m0[0] * s, m.m0[1] * s, m.m0[2] * s },
    { m.m1[0] * s, m.m1[1] * s, m.m1[2] * s },
    { m.m2[0] * s, m.m2[1] * s, m.m2[2] * s },
  };
}

bnd_m3 matrix_sub(bnd_m3 a, bnd_m3 b) {
  return (bnd_m3){
    { a.m0[0] - b.m0[0], a.m0[1] - b.m0[1], a.m0[2] - b.m0[2] },
    { a.m1[0] - b.m1[0], a.m1[1] - b.m1[1], a.m1[2] - b.m1[2] },
    { a.m2[0] - b.m2[0], a.m2[1] - b.m2[1], a.m2[2] - b.m2[2] }
  };
}

bnd_m3 bnd_m3_initial_inertia(bnd_v3 inertia) {
  return (bnd_m3){ { inertia.x, 0, 0 }, { 0, inertia.y, 0 }, { 0, 0, inertia.z } };
}

bnd_m3 bnd_m3_inertia(bnd_m3 initial_inertia, bnd_quat q) {
  float a2 = q.x * q.x;
  float b2 = q.y * q.y;
  float c2 = q.z * q.z;
  float ac = q.x * q.z;
  float ab = q.x * q.y;
  float bc = q.y * q.z;
  float ad = q.w * q.x;
  float bd = q.w * q.y;
  float cd = q.w * q.z;

  bnd_m3 rotation;

  rotation.m0[0] = 1 - 2 * (b2 + c2);
  rotation.m0[1] = 2 * (ab - cd);
  rotation.m0[2] = 2 * (ac + bd);

  rotation.m1[0] = 2 * (ab + cd);
  rotation.m1[1] = 1 - 2 * (a2 + c2);
  rotation.m1[2] = 2 * (bc - ad);

  rotation.m2[0] = 2 * (ac - bd);
  rotation.m2[1] = 2 * (bc + ad);
  rotation.m2[2] = 1 - 2 * (a2 + b2);

  return bnd_m3_multiply(bnd_m3_multiply(rotation, initial_inertia), bnd_m3_transpose(rotation));
}

bnd_m3 bnd_m3_displacement_inertia(bnd_m3 i0, bnd_v3 offset, float mass) {
  float r = bnd_v3_dot(offset, offset);

  bnd_m3 a = { 0 };
  a.m0[0] = r;
  a.m1[1] = r;
  a.m2[2] = r;

  bnd_m3 b;
  b.m0[0] = offset.x * offset.x;
  b.m0[1] = offset.x * offset.y;
  b.m0[2] = offset.x * offset.z;

  b.m1[0] = b.m0[1];
  b.m1[1] = offset.y * offset.y;
  b.m1[2] = offset.y * offset.z;

  b.m2[0] = b.m0[2];
  b.m2[1] = b.m1[2];
  b.m2[2] = offset.z * offset.z;

  return bnd_m3_add(i0, bnd_m3_scale(matrix_sub(a, b), mass));
}

bnd_quat integrate_rotation_midpoint(bnd_quat rotation, bnd_v3 angular_momentum, bnd_m3 base_inv_inertia, float dt) {
  bnd_m3 inv_inertia = bnd_m3_inertia(base_inv_inertia, rotation);
  bnd_v3 omega = bnd_m3_rotate(angular_momentum, inv_inertia);

  const float qdt = 0.25f * dt;
  const float hdt = 0.5f * dt;

  float half_angle = bnd_v3_len(omega) * qdt;
  bnd_quat half_step;
  if (half_angle < 1e-6f) {
    half_step = (bnd_quat){ omega.x * qdt, omega.y * qdt, omega.z * qdt, 1.0f };
    half_step = bnd_quat_normalize(half_step);
  } else {
    float scale_factor = sinf(half_angle) / bnd_v3_len(omega);
    half_step = (bnd_quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(half_angle) };
  }

  bnd_quat mid_rotation = bnd_quat_normalize(bnd_quat_mul(half_step, rotation));

  inv_inertia = bnd_m3_inertia(base_inv_inertia, mid_rotation);
  omega = bnd_m3_rotate(angular_momentum, inv_inertia);

  float angle = bnd_v3_len(omega) * hdt;
  if (angle < 1e-6f) {
    bnd_quat step = (bnd_quat){ omega.x * hdt, omega.y * hdt, omega.z * hdt, 1.0f };
    step = bnd_quat_normalize(step);
    return bnd_quat_normalize(bnd_quat_mul(step, rotation));
  }

  float scale_factor = sinf(angle) / bnd_v3_len(omega);
  bnd_quat step = (bnd_quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(angle) };

  return bnd_quat_normalize(bnd_quat_mul(step, rotation));
}

float sqr_distance_to_line_segment(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 *closest) {
  bnd_v3 d = bnd_v3_sub(b, a);
  bnd_v3 ao = bnd_v3_sub(a, from);

  float t = -1.0 * bnd_v3_dot(ao, d);
  t /= bnd_v3_lensqr(d);

  if (t <= 0) {
    *closest = a;
    return bnd_v3_distancesqr(a, from);
  } else if (t >= 1) {
    *closest = b;
    return bnd_v3_distancesqr(b, from);
  } else {
    *closest = bnd_v3_add(a, bnd_v3_scale(d, t));

    return bnd_v3_distancesqr(*closest, from);
  }
}

float sqr_distance_to_triangle(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_v3 *closest) {
  bnd_v3 d1 = bnd_v3_sub(b, a);
  bnd_v3 d2 = bnd_v3_sub(c, a);
  bnd_v3 ao = bnd_v3_sub(a, from);

  float v = bnd_v3_dot(d1, d1);
  float w = bnd_v3_dot(d2, d2);
  float p = bnd_v3_dot(ao, d1);
  float q = bnd_v3_dot(ao, d2);
  float r = bnd_v3_dot(d1, d2);

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
    d1 = bnd_v3_scale(d1, s);
    d2 = bnd_v3_scale(d2, t);

    *closest = bnd_v3_add(a, bnd_v3_add(d1, d2));
    distance = bnd_v3_distancesqr(*closest, from);
  } else {
    distance = sqr_distance_to_line_segment(from, a, b, closest);

    bnd_v3 closest_2;
    float distance2 = sqr_distance_to_line_segment(from, a, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }

    distance2 = sqr_distance_to_line_segment(from, b, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }
  }

  return distance;
}


bnd_m3 quat_as_matrix(bnd_quat q) {
  bnd_m3 result;

  float a2 = q.x*q.x;
  float b2 = q.y*q.y;
  float c2 = q.z*q.z;
  float ac = q.x*q.z;
  float ab = q.x*q.y;
  float bc = q.y*q.z;
  float ad = q.w*q.x;
  float bd = q.w*q.y;
  float cd = q.w*q.z;

  result.m0[0] = 1 - 2*(b2 + c2);
  result.m1[0] = 2*(ab + cd);
  result.m2[0] = 2*(ac - bd);

  result.m0[1] = 2*(ab - cd);
  result.m1[1] = 1 - 2*(a2 + c2);
  result.m2[1] = 2*(bc + ad);

  result.m0[2] = 2*(ac + bd);
  result.m1[2] = 2*(bc - ad);
  result.m2[2] = 1 - 2*(a2 + b2);

  return result;
}
