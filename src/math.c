#define RAYMATH_IMPLEMENTATION
#include "bandura.h"

m3 matrix_identity() {
  return (m3) {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
}

v3 matrix_rotate(v3 v, m3 m) {
  return (v3) {
    dot(*(v3*)m.m0, v),
    dot(*(v3*)m.m1, v),
    dot(*(v3*)m.m2, v)
  };
}

m3 matrix_transpose(m3 m) {
  return (m3) {
    { m.m0[0], m.m1[0], m.m2[0] },
    { m.m0[1], m.m1[1], m.m2[1] },
    { m.m0[2], m.m1[2], m.m2[2] }
  };
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
  return (m3) {
    { x.x, y.x, z.x },
    { x.y, y.y, z.y },
    { x.z, y.z, z.z }
  };
}

m3 matrix_negate(m3 m) {
  return (m3) {
    {-m.m0[0], -m.m0[1], -m.m0[2]},
    {-m.m1[0], -m.m1[1], -m.m1[2]},
    {-m.m2[0], -m.m2[1], -m.m2[2]}
  };
}

m3 matrix_inverse(m3 m) {
  float t4 = m.m0[0] * m.m1[1];
  float t6 = m.m0[0] * m.m1[2];
  float t8 = m.m0[1] * m.m1[0];
  float t10 = m.m0[2] * m.m1[0];
  float t12 = m.m0[1] * m.m2[0];
  float t14 = m.m0[2] * m.m2[0];

  // Calculate the determinant
  float t16 = (t4 * m.m2[2] - t6 * m.m2[1] - t8 * m.m2[2]+
    t10 * m.m2[1] + t12 * m.m1[2] - t14 * m.m1[1]);

  // Make sure the determinant is non-zero.
  if (t16 == (float)0.0f) {
    return m;
  }

  float t17 = 1/t16;

  m3 result;
  result.m0[0] = (m.m1[1] * m.m2[2] - m.m1[2] * m.m2[1]) * t17;
  result.m0[1] = -(m.m0[1] * m.m2[2] - m.m0[2] * m.m2[1]) * t17;
  result.m0[2] = (m.m0[1] * m.m1[2] - m.m0[2] * m.m1[1]) * t17;
  result.m1[0] = -(m.m1[0] * m.m2[2] - m.m1[2] * m.m2[0]) * t17;
  result.m1[1] = (m.m0[0] * m.m2[2] - t14) * t17;
  result.m1[2] = -(t6-t10) * t17;
  result.m2[0] = (m.m1[0] * m.m2[1] - m.m1[1] * m.m2[0]) * t17;
  result.m2[1] = -(m.m0[0] * m.m2[1] - t12) * t17;
  result.m2[2] = (t4-t8) * t17;

  return result;
}

m3 matrix_skew_symmetric(v3 v) {
  return (m3) {
    { 0, -v.z, v.y },
    { v.z, 0, -v.x },
    { -v.y, v.x, 0 }
  };
}

m3 matrix_add(m3 a, m3 b) {
  return (m3) {
    { a.m0[0] + b.m0[0], a.m0[1] + b.m0[1], a.m0[2] + b.m0[2] },
    { a.m1[0] + b.m1[0], a.m1[1] + b.m1[1], a.m1[2] + b.m1[2] },
    { a.m2[0] + b.m2[0], a.m2[1] + b.m2[1], a.m2[2] + b.m2[2] }
  };
}

m3 matrix_scale(m3 m, float s) {
  return (m3) {
    { m.m0[0] * s, m.m0[1] * s, m.m0[2] * s },
    { m.m1[0] * s, m.m1[1] * s, m.m1[2] * s },
    { m.m2[0] * s, m.m2[1] * s, m.m2[2] * s },
  };
}

m3 matrix_sub(m3 a, m3 b) {
  return (m3) {
    { a.m0[0] - b.m0[0], a.m0[1] - b.m0[1], a.m0[2] - b.m0[2] },
    { a.m1[0] - b.m1[0], a.m1[1] - b.m1[1], a.m1[2] - b.m1[2] },
    { a.m2[0] - b.m2[0], a.m2[1] - b.m2[1], a.m2[2] - b.m2[2] }
  };
}

m3 matrix_initial_inertia(v3 inertia) {
  return (m3) {
    {inertia.x, 0, 0},
    {0, inertia.y, 0},
    {0, 0, inertia.z}
  };
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

  rotation.m0[0] = 1 - 2*(b2 + c2);
  rotation.m0[1] = 2*(ab - cd);
  rotation.m0[2] = 2*(ac + bd);

  rotation.m1[0] = 2*(ab + cd);
  rotation.m1[1] = 1 - 2*(a2 + c2);
  rotation.m1[2] = 2*(bc - ad);

  rotation.m2[0] = 2*(ac - bd);
  rotation.m2[1] = 2*(bc + ad);
  rotation.m2[2] = 1 - 2*(a2 + b2);

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
