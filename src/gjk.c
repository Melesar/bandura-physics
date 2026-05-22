#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <float.h>

#define TOLERANCE FLT_EPSILON

const bnd_v3 initial_direction = (bnd_v3){ 1, 0, 0 };

inline static int sign(float x) {
  if (fabsf(x) < TOLERANCE) {
    return 0;
  }

  return x > 0 ? 1 : -1;
}

static inline bool is_zero(float x) {
  return fabsf(x) < TOLERANCE;
}

static void simplex_add_point(simplex *s, support_point p) {
  s->points[3] = s->points[2];
  s->points[2] = s->points[1];
  s->points[1] = s->points[0];
  s->points[0] = p;

  s->size += 1;
}

static bool simplex_update_2(simplex *s, bnd_v3 *direction) {
  bnd_v3 a = s->points[0].v;
  bnd_v3 b = s->points[1].v;
  bnd_v3 ab = bnd_v3_sub(b, a);
  bnd_v3 ao = bnd_v3_negate(a);

  bnd_v3 cr = bnd_v3_cross(ab, ao);
  float c = bnd_v3_dot(ab, ao);

  if (bnd_v3_lensqr(cr) < TOLERANCE && c > 0) {
    *direction = bnd_v3_zero();
    return false;
  }

  if (is_zero(c) || c > 0) {
    *direction = bnd_v3_cross(cr, ab);
  } else {
    s->size = 1;
    *direction = ao;
  }

  return false;
}

static bool simplex_update_3(simplex *s, bnd_v3 *direction) {
  support_point a = s->points[0];
  support_point b = s->points[1];
  support_point c = s->points[2];

  if (bnd_v3_distancesqr(a.v, b.v) < TOLERANCE || bnd_v3_distancesqr(a.v, c.v) < TOLERANCE) {
    *direction = bnd_v3_zero();
    return false;
  }

  bnd_v3 ab = bnd_v3_sub(b.v, a.v);
  bnd_v3 ac = bnd_v3_sub(c.v, a.v);
  bnd_v3 ao = bnd_v3_negate(a.v);

  bnd_v3 abc = bnd_v3_cross(ab, ac);

  float d1 = bnd_v3_dot(bnd_v3_cross(abc, ac), ao);
  if (is_zero(d1) || d1 > 0) {
    float d2 = bnd_v3_dot(ac, ao);
    if (is_zero(d2) || d2 > 0) {
      s->points[1] = c;
      s->size = 2;
      *direction = bnd_v3_cross(bnd_v3_cross(ac, ao), ac);
    } else {
    do_simplex3_edge_ab:;
      float d3 = bnd_v3_dot(ab, ao);
      if (is_zero(d3) || d3 > 0) {
        s->size = 2;
        *direction = bnd_v3_cross(bnd_v3_cross(ab, ao), ab);
      } else {
        s->size = 1;
        *direction = ao;
      }
    }
  } else {
    float d4 = bnd_v3_dot(bnd_v3_cross(ab, abc), ao);
    if (is_zero(d4) || d4 > 0) {
      goto do_simplex3_edge_ab;
    } else {
      float d5 = bnd_v3_dot(abc, ao);
      if (is_zero(d5) || d5 > 0) {
        *direction = abc;
      } else {
        support_point tmp = s->points[1];
        s->points[1] = s->points[2];
        s->points[2] = tmp;
        *direction = bnd_v3_negate(abc);
      }
    }
  }

  return false;
}

static bool simplex_update_4(simplex *s, bnd_v3 *direction) {
  support_point a = s->points[0];
  support_point b = s->points[1];
  support_point c = s->points[2];
  support_point d = s->points[3];

  bnd_v3 ab = bnd_v3_sub(b.v, a.v);
  bnd_v3 ac = bnd_v3_sub(c.v, a.v);
  bnd_v3 ad = bnd_v3_sub(d.v, a.v);
  bnd_v3 ao = bnd_v3_negate(a.v);

  bnd_v3 abc = bnd_v3_cross(ab, ac);
  bnd_v3 acd = bnd_v3_cross(ac, ad);
  bnd_v3 adb = bnd_v3_cross(ad, ab);

  int b_acd = sign(bnd_v3_dot(ab, acd));
  int c_adb = sign(bnd_v3_dot(ac, adb));
  int d_abc = sign(bnd_v3_dot(ad, abc));

  bool acd_o = sign(bnd_v3_dot(acd, ao)) == b_acd;
  bool adb_o = sign(bnd_v3_dot(adb, ao)) == c_adb;
  bool abc_o = sign(bnd_v3_dot(abc, ao)) == d_abc;

  if (acd_o && adb_o && abc_o) {
    return true;
  } else if (!acd_o) {
    s->points[1] = c;
    s->points[2] = d;
    s->size = 3;
  } else if (!adb_o) {
    s->points[2] = b;
    s->points[1] = d;
    s->size = 3;
  } else if (!abc_o) {
    s->size = 3;
  }

  return simplex_update_3(s, direction);
}

static bool simplex_update(simplex *s, bnd_v3 *direction) {
  switch (s->size) {
    case 4:
      return simplex_update_4(s, direction);

    case 3:
      return simplex_update_3(s, direction);

    case 2:
      return simplex_update_2(s, direction);
  }

  return false;
}

bool gjk_check_intersection(const bnd_world *world, const collision_detection_context *ctx, simplex *simplex) {
  PROFILE_FUNCTION

  bnd_v3 direction = initial_direction;

  simplex->size = 0;

  support_point support_point = support(ctx, direction);
  simplex_add_point(simplex, support_point);
  direction = bnd_v3_normalize(bnd_v3_negate(support_point.v));

  count_t iterations = 0;
  for (iterations = 0; iterations < world->config.collision_detection.max_gjk_iterations; ++iterations) {
    support_point = support(ctx, direction);

    if (bnd_v3_dot(support_point.v, direction) < 0) {
      return false;
    }

    simplex_add_point(simplex, support_point);

    if (simplex_update(simplex, &direction)) {
      return true;
    }

    if (bnd_v3_lensqr(direction) < TOLERANCE) {
      return false;
    }

    direction = bnd_v3_normalize(direction);
  }

  return false;
}
