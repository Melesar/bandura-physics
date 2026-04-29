#include "bnd-core.h"
#include <float.h>

#define TOLERANCE FLT_EPSILON

const v3 initial_direction = vec3(1, 0, 0);

inline static int sign(float x) {
  if (fabsf(x) < TOLERANCE) {
    return 0;
  }

  return x > 0 ? 1 : -1;
}

static inline bool is_zero(float x) { return fabsf(x) < TOLERANCE; }

static void simplex_add_point(simplex *s, support_point p) {
  s->points[3] = s->points[2];
  s->points[2] = s->points[1];
  s->points[1] = s->points[0];
  s->points[0] = p;

  s->size += 1;
}

static bool simplex_update_2(simplex *s, v3 *direction) {
  v3 a = s->points[0].v;
  v3 b = s->points[1].v;
  v3 ab = sub(b, a);
  v3 ao = negate(a);

  v3 cr = cross(ab, ao);
  float c = dot(ab, ao);

  if (lensq(cr) < TOLERANCE && c > 0) {
    *direction = zero();
    return false;
  }

  if (is_zero(c) || c > 0) {
    *direction = cross(cr, ab);
  } else {
    s->size = 1;
    *direction = ao;
  }

  return false;
}

static bool simplex_update_3(simplex *s, v3 *direction) {
  support_point a = s->points[0];
  support_point b = s->points[1];
  support_point c = s->points[2];

  if (distancesqr(a.v, b.v) < TOLERANCE || distancesqr(a.v, c.v) < TOLERANCE) {
    *direction = zero();
    return false;
  }

  v3 ab = sub(b.v, a.v);
  v3 ac = sub(c.v, a.v);
  v3 ao = negate(a.v);

  v3 abc = cross(ab, ac);

  float d1 = dot(cross(abc, ac), ao);
  if (is_zero(d1) || d1 > 0) {
    float d2 = dot(ac, ao);
    if (is_zero(d2) || d2 > 0) {
      s->points[1] = c;
      s->size = 2;
      *direction = cross(cross(ac, ao), ac);
    } else {
    do_simplex3_edge_ab:;
      float d3 = dot(ab, ao);
      if (is_zero(d3) || d3 > 0) {
        s->size = 2;
        *direction = cross(cross(ab, ao), ab);
      } else {
        s->size = 1;
        *direction = ao;
      }
    }
  } else {
    float d4 = dot(cross(ab, abc), ao);
    if (is_zero(d4) || d4 > 0) {
      goto do_simplex3_edge_ab;
    } else {
      float d5 = dot(abc, ao);
      if (is_zero(d5) || d5 > 0) {
        *direction = abc;
      } else {
        support_point tmp = s->points[1];
        s->points[1] = s->points[2];
        s->points[2] = tmp;
        *direction = negate(abc);
      }
    }
  }

  return false;
}

static bool simplex_update_4(simplex *s, v3 *direction) {
  support_point a = s->points[0];
  support_point b = s->points[1];
  support_point c = s->points[2];
  support_point d = s->points[3];

  v3 ab = sub(b.v, a.v);
  v3 ac = sub(c.v, a.v);
  v3 ad = sub(d.v, a.v);
  v3 ao = negate(a.v);

  v3 abc = cross(ab, ac);
  v3 acd = cross(ac, ad);
  v3 adb = cross(ad, ab);

  int b_acd = sign(dot(ab, acd));
  int c_adb = sign(dot(ac, adb));
  int d_abc = sign(dot(ad, abc));

  bool acd_o = sign(dot(acd, ao)) == b_acd;
  bool adb_o = sign(dot(adb, ao)) == c_adb;
  bool abc_o = sign(dot(abc, ao)) == d_abc;

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

static bool simplex_update(simplex *s, v3 *direction) {
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

bool gjk_check_intersection_bodies(bnd_world *world, bnd_body_handle body_1, bnd_body_handle body_2, simplex *simplex) {
  count_t n;
  collision_detection_context ctx = {
      .data_a = body_1.type == BND_DYNAMIC ? (common_data *)&world->dynamics : &world->statics,
      .data_b = body_2.type == BND_DYNAMIC ? (common_data *)&world->dynamics : &world->statics,
      .body_a = handle_to_inner_index(world, body_1),
      .body_b = handle_to_inner_index(world, body_2),
      .shape_a = bnd_get_shapes(world, body_1, &n)[0],
      .shape_b = bnd_get_shapes(world, body_2, &n)[0]};

  return gjk_check_intersection(world, &ctx, simplex);
}

bool gjk_check_intersection(bnd_world *world, const collision_detection_context *ctx, simplex *simplex) {
  v3 direction = initial_direction;

  simplex->size = 0;

  support_point support_point = support(ctx, direction);
  simplex_add_point(simplex, support_point);
  direction = normalize(negate(support_point.v));

  count_t iterations = 0;
  for (iterations = 0; iterations < world->config.collision_detection.max_gjk_iterations; ++iterations) {
    support_point = support(ctx, direction);

    if (dot(support_point.v, direction) < 0) {
      return false;
    }

    simplex_add_point(simplex, support_point);

    if (simplex_update(simplex, &direction)) {
      return true;
    }

    if (lensq(direction) < TOLERANCE) {
      return false;
    }

    direction = normalize(direction);
  }

  world->stats.incomplete_collision_detections += 1;

  return false;
}
