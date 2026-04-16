#include "bandura.h"
#include "physics.h"
#include "trace.h"
#include <float.h>

#define TOLERANCE FLT_EPSILON

#ifdef COLLISIONS_DEBUG
#define COLLISION_TRACE(...) trace_log(__VA_ARGS__)
#else
#define COLLISION_TRACE(...)
#endif

typedef struct {
  const common_data *data;
  body_shape shape;
  count_t index;
} support_context;

typedef v3 (*support_func)(const support_context *, v3);

const v3 initial_direction = vec3(1, 0, 0);

inline static int sign(float x) {
  if (fabsf(x) < TOLERANCE) {
    return 0;
  }

  return x > 0 ? 1 : -1;
}

static inline bool is_zero(float x) { return fabsf(x) < TOLERANCE; }

static v3 body_center(const support_context *ctx) {
  v3 shape_offset = ctx->shape.offset;
  quat global_rotation = ctx->data->rotations[ctx->index];
  v3 body_position = ctx->data->positions[ctx->index];

  v3 center = shape_offset;
  center = rotate(center, global_rotation);
  center = add(center, body_position);

  return center;
}

static quat body_rotation(const support_context *ctx) {
  return qmul(ctx->data->rotations[ctx->index], ctx->shape.rotation);
}

static v3 sphere_support(const support_context *ctx, v3 direction) {
  v3 center = add(ctx->data->positions[ctx->index], ctx->shape.offset);
  float radius = ctx->shape.sphere.radius;

  return add(center, scale(direction, radius));
}

static v3 box_support(const support_context *ctx, v3 direction) {
  v3 center = body_center(ctx);
  quat rotation = body_rotation(ctx);
  quat inv_rotation = qinvert(rotation);

  v3 local_direction = normalize(rotate(direction, inv_rotation));
  v3 v = vec3((local_direction.x > 0 ? 1 : -1) * ctx->shape.box.size.x * 0.5,
              (local_direction.y > 0 ? 1 : -1) * ctx->shape.box.size.y * 0.5,
              (local_direction.z > 0 ? 1 : -1) * ctx->shape.box.size.z * 0.5);

  v = rotate(v, rotation);
  v = add(center, v);

  return v;
}

static v3 cylinder_support(const support_context *ctx, v3 direction) {
  v3 center = body_center(ctx);
  quat rotation = body_rotation(ctx);
  quat inv_rotation = qinvert(rotation);

  v3 local_direction = normalize(rotate(direction, inv_rotation));

  float radius = ctx->shape.cylinder.radius;
  float height = ctx->shape.cylinder.height;

  v3 v;
  float y = (local_direction.y > 0 ? 1 : -1) * height * 0.5;
  if (fabsf(local_direction.y) - 1.0 < 0) {
    float t = 1.0 / sqrtf(local_direction.x * local_direction.x + local_direction.z * local_direction.z);

    v = vec3(radius * local_direction.x * t, y, radius * local_direction.z * t);
  } else {
    v = vec3(radius, y, 0);
  }

  v = rotate(v, rotation);
  v = add(center, v);

  return v;
}

support_func support_functions[] = {box_support, sphere_support, cylinder_support};

static v3 support(const collision_detection_context *ctx, v3 direction) {
  support_context sa = {ctx->data_a, ctx->shape_a, ctx->body_a};
  support_context sb = {ctx->data_b, ctx->shape_b, ctx->body_b};

  v3 s1 = support_functions[ctx->shape_a.type](&sa, direction);
  v3 s2 = support_functions[ctx->shape_b.type](&sb, negate(direction));

  return sub(s1, s2);
}

v3 support_bodies(physics_world *world, v3 direction, body_handle body_1, body_handle body_2) {
  collision_detection_context ctx = {
      .body_a = handle_to_inner_index(world, body_1),
      .body_b = handle_to_inner_index(world, body_2),
      .data_a = as_common_const(world, body_1.type),
      .data_b = as_common_const(world, body_2.type),
      .shape_a = *shapes_get(world, ctx.data_a->shapes[ctx.body_a]),
      .shape_b = *shapes_get(world, ctx.data_b->shapes[ctx.body_b]),
  };

  return support(&ctx, normalize(direction));
}

static void simplex_add_point(simplex *s, v3 p) {
  s->points[3] = s->points[2];
  s->points[2] = s->points[1];
  s->points[1] = s->points[0];
  s->points[0] = p;

  s->size += 1;
}

static bool simplex_update_2(simplex *s, v3 *direction) {
  COLLISION_TRACE("[BND] Simplex update 2\n");
  v3 a = s->points[0];
  v3 b = s->points[1];
  v3 ab = sub(b, a);
  v3 ao = negate(a);

  v3 cr = cross(ab, ao);
  float c = dot(ab, ao);

  if (lensq(cr) < TOLERANCE && c > 0) {
    COLLISION_TRACE("[BND] Len2\n");
    *direction = zero();
    return false;
  }

  if (is_zero(c) || c > 0) {
    COLLISION_TRACE("[BND] Triple cross 2\n");
    *direction = cross(cr, ab);
  } else {
    s->size = 1;
    COLLISION_TRACE("[BND] AO 1\n");
    *direction = ao;
  }

  return false;
}

static bool simplex_update_3(simplex *s, v3 *direction) {
  COLLISION_TRACE("[BND] Simplex update 3\n");
  v3 a = s->points[0];
  v3 b = s->points[1];
  v3 c = s->points[2];

  if (distancesqr(a, b) < TOLERANCE || distancesqr(a, c) < TOLERANCE) {
    *direction = zero();
    return false;
  }

  v3 ab = sub(b, a);
  v3 ac = sub(c, a);
  v3 ao = negate(a);

  v3 abc = cross(ab, ac);

  float d1 = dot(cross(abc, ac), ao);
  if (is_zero(d1) || d1 > 0) {
    float d2 = dot(ac, ao);
    if (is_zero(d2) || d2 > 0) {
      s->points[1] = c;
      s->size = 2;
      *direction = cross(cross(ac, ao), ac);
      COLLISION_TRACE("[BND] Triple cross 1\n");
    } else {
    do_simplex3_edge_ab:;
      float d3 = dot(ab, ao);
      if (is_zero(d3) || d3 > 0) {
        s->size = 2;
        *direction = cross(cross(ab, ao), ab);
        COLLISION_TRACE("[BND] Triple cross 2\n");
      } else {
        s->size = 1;
        *direction = ao;
        COLLISION_TRACE("[BND] AO\n");
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
        COLLISION_TRACE("[BND] ABC\n");
      } else {
        v3 tmp = s->points[1];
        s->points[1] = s->points[2];
        s->points[2] = tmp;
        *direction = negate(abc);
        COLLISION_TRACE("[BND] -ABC\n");
      }
    }
  }

  return false;
}

static bool simplex_update_4(simplex *s, v3 *direction) {
  COLLISION_TRACE("[BND] Simplex update 4\n");
  v3 a = s->points[0];
  v3 b = s->points[1];
  v3 c = s->points[2];
  v3 d = s->points[3];

  v3 ab = sub(b, a);
  v3 ac = sub(c, a);
  v3 ad = sub(d, a);
  v3 ao = negate(a);

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

    COLLISION_TRACE("[BND] ACD_O\n");
  } else if (!adb_o) {
    s->points[2] = b;
    s->points[1] = d;
    s->size = 3;
    COLLISION_TRACE("[BND] ADB_O\n");
  } else if (!abc_o) {
    s->size = 3;
    COLLISION_TRACE("[BND] ABC_O\n");
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

bool gjk_check_intersection_bodies(physics_world *world, body_handle body_1, body_handle body_2, simplex *simplex) {
  count_t n;
  collision_detection_context ctx = {
      .data_a = body_1.type == BODY_DYNAMIC ? (common_data *)&world->dynamics : &world->statics,
      .data_b = body_2.type == BODY_DYNAMIC ? (common_data *)&world->dynamics : &world->statics,
      .body_a = handle_to_inner_index(world, body_1),
      .body_b = handle_to_inner_index(world, body_2),
      .shape_a = physics_get_shapes(world, body_1, &n)[0],
      .shape_b = physics_get_shapes(world, body_2, &n)[0]};

  return gjk_check_intersection(world, &ctx, simplex);
}

bool gjk_check_intersection(physics_world *world, const collision_detection_context *ctx, simplex *simplex) {
  v3 direction = initial_direction;

  COLLISION_TRACE("[BND] GJK start\n");

  simplex->size = 0;

  v3 support_point = support(ctx, direction);
  simplex_add_point(simplex, support_point);
  direction = normalize(negate(support_point));

  COLLISION_TRACE("[BND] Initial point: (%.2f, %.2f, %.2f)\n", support_point.x, support_point.y, support_point.z);

  count_t iterations = 0;
  for (iterations = 0; iterations < world->config.max_gjk_iterations; ++iterations) {
    support_point = support(ctx, direction);

    COLLISION_TRACE("[BND] Iteration %u. Support (%.2f, %.2f, %.2f)\n", iterations, support_point.x, support_point.y,
                    support_point.z);

    if (dot(support_point, direction) < 0) {
      COLLISION_TRACE("[BND] Dot is negative. No collision\n");
      return false;
    }

    simplex_add_point(simplex, support_point);

    if (simplex_update(simplex, &direction)) {
      COLLISION_TRACE("[BND] GJK finished, collision found\n");
      return true;
    }

    COLLISION_TRACE("[BND] New direction: (%.4f, %.4f, %.4f)\n", direction.x, direction.y, direction.z);

    if (lensq(direction) < TOLERANCE) {
      COLLISION_TRACE("[BND] Direction is zero, no collision\n");
      return false;
    }

    direction = normalize(direction);
  }

  world->stats.incomplete_collision_detections += 1;

  return false;
}
