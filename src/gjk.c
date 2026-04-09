#include "bandura.h"
#include "physics.h"

#define TOLERANCE 0.0001

typedef struct {
  const common_data *data;
  body_shape shape;
  count_t index;
} support_context;

typedef v3 (*support_func)(const support_context *, v3);

const v3 initial_direction = vec3(1, 0, 0);

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

static void simplex_add_point(simplex *s, v3 p) {
  s->points[3] = s->points[2];
  s->points[2] = s->points[1];
  s->points[1] = s->points[0];
  s->points[0] = p;

  s->size += 1;
}

static void simplex_update_1(const simplex *s, v3 *direction) { *direction = normalize(negate(s->points[0])); }

static void simplex_update_2(simplex *s, v3 *direction) {
  v3 ab = sub(s->points[1], s->points[0]);
  v3 ao = negate(s->points[0]);

  if (dot(ab, ao) > 0) {
    *direction = normalize(cross(cross(ab, ao), ab));
  } else {
    s->size = 1;
    *direction = normalize(ao);
  }
}

static void simplex_update_3(simplex *s, v3 *direction) {
  v3 a = s->points[0];
  v3 b = s->points[1];
  v3 c = s->points[2];

  v3 ab = sub(b, a);
  v3 ac = sub(c, a);
  v3 ao = negate(a);

  v3 abc = cross(ab, ac);
  if (dot(cross(abc, ac), ao) > 0) {
    if (dot(ac, ao) > 0) {
      s->points[1] = c;
      s->size = 2;
      *direction = normalize(cross(cross(ac, ao), ac));
    } else {
      s->size = 2;
      simplex_update_2(s, direction);
    }
  } else {
    if (dot(cross(ab, abc), ao) > 0) {
      s->size = 2;
      simplex_update_2(s, direction);
    } else {
      if (dot(abc, ao) > 0) {
        *direction = normalize(abc);
      } else {
        s->points[2] = b;
        s->points[1] = c;

        *direction = normalize(negate(abc));
      }
    }
  }
}

static bool simplex_update_4(simplex *s, v3 *direction) {
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

  if (dot(abc, ao) > 0) {
    s->size = 3;

    simplex_update_3(s, direction);
  } else if (dot(acd, ao) > 0) {
    s->points[1] = c;
    s->points[2] = d;
    s->size = 3;

    simplex_update_3(s, direction);
  } else if (dot(adb, ao) > 0) {
    s->points[1] = d;
    s->points[2] = b;
    s->size = 3;

    simplex_update_3(s, direction);
  } else {
    return true;
  }

  return false;
}

static bool simplex_update(simplex *s, v3 *direction) {
  switch (s->size) {
    case 4:
      return simplex_update_4(s, direction);

    case 3:
      simplex_update_3(s, direction);
      return false;

    case 2:
      simplex_update_2(s, direction);
      return false;

    case 1:
      simplex_update_1(s, direction);
      return false;
  }

  return false;
}

bool gjk_check_intersection(physics_world *world, const collision_detection_context *ctx) {
  v3 direction = initial_direction;
  simplex simplex = {0};

  v3 support_point = support(ctx, direction);
  simplex_add_point(&simplex, support_point);
  direction = negate(support_point);

  count_t iterations = 0;
  for (iterations = 0; iterations < world->config.max_gjk_iterations; ++iterations) {
    support_point = support(ctx, direction);

    if (dot(support_point, direction) < TOLERANCE) {
      return false;
    }

    simplex_add_point(&simplex, support_point);

    if (simplex_update(&simplex, &direction)) {
      return true;
    }

    if (len(direction) < TOLERANCE) {
      return false;
    }
  }

  world->stats.incomplete_collision_detections += 1;

  return false;
}
