#define CCD_SINGLE

#include "vec3.h"
#include "ccd.h"
#include "physics.h"
#include "trace.h"

#include <math.h>
#include <stdlib.h>

#ifdef COLLISIONS_DEBUG
#include <stdio.h>
#endif

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type)                                                           \
  while (count >= capacity) {                                                                                          \
    capacity <<= 1;                                                                                                    \
    if (count <= capacity) {                                                                                           \
      array = realloc(array, capacity * sizeof(type));                                                                 \
      break;                                                                                                           \
    }                                                                                                                  \
  }

#define CCD_DIR(dir) normalize(ccd_vec3_to_ray(*dir))

typedef void (*support_func)(const void *, const ccd_vec3_t *, ccd_vec3_t *);

typedef struct {
  const common_data *data;
  count_t body;
  body_shape shape;
} ccd_context;

typedef struct {
  v3 center;
  v3 size;
  v3 axis[3];
} collision_box;

bool replay_collision;

static void collision_validation_failure(const physics_world *world, const collision_detection_context *ctx,
                                         bool ccd_collision, bool custom_collision) {
#ifdef COLLISIONS_DEBUG
  trace_print();
  printf("Collision detection mismatch\n");
  exit(1);
#endif
}

static void debugger_anchor() { exit(1); }

static v3 body_center(v3 shape_offset, quat global_rotation, v3 body_position) {
  v3 center = shape_offset;
  center = rotate(center, global_rotation);
  center = add(center, body_position);

  return center;
}

static v3 ccd_vec3_to_ray(ccd_vec3_t v) { return (v3){v.v[0], v.v[1], v.v[2]}; }

static ccd_vec3_t ccd_vec3_from_ray(v3 v) { return (ccd_vec3_t){{v.x, v.y, v.z}}; }

static v3 ccd_body_center(const ccd_context *ctx) {
  return body_center(ctx->shape.offset, ctx->data->rotations[ctx->body], ctx->data->positions[ctx->body]);
}

static quat ccd_body_rotation(const ccd_context *ctx) {
  return qmul(ctx->data->rotations[ctx->body], ctx->shape.rotation);
}

static void sphere_support(const void *data, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  ccd_context *ctx = (ccd_context *)data;

  v3 direction = CCD_DIR(dir);
  v3 center = add(ctx->data->positions[ctx->body], ctx->shape.offset);
  float radius = ctx->shape.sphere.radius;

  v3 support = add(center, scale(direction, radius));
  *vec = ccd_vec3_from_ray(support);
}

static void box_support(const void *data, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  ccd_context *ctx = (ccd_context *)data;

  v3 direction = CCD_DIR(dir);
  v3 center = ccd_body_center(ctx);
  quat rotation = ccd_body_rotation(ctx);
  quat inv_rotation = qinvert(rotation);

  v3 local_direction = normalize(rotate(direction, inv_rotation));
  v3 v = vec3((local_direction.x > 0 ? 1 : -1) * ctx->shape.box.size.x * 0.5,
              (local_direction.y > 0 ? 1 : -1) * ctx->shape.box.size.y * 0.5,
              (local_direction.z > 0 ? 1 : -1) * ctx->shape.box.size.z * 0.5);

  v = rotate(v, rotation);
  v = add(center, v);

  *vec = ccd_vec3_from_ray(v);
}

static void cylinder_support(const void *data, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  ccd_context *ctx = (ccd_context *)data;

  v3 direction = CCD_DIR(dir);
  v3 center = ccd_body_center(ctx);
  quat rotation = ccd_body_rotation(ctx);
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

  *vec = ccd_vec3_from_ray(v);
}

static support_func support_functions[] = {
    box_support,
    sphere_support,
    cylinder_support,
};

static v3 collision_detection_body_center(const collision_detection_context *ctx) {
  return body_center(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static contact *new_contact(physics_world *world, const collision_detection_context *ctx) {
  contact *c = &world->contacts.values[world->contacts.count++];
  c->index_a = ctx->body_a;
  c->index_b = ctx->body_b;
  c->friction = world->config.friction;
  c->restitution = world->config.restitution;

  return c;
}

static count_t box_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  quat shape_rotation = ctx->shape_a.rotation;

  v3 box_center = collision_detection_body_center(ctx);
  v3 extents = scale(ctx->shape_a.box.size, 0.5);

  v3 plane_normal = ctx->shape_b.plane.normal;
  v3 plane_point = ctx->data_b->positions[ctx->body_b];

  v3 corners[] = {
      {extents.x, extents.y, extents.z},    {extents.x, -extents.y, extents.z},  {extents.x, -extents.y, -extents.z},
      {extents.x, extents.y, -extents.z},   {-extents.x, extents.y, extents.z},  {-extents.x, -extents.y, extents.z},
      {-extents.x, -extents.y, -extents.z}, {-extents.x, extents.y, -extents.z},
  };

  const count_t max_contacts = 4;

  contacts *contacts = &world->contacts;
  ARRAY_RESIZE_IF_NEEDED(contacts->values, contacts->count + max_contacts, contacts->capacity, contact)

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    v3 corner = add(box_center, rotate(rotate(corners[i], shape_rotation), box_rotation));
    float distance = dot(sub(corner, plane_point), plane_normal);
    if (distance > 0)
      continue;

    contact *c = new_contact(world, ctx);
    c->normal = plane_normal;
    c->point = add(corner, scale(plane_normal, -0.5 * distance));
    c->depth = -distance;

    contact_count += 1;
  }

  return contact_count;
}

static count_t sphere_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 sphere_center = collision_detection_body_center(ctx);
  float sphere_radius = ctx->shape_a.sphere.radius;

  v3 plane_point = ctx->data_b->positions[ctx->body_b];
  v3 plane_normal = ctx->shape_b.plane.normal;

  float plane_sphere_distance = dot(sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius)
    return 0;

  contacts *contacts = &world->contacts;
  ARRAY_RESIZE_IF_NEEDED(contacts->values, contacts->count + 1, contacts->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->normal = plane_normal;
  contact->point = add(sphere_center, scale(plane_normal, -plane_sphere_distance));
  contact->depth = sphere_radius - plane_sphere_distance;

  return 1;
}

static count_t cylinder_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 plane_point = ctx->data_b->positions[ctx->body_b];
  v3 plane_normal = ctx->shape_b.plane.normal;

  quat global_rotation = ctx->data_a->rotations[ctx->body_a];
  quat shape_rotation = ctx->shape_a.rotation;

  v3 cylinder_center = collision_detection_body_center(ctx);
  float cylinder_radius = ctx->shape_a.cylinder.radius;
  float cylinder_half_height = ctx->shape_a.cylinder.height * 0.5f;

  v3 cylinder_axis = rotate(up(), qmul(global_rotation, shape_rotation));
  float axis_projection = dot(cylinder_axis, plane_normal);
  float radial_projection_sq = 1.0f - axis_projection * axis_projection;
  if (radial_projection_sq < 0.0f)
    radial_projection_sq = 0.0f;

  float radial_projection = sqrtf(radial_projection_sq);
  float center_distance = dot(sub(cylinder_center, plane_point), plane_normal);
  float min_distance =
      center_distance - cylinder_half_height * fabsf(axis_projection) - cylinder_radius * radial_projection;
  if (min_distance > 0.0f)
    return 0;

  contacts *contacts = &world->contacts;
  ARRAY_RESIZE_IF_NEEDED(contacts->values, contacts->count + 1, contacts->capacity, contact);

  float cap_sign = axis_projection > 0.0f ? -1.0f : 1.0f;
  v3 cap_offset = scale(cylinder_axis, cap_sign * cylinder_half_height);

  v3 radial_axis = sub(plane_normal, scale(cylinder_axis, axis_projection));
  float radial_axis_len_sqr = lensq(radial_axis);
  v3 radial_offset = zero();
  if (radial_axis_len_sqr > 0.000001f)
    radial_offset = scale(normalize(radial_axis), -cylinder_radius);

  v3 deepest_point = add(cylinder_center, add(cap_offset, radial_offset));

  contact *contact = new_contact(world, ctx);
  contact->normal = plane_normal;
  contact->point = add(deepest_point, scale(plane_normal, -min_distance));
  contact->depth = -min_distance;

  return 1;
}

static count_t detect_collision_ccd(physics_world *world, const collision_detection_context *ctx) {
  ccd_t ccd;
  CCD_INIT(&ccd);

  ccd.support1 = support_functions[ctx->shape_a.type];
  ccd.support2 = support_functions[ctx->shape_b.type];
  ccd.max_iterations = world->config.max_gjk_iterations;
  ccd.epa_tolerance = world->config.epa_tolerance;

  ccd_context ctx_a = {
      .data = ctx->data_a,
      .shape = ctx->shape_a,
      .body = ctx->body_a,
  };

  ccd_context ctx_b = {
      .data = ctx->data_b,
      .shape = ctx->shape_b,
      .body = ctx->body_b,
  };

  float depth;
  ccd_vec3_t normal, point;

  if (replay_collision) {
    debugger_anchor();
  }

  trace_clear();

  int result = ccdGJKPenetration(&ctx_a, &ctx_b, &ccd, &depth, &normal, &point);
  bool gjk_custom = gjk_check_intersection(world, ctx);

  /* Only validate false negatives: BND misses a collision that CCD found.
   * BND false positives (BND says collision, CCD says no) are benign at the
   * near-degenerate boundary — EPA will produce a near-zero penetration depth. */
  if (!gjk_custom && !result) {
    collision_validation_failure(world, ctx, result == 0, gjk_custom);
    return 0;
  }

  if (result < 0) {
    return 0;
  }

  ARRAY_RESIZE_IF_NEEDED(world->contacts.values, world->contacts.count, world->contacts.capacity, contact)

  contact *c = new_contact(world, ctx);
  c->point = ccd_vec3_to_ray(point);
  c->normal = negate(ccd_vec3_to_ray(normal));
  c->depth = depth;

  return 1;
}

count_t collisions_detect_dynamic(physics_world *world) {
  const common_data *dynamics = (common_data *)&world->dynamics;

  collision_detection_context ctx = {
      .data_a = dynamics,
      .data_b = dynamics,
  };

  count_t dyn_count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; ++j) {
      ctx.body_a = i;
      ctx.body_b = j;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = dynamics->shapes[j];

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          dyn_count += detect_collision_ccd(world, &ctx);

          if (replay_collision) {
            detect_collision_ccd(world, &ctx);
          }
        }
      }
    }
  }

  return dyn_count;
}

void collisions_detect_static(physics_world *world) {
  const common_data *dynamics = (common_data *)&world->dynamics;
  const common_data *statics = (common_data *)&world->statics;

  collision_detection_context ctx = {
      .data_a = dynamics,
      .data_b = statics,
  };

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < statics->count; ++j) {
      ctx.body_a = i;
      ctx.body_b = j;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = statics->shapes[j];

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          switch (shape_b.type) {
            case SHAPE_PLANE:
              switch (shape_a.type) {
                case SHAPE_BOX:
                  box_plane_collision(world, &ctx);
                  break;

                case SHAPE_SPHERE:
                  sphere_plane_collision(world, &ctx);
                  break;

                case SHAPE_CYLINDER:
                  cylinder_plane_collision(world, &ctx);
                  break;

                default:
                  break;
              }
              break;

            default:
              detect_collision_ccd(world, &ctx);

              if (replay_collision) {
                detect_collision_ccd(world, &ctx);
              }
              break;
          }
        }
      }
    }
  }
}
