#define CCD_SINGLE

#include "vec3.h"
#include "ccd.h"

#include "physics.h"
#include <math.h>
#include <stdlib.h>

v3 ccd_vec3_to_ray(ccd_vec3_t v) {
  return (v3) { v.v[0], v.v[1], v.v[2] };
}

ccd_vec3_t ccd_vec3_from_ray(v3 v) {
  return (ccd_vec3_t) { { v.x, v.y, v.z } };
}

void sphere_support(const void *data, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  v3 direction = ccd_vec3_to_ray(*dir);
}

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type)                                                           \
  while (count >= capacity) {                                                                                          \
    capacity <<= 1;                                                                                                    \
    if (count <= capacity) {                                                                                           \
      array = realloc(array, capacity * sizeof(type));                                                                 \
      break;                                                                                                           \
    }                                                                                                                  \
  }

typedef struct {
  v3 center;
  v3 size;
  v3 axis[3];
} collision_box;

static contact *new_contact(physics_world *world, const collision_detection_context *ctx) {
  contact *c = &world->contacts.values[world->contacts.count++];
  c->index_a = ctx->body_a;
  c->index_b = ctx->body_b;
  c->friction = world->config.friction;
  c->restitution = world->config.restitution;

  return c;
}

typedef count_t (*collision_func)(physics_world *world, const collision_detection_context *ctx);

static v3 body_center(v3 shape_offset, quat global_rotation, v3 body_position) {
  v3 center = shape_offset;
  center = rotate(center, global_rotation);
  center = add(center, body_position);

  return center;
}

static v3 body_a_center(const collision_detection_context *ctx) {
  return body_center(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static count_t box_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  quat shape_rotation = ctx->shape_a.rotation;

  v3 box_center = body_a_center(ctx);
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
  v3 sphere_center = body_a_center(ctx);
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

  v3 cylinder_center = body_a_center(ctx);
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

count_t collisions_detect_dynamic(physics_world *world) {
  const common_data *dynamics = (common_data *)&world->dynamics;

  collision_detection_context ctx = {.world = world};
  ctx.data_a = dynamics;
  ctx.data_b = dynamics;

  collision_detection_context inv_ctx = ctx;

  count_t dyn_count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; ++j) {
      ctx.body_a = i;
      ctx.body_b = j;
      inv_ctx.body_a = j;
      inv_ctx.body_b = i;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = dynamics->shapes[j];

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          inv_ctx.shape_a = ctx.shape_b;
          inv_ctx.shape_b = ctx.shape_a;
        }
      }
    }
  }

  return dyn_count;
}

void collisions_detect_static(physics_world *world) {
  const common_data *dynamics = (common_data *)&world->dynamics;
  const common_data *statics = (common_data *)&world->statics;

  collision_detection_context ctx = {.world = world};
  collision_detection_context inv_ctx = ctx;

  ctx.data_a = dynamics;
  ctx.data_b = statics;

  inv_ctx.data_a = statics;
  inv_ctx.data_b = dynamics;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < statics->count; ++j) {
      ctx.body_a = i;
      ctx.body_b = j;
      inv_ctx.body_a = j;
      inv_ctx.body_b = i;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = statics->shapes[j];

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;
        inv_ctx.shape_b = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;
          inv_ctx.shape_a = shape_b;

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
          }
        }
      }
    }
  }
}
