#include "bnd-core.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct {
  v3 center;
  v3 size;
  v3 axis[3];
} collision_box;

static v3 body_center(v3 shape_offset, quat global_rotation, v3 body_position) {
  v3 center = shape_offset;
  center = rotate(center, global_rotation);
  center = add(center, body_position);

  return center;
}

static v3 collision_detection_body_center(const collision_detection_context *ctx) {
  return body_center(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static contact *new_contact(bnd_world *world, const collision_detection_context *ctx) {
  return contacts_new_default(world, ctx->body_a, ctx->body_b);
}

static count_t sphere_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  v3 center_a = ctx->data_a->positions[ctx->body_a];
  v3 center_b = ctx->data_b->positions[ctx->body_b];

  float radius_a = ctx->shape_a.sphere.radius;
  float radius_b = ctx->shape_b.sphere.radius;

  v3 offset = sub(center_a, center_b);
  float distance = len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return 0;
  }

  v3 normal = scale(offset, 1 / distance);

  contact *c = new_contact(world, ctx);
  c->point = add(center_b, scale(normal, radius_b + penetration));
  c->normal = normal;
  c->depth = -penetration;

  return 1;
}

static count_t box_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
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

  contacts_ensure_capacity(world, max_contacts);

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

static count_t sphere_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  v3 sphere_center = collision_detection_body_center(ctx);
  float sphere_radius = ctx->shape_a.sphere.radius;

  v3 plane_point = ctx->data_b->positions[ctx->body_b];
  v3 plane_normal = ctx->shape_b.plane.normal;

  float plane_sphere_distance = dot(sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius)
    return 0;

  contact *contact = new_contact(world, ctx);
  contact->normal = plane_normal;
  contact->point = add(sphere_center, scale(plane_normal, -plane_sphere_distance));
  contact->depth = sphere_radius - plane_sphere_distance;

  return 1;
}

static count_t cylinder_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
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

static count_t detect_collisions(bnd_world *world, const collision_detection_context *ctx) {
  simplex s;
  if (!gjk_check_intersection(world, ctx, &s)) {
    return 0;
  }

  contact *c = new_contact(world, ctx);
  epa_get_contact(ctx, &s, world->config.collision_detection.epa_tolerance, c);

  return 1;
}

count_t collisions_detect_dynamic(bnd_world *world) {
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
        bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          if (shape_a.type == SHAPE_SPHERE && shape_b.type == SHAPE_SPHERE) {
            dyn_count += sphere_sphere_collision(world, &ctx);
          } else {
            dyn_count += detect_collisions(world, &ctx);
          }
        }
      }
    }
  }

  return dyn_count;
}

void collisions_detect_static(bnd_world *world) {
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
        bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          if (shape_a.type == SHAPE_SPHERE && shape_b.type == SHAPE_SPHERE) {
            sphere_sphere_collision(world, &ctx);
            continue;
          }

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
              detect_collisions(world, &ctx);
              break;
          }
        }
      }
    }
  }
}
