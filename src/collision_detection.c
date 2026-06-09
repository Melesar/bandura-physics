#include "bnd-core.h"
#include "bnd-math.h"

#include "profiler.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

typedef count_t (*collision_detection_func)(bnd_world *world, const collision_detection_context *ctx);

typedef enum {
  QUAD_LEFT = 1,
  QUAD_RIGHT = 2,
  QUAD_BOTTOM = 4,
  QUAD_TOP = 8,
} outcode;

typedef struct {
  collision_detection_func func;
  bool primary;
} collision_detection_entry;

static collision_detection_entry collision_detection_table[BND_SHAPES_COUNT][BND_SHAPES_COUNT];

static collision_detection_context ctx_inverse(collision_detection_context ctx) {
  return (collision_detection_context){
    .world = ctx.world,
    .data_a = ctx.data_b,
    .data_b = ctx.data_a,
    .body_a = ctx.body_b,
    .body_b = ctx.body_a,
    .shape_a = ctx.shape_b,
    .shape_b = ctx.shape_a,
  };
}

static bnd_v3 body_center_ex(bnd_v3 shape_offset, bnd_quat global_rotation, bnd_v3 body_position) {
  bnd_v3 center = shape_offset;
  center = bnd_v3_rotate(center, global_rotation);
  center = bnd_v3_add(center, body_position);

  return center;
}

static bnd_v3 body_a_center(const collision_detection_context *ctx) {
  return body_center_ex(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static bnd_v3 body_b_center(const collision_detection_context *ctx) {
  return body_center_ex(ctx->shape_b.offset, ctx->data_b->rotations[ctx->body_b], ctx->data_b->positions[ctx->body_b]);
}

bnd_v3 body_center(const shape_context *ctx) {
  return body_center_ex(ctx->shape.offset, ctx->data->rotations[ctx->index], ctx->data->positions[ctx->index]);
}

bnd_quat body_a_rotation(const collision_detection_context *ctx) {
  return bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
}

bnd_quat body_b_rotation(const collision_detection_context *ctx) {
  return bnd_quat_mul(ctx->data_b->rotations[ctx->body_b], ctx->shape_b.rotation);
}

bnd_quat body_rotation(const shape_context *ctx) {
  return bnd_quat_mul(ctx->data->rotations[ctx->index], ctx->shape.rotation);
}

bool aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b) {
  const bnd_aabb *a = &data_a->aabbs[index_a];
  const bnd_aabb *b = &data_b->aabbs[index_b];

  if (fabsf(a->center.x - b->center.x) > a->half_extents.x + b->half_extents.x) {
    return false;
  }

  if (fabsf(a->center.y - b->center.y) > a->half_extents.y + b->half_extents.y) {
    return false;
  }

  if (fabsf(a->center.z - b->center.z) > a->half_extents.z + b->half_extents.z) {
    return false;
  }

  return true;
}

static support_point sphere_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = bnd_v3_add(ctx->data->positions[ctx->index], ctx->shape.offset);
  float radius = ctx->shape.value.sphere.radius;

  return (support_point) { bnd_v3_add(center, bnd_v3_scale(direction, radius)), 0 };
}

static support_point box_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);
  bnd_quat inv_rotation = bnd_quat_invert(rotation);

  bnd_v3 local_direction = bnd_v3_normalize(bnd_v3_rotate(direction, inv_rotation));
  bnd_v3 v = (bnd_v3) {
    (local_direction.x > 0 ? 1 : -1) * ctx->shape.value.box.size.x * 0.5,
    (local_direction.y > 0 ? 1 : -1) * ctx->shape.value.box.size.y * 0.5,
    (local_direction.z > 0 ? 1 : -1) * ctx->shape.value.box.size.z * 0.5
  };

  v = bnd_v3_rotate(v, rotation);
  v = bnd_v3_add(center, v);

  uint16_t index = 0;
  if (local_direction.x > 0 && local_direction.z > 0) {
    index = 1;
  } else if (local_direction.x <= 0 && local_direction.z > 0) {
    index = 2;
  } else if (local_direction.x <= 0 && local_direction.z <= 0) {
    index = 3;
  }
  index += local_direction.y > 0 ? 4 : 0;

  return (support_point) { v, index };
}

static support_point capsule_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);
  bnd_quat inv_rotation = bnd_quat_invert(rotation);

  bnd_v3 local_direction = bnd_v3_normalize(bnd_v3_rotate(direction, inv_rotation));

  float radius = ctx->shape.value.capsule.radius;
  float height = ctx->shape.value.capsule.height;

  bnd_v3 cap = { 0, (local_direction.y >= 0 ? 1 : -1) * height * 0.5f, 0 };
  bnd_v3 p = bnd_v3_add(cap, bnd_v3_scale(local_direction, radius));

  p = bnd_v3_rotate(p, rotation);
  p = bnd_v3_add(p, center);

  return (support_point) { p, 0 };
}

static support_point mesh_support(const shape_context *ctx, bnd_v3 direction) {
  const mesh_storage *meshes = &ctx->world->meshes;
  const bnd_mesh_handle mesh_handle = ctx->shape.value.mesh;

  bnd_quat rotation = body_rotation(ctx);
  bnd_v3 position = body_center(ctx);
  bnd_v3 local_direction = bnd_v3_rotate(direction, bnd_quat_invert(rotation));

  bnd_mesh mesh = meshes->meshes[mesh_handle];
  count_t submesh_start = mesh.submesh_offset;
  count_t submesh_end = submesh_start + mesh.submesh_count;

  float max_dot = -FLT_MAX;
  count_t max_vertex = ~0;
  for (count_t mesh_index = submesh_start; mesh_index < submesh_end; ++mesh_index) {
    submesh submesh = meshes->submeshes[mesh_index];
    count_t vertex_start = submesh.vertex_offset;
    count_t vertex_end = vertex_start + submesh.vertex_count;

    for (count_t vertex_index = vertex_start; vertex_index < vertex_end; ++vertex_index) {
      bnd_v3 vertex = meshes->verticies[vertex_index];
      float d = bnd_v3_dot(vertex, local_direction);

      if (d > max_dot) {
        max_dot = d;
        max_vertex = vertex_index;
      }
    }
  }

  bnd_v3 support = meshes->verticies[max_vertex];
  support = bnd_v3_rotate(support, rotation);
  support = bnd_v3_add(support, position);

  return (support_point) { support, max_vertex };
}

support_func support_functions[] = { box_support, sphere_support, capsule_support, mesh_support };

body_support support(const collision_detection_context *ctx, bnd_v3 direction) {
  PROFILE_FUNCTION

  shape_context sa = { ctx->world, ctx->data_a, ctx->shape_a, ctx->body_a };
  shape_context sb = { ctx->world, ctx->data_b, ctx->shape_b, ctx->body_b };

  body_support result;
  result.p1 = support_functions[ctx->shape_a.type](&sa, direction);
  result.p2 = support_functions[ctx->shape_b.type](&sb, bnd_v3_negate(direction));
  result.p = bnd_v3_sub(result.p1.point, result.p2.point);

  return result;
}

static contact *new_contact(bnd_world *world, const collision_detection_context *ctx) {
  return contacts_new_default(world, ctx->body_a, ctx->body_b);
}

static count_t sphere_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 center_a = ctx->data_a->positions[ctx->body_a];
  bnd_v3 center_b = ctx->data_b->positions[ctx->body_b];

  float radius_a = ctx->shape_a.value.sphere.radius;
  float radius_b = ctx->shape_b.value.sphere.radius;

  bnd_v3 offset = bnd_v3_sub(center_a, center_b);
  float distance = bnd_v3_len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return 0;
  }

  bnd_v3 normal = distance > EPSILON ? bnd_v3_scale(offset, 1 / distance) : bnd_v3_up();

  contact *c = new_contact(world, ctx);
  if (c == NULL) {
    return 0;
  }

  c->point = bnd_v3_add(center_b, bnd_v3_scale(normal, radius_b + penetration));
  c->normal = normal;
  c->depth = -penetration;

  return 1;
}

static count_t capsule_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 capsule_center = body_a_center(ctx);
  bnd_quat capsule_rotation = body_a_rotation(ctx);
  bnd_quat capsule_inv_rotation = bnd_quat_invert(capsule_rotation);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_half_height = ctx->shape_a.value.capsule.height * 0.5;

  bnd_v3 sphere_center = body_b_center(ctx);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, capsule_center), capsule_inv_rotation);
  float sphere_radius = ctx->shape_b.value.sphere.radius;

  if (fabsf(local_sphere_center.y) < capsule_half_height) {
    bnd_v3 horizontal_offset = { local_sphere_center.x, 0, local_sphere_center.z };
    float horizontal_distance = bnd_v3_len(horizontal_offset);

    if (horizontal_distance < capsule_radius) {
      contact *c = new_contact(world, ctx);
      if (c == NULL) {
        return 0;
      }

      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = horizontal_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_right(), capsule_rotation);
      c->depth = capsule_radius - horizontal_distance + sphere_radius;

      return 1;
    } else if (horizontal_distance < capsule_radius + sphere_radius) {
      contact *c = new_contact(world, ctx);
      if (c == NULL) {
        return 0;
      }

      bnd_v3 closest = bnd_v3_scale(horizontal_offset, capsule_radius / horizontal_distance);
      closest.y = local_sphere_center.y;
      closest = bnd_v3_rotate(closest, capsule_rotation);

      c->point = bnd_v3_add(capsule_center, closest);
      c->normal = bnd_v3_normalize(bnd_v3_negate(closest));
      c->depth = sphere_radius - horizontal_distance + capsule_radius;

      return 1;
    } else {
      return 0;
    }
  } else {
    bnd_v3 local_caps[] = {
      (bnd_v3) { 0, capsule_half_height, 0 },
      (bnd_v3) { 0, -capsule_half_height, 0 },
    };

    bnd_v3 cap = local_sphere_center.y > capsule_half_height ? local_caps[0] : local_caps[1];
    bnd_v3 cap_offset = (bnd_v3) { local_sphere_center.x, local_sphere_center.y - cap.y, local_sphere_center.z };

    float cap_distance = bnd_v3_len(cap_offset);
    if (cap_distance < capsule_radius) {
      contact *c = new_contact(world, ctx);
      if (c == NULL) {
        return 0;
      }

      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = cap_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
      c->depth = capsule_radius - cap_distance + sphere_radius;

      return 1;
    } else if (cap_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(cap_offset, capsule_radius / cap_distance);
      closest = bnd_v3_add(cap, closest);

      contact *c = new_contact(world, ctx);
      if (c == NULL) {
        return 0;
      }

      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      c->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation));
      c->depth = sphere_radius - cap_distance + capsule_radius;

      return 1;
    } else {
      return 0;
    }
  }
}

static count_t box_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 half_extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5);
  bnd_v3 box_center = body_a_center(ctx);
  bnd_quat box_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_quat inv_box_rotation = bnd_quat_invert(box_rotation);

  bnd_v3 sphere_center = bnd_v3_add(ctx->data_b->positions[ctx->body_b], ctx->shape_b.offset);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, box_center), inv_box_rotation);
  float r = ctx->shape_b.value.sphere.radius;

  bnd_v3 closest = {
    fmaxf(-half_extents.x, fminf(local_sphere_center.x, half_extents.x)),
    fmaxf(-half_extents.y, fminf(local_sphere_center.y, half_extents.y)),
    fmaxf(-half_extents.z, fminf(local_sphere_center.z, half_extents.z))
  };

  float distancesqr = bnd_v3_distancesqr(closest, local_sphere_center);
  if (distancesqr > r * r) {
    return 0;
  }

  float *s = (float *)&half_extents;
  float *c = (float *)&closest;

  float depth = 0;
  float local_normal[3] = {0};

  if (fabsf(distancesqr) < EPSILON) {
    // Sphere is inside the box
    float min_dist = FLT_MAX;
    int min_axis = -1;
    for (int i = 0; i < 3; ++i) {
      float dist = s[i] - fabsf(c[i]);
      if (dist < min_dist) {
        min_dist = dist;
        min_axis = i;
      }
    }

    local_normal[min_axis] = c[min_axis] > 0 ? -1 : 1;
    depth = min_dist + r;
  } else {
    bnd_v3 diff = bnd_v3_sub(closest, local_sphere_center);
    float dist = sqrtf(distancesqr);
    depth = r - dist;

    diff = bnd_v3_scale(diff, 1.0f / dist);
    memcpy(&local_normal, &diff, sizeof(bnd_v3));
  }

  contact *contact = new_contact(world, ctx);
  if (contact == NULL) {
    return 0;
  }

  memcpy(&contact->normal, local_normal, sizeof(bnd_v3));

  contact->point = bnd_v3_add(box_center, bnd_v3_rotate(closest, box_rotation));
  contact->normal = bnd_v3_rotate(contact->normal, box_rotation);
  contact->depth = depth;

  return 1;
}

static count_t box_capsule_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 box_half_size = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5);
  bnd_quat box_rotation = body_a_rotation(ctx);
  bnd_quat inv_box_rotation = bnd_quat_invert(box_rotation);

  bnd_v3 capsule_center = body_b_center(ctx);
  float capsule_height = ctx->shape_b.value.capsule.height;
  float capsule_radius = ctx->shape_b.value.capsule.radius;
  bnd_quat capsule_rotation = body_b_rotation(ctx);

  bnd_v3 local_caps[2];
  for (count_t i = 0; i < 2; ++i) {
    bnd_v3 cap = { 0, (i == 0 ? 1 : -1) * 0.5 * capsule_height, 0 };
    cap = bnd_v3_rotate(cap, capsule_rotation);
    cap = bnd_v3_add(cap, capsule_center);
    cap = bnd_v3_rotate(cap, inv_box_rotation);

    local_caps[i] = cap;
  }

  float *half_sizes = (float *)&box_half_size;
  for (count_t axis_normal = 0; axis_normal < 3; ++axis_normal) {
    count_t axis_a = (axis_normal + 1) % 3;
    count_t axis_b = (axis_normal + 2) % 3;
    int sign = 1;

    bnd_v3 p;
    float *pp = (float *)&p;
    pp[axis_a] = -half_sizes[axis_a];
    pp[axis_b] = -half_sizes[axis_b];

    for (count_t i = 0; i < 2; ++i) {
      pp[axis_normal] = sign * half_sizes[axis_normal];

      bnd_v3 offsets[] = { bnd_v3_sub(local_caps[0], p), bnd_v3_sub(local_caps[1], p) };
      float *po[] = { (float*)&offsets[0], (float*)&offsets[1] };
      float signed_distances[] = {sign * po[0][axis_normal], sign * po[1][axis_normal] };

      if (signed_distances[0] > capsule_radius && signed_distances[1] > capsule_radius) {
        // Both cylinder caps are above the face and further than the radius. Two shapes do not intersect.
        return 0;
      }

      if (signed_distances[0] < -half_sizes[axis_normal] || signed_distances[1] < -half_sizes[axis_normal]) {
        // A cap is on the other side of the box
        sign = -1;
        continue;
      }

      float a_max = 2.0 * half_sizes[axis_a];
      float b_max = 2.0 * half_sizes[axis_b];

      // Cohen–Sutherland algorithm to determine if a line intersects a rectangle.
      // Here it's used to detect if the capsule's axis passes through the face when projected on its plane.
      //
      // https://en.wikipedia.org/wiki/Cohen%E2%80%93Sutherland_algorithm
      uint8_t outcodes[2] = { 0 };
      for (count_t k = 0; k < 2; ++k) {
        if (po[k][axis_a] < 0) outcodes[k] |= QUAD_LEFT;
        else if (po[k][axis_a] > a_max) outcodes[k] |= QUAD_RIGHT;
        if (po[k][axis_b] < 0) outcodes[k] |= QUAD_BOTTOM;
        else if (po[k][axis_b] > b_max) outcodes[k] |= QUAD_TOP;
      }

      if (outcodes[0] & outcodes[1]) {
        // The capsule misses the box's face.
        sign = -1;
        continue;
      }

      bnd_v3 box_center = body_a_center(ctx);

      count_t contacts_count = 0;
      for (count_t k = 0; k < 2; ++k) {
        if (signed_distances[k] > capsule_radius) {
          continue;
        }

        contact *c = new_contact(world, ctx);
        if (c == NULL) {
          return contacts_count;
        }

        bnd_v3 point = { 0 };
        pp = (float*)&point;
        pp[axis_normal] = sign * half_sizes[axis_normal];

        float *pcaps[] = { (float*)&local_caps[0], (float*)&local_caps[1] };
        if (outcodes[k] == 0) {
          // Capsule's cap projection is inside the face.
          pp[axis_a] = pcaps[k][axis_a];
          pp[axis_b] = pcaps[k][axis_b];
        } else {
          // Clip the projection to the face's boundary.
          float x0 = pcaps[0][axis_a];
          float x1 = pcaps[1][axis_a];
          float y0 = pcaps[0][axis_b];
          float y1 = pcaps[1][axis_b];
          float xmin = -half_sizes[axis_a];
          float xmax = half_sizes[axis_a];
          float ymin = -half_sizes[axis_b];
          float ymax = half_sizes[axis_b];
          if (outcodes[k] & QUAD_TOP) {
            pp[axis_a] = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
            pp[axis_b] = ymax;
          } else if (outcodes[k] & QUAD_BOTTOM) {
            pp[axis_a] = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
            pp[axis_b] = ymin;
          } else if (outcodes[k] & QUAD_RIGHT) {
            pp[axis_b] = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
            pp[axis_a] = xmax;
          } else if (outcodes[k] & QUAD_LEFT) {
            pp[axis_b] = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
            pp[axis_a] = xmin;
          }
        }

        point = bnd_v3_rotate(point, box_rotation);
        point = bnd_v3_add(point, box_center);

        bnd_v3 normal = { 0 };
        ((float*)&normal)[axis_normal] = sign;
        normal = bnd_v3_rotate(normal, box_rotation);
        normal = bnd_v3_negate(normal);

        c->point = point;
        c->normal = normal;
        c->depth = capsule_radius - signed_distances[k];

        contacts_count += 1;
      }

      return contacts_count;
    }
  }

  return 0;
}

static count_t box_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  bnd_quat shape_rotation = ctx->shape_a.rotation;

  bnd_v3 box_center = body_a_center(ctx);
  bnd_v3 extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5);

  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];

  bnd_v3 corners[] = {
    { extents.x, extents.y, extents.z },
    { extents.x, -extents.y, extents.z },
    { extents.x, -extents.y, -extents.z },
    { extents.x, extents.y, -extents.z },
    { -extents.x, extents.y, extents.z },
    { -extents.x, -extents.y, extents.z },
    { -extents.x, -extents.y, -extents.z },
    { -extents.x, extents.y, -extents.z },
  };

  const count_t max_contacts = 4;

  bnd_error e = contacts_ensure_capacity(world, max_contacts);
  if (e.type != BND_OK) {
    return 0;
  }

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    bnd_v3 corner = bnd_v3_add(box_center, bnd_v3_rotate(bnd_v3_rotate(corners[i], shape_rotation), box_rotation));
    float distance = bnd_v3_dot(bnd_v3_sub(corner, plane_point), plane_normal);
    if (distance > 0) {
      continue;
    }

    contact *c = new_contact(world, ctx);
    if (c == NULL) {
      return 0;
    }
    c->normal = plane_normal;
    c->point = bnd_v3_add(corner, bnd_v3_scale(plane_normal, -0.5 * distance));
    c->depth = -distance;

    contact_count += 1;
  }

  return contact_count;
}

static count_t sphere_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 sphere_center = body_a_center(ctx);
  float sphere_radius = ctx->shape_a.value.sphere.radius;

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  float plane_sphere_distance = bnd_v3_dot(bnd_v3_sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius) {
    return 0;
  }

  contact *contact = new_contact(world, ctx);
  if (contact == NULL) {
    return 0;
  }

  contact->normal = plane_normal;
  contact->point = bnd_v3_add(sphere_center, bnd_v3_scale(plane_normal, -plane_sphere_distance));
  contact->depth = sphere_radius - plane_sphere_distance;

  return 1;
}

static count_t capsule_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 capsule_center = body_a_center(ctx);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_height = ctx->shape_a.value.capsule.height;

  bnd_quat capsule_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_v3 capsule_axis = bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
  bnd_v3 cap_top = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, capsule_height * 0.5));
  bnd_v3 cap_bottom = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, -capsule_height * 0.5));

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 points[] = { cap_top, cap_bottom };

  count_t contact_count = 0;
  for (int i = 0; i < 2; ++i) {
    bnd_v3 offset = bnd_v3_sub(points[i], plane_point);
    float d = bnd_v3_dot(offset, plane_normal);
    if (d > capsule_radius) {
      continue;
    }

    contact *c = new_contact(world, ctx);
    if (c == NULL) {
      return contact_count;
    }

    c->point = bnd_v3_add(points[i], bnd_v3_scale(plane_normal, -d));
    c->normal = plane_normal;
    c->depth = capsule_radius - d;

    contact_count += 1;
  }

  return contact_count;
}

static count_t mesh_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 mesh_center = body_a_center(ctx);
  bnd_quat mesh_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_quat inv_mesh_rotation = bnd_quat_invert(mesh_rotation);

  bnd_v3 local_normal = bnd_v3_rotate(plane_normal, inv_mesh_rotation);
  bnd_v3 local_point = bnd_v3_rotate(bnd_v3_sub(plane_point, mesh_center), inv_mesh_rotation);

  const mesh_storage *meshes = &world->meshes;
  const bnd_mesh_handle mesh_handle = ctx->shape_a.value.mesh;

  bnd_mesh mesh = meshes->meshes[mesh_handle];
  count_t submesh_start = mesh.submesh_offset;
  count_t submesh_end = submesh_start + mesh.submesh_count;

  float min_dot = FLT_MAX;
  count_t collision_vertex = 0;
  for (count_t mesh_index = submesh_start; mesh_index < submesh_end; ++mesh_index) {
    count_t vertex_start = meshes->submeshes[mesh_index].vertex_offset;
    count_t vertex_end = vertex_start + meshes->submeshes[mesh_index].vertex_count;

    for (count_t vertex_index = vertex_start; vertex_index < vertex_end; ++vertex_index) {
      bnd_v3 vertex = meshes->verticies[vertex_index];
      bnd_v3 offset = bnd_v3_sub(vertex, local_point);
      float d = bnd_v3_dot(offset, local_normal);

      if (d < min_dot) {
        min_dot = d;
        collision_vertex = vertex_index;
      }
    }
  }

  if (min_dot > 0) {
    return 0;
  }

  bnd_v3 point = meshes->verticies[collision_vertex];
  point = bnd_v3_rotate(point, mesh_rotation);
  point = bnd_v3_add(point, mesh_center);
  point = bnd_v3_add(point, bnd_v3_scale(plane_normal, -min_dot)); // Project the deepest vertex back on the plane.

  contact *c = new_contact(world, ctx);
  if (c == NULL) {
    return 0;
  }

  c->point = point;
  c->normal = plane_normal;
  c->depth = -min_dot;

  return 1;
}

static count_t polytope_polytope_collision(bnd_world *world, const collision_detection_context *ctx) {
  simplex s;
  if (!gjk_check_intersection(world, ctx, &s)) {
    return 0;
  }

  contact *c = new_contact(world, ctx);
  if (c == NULL) {
    return 0;
  }

  epa_get_contact(ctx, &s, world->config.advanced.epa_tolerance, c);

  return 1;
}

static count_t collisions_detect(bnd_world *world, const common_data *data_b, bool loop_all) {
  const common_data *dynamics = (common_data *)&world->dynamics;

  collision_detection_context ctx = {
    .world = world,
    .data_a = dynamics,
    .data_b = data_b,
  };

  count_t count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    count_t until = loop_all ? data_b->count : i;
    for (count_t j = 0; j < until; ++j) {
      if (!aabb_intersect(dynamics, data_b, i, j)) {
        continue;
      }

      ctx.body_a = i;
      ctx.body_b = j;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = data_b->shapes[j];

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
          ctx.shape_b = shape_b;

          collision_detection_entry entry = collision_detection_table[shape_a.type][shape_b.type];
          if (entry.func == NULL) {
            continue;
          }

          if (entry.primary) {
            count += entry.func(world, &ctx);
          } else {
            collision_detection_context inv_ctx = ctx_inverse(ctx);
            count_t new_contacts = entry.func(world, &inv_ctx);

            for (count_t k = world->contacts.count - new_contacts; k < world->contacts.count; ++k) {
              contact *c = &world->contacts.values[k];
              c->index_a = ctx.body_a;
              c->index_b = ctx.body_b;
              c->normal = bnd_v3_negate(c->normal);
            }

            count += new_contacts;
          }
        }
      }
    }
  }

  return count;
}

count_t collisions_detect_dynamic(bnd_world *world) {
  const common_data *dynamics = (common_data *)&world->dynamics;
  return collisions_detect(world, dynamics, false);
}

void collisions_detect_static(bnd_world *world) {
  const common_data *statics = (common_data *)&world->statics;
  collisions_detect(world, statics, true);
}

void collision_detection_init(bnd_world *world) {
  memset(collision_detection_table, 0, sizeof(collision_detection_table));

  collision_detection_table[BND_SPHERE][BND_SPHERE] = (collision_detection_entry) { sphere_sphere_collision, true };
  collision_detection_table[BND_BOX][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true };
  collision_detection_table[BND_CAPSULE][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true };
  collision_detection_table[BND_MESH][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, true };

  collision_detection_table[BND_BOX][BND_SPHERE] = (collision_detection_entry) { box_sphere_collision, true };
  collision_detection_table[BND_SPHERE][BND_BOX] = (collision_detection_entry) { box_sphere_collision, false };
  collision_detection_table[BND_BOX][BND_CAPSULE] = (collision_detection_entry) { box_capsule_collision, true };
  collision_detection_table[BND_CAPSULE][BND_BOX] = (collision_detection_entry) { box_capsule_collision, false };
  collision_detection_table[BND_CAPSULE][BND_SPHERE] = (collision_detection_entry) { capsule_sphere_collision, true };
  collision_detection_table[BND_SPHERE][BND_CAPSULE] = (collision_detection_entry) { capsule_sphere_collision, false };

  collision_detection_table[BND_MESH][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true };
  collision_detection_table[BND_BOX][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false };
  collision_detection_table[BND_MESH][BND_SPHERE] = (collision_detection_entry) { polytope_polytope_collision, true };
  collision_detection_table[BND_SPHERE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false };
  collision_detection_table[BND_MESH][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true };
  collision_detection_table[BND_CAPSULE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false };

  collision_detection_table[BND_BOX][BND_PLANE] = (collision_detection_entry) { box_plane_collision, true };
  collision_detection_table[BND_SPHERE][BND_PLANE] = (collision_detection_entry) { sphere_plane_collision, true };
  collision_detection_table[BND_CAPSULE][BND_PLANE] = (collision_detection_entry) { capsule_plane_collision, true };
  collision_detection_table[BND_MESH][BND_PLANE] = (collision_detection_entry) { mesh_plane_collision, true };
}
