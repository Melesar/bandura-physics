#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include "profiler.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#define PROFILING_BLOCK_NAME "Contacts cache"

typedef bool (*collision_detection_func)(const collision_detection_context *ctx, contact_manifold *manifold);

typedef struct {
  collision_detection_func func;
  bool primary;
  bool use_cache;
} collision_detection_entry;

static collision_detection_entry collision_detection_table[BND_SHAPES_COUNT][BND_SHAPES_COUNT];

static void box_corners(bnd_v3 half_extents, bnd_v3 corners[8])  {
  corners[0] = (bnd_v3){ half_extents.x, half_extents.y, half_extents.z };
  corners[1] = (bnd_v3){ half_extents.x, half_extents.y, -half_extents.z };
  corners[2] = (bnd_v3){ half_extents.x, -half_extents.y, half_extents.z };
  corners[3] = (bnd_v3){ half_extents.x, -half_extents.y, -half_extents.z };
  corners[4] = (bnd_v3){ -half_extents.x, half_extents.y, half_extents.z };
  corners[5] = (bnd_v3){ -half_extents.x, half_extents.y, -half_extents.z };
  corners[6] = (bnd_v3){ -half_extents.x, -half_extents.y, half_extents.z };
  corners[7] = (bnd_v3){ -half_extents.x, -half_extents.y, -half_extents.z };
}

static collision_detection_context ctx_inverse(collision_detection_context ctx) {
  return (collision_detection_context){
    .world = ctx.world,
    .data_a = ctx.data_b,
    .data_b = ctx.data_a,
    .contacts_offset = ctx.contacts_offset,
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

static bool aabb_intersect(const bnd_aabb *a, const bnd_aabb *b) {
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

bool body_aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b) {
  const bnd_aabb *a = &data_a->aabbs[index_a];
  const bnd_aabb *b = &data_b->aabbs[index_b];

  return aabb_intersect(a, b);
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
    (local_direction.x > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.x * 0.5f,
    (local_direction.y > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.y * 0.5f,
    (local_direction.z > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.z * 0.5f
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
  count_t max_vertex = UINT32_MAX;
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

  return (support_point) { support, (uint16_t)(max_vertex & 0xFFFF) };
}

support_func support_functions[] = { box_support, sphere_support, capsule_support, mesh_support };

body_support support(const collision_detection_context *ctx, bnd_v3 direction) {
  shape_context sa = { ctx->world, ctx->data_a, ctx->shape_a, ctx->body_a };
  shape_context sb = { ctx->world, ctx->data_b, ctx->shape_b, ctx->body_b };

  body_support result;
  result.p1 = support_functions[ctx->shape_a.type](&sa, direction);
  result.p2 = support_functions[ctx->shape_b.type](&sb, bnd_v3_negate(direction));
  result.p = bnd_v3_sub(result.p1.point, result.p2.point);

  return result;
}

static bool sphere_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 center_a = body_a_center(ctx);
  bnd_v3 center_b = body_b_center(ctx);

  float radius_a = ctx->shape_a.value.sphere.radius;
  float radius_b = ctx->shape_b.value.sphere.radius;

  bnd_v3 offset = bnd_v3_sub(center_a, center_b);
  float distance = bnd_v3_len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return false;
  }

  manifold->count = 1;
  manifold->normal = distance > EPSILON ? bnd_v3_scale(offset, 1 / distance) : bnd_v3_up();

  manifold->points[0].point = bnd_v3_add(center_b, bnd_v3_scale(manifold->normal, radius_b + penetration));
  manifold->points[0].depth = -penetration;

  return true;
}

static bool capsule_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 capsule_center = body_a_center(ctx);
  bnd_quat capsule_rotation = body_a_rotation(ctx);
  bnd_quat capsule_inv_rotation = bnd_quat_invert(capsule_rotation);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_half_height = ctx->shape_a.value.capsule.height * 0.5f;

  bnd_v3 sphere_center = body_b_center(ctx);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, capsule_center), capsule_inv_rotation);
  float sphere_radius = ctx->shape_b.value.sphere.radius;

  if (fabsf(local_sphere_center.y) < capsule_half_height) {
    bnd_v3 horizontal_offset = { local_sphere_center.x, 0.0f, local_sphere_center.z };
    float horizontal_distance = bnd_v3_len(horizontal_offset);

    if (horizontal_distance < capsule_radius) {
      manifold->count = 1;
      manifold->normal = horizontal_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_right(), capsule_rotation);

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      manifold->points[0].depth = capsule_radius - horizontal_distance + sphere_radius;

      return true;
    } else if (horizontal_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(horizontal_offset, capsule_radius / horizontal_distance);
      closest.y = local_sphere_center.y;

      manifold->count = 1;
      manifold->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation));

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      manifold->points[0].depth = sphere_radius - horizontal_distance + capsule_radius;

      return true;
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
      manifold->count = 1;
      manifold->normal = cap_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_up(), capsule_rotation);

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      manifold->points[0].depth = capsule_radius - cap_distance + sphere_radius;

      return true;
    } else if (cap_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(cap_offset, capsule_radius / cap_distance);
      closest = bnd_v3_add(cap, closest);

      manifold->count = 1;
      manifold->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation));

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      manifold->points[0].depth = sphere_radius - cap_distance + capsule_radius;

      return true;
    }
  }

  return false;
}

static bool box_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 half_extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5);
  bnd_v3 box_center = body_a_center(ctx);
  bnd_quat box_rotation = body_a_rotation(ctx);
  bnd_quat inv_box_rotation = bnd_quat_invert(box_rotation);

  bnd_v3 sphere_center = body_b_center(ctx);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, box_center), inv_box_rotation);
  float r = ctx->shape_b.value.sphere.radius;

  bnd_v3 closest = {
    fmaxf(-half_extents.x, fminf(local_sphere_center.x, half_extents.x)),
    fmaxf(-half_extents.y, fminf(local_sphere_center.y, half_extents.y)),
    fmaxf(-half_extents.z, fminf(local_sphere_center.z, half_extents.z))
  };

  float distancesqr = bnd_v3_distancesqr(closest, local_sphere_center);
  if (distancesqr > r * r) {
    return false;
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

    local_normal[min_axis] = c[min_axis] > 0 ? -1.0f : 1.0f;
    depth = min_dist + r;
  } else {
    bnd_v3 diff = bnd_v3_sub(closest, local_sphere_center);
    float dist = sqrtf(distancesqr);
    depth = r - dist;

    diff = bnd_v3_scale(diff, 1.0f / dist);
    memcpy(&local_normal, &diff, sizeof(bnd_v3));
  }

  bnd_v3 normal;
  memcpy(&normal, local_normal, sizeof(normal));

  manifold->count = 1;
  manifold->normal = bnd_v3_rotate(normal, box_rotation);
  manifold->points[0].point = bnd_v3_add(box_center, bnd_v3_rotate(closest, box_rotation));
  manifold->points[0].depth = depth;

  return true;
}

static bool box_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  bnd_quat shape_rotation = ctx->shape_a.rotation;

  bnd_v3 box_center = body_a_center(ctx);
  bnd_v3 extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5f);

  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];

  bnd_v3 corners[8];
  box_corners(extents, corners);

  const count_t max_contacts = 4;

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    bnd_v3 corner = bnd_v3_add(box_center, bnd_v3_rotate(bnd_v3_rotate(corners[i], shape_rotation), box_rotation));
    float distance = bnd_v3_dot(bnd_v3_sub(corner, plane_point), plane_normal);
    if (distance > 0) {
      continue;
    }

    manifold->points[contact_count].point = bnd_v3_add(corner, bnd_v3_scale(plane_normal, -0.5f * distance));
    manifold->points[contact_count].depth = -distance;

    contact_count += 1;
  }

  manifold->normal = plane_normal;
  manifold->count = contact_count;

  return contact_count > 0;
}

static bool sphere_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 sphere_center = body_a_center(ctx);
  float sphere_radius = ctx->shape_a.value.sphere.radius;

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  float plane_sphere_distance = bnd_v3_dot(bnd_v3_sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius) {
    return false;
  }

  manifold->count = 1;
  manifold->normal = plane_normal;
  manifold->points[0].point = bnd_v3_add(sphere_center, bnd_v3_scale(plane_normal, -plane_sphere_distance));
  manifold->points[0].depth = sphere_radius - plane_sphere_distance;

  return true;
}

static bool capsule_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 capsule_center = body_a_center(ctx);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_height = ctx->shape_a.value.capsule.height;

  bnd_quat capsule_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_v3 capsule_axis = bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
  bnd_v3 cap_top = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, capsule_height * 0.5f));
  bnd_v3 cap_bottom = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, -capsule_height * 0.5f));

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

    manifold->points[contact_count].point = bnd_v3_add(points[i], bnd_v3_scale(plane_normal, -d));
    manifold->points[contact_count].depth = capsule_radius - d;

    contact_count += 1;
  }

  manifold->normal = plane_normal;
  manifold->count = contact_count;

  return contact_count > 0;
}

static bool mesh_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 mesh_center = body_a_center(ctx);
  bnd_quat mesh_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_quat inv_mesh_rotation = bnd_quat_invert(mesh_rotation);

  bnd_v3 local_normal = bnd_v3_rotate(plane_normal, inv_mesh_rotation);
  bnd_v3 local_point = bnd_v3_rotate(bnd_v3_sub(plane_point, mesh_center), inv_mesh_rotation);

  const mesh_storage *meshes = &ctx->world->meshes;
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
    return false;
  }

  bnd_v3 point = meshes->verticies[collision_vertex];
  point = bnd_v3_rotate(point, mesh_rotation);
  point = bnd_v3_add(point, mesh_center);
  point = bnd_v3_add(point, bnd_v3_scale(plane_normal, -min_dot)); // Project the deepest vertex back on the plane.

  manifold->count = 1;
  manifold->normal = plane_normal;
  manifold->points[0].point = point;
  manifold->points[0].depth = -min_dot;

  return true;
}

static bool polytope_polytope_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  simplex s;
  if (!gjk_check_intersection(ctx->world, ctx, &s)) {
    return false;
  }

  epa_get_contact(ctx, &s, ctx->world->config.collision_detection.epa_tolerance, manifold);
  return true;
}

bnd_error collision_detection_epa_context(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, collision_detection_context *ctx) {
  char *message = "EPA debugging requires two distinct single-shape bodies that use EPA collision detection";

  if (body_a.type > BND_BODY_STATIC || body_b.type > BND_BODY_STATIC) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle has an invalid body type" };
  }

  PROPAGATE_ERROR(bnd_handle_valid(world, body_a))
  PROPAGATE_ERROR(bnd_handle_valid(world, body_b))

  if (body_a.type == body_b.type && body_a.index == body_b.index) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  const common_data *data_a = as_common_const(world, body_a.type);
  const common_data *data_b = as_common_const(world, body_b.type);
  count_t index_a = handle_to_inner_index(world, body_a);
  count_t index_b = handle_to_inner_index(world, body_b);

  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_DYNAMIC) {
    const common_data *tmp_data = data_a;
    data_a = data_b;
    data_b = tmp_data;

    count_t tmp_index = index_a;
    index_a = index_b;
    index_b = tmp_index;
  }

  if (data_a == data_b && index_a == index_b) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  body_shapes shapes_a = data_a->shapes[index_a];
  body_shapes shapes_b = data_b->shapes[index_b];
  if (shapes_a.count != 1 || shapes_b.count != 1) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  *ctx = (collision_detection_context) {
    .world = world,
    .data_a = data_a,
    .data_b = data_b,
    .body_a = index_a,
    .body_b = index_b,
    .shape_a = shapes_get(world, shapes_a)[0],
    .shape_b = shapes_get(world, shapes_b)[0],
  };

  collision_detection_entry entry = collision_detection_table[ctx->shape_a.type][ctx->shape_b.type];
  if (entry.func != polytope_polytope_collision) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  if (!entry.primary) {
    *ctx = ctx_inverse(*ctx);
  }

  return OK;
}

// count_t collisions_detect(bnd_world *world, count_t contacts_offset, bnd_body_type type) {
//   const common_data *dynamics = as_common_const(world, BND_BODY_DYNAMIC);
//   const common_data *data_b = as_common_const(world, type);

//   collision_detection_context ctx = {
//     .world = world,
//     .data_a = dynamics,
//     .data_b = data_b,
//   };

//   count_t count = 0;

//   for (count_t i = 0; i < dynamics->count; ++i) {
//     count_t until = type == BND_BODY_DYNAMIC ? i : data_b->count;
//     for (count_t j = 0; j < until; ++j) {
//       bnd_collision_mask validation_mask = layer_to_mask(dynamics->collision_layers[i]);
//       bnd_collision_mask reference_mask = world->matrix.matrix[data_b->collision_layers[j]];
//       if ((reference_mask & validation_mask) == 0) {
//         continue;
//       }

//       if (!aabb_intersect(dynamics, data_b, i, j)) {
//         continue;
//       }

//       ctx.body_a = i;
//       ctx.body_b = j;

//       body_shapes shapes_a = dynamics->shapes[i];
//       body_shapes shapes_b = data_b->shapes[j];

//       uint64_t cached_contacts_mask = 0;
//       count_t pair_contacts_count = 0;
//       count_t pair_offset = contacts_offset + count;

//       for (count_t sa = 0; sa < shapes_a.count; ++sa) {
//         bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
//         ctx.shape_a = shape_a;

//         for (count_t sb = 0; sb < shapes_b.count; ++sb) {
//           bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
//           count_t shape_offset = pair_offset + pair_contacts_count;

//           ctx.shape_b = shape_b;
//           ctx.contacts_offset = shape_offset;

//           collision_detection_entry entry = collision_detection_table[shape_a.type][shape_b.type];
//           if (entry.func == NULL) {
//             continue;
//           }

//           collision_detection_context context = entry.primary ? ctx : ctx_inverse(ctx);
//           contact_manifold manifold = entry.func(world, &context);
//           count_t shape_contacts_count = manifold.count;

//           if (shape_contacts_count > 0 && IS_ERROR(contacts_ensure_capacity(world, shape_offset, shape_contacts_count))) {
//             continue;
//           }

//           for (count_t k = 0; k < shape_contacts_count; ++k) {
//             contact *c = new_contact(&context, k);
//             c->point = manifold.points[k].point;
//             c->normal = manifold.normal;
//             c->depth = manifold.points[k].depth;
//             c->features = manifold.points[k].features;
//           }

//           if (!entry.primary) {
//             for (count_t k = 0; k < shape_contacts_count; ++k) {
//               contact *c = &world->contacts.values[shape_offset + k];
//               c->index_a = ctx.body_a;
//               c->index_b = ctx.body_b;
//               c->normal = bnd_v3_negate(c->normal);

//               if (entry.use_cache) {
//                 bnd_v3 tmp_witness = c->features.witness_a;
//                 c->features.witness_a = c->features.witness_b;
//                 c->features.witness_b = tmp_witness;
//                 c->features.normal = bnd_v3_negate(c->features.normal);
//               }
//             }
//           }

//           if (entry.use_cache) {
//             uint64_t mask = (1 << shape_contacts_count) - 1;
//             cached_contacts_mask |= mask << pair_contacts_count;
//           }

//           pair_contacts_count += shape_contacts_count;
//         }
//       }

//       bool is_trigger = data_b->flags[j] & BODY_FLAG_TRIGGER;
//       if (is_trigger && pair_contacts_count > 0) {
//         // It's not ideal to process triggers here, since we've already done a lot of useless work.
//         // This is going to change with the introduction of contact islands.

//         if (events_subscribed((const common_data *)dynamics, i, BND_EVENT_TRIGGER)) {
//           events_push(world, (common_data *)dynamics, i, (bnd_event) {
//             .type = BND_EVENT_TRIGGER,
//             .trigger = { .other = make_body_handle(world, type, j) }
//           });
//         }

//         if (events_subscribed(data_b, j, BND_EVENT_TRIGGER)) {
//           events_push(world, (common_data *)data_b, j, (bnd_event) {
//             .type = BND_EVENT_TRIGGER,
//             .trigger = { .other = make_body_handle(world, BND_BODY_DYNAMIC, i) }
//           });
//         }

//         break;
//       }

//       count_t filtered_contact_indices[MAX_CONTACTS_PER_PAIR] = {0};
//       if (cached_contacts_mask == 0) {
//         if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
//           contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);
//           pair_contacts_count = MAX_CONTACTS_PER_PAIR;
//         }

//         count += pair_contacts_count;
//         continue;
//       }

//       PROFILER_BLOCK_START(PROFILING_BLOCK_NAME);

//       cache_entry *cached_entry = contacts_cache_query(world, world->contacts.values + pair_offset, type);
//       if (cached_entry == NULL) {
//         PROFILER_BLOCK_END;
//         continue;
//       }

//       float distance_threshold = world->config.advanced.contacts_cache.feature_distance_threshold;
//       float distance_threshold_sqr = distance_threshold * distance_threshold;
//       float separation_threshold = world->config.advanced.contacts_cache.separation_threshold;

//       bnd_v3 position_a = ctx.data_a->positions[ctx.body_a];
//       bnd_v3 position_b = ctx.data_b->positions[ctx.body_b];
//       bnd_quat rotation_a = ctx.data_a->rotations[ctx.body_a];
//       bnd_quat rotation_b = ctx.data_b->rotations[ctx.body_b];

//       uint8_t picked_features = 0;
//       for (count_t k = 0; k < pair_contacts_count; ++k) {
//         if ((cached_contacts_mask & (UINT64_C(1) << k)) == 0) {
//           continue;
//         }

//         contact *c = &world->contacts.values[pair_offset + k];
//         contact_features *features = &c->features;

//         bnd_quat inv_rotation_a = bnd_quat_invert(rotation_a);
//         bnd_quat inv_rotation_b = bnd_quat_invert(rotation_b);
//         features->witness_a = bnd_v3_rotate(bnd_v3_sub(features->witness_a, position_a), inv_rotation_a);
//         features->witness_b = bnd_v3_rotate(bnd_v3_sub(features->witness_b, position_b), inv_rotation_b);
//         features->normal = bnd_v3_rotate(features->normal, inv_rotation_a);

//         count_t matched_slot = cached_entry->feature_count;
//         for (count_t h = 0; h < cached_entry->feature_count; ++h) {
//           const contact_features *cached_features = &cached_entry->features[h];

//           float distance_a_sqr = bnd_v3_distancesqr(cached_features->witness_a, features->witness_a);
//           float distance_b_sqr = bnd_v3_distancesqr(cached_features->witness_b, features->witness_b);

//           if (distance_a_sqr <= distance_threshold_sqr && distance_b_sqr <= distance_threshold_sqr) {
//             matched_slot = h;
//             break;
//           }
//         }

//         count_t feature_count = cached_entry->feature_count;
//         if (matched_slot < feature_count) {
//           cached_entry->features[matched_slot] = *features;
//           picked_features |= 1 << matched_slot;
//         } else if (cached_entry->feature_count < MAX_CONTACTS_PER_PAIR) {
//           cached_entry->features[feature_count] = *features;
//           cached_entry->feature_count += 1;

//           picked_features |= 1 << feature_count;
//         }
//       }

//       count_t fresh_contacts_count = pair_contacts_count;
//       count_t contacts_from_cache = 0;
//       for (count_t h = 0; h < cached_entry->feature_count; ++h) {
//         if (picked_features & (1 << h)) {
//           continue;
//         }

//         const contact_features *cached_features = &cached_entry->features[h];

//         bnd_v3 witness_a_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_a, rotation_a), position_a);
//         bnd_v3 witness_b_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_b, rotation_b), position_b);
//         bnd_v3 normal_world = bnd_v3_rotate(cached_features->normal, rotation_a);

//         float separation = bnd_v3_dot(bnd_v3_sub(witness_a_world, witness_b_world), normal_world);
//         if (separation > separation_threshold) {
//           continue;
//         }

//         count_t contact_offset = pair_offset + fresh_contacts_count + contacts_from_cache;
//         if (IS_ERROR(contacts_ensure_capacity(world, contact_offset, 1))) {
//           break;
//         }

//         contact *c = &world->contacts.values[contact_offset];
//         c->index_a = ctx.body_a;
//         c->index_b = ctx.body_b;
//         c->point = bnd_v3_scale(bnd_v3_add(witness_a_world, witness_b_world), 0.5f);
//         c->normal = normal_world;
//         c->depth = -separation;
//         c->features = *cached_features;
//         c->restitution = mix_restitution(&ctx);
//         c->friction = mix_friction(&ctx);

//         contacts_from_cache += 1;
//       }

//       pair_contacts_count += contacts_from_cache;

//       if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
//         contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);

//         count_t feature_count = 0;
//         for (count_t k = 0; k < MAX_CONTACTS_PER_PAIR; ++k) {
//           contact *c = &world->contacts.values[pair_offset + k];
//           count_t original_contact_index = filtered_contact_indices[k];

//           bool fresh_cashable = original_contact_index < fresh_contacts_count && cached_contacts_mask & ((uint64_t)1 << original_contact_index);
//           bool from_cache = original_contact_index >= fresh_contacts_count;
//           if (fresh_cashable || from_cache) {
//             cached_entry->features[feature_count++] = c->features;
//           }
//         }

//         cached_entry->feature_count = feature_count;
//         pair_contacts_count = MAX_CONTACTS_PER_PAIR;
//       }

//       count += pair_contacts_count;

//       PROFILER_BLOCK_END;
//     }
//   }

//   return count;
// }

void collision_detection_init(void) {
  memset(collision_detection_table, 0, sizeof(collision_detection_table));

  collision_detection_table[BND_SPHERE][BND_SPHERE]   = (collision_detection_entry) { sphere_sphere_collision, true, false };
  collision_detection_table[BND_BOX][BND_BOX]         = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_MESH][BND_MESH]       = (collision_detection_entry) { polytope_polytope_collision, true, true };

  collision_detection_table[BND_BOX][BND_SPHERE]      = (collision_detection_entry) { box_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_BOX]      = (collision_detection_entry) { box_sphere_collision, false, false };
  collision_detection_table[BND_BOX][BND_CAPSULE]     = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_BOX]     = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_CAPSULE][BND_SPHERE]  = (collision_detection_entry) { capsule_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_CAPSULE]  = (collision_detection_entry) { capsule_sphere_collision, false, false };

  collision_detection_table[BND_MESH][BND_BOX]        = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_BOX][BND_MESH]        = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_MESH][BND_SPHERE]     = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_MESH]     = (collision_detection_entry) { polytope_polytope_collision, false, false };
  collision_detection_table[BND_MESH][BND_CAPSULE]    = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_MESH]    = (collision_detection_entry) { polytope_polytope_collision, false, false };

  collision_detection_table[BND_BOX][BND_PLANE]       = (collision_detection_entry) { box_plane_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_PLANE]    = (collision_detection_entry) { sphere_plane_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_PLANE]   = (collision_detection_entry) { capsule_plane_collision, true, false };
  collision_detection_table[BND_MESH][BND_PLANE]      = (collision_detection_entry) { mesh_plane_collision, true, false };
}

bnd_error for_each_broad_contact(bnd_world *world, broad_contact_iterator func, void *custom_data) {
  broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };
  for (count_t k = 0; k < 2; ++k) {
    broad_contacts_set *contacts = sets[k];

    count_t body_contact_index = contacts->first;
    while (body_contact_index != UINT32_MAX) {
      broad_phase_contact *body_contact = &contacts->contacts[body_contact_index];

      count_t shape_contact_index = body_contact_index;
      while (shape_contact_index != UINT32_MAX) {
        broad_phase_contact *shape_contact = &contacts->contacts[shape_contact_index];

        PROPAGATE_ERROR(func(world, contacts, (bnd_body_type)k, shape_contact, shape_contact_index, custom_data));

        shape_contact_index = shape_contact->next;
      }

      body_contact_index = body_contact->next_body;
    }
  }

  return OK;
}

static void update_contact_status(broad_phase_contact *contact, bool is_intersection) {
  bool did_touch = contact->status & CONTACT_TOUCHING;

  if (is_intersection) {
    contact->status |= CONTACT_TOUCHING; 
    if (!did_touch) {
      contact->status |= CONTACT_BEGAN_TOUCHING;
    } else {
      contact->status &= ~CONTACT_BEGAN_TOUCHING;
    }
  } else if (did_touch) {
    contact->status |= CONTACT_FINISHED_TOUCHING;
    contact->status &= ~CONTACT_TOUCHING;
  } else {
    contact->status &= ~CONTACT_FINISHED_TOUCHING;
  }
}


static void update_manifold(const collision_detection_context *ctx, contact_manifold *target, const contact_manifold *new_manifold, const contact_manifold *old_manifold) {
  *target = *new_manifold;

  if (new_manifold->count == 0) {
    return;
  }

  const float distance = ctx->world->config.collision_detection.feature_distance_threshold;
  const float distance_sqr = distance * distance;

  const float separation_threshold = ctx->world->config.collision_detection.separation_threshold;

  bnd_v3 position_a = body_a_center(ctx);
  bnd_v3 position_b = body_b_center(ctx);
  bnd_quat rotation_a = body_a_rotation(ctx);
  bnd_quat rotation_b = body_b_rotation(ctx);
  bnd_quat inverse_a = bnd_quat_invert(rotation_a);
  bnd_quat inverse_b = bnd_quat_invert(rotation_b);

  bnd_v3 old_world_normal = bnd_v3_rotate(old_manifold->local_normal, rotation_b);
  bnd_v3 new_world_normal = new_manifold->normal;
  target->local_normal = bnd_v3_rotate(new_world_normal, inverse_b);

  contact_point candidates[MAX_CONTACTS_PER_PAIR * 2];

  count_t count = new_manifold->count;
  for (count_t i = 0; i < count; ++i) {
    candidates[i] = new_manifold->points[i];

    candidates[i].witness_a = bnd_v3_rotate(bnd_v3_sub(candidates[i].witness_a, position_a), inverse_a);
    candidates[i].witness_b = bnd_v3_rotate(bnd_v3_sub(candidates[i].witness_b, position_b), inverse_b);
  }

  bool normals_align = bnd_v3_dot(old_world_normal, new_world_normal) >= 0.95f;
  count_t cached_count = normals_align ? old_manifold->count : 0;

  for (count_t i = 0; i < cached_count; ++i) {
    contact_point point = old_manifold->points[i];

    bnd_v3 witness_a = bnd_v3_add(bnd_v3_rotate(point.witness_a, rotation_a), position_a);
    bnd_v3 witness_b = bnd_v3_add(bnd_v3_rotate(point.witness_b, rotation_b), position_b);

    bnd_v3 offset = bnd_v3_sub(witness_a, witness_b);
    float separation = bnd_v3_dot(offset, new_world_normal);
    bnd_v3 drift = bnd_v3_sub(offset, bnd_v3_scale(new_world_normal, separation));
    if (separation > separation_threshold || bnd_v3_lensqr(drift) > distance_sqr) {
      continue;
    }

    count_t match = count;
    for (count_t j = 0; j < count; ++j) {
      if (bnd_v3_distancesqr(point.witness_a, candidates[j].witness_a) <= distance_sqr &&
          bnd_v3_distancesqr(point.witness_b, candidates[j].witness_b) <= distance_sqr) {
        match = j;
        break;
      }
    }

    if (match < count) {
      // Pass the stored impulses to the new point.
      candidates[match].normal_impulse = point.normal_impulse;
      memcpy(candidates[match].tangential_impulse, point.tangential_impulse, sizeof(point.tangential_impulse));
      continue;
    }

    point.point = bnd_v3_scale(bnd_v3_add(witness_a, witness_b), 0.5f);
    point.depth = -separation;
    candidates[count++] = point;
  }

  if (count > MAX_CONTACTS_PER_PAIR) {
    contact_point reduction[MAX_CONTACTS_PER_PAIR * 2] = {0};
    for (count_t i = 0; i < count; ++i) {
      reduction[i].point = candidates[i].point;
      reduction[i].depth = candidates[i].depth;
    }

    count_t selected[MAX_CONTACTS_PER_PAIR];
    contacts_filter_largest_surface_area(reduction, count, new_world_normal, selected);

    for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
      target->points[i] = candidates[selected[i]];
    }
    target->count = MAX_CONTACTS_PER_PAIR;
  } else {
    memcpy(target->points, candidates, count * sizeof(contact_point));
    target->count = count;
  }
}

static bnd_error detect_narrow_collisions(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data) {
  common_data *data_a = (common_data *)&world->dynamics;
  common_data *data_b = type == BND_BODY_DYNAMIC ? (common_data *)&world->dynamics : (common_data *)&world->statics;

  count_t body_a = data_a->outer_lookup[contact->body_a].index;
  count_t body_b = data_b->outer_lookup[contact->body_b].index;

  bnd_body_shape *shapes_a = shapes_get(world, data_a->shapes[body_a]);
  bnd_body_shape *shapes_b = shapes_get(world, data_b->shapes[body_b]);
  collision_detection_context ctx = {
    world,
    data_a,
    data_b,
    0,
    body_a,
    body_b,
    shapes_a[contact->shape_a],
    shapes_b[contact->shape_b],
  };

  contact_manifold new_manifold = {0};
  collision_detection_entry entry = collision_detection_table[ctx.shape_a.type][ctx.shape_b.type];
  collision_detection_context context = entry.primary ? ctx : ctx_inverse(ctx);

  bool intersection = entry.func(&context, &new_manifold);

  if (!entry.primary) {
    new_manifold.normal = bnd_v3_negate(new_manifold.normal);
    for (count_t i = 0; i < new_manifold.count; ++i) {
      contact_point *point = &new_manifold.points[i];

      bnd_v3 witness = point->witness_a;
      point->witness_a = point->witness_b;
      point->witness_b = witness;
    }
  }

  contact_manifold current_manifold = contact->manifold;

  if (entry.use_cache) {
    update_manifold(&ctx, &contact->manifold, &new_manifold, &current_manifold);
  } else {
    contact->manifold = new_manifold;
  }

  update_contact_status(contact, intersection);

  return OK;
}

bnd_error run_narrow_phase(bnd_world *world) {
  return for_each_broad_contact(world, detect_narrow_collisions, NULL);
}

static bool find_existing_shapes_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts, count_t shape_a, count_t shape_b, broad_phase_contact **contact, broad_phase_contact **prev_contact, count_t *contact_index, count_t *prev_contact_index) {
  count_t index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[index];

  *prev_contact = NULL;
  *prev_contact_index = UINT32_MAX;
  if (c->shape_a == shape_a && c->shape_b == shape_b) {
    // Shapes contact in question is a root body contact. Now find the previous body contact.
    *contact = c;
    *contact_index = index;

    count_t next = contacts->first;
    while (next != index) {
      *prev_contact_index = next;
      *prev_contact = &contacts->contacts[next];
      next = (*prev_contact)->next_body;
    }

    return true;
  }

  broad_phase_contact *prev = NULL;
  do {
    if (c->next == UINT32_MAX) {
      return false;
    }

    *prev_contact_index = index;
    *contact_index = c->next;
    prev = c;

    index = c->next;
    c = &contacts->contacts[index];
  } while(c->shape_a != shape_a || c->shape_b != shape_b);

  *contact = c;
  *prev_contact = prev;

  return true;
}

static void free_list_append(broad_contacts_set *contacts, broad_phase_contact *contact, count_t index) {
  count_t current_head = contacts->free_list;
  contacts->free_list = index;
  contact->next = current_head;
}

static bool free_list_pop(broad_contacts_set *contacts, count_t *index) {
  if (contacts->free_list == UINT32_MAX) {
    return false;
  }

  *index = contacts->free_list;
  contacts->free_list = contacts->contacts[contacts->free_list].next;

  return true;
}

static bnd_error new_contact_index(bnd_world *world, broad_contacts_set *contacts, count_t *index) {
  if (contacts->next == contacts->capacity && contacts->free_list == UINT32_MAX) {
    PROPAGATE_ERROR(resize_force(world->allocator, (void **)&contacts->contacts, sizeof(broad_phase_contact), ALIGNMENT_BROAD_CONTACT, 2, &contacts->capacity));
  }

  if (!free_list_pop(contacts, index)) {
    *index = contacts->next++;
  }

  return OK;
}

static bnd_error create_shapes_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts,  broad_phase_contact **new_contact) {
  count_t index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[index];

  while (c->next != UINT32_MAX) {
    index = c->next;
    c = &contacts->contacts[index];
  }

  PROPAGATE_ERROR(new_contact_index(world, contacts, &index));

  c->next = index;

  *new_contact = &contacts->contacts[index];

  return OK;
}

static bnd_error create_body_contact(bnd_world *world, uint64_t hash_key, broad_contacts_set *contacts, broad_phase_contact **new_contact, count_t *hash_slot) {
  count_t slot, contact_index;

  PROPAGATE_ERROR(new_contact_index(world, contacts, &contact_index))
  PROPAGATE_ERROR(hash_table_resize_if_needed(world, 1))

  assert(hash_table_find_empty_slot(&world->contacts, hash_key, &slot));
  world->contacts.keys[slot] = hash_key;
  world->contacts.indices[slot] = contact_index;
  world->contacts.hash_table_entry_count += 1;

  if (contacts->last != UINT32_MAX) {
    broad_phase_contact *last_contact = &contacts->contacts[contacts->last];
    last_contact->next_body = contact_index;
  }

  contacts->last = contact_index;

  if (contacts->first == UINT32_MAX) {
    contacts->first = contact_index;
  }

  *new_contact = &contacts->contacts[contact_index];
  *hash_slot = slot;

  return OK;
}

static void remove_body_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts, broad_phase_contact *contact, broad_phase_contact *prev_contact, count_t index, count_t prev_index, bool *body_contact_still_exists) {
  if (contact->next == UINT32_MAX) {
    // Body contact has no attached shape contacts. Remove it altogether.
    world->contacts.keys[hash_slot] = HASH_TABLE_TOMBSTONE;
    world->contacts.hash_table_entry_count -= 1;

    if (prev_contact) {
      prev_contact->next_body = contact->next_body;
    } else {
      contacts->first = contact->next_body;
    }

    if (contacts->last == index) {
      contacts->last = prev_index;
    }

    *body_contact_still_exists = false;
  } else {
    // Make the next attached shape contact the new body contact.
    broad_phase_contact *next_contact = &contacts->contacts[contact->next];
    next_contact->next_body = contact->next_body;
    if (prev_contact) {
      prev_contact->next_body = contact->next;
    } else {
      contacts->first = contact->next;
    }

    if (contacts->last == index) {
      contacts->last = contact->next;
    }

    world->contacts.indices[hash_slot] = contact->next;
    *body_contact_still_exists = true;
  }
}

static void remove_all_shape_contacts(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts) {
  world->contacts.keys[hash_slot] = HASH_TABLE_TOMBSTONE;
  world->contacts.hash_table_entry_count -= 1;

  count_t contact_index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[contact_index];

  if (contacts->first == contact_index) {
    contacts->first = c->next_body;

    if (contacts->last == contact_index) {
      // At this point both will be UINT32_MAX
      contacts->last = c->next_body;
    }
  } else {
    count_t prev_index = contacts->first;
    broad_phase_contact *prev_contact = &contacts->contacts[prev_index];
    count_t next_index = prev_contact->next_body;

    while(next_index != contact_index) {
      prev_index = next_index;
      prev_contact = &contacts->contacts[prev_index];
      next_index = prev_contact->next_body;
    }

    prev_contact->next_body = c->next_body;

    if (contacts->last == contact_index) {
      contacts->last = prev_index;
    }
  }

  do {
    count_t next_contact = c->next;
    free_list_append(contacts, c, contact_index);

    contact_index = next_contact;
    if (next_contact != UINT32_MAX) {
      c = &contacts->contacts[next_contact];
    }
  } while (contact_index != UINT32_MAX);
}

static void init_contact(broad_phase_contact *contact, uint64_t key, const collision_detection_context *ctx, count_t shape_a, count_t shape_b, broad_contact_status status) {
  contact->key = key;
  contact->body_a = ctx->data_a->inner_lookup[ctx->body_a];
  contact->body_b = ctx->data_b->inner_lookup[ctx->body_b];
  contact->shape_a = shape_a;
  contact->shape_b = shape_b;

  memset(&contact->manifold, 0, sizeof(contact_manifold));

  // Per-shape materials maybe??
  contact->friction = mix_friction(ctx);
  contact->restitution = mix_restitution(ctx);

  contact->next = UINT32_MAX;
  contact->next_body = UINT32_MAX;
  contact->status = status;
}

static bnd_error run_broad_phase_typed(bnd_world *world, broad_contacts_set *contact_set, bnd_body_type type) {
  collision_detection_context ctx = { .world = world };

  contacts *contacts = &world->contacts;

  common_data *data_a = (common_data *) &world->dynamics;
  common_data *data_b = (common_data *) as_common(world, type);


  for (count_t i = 0; i < data_a->count; ++i) {
    count_t until = type == BND_BODY_DYNAMIC ? i : data_b->count;
    for (count_t j = 0; j < until; j++) {
      count_t body_a = i;
      count_t body_b = j;
      if (type == BND_BODY_DYNAMIC && data_a->inner_lookup[i] > data_b->inner_lookup[j]) {
        body_a = j;
        body_b = i;
      }

      uint8_t flags_a = data_a->flags[body_a];
      uint8_t flags_b = data_b->flags[body_b];

      uint8_t trigger_flags = flags_a & BODY_FLAG_TRIGGER;
      trigger_flags |= (flags_b & BODY_FLAG_TRIGGER) << 1;

      broad_contact_status contact_trigger_status = (trigger_flags << 3) & CONTACT_TRIGGER_BOTH;
      if (contact_trigger_status == CONTACT_TRIGGER_BOTH) {
        continue;
      }

      ctx.data_a = data_a;
      ctx.data_b = data_b;

      ctx.body_a = body_a;
      ctx.body_b = body_b;

      bnd_collision_mask validation_mask = layer_to_mask(data_a->collision_layers[body_a]);
      bnd_collision_mask reference_mask = world->matrix.matrix[data_b->collision_layers[body_b]];

      if ((reference_mask & validation_mask) == 0) {
        continue;
      }

      bool potential_overlap = body_aabb_intersect(data_a, data_b, body_a, body_b);

      count_t slot;
      uint64_t key = hash_table_create_key(data_a, data_b, body_a, body_b, type);
      bool body_contact_exists = hash_table_find_slot_for_key(contacts, key, &slot);

      if (potential_overlap) {
        body_shapes shapes_a = data_a->shapes[body_a];
        body_shapes shapes_b = data_b->shapes[body_b];

        bnd_body_shape *shapes_buffer_a = shapes_get(world, shapes_a);
        bnd_body_shape *shapes_buffer_b = shapes_get(world, shapes_b);

        for (count_t sa = 0; sa < shapes_a.count; ++sa) {
          ctx.shape_a = shapes_buffer_a[sa];
          for (count_t sb = 0; sb < shapes_b.count; ++sb) {
            ctx.shape_b = shapes_buffer_b[sb];
            
            bnd_aabb a = {
              .center = body_a_center(&ctx),
              .half_extents = bounding_box_extents(world, ctx.shape_a.type, ctx.shape_a.value, body_a_rotation(&ctx))
            };

            bnd_aabb b = {
              .center = body_b_center(&ctx),
              .half_extents = bounding_box_extents(world, ctx.shape_b.type, ctx.shape_b.value, body_b_rotation(&ctx)),
            };

            bool shapes_overlap = aabb_intersect(&a, &b);
 
            count_t contact_index, prev_contact_index;
            broad_phase_contact *shapes_contact, *prev_contact;
            if (shapes_overlap) {
              if (body_contact_exists) {
                if (!find_existing_shapes_contact(world, slot, contact_set, sa, sb, &shapes_contact, &prev_contact, &contact_index, &prev_contact_index)) {
                  // Shapes potentially overlap, there is a root contact for the bodies, but not for the shapes.
                  PROPAGATE_ERROR(create_shapes_contact(world, slot, contact_set, &shapes_contact));
                  init_contact(shapes_contact, key, &ctx, sa, sb, contact_trigger_status);
                } else {
                  // Shapes potentially overlap and there is already a contact - do nothing.
                }
              } else { 
                // Shapes potentially overlap but there is not even a body contact. 
                PROPAGATE_ERROR(create_body_contact(world, key, contact_set, &shapes_contact, &slot));
                init_contact(shapes_contact, key, &ctx, sa, sb, contact_trigger_status);
                body_contact_exists = true;
              }
            } else { 
              // Shapes cannot overlap at all.
              if (body_contact_exists) {
                if (find_existing_shapes_contact(world, slot, contact_set, sa, sb, &shapes_contact, &prev_contact, &contact_index, &prev_contact_index)) {
                  // Should remove the existing contact.
                  if (prev_contact == NULL || shapes_contact->key != prev_contact->key) {
                    // Prev contact is for a different body or doesn't exist. That means that current contact is a root body contact.
                    remove_body_contact(world, slot, contact_set, shapes_contact, prev_contact, contact_index, prev_contact_index, &body_contact_exists);
                  } else {
                    // Simply remove one of the shape contacts.
                    prev_contact->next = shapes_contact->next;
                  }

                  free_list_append(contact_set, shapes_contact, contact_index);
                } else {
                  // No overlap and no contact - all good.
                }
              }
            }
          }
        }
      } else if (!potential_overlap && body_contact_exists) {
        remove_all_shape_contacts(world, slot, contact_set);
      }
    }
  }

  return OK;
}

void contacts_remove_for_body(bnd_world *world, bnd_body_handle handle) {
  broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };

  for (count_t k = 0; k < 2; ++k) {
    if (k == 0 && handle.type == BND_BODY_STATIC) {
      continue;
    }

    broad_contacts_set *contacts = sets[k];
    count_t index = contacts->first;
    while(index != UINT32_MAX) {
      broad_phase_contact *contact = &contacts->contacts[index];
      index = contact->next_body;

      bool static_match = handle.type == BND_BODY_STATIC && contact->body_b == handle.index;
      bool full_dynamic_match = k == 0 && handle.type == BND_BODY_DYNAMIC && (contact->body_a == handle.index || contact->body_b == handle.index);
      bool partial_dynamic_match = k == 1 && handle.type == BND_BODY_DYNAMIC && contact->body_a == handle.index;
      if (static_match || full_dynamic_match || partial_dynamic_match) {
        count_t slot;
        if (hash_table_find_slot_for_key(&world->contacts, contact->key, &slot)) {
          remove_all_shape_contacts(world, slot, contacts);
        }
      }
    }
  }
}

bnd_error run_broad_phase(bnd_world *world) {
  PROFILER_FUNCTION_START
  
  bnd_error e = run_broad_phase_typed(world, &world->contacts.dynamics, BND_BODY_DYNAMIC);
  if (IS_ERROR(e)) {
    PROFILER_FUNCTION_END
    return e;
  }

  e = run_broad_phase_typed(world, &world->contacts.statics, BND_BODY_STATIC);
  if (IS_ERROR(e)) {
    PROFILER_FUNCTION_END
    return e;
  }


  PROFILER_FUNCTION_END
  return OK;
}

#ifdef BND_TESTS

#include "library_testing.h"
#include "testing.h"

static collision_detection_context manifold_test_context(bnd_world *world) {
  add_dynamic_sphere(world, 1);
  add_static_sphere(world, 1);
  return (collision_detection_context) {
    .world = world, .data_a = (common_data *)&world->dynamics, .data_b = (common_data *)&world->statics,
  };
}

static contact_manifold manifold_test_point(float x, float z, float depth) {
  return (contact_manifold) {
    .count = 1,
    .normal = {0, 1, 0},
    .points = {
      {
        .point = {x, -depth * 0.5f, z},
        .depth = depth,
        .witness_a = {x, -depth, z},
        .witness_b = {x, 0, z}
      }
    },
  };
}

static void test_manifold_materializes_local_witnesses(void) {
  bnd_world *world = test_world();
  collision_detection_context ctx = manifold_test_context(world);
  contact_manifold empty = {0}, cached, result;
  contact_manifold fresh = manifold_test_point(1, 0, 0.2f);
  update_manifold(&ctx, &cached, &fresh, &empty);

  bnd_quat rotation = (bnd_quat){0, 0, sinf(0.25f), cosf(0.25f)};
  bnd_v3 translation = {2, 3, 4};
  world->dynamics.rotations[0] = world->statics.rotations[0] = rotation;
  world->dynamics.positions[0] = world->statics.positions[0] = translation;
  fresh = manifold_test_point(-1, 0, 0.1f);
  fresh.normal = bnd_v3_rotate(fresh.normal, rotation);
  fresh.points[0].point = bnd_v3_add(bnd_v3_rotate(fresh.points[0].point, rotation), translation);
  fresh.points[0].witness_a = bnd_v3_add(bnd_v3_rotate(fresh.points[0].witness_a, rotation), translation);
  fresh.points[0].witness_b = bnd_v3_add(bnd_v3_rotate(fresh.points[0].witness_b, rotation), translation);
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 2);
  expect_v3_near(result.points[1].point, bnd_v3_add(bnd_v3_rotate(cached.points[0].point, rotation), translation));
  expect_float_near(result.points[1].depth, 0.2f);
  expect_v3_near(result.points[0].witness_a, ((bnd_v3){-1, -0.1f, 0}));
  expect_v3_near(result.points[1].witness_a, cached.points[0].witness_a);
  bnd_teardown(world);
}

static void test_manifold_matches_fresh_points_and_keeps_impulses(void) {
  bnd_world *world = test_world();
  collision_detection_context ctx = manifold_test_context(world);
  contact_manifold empty = {0}, cached, result;
  contact_manifold fresh = manifold_test_point(0, 0, 0.1f);
  update_manifold(&ctx, &cached, &fresh, &empty);
  cached.points[0].normal_impulse = 2.0f;
  cached.points[0].tangential_impulse[0] = 1;
  cached.points[0].tangential_impulse[1] = 0;
  fresh = manifold_test_point(0.005f, 0, 0.11f);
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 1);
  expect_float_near(result.points[0].depth, 0.11f);
  expect_v3_near(result.points[0].point, fresh.points[0].point);
  expect_float_near(result.points[0].normal_impulse, cached.points[0].normal_impulse);
  expect_float_near(result.points[0].tangential_impulse[0], cached.points[0].tangential_impulse[0]);
  expect_float_near(result.points[0].tangential_impulse[1], cached.points[0].tangential_impulse[1]);
  expect_float_near(result.points[0].tangential_impulse[2], cached.points[0].tangential_impulse[2]);
  expect_float_near(result.points[0].tangential_impulse[3], cached.points[0].tangential_impulse[3]);
  bnd_teardown(world);
}

static void test_manifold_discards_invalid_cached_points(void) {
  bnd_world *world = test_world();
  collision_detection_context ctx = manifold_test_context(world);
  contact_manifold empty = {0}, cached, result;
  contact_manifold fresh = manifold_test_point(1, 0, 0.1f);
  update_manifold(&ctx, &cached, &fresh, &empty);
  fresh = manifold_test_point(-1, 0, 0.1f);

  world->dynamics.positions[0].y = 0.2f;
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 1); // Normal separation.
  world->dynamics.positions[0] = (bnd_v3){0.1f, 0, 0};
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 1); // Tangential drift.
  world->dynamics.positions[0] = bnd_v3_zero();
  cached.normal = bnd_v3_right();
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 1); // Different contact face.
  cached.normal = bnd_v3_up();
  update_manifold(&ctx, &result, &empty, &cached);
  assert(result.count == 0); // A narrow-phase miss clears the cache.
  bnd_teardown(world);
}

static void test_manifold_reduces_to_largest_surface(void) {
  bnd_world *world = test_world();
  collision_detection_context ctx = manifold_test_context(world);
  contact_manifold cached = {0}, result;
  float corners[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  for (count_t i = 0; i < 4; ++i) {
    contact_manifold fresh = manifold_test_point(corners[i][0], corners[i][1], i == 0 ? 0.2f : 0.1f);
    update_manifold(&ctx, &result, &fresh, &cached);
    cached = result;
  }
  assert(cached.count == 4);
  contact_manifold fresh = manifold_test_point(0, 0, 0.1f);
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 4);
  for (count_t i = 0; i < result.count; ++i) {
    expect_float_near(fabsf(result.points[i].point.x), 1);
    expect_float_near(fabsf(result.points[i].point.z), 1);
  }
  // The selected points, including their local features, survive another frame unchanged.
  cached = result;
  update_manifold(&ctx, &result, &fresh, &cached);
  assert(result.count == 4);
  for (count_t i = 0; i < result.count; ++i) {
    expect_v3_near(result.points[i].point, cached.points[i].point);
    expect_v3_near(result.points[i].witness_a, cached.points[i].witness_a);
  }
  bnd_teardown(world);
}

static void test_manifold_inverted_dispatch_and_contact_end(void) {
  bnd_world *world = test_world();
  bnd_result_handle capsule = bnd_add_capsule_dynamic(world, 1, 0.5f, 1);
  bnd_result_handle box = bnd_add_box_static(world, (bnd_v3){2, 2, 2});
  expect_ok(capsule.error);
  expect_ok(box.error);
  expect_ok(bnd_set_position(world, capsule.value, (bnd_v3){3.2f, 2, 4}));
  expect_ok(bnd_set_position(world, box.value, (bnd_v3){2, 2, 4}));
  expect_ok(run_broad_phase(world));
  expect_ok(run_narrow_phase(world));
  assert(world->contacts.statics.first != UINT32_MAX);
  broad_phase_contact *c = &world->contacts.statics.contacts[world->contacts.statics.first];
  assert(c->manifold.count == 1);
  assert(c->status & CONTACT_BEGAN_TOUCHING);
  assert(c->manifold.normal.x > 0.9f);
  contact_point point = c->manifold.points[0];
  bnd_v3 a = bnd_v3_add(point.witness_a, world->dynamics.positions[0]);
  bnd_v3 b = bnd_v3_add(point.witness_b, world->statics.positions[0]);
  expect_v3_near(point.point, bnd_v3_scale(bnd_v3_add(a, b), 0.5f));
  expect_float_near(point.depth, -bnd_v3_dot(bnd_v3_sub(a, b), c->manifold.normal));
  expect_float_near(point.witness_b.x, 1);
  assert(point.witness_a.x < 0);
  expect_ok(run_narrow_phase(world));
  assert(c->manifold.count == 1);
  assert(c->status & CONTACT_TOUCHING);
  assert(!(c->status & CONTACT_BEGAN_TOUCHING));

  // Keep the broad contact to verify that a narrow-phase miss clears its manifold.
  expect_ok(bnd_set_position(world, capsule.value, (bnd_v3){10, 2, 4}));
  expect_ok(run_narrow_phase(world));
  assert(c->manifold.count == 0);
  assert(c->status & CONTACT_FINISHED_TOUCHING);
  assert(!(c->status & CONTACT_TOUCHING));
  bnd_teardown(world);
}


static count_t broad_phase_inner_index(const bnd_world *world, bnd_body_handle handle);

static void broad_phase_refresh(bnd_world *world) {
  expect_ok(run_broad_phase(world));
}

static void broad_phase_move_body(bnd_world *world, bnd_body_handle handle, bnd_v3 position) {
  bnd_result_v3 old_position = bnd_get_position(world, handle);
  expect_ok(old_position.error);
  expect_ok(bnd_set_position(world, handle, position));

  common_data *data = (common_data *) as_common(world, handle.type);
  count_t index = broad_phase_inner_index(world, handle);
  data->aabbs[index].center = bnd_v3_add(data->aabbs[index].center, bnd_v3_sub(position, old_position.value));
}

static count_t broad_phase_inner_index(const bnd_world *world, bnd_body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->outer_lookup[handle.index].index;
}

static broad_contacts_set *broad_phase_set(bnd_world *world, bnd_body_type type) {
  return type == BND_BODY_DYNAMIC ? &world->contacts.dynamics : &world->contacts.statics;
}

static broad_phase_contact *broad_phase_contact_for_pair(bnd_world *world, bnd_body_handle a, bnd_body_handle b, bnd_body_type type) {
  common_data *data_a = (common_data *) &world->dynamics;
  common_data *data_b = (common_data *) as_common(world, type);
  count_t index_a = broad_phase_inner_index(world, a);
  count_t index_b = broad_phase_inner_index(world, b);
  uint64_t key = hash_table_create_key(data_a, data_b, index_a, index_b, type);
  count_t slot;

  if (!hash_table_find_slot_for_key(&world->contacts, key, &slot)) {
    return NULL;
  }

  return &broad_phase_set(world, type)->contacts[world->contacts.indices[slot]];
}

static count_t broad_phase_chain_count(const broad_contacts_set *set, count_t root_index) {
  count_t count = 0;
  count_t index = root_index;
  while (index != UINT32_MAX) {
    count += 1;
    index = set->contacts[index].next;
  }
  return count;
}

static count_t broad_phase_root_count(const broad_contacts_set *set) {
  count_t count = 0;
  for (count_t index = set->first; index != UINT32_MAX; index = set->contacts[index].next_body) {
    count += 1;
  }
  return count;
}

static count_t broad_phase_free_count(const broad_contacts_set *set) {
  count_t count = 0;
  for (count_t index = set->free_list; index != UINT32_MAX; index = set->contacts[index].next) {
    count += 1;
  }
  return count;
}

static count_t broad_phase_contacts_for_body(const bnd_world *world, bnd_body_handle handle) {
  const broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };
  count_t count = 0;

  for (count_t k = 0; k < 2; ++k) {
    const broad_contacts_set *set = sets[k];
    for (count_t index = set->first; index != UINT32_MAX; index = set->contacts[index].next_body) {
      const broad_phase_contact *contact = &set->contacts[index];
      bool dynamic_match = handle.type == BND_BODY_DYNAMIC &&
        (contact->body_a == handle.index || (k == 0 && contact->body_b == handle.index));
      bool static_match = handle.type == BND_BODY_STATIC && k == 1 && contact->body_b == handle.index;

      if (dynamic_match || static_match) {
        count += broad_phase_chain_count(set, index);
      }
    }
  }

  return count;
}

static bnd_body_handle add_two_shape_dynamic(bnd_world *world) {
  bnd_body_shape shapes[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {-0.5f, 0, 0}, .rotation = bnd_quat_identity() },
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = { 0.5f, 0, 0}, .rotation = bnd_quat_identity() },
  };
  float masses[] = {1.0f, 1.0f};
  bnd_result_handle result = bnd_add_compound_body_dynamic(world, shapes, masses, 2);
  expect_ok(result.error);
  return result.value;
}

static bnd_body_handle add_two_shape_static(bnd_world *world) {
  bnd_body_shape shapes[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {-0.5f, 0, 0}, .rotation = bnd_quat_identity() },
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = { 0.5f, 0, 0}, .rotation = bnd_quat_identity() },
  };
  bnd_result_handle result = bnd_add_compound_body_static(world, shapes, 2);
  expect_ok(result.error);
  return result.value;
}

static void broad_phase_assert_set_links(const broad_contacts_set *set) {
  count_t roots = 0;
  count_t shape_entries = 0;
  count_t previous = UINT32_MAX;

  for (count_t index = set->first; index != UINT32_MAX; index = set->contacts[index].next_body) {
    const broad_phase_contact *root = &set->contacts[index];
    assert(index != UINT32_MAX);
    assert(root->next_body == UINT32_MAX || set->contacts[root->next_body].key != 0);
    assert(root->key & UINT64_C(0x8000000000000000));
    assert(previous == UINT32_MAX || set->contacts[previous].next_body == index);
    shape_entries += broad_phase_chain_count(set, index);
    previous = index;
    roots += 1;
  }

  assert((roots == 0) == (set->first == UINT32_MAX && set->last == UINT32_MAX));
  assert((roots > 0) == (set->last != UINT32_MAX));
  assert(set->next >= shape_entries + broad_phase_free_count(set));
}

static void test_broad_phase_creates_and_deduplicates_contacts(void) {
  bnd_world *world = test_world();
  bnd_body_handle a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle b = add_dynamic_sphere(world, 1.0f);

  broad_phase_refresh(world);

  broad_contacts_set *set = &world->contacts.dynamics;
  assert(world->contacts.hash_table_entry_count == 1);
  assert(broad_phase_root_count(set) == 1);
  assert(set->first == set->last);
  assert(set->next == 1);
  assert(set->free_list == UINT32_MAX);

  broad_phase_contact *contact = broad_phase_contact_for_pair(world, a, b, BND_BODY_DYNAMIC);
  assert(contact != NULL);
  assert(contact->shape_a == 0 && contact->shape_b == 0);
  assert(contact->next == UINT32_MAX && contact->next_body == UINT32_MAX);
  assert(contact->key & UINT64_C(0x8000000000000000));

  count_t first = set->first;
  broad_phase_refresh(world);
  assert(world->contacts.hash_table_entry_count == 1);
  assert(set->first == first && set->next == 1);
  broad_phase_assert_set_links(set);

  bnd_teardown(world);
}

static void test_broad_phase_keeps_dynamic_and_static_sets_independent(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle dynamic_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);

  broad_phase_refresh(world);

  assert(broad_phase_root_count(&world->contacts.dynamics) == 1);
  assert(broad_phase_root_count(&world->contacts.statics) == 2);
  assert(world->contacts.hash_table_entry_count == 3);
  assert(broad_phase_contact_for_pair(world, dynamic_a, dynamic_b, BND_BODY_DYNAMIC) != NULL);
  assert(broad_phase_contact_for_pair(world, dynamic_a, static_body, BND_BODY_STATIC) != NULL);
  broad_phase_assert_set_links(&world->contacts.dynamics);
  broad_phase_assert_set_links(&world->contacts.statics);

  bnd_teardown(world);
}

static void test_broad_phase_uses_shape_aabbs_and_inclusive_boundaries(void) {
  bnd_world *world = test_world();
  bnd_body_shape dynamic_shapes[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {-5, 0, 0}, .rotation = bnd_quat_identity() },
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = { 5, 0, 0}, .rotation = bnd_quat_identity() },
  };
  float masses[] = {1.0f, 1.0f};
  bnd_body_handle dynamic = bnd_add_compound_body_dynamic(world, dynamic_shapes, masses, 2).value;
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);

  broad_phase_refresh(world);
  assert(broad_phase_contact_for_pair(world, dynamic, static_body, BND_BODY_STATIC) == NULL);

  broad_phase_move_body(world, static_body, (bnd_v3){-3, 0, 0});
  broad_phase_refresh(world);
  broad_phase_contact *contact = broad_phase_contact_for_pair(world, dynamic, static_body, BND_BODY_STATIC);
  assert(contact != NULL);
  assert(contact->shape_a == 0 && contact->shape_b == 0);

  bnd_body_handle boundary_dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle boundary_static = add_static_sphere(world, 1.0f);
  broad_phase_refresh(world);

  /* Two unit spheres at distance two touch and must be retained. */
  broad_phase_move_body(world, boundary_static, (bnd_v3){2, 0, 0});
  broad_phase_refresh(world);
  assert(broad_phase_contact_for_pair(world, boundary_dynamic, boundary_static, BND_BODY_STATIC) != NULL);

  bnd_teardown(world);
}

static void test_broad_phase_promotes_root_and_removes_final_shape_contact(void) {
  bnd_world *world = test_world();
  bnd_body_shape one_shape[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() },
  };
  bnd_body_shape two_shapes[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {-1, 0, 0}, .rotation = bnd_quat_identity() },
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = { 1, 0, 0}, .rotation = bnd_quat_identity() },
  };
  float mass[] = {1.0f};
  bnd_body_handle dynamic = bnd_add_compound_body_dynamic(world, one_shape, mass, 1).value;
  bnd_body_handle static_body = bnd_add_compound_body_static(world, two_shapes, 2).value;

  broad_phase_refresh(world);
  broad_phase_contact *root = broad_phase_contact_for_pair(world, dynamic, static_body, BND_BODY_STATIC);
  assert(root != NULL);
  assert(broad_phase_chain_count(&world->contacts.statics, (count_t)(root - world->contacts.statics.contacts)) == 2);
  assert(root->shape_a == 0 && root->shape_b == 0);

  broad_phase_move_body(world, static_body, (bnd_v3){-2, 0, 0});
  broad_phase_refresh(world);
  root = broad_phase_contact_for_pair(world, dynamic, static_body, BND_BODY_STATIC);
  assert(root != NULL);
  assert(root->shape_a == 0 && root->shape_b == 1);
  assert(root->next == UINT32_MAX);
  assert(world->contacts.hash_table_entry_count == 1);
  broad_phase_assert_set_links(&world->contacts.statics);

  broad_phase_move_body(world, static_body, (bnd_v3){5, 0, 0});
  broad_phase_refresh(world);
  assert(broad_phase_contact_for_pair(world, dynamic, static_body, BND_BODY_STATIC) == NULL);
  assert(world->contacts.hash_table_entry_count == 0);
  assert(world->contacts.statics.first == UINT32_MAX);
  assert(world->contacts.statics.last == UINT32_MAX);
  assert(broad_phase_free_count(&world->contacts.statics) == 2);

  bnd_teardown(world);
}

static void test_broad_phase_repairs_body_list_when_middle_pair_is_removed(void) {
  bnd_world *world = test_world();
  bnd_body_handle bodies[6];
  for (count_t i = 0; i < 6; ++i) {
    bodies[i] = add_dynamic_sphere(world, 1.0f);
    broad_phase_move_body(world, bodies[i], (bnd_v3){(float)(i / 2) * 10.0f, 0, 0});
  }

  broad_phase_refresh(world);
  broad_contacts_set *set = &world->contacts.dynamics;
  assert(broad_phase_root_count(set) == 3);
  assert(world->contacts.hash_table_entry_count == 3);

  broad_phase_move_body(world, bodies[2], (bnd_v3){100, 0, 0});
  broad_phase_move_body(world, bodies[3], (bnd_v3){110, 0, 0});
  broad_phase_refresh(world);
  assert(broad_phase_root_count(set) == 2);
  assert(world->contacts.hash_table_entry_count == 2);
  assert(set->first != UINT32_MAX && set->last != UINT32_MAX);
  assert(set->first != set->last);
  assert(set->contacts[set->first].next_body == set->last);
  assert(set->contacts[set->last].next_body == UINT32_MAX);
  broad_phase_assert_set_links(set);

  bnd_teardown(world);
}

static void test_broad_phase_reuses_contacts_and_resizes_storage(void) {
  bnd_config config = test_config();
  config.memory.contacts_capacity = 2;
  config.memory.hash_table_capacity = 2;
  config.memory.dynamics_capacity = 8;
  bnd_world *world = bnd_init(config);
  assert(world != NULL);

  bnd_body_handle bodies[4];
  for (count_t i = 0; i < 4; ++i) {
    bodies[i] = add_dynamic_sphere(world, 1.0f);
    broad_phase_move_body(world, bodies[i], (bnd_v3){(float)(i / 2) * 10.0f, 0, 0});
  }

  broad_phase_refresh(world);
  assert(broad_phase_root_count(&world->contacts.dynamics) == 2);
  assert(world->contacts.dynamics.capacity > 1);
  assert(world->contacts.hash_table_capacity > 2);
  assert(world->contacts.hash_table_entry_count == 2);

  broad_phase_move_body(world, bodies[0], (bnd_v3){100, 0, 0});
  broad_phase_move_body(world, bodies[1], (bnd_v3){110, 0, 0});
  broad_phase_refresh(world);
  assert(broad_phase_root_count(&world->contacts.dynamics) == 1);
  assert(broad_phase_free_count(&world->contacts.dynamics) == 1);

  broad_phase_move_body(world, bodies[0], (bnd_v3){0, 0, 0});
  broad_phase_move_body(world, bodies[1], (bnd_v3){0, 0, 0});
  broad_phase_refresh(world);
  assert(broad_phase_root_count(&world->contacts.dynamics) == 2);
  assert(broad_phase_free_count(&world->contacts.dynamics) == 0);
  broad_phase_assert_set_links(&world->contacts.dynamics);

  bnd_teardown(world);
}

static void test_broad_phase_survives_dynamic_inner_reordering(void) {
  bnd_world *world = test_world();
  bnd_body_handle a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle unrelated = add_dynamic_sphere(world, 1.0f);

  broad_phase_refresh(world);
  assert(broad_phase_contact_for_pair(world, a, b, BND_BODY_DYNAMIC) != NULL);

  expect_ok(bnd_put_to_sleep(world, unrelated));
  broad_phase_refresh(world);
  broad_phase_contact *contact = broad_phase_contact_for_pair(world, a, b, BND_BODY_DYNAMIC);
  assert(contact != NULL);
  assert(world->contacts.hash_table_entry_count == 3);
  broad_phase_assert_set_links(&world->contacts.dynamics);

  bnd_teardown(world);
}

static void test_removing_dynamic_body_removes_all_its_broad_phase_contacts(void) {
  bnd_world *world = test_world();
  bnd_body_handle far_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle removed = add_two_shape_dynamic(world);
  bnd_body_handle peer_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle peer_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle far_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle near_static = add_static_sphere(world, 1.0f);
  bnd_body_handle far_static = add_static_sphere(world, 1.0f);
  broad_phase_move_body(world, far_a, (bnd_v3){10, 0, 0});
  broad_phase_move_body(world, far_b, (bnd_v3){10, 0, 0});
  broad_phase_move_body(world, far_static, (bnd_v3){10, 0, 0});
  broad_phase_refresh(world);

  assert(broad_phase_contacts_for_body(world, removed) == 6);
  expect_ok(bnd_remove_body(world, removed));

  assert(broad_phase_contacts_for_body(world, removed) == 0);
  assert(world->contacts.hash_table_entry_count == 6);
  assert(broad_phase_contact_for_pair(world, peer_a, peer_b, BND_BODY_DYNAMIC) != NULL);
  assert(broad_phase_contact_for_pair(world, far_a, far_b, BND_BODY_DYNAMIC) != NULL);
  assert(broad_phase_contact_for_pair(world, peer_a, near_static, BND_BODY_STATIC) != NULL);
  assert(broad_phase_contact_for_pair(world, far_a, far_static, BND_BODY_STATIC) != NULL);
  broad_phase_assert_set_links(&world->contacts.dynamics);
  broad_phase_assert_set_links(&world->contacts.statics);

  bnd_teardown(world);
}

static void test_removing_static_body_removes_all_its_broad_phase_contacts(void) {
  bnd_world *world = test_world();
  bnd_body_handle far_dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle dynamic_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle dynamic_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle far_static = add_static_sphere(world, 1.0f);
  bnd_body_handle removed = add_two_shape_static(world);
  broad_phase_move_body(world, far_dynamic, (bnd_v3){10, 0, 0});
  broad_phase_move_body(world, far_static, (bnd_v3){10, 0, 0});
  broad_phase_refresh(world);

  assert(broad_phase_contacts_for_body(world, removed) == 4);
  expect_ok(bnd_remove_body(world, removed));

  assert(broad_phase_contacts_for_body(world, removed) == 0);
  assert(world->contacts.hash_table_entry_count == 2);
  assert(broad_phase_contact_for_pair(world, dynamic_a, dynamic_b, BND_BODY_DYNAMIC) != NULL);
  assert(broad_phase_contact_for_pair(world, far_dynamic, far_static, BND_BODY_STATIC) != NULL);
  broad_phase_assert_set_links(&world->contacts.dynamics);
  broad_phase_assert_set_links(&world->contacts.statics);

  bnd_teardown(world);
}

static void test_changing_collision_layer_removes_all_body_broad_phase_contacts(void) {
  bnd_world *world = test_world();
  expect_ok(bnd_set_layers_count(world, 2));
  bnd_body_handle far_dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle changed = add_two_shape_dynamic(world);
  bnd_body_handle spacer = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle peer = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle far_static = add_static_sphere(world, 1.0f);
  bnd_body_handle near_static = add_static_sphere(world, 1.0f);
  broad_phase_move_body(world, far_dynamic, (bnd_v3){10, 0, 0});
  broad_phase_move_body(world, spacer, (bnd_v3){20, 0, 0});
  broad_phase_move_body(world, far_static, (bnd_v3){10, 0, 0});
  broad_phase_refresh(world);

  assert(broad_phase_contacts_for_body(world, changed) == 4);
  expect_ok(bnd_set_collision_layer(world, changed, 1));

  assert(broad_phase_contacts_for_body(world, changed) == 0);
  assert(world->contacts.hash_table_entry_count == 2);
  assert(broad_phase_contact_for_pair(world, peer, near_static, BND_BODY_STATIC) != NULL);
  assert(broad_phase_contact_for_pair(world, far_dynamic, far_static, BND_BODY_STATIC) != NULL);
  broad_phase_assert_set_links(&world->contacts.dynamics);
  broad_phase_assert_set_links(&world->contacts.statics);

  bnd_teardown(world);
}

static void test_changing_trigger_status_removes_all_body_broad_phase_contacts(void) {
  bnd_world *world = test_world();
  bnd_body_handle far_dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle near_dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle far_static = add_static_sphere(world, 1.0f);
  bnd_body_handle changed = add_two_shape_static(world);
  broad_phase_move_body(world, far_dynamic, (bnd_v3){10, 0, 0});
  broad_phase_move_body(world, far_static, (bnd_v3){10, 0, 0});
  broad_phase_refresh(world);

  assert(broad_phase_contacts_for_body(world, changed) == 2);
  expect_ok(bnd_set_trigger(world, changed, true));
  assert(broad_phase_contacts_for_body(world, changed) == 0);
  assert(broad_phase_contact_for_pair(world, far_dynamic, far_static, BND_BODY_STATIC) != NULL);

  broad_phase_refresh(world);
  assert(broad_phase_contacts_for_body(world, changed) == 2);
  expect_ok(bnd_set_trigger(world, changed, false));

  assert(broad_phase_contacts_for_body(world, changed) == 0);
  assert(world->contacts.hash_table_entry_count == 1);
  assert(broad_phase_contact_for_pair(world, far_dynamic, far_static, BND_BODY_STATIC) != NULL);
  assert(broad_phase_contact_for_pair(world, near_dynamic, changed, BND_BODY_STATIC) == NULL);
  broad_phase_assert_set_links(&world->contacts.statics);

  bnd_teardown(world);
}

void broad_phase_tests(void) {
  TESTS_BEGIN("Broad phase")
    TEST(test_broad_phase_creates_and_deduplicates_contacts)
    TEST(test_broad_phase_keeps_dynamic_and_static_sets_independent)
    TEST(test_broad_phase_uses_shape_aabbs_and_inclusive_boundaries)
    TEST(test_broad_phase_promotes_root_and_removes_final_shape_contact)
    TEST(test_broad_phase_repairs_body_list_when_middle_pair_is_removed)
    TEST(test_broad_phase_reuses_contacts_and_resizes_storage)
    TEST(test_broad_phase_survives_dynamic_inner_reordering)
    TEST(test_removing_dynamic_body_removes_all_its_broad_phase_contacts)
    TEST(test_removing_static_body_removes_all_its_broad_phase_contacts)
    TEST(test_changing_collision_layer_removes_all_body_broad_phase_contacts)
    TEST(test_changing_trigger_status_removes_all_body_broad_phase_contacts)
  TESTS_END;
}

void manifold_tests(void) {
  TESTS_BEGIN("Contact manifolds")
    TEST(test_manifold_materializes_local_witnesses)
    TEST(test_manifold_matches_fresh_points_and_keeps_impulses)
    TEST(test_manifold_discards_invalid_cached_points)
    TEST(test_manifold_reduces_to_largest_surface)
    TEST(test_manifold_inverted_dispatch_and_contact_end)
  TESTS_END;
}

#endif
