#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include "profiler.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define PROFILING_BLOCK_NAME "Contacts cache"

typedef count_t (*collision_detection_func)(bnd_world *world, const collision_detection_context *ctx);

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

static contact *new_contact(const collision_detection_context *ctx, count_t offset) {
  contact *c = &ctx->world->contacts.values[ctx->contacts_offset + offset];
  c->index_a = ctx->body_a;
  c->index_b = ctx->body_b;
  c->friction = mix_friction(ctx);
  c->restitution = mix_restitution(ctx);
  c->from_cache = false;

  return c;
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

static count_t sphere_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 center_a = body_a_center(ctx);
  bnd_v3 center_b = body_b_center(ctx);

  float radius_a = ctx->shape_a.value.sphere.radius;
  float radius_b = ctx->shape_b.value.sphere.radius;

  bnd_v3 offset = bnd_v3_sub(center_a, center_b);
  float distance = bnd_v3_len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return 0;
  }

  bnd_v3 normal = distance > EPSILON ? bnd_v3_scale(offset, 1 / distance) : bnd_v3_up();

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
  c->point = bnd_v3_add(center_b, bnd_v3_scale(normal, radius_b + penetration));
  c->normal = normal;
  c->depth = -penetration;

  return 1;
}

static count_t capsule_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

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
      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = horizontal_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_right(), capsule_rotation);
      c->depth = capsule_radius - horizontal_distance + sphere_radius;

      return 1;
    } else if (horizontal_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(horizontal_offset, capsule_radius / horizontal_distance);
      closest.y = local_sphere_center.y;

      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      c->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation));
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
      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = cap_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
      c->depth = capsule_radius - cap_distance + sphere_radius;

      return 1;
    } else if (cap_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(cap_offset, capsule_radius / cap_distance);
      closest = bnd_v3_add(cap, closest);

      contact *c = new_contact(ctx, 0);
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

    local_normal[min_axis] = c[min_axis] > 0 ? -1.0f : 1.0f;
    depth = min_dist + r;
  } else {
    bnd_v3 diff = bnd_v3_sub(closest, local_sphere_center);
    float dist = sqrtf(distancesqr);
    depth = r - dist;

    diff = bnd_v3_scale(diff, 1.0f / dist);
    memcpy(&local_normal, &diff, sizeof(bnd_v3));
  }

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *contact = new_contact(ctx, 0);
  memcpy(&contact->normal, local_normal, sizeof(bnd_v3));

  contact->point = bnd_v3_add(box_center, bnd_v3_rotate(closest, box_rotation));
  contact->normal = bnd_v3_rotate(contact->normal, box_rotation);
  contact->depth = depth;

  return 1;
}

static count_t box_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  bnd_quat shape_rotation = ctx->shape_a.rotation;

  bnd_v3 box_center = body_a_center(ctx);
  bnd_v3 extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5f);

  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];

  bnd_v3 corners[8];
  box_corners(extents, corners);

  const count_t max_contacts = 4;

  bnd_error e = contacts_ensure_capacity(world, ctx->contacts_offset, max_contacts);
  if (IS_ERROR(e)) {
    return 0;
  }

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    bnd_v3 corner = bnd_v3_add(box_center, bnd_v3_rotate(bnd_v3_rotate(corners[i], shape_rotation), box_rotation));
    float distance = bnd_v3_dot(bnd_v3_sub(corner, plane_point), plane_normal);
    if (distance > 0) {
      continue;
    }

    contact *c = new_contact(ctx, contact_count);
    c->normal = plane_normal;
    c->point = bnd_v3_add(corner, bnd_v3_scale(plane_normal, -0.5f * distance));
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

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *contact = new_contact(ctx, 0);
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

    if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset + contact_count, 1))) {
      return contact_count;
    }

    contact *c = new_contact(ctx, contact_count);
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

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
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

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
  epa_get_contact(world, ctx, &s, world->config.advanced.epa_tolerance, c);

  return 1;
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

count_t collisions_detect(bnd_world *world, count_t contacts_offset, bnd_body_type type) {
  const common_data *dynamics = as_common_const(world, BND_BODY_DYNAMIC);
  const common_data *data_b = as_common_const(world, type);

  collision_detection_context ctx = {
    .world = world,
    .data_a = dynamics,
    .data_b = data_b,
  };

  count_t count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    count_t until = type == BND_BODY_DYNAMIC ? i : data_b->count;
    for (count_t j = 0; j < until; ++j) {
      bnd_collision_mask validation_mask = layer_to_mask(dynamics->collision_layers[i]);
      bnd_collision_mask reference_mask = world->matrix.matrix[data_b->collision_layers[j]];
      if ((reference_mask & validation_mask) == 0) {
        continue;
      }

      if (!aabb_intersect(dynamics, data_b, i, j)) {
        continue;
      }

      ctx.body_a = i;
      ctx.body_b = j;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = data_b->shapes[j];

      uint64_t cached_contacts_mask = 0;
      count_t pair_contacts_count = 0;
      count_t pair_offset = contacts_offset + count;

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
          count_t shape_offset = pair_offset + pair_contacts_count;

          ctx.shape_b = shape_b;
          ctx.contacts_offset = shape_offset;

          collision_detection_entry entry = collision_detection_table[shape_a.type][shape_b.type];
          if (entry.func == NULL) {
            continue;
          }

          collision_detection_context context = entry.primary ? ctx : ctx_inverse(ctx);
          count_t shape_contacts_count = entry.func(world, &context);

          if (!entry.primary) {
            for (count_t k = 0; k < shape_contacts_count; ++k) {
              contact *c = &world->contacts.values[shape_offset + k];
              c->index_a = ctx.body_a;
              c->index_b = ctx.body_b;
              c->normal = bnd_v3_negate(c->normal);

              if (entry.use_cache) {
                bnd_v3 tmp_witness = c->features.witness_a;
                c->features.witness_a = c->features.witness_b;
                c->features.witness_b = tmp_witness;
                c->features.normal = bnd_v3_negate(c->features.normal);
              }
            }
          }

          if (entry.use_cache) {
            uint64_t mask = (1 << shape_contacts_count) - 1;
            cached_contacts_mask |= mask << pair_contacts_count;
          }

          pair_contacts_count += shape_contacts_count;
        }
      }

      bool is_trigger = data_b->flags[j] & BODY_FLAG_TRIGGER;
      if (is_trigger && pair_contacts_count > 0) {
        // It's not ideal to process triggers here, since we've already done a lot of useless work.
        // This is going to change with the introduction of contact islands.

        if (events_subscribed((const common_data *)dynamics, i, BND_EVENT_TRIGGER)) {
          events_push(world, (common_data *)dynamics, i, (bnd_event) {
            .type = BND_EVENT_TRIGGER,
            .trigger = { .other = make_body_handle(world, type, j) }
          });
        }

        if (events_subscribed(data_b, j, BND_EVENT_TRIGGER)) {
          events_push(world, (common_data *)data_b, j, (bnd_event) {
            .type = BND_EVENT_TRIGGER,
            .trigger = { .other = make_body_handle(world, BND_BODY_DYNAMIC, i) }
          });
        }

        break;
      }

      count_t filtered_contact_indices[MAX_CONTACTS_PER_PAIR] = {0};
      if (cached_contacts_mask == 0) {
        if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
          contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);
          pair_contacts_count = MAX_CONTACTS_PER_PAIR;
        }

        count += pair_contacts_count;
        continue;
      }

      PROFILER_BLOCK_START(PROFILING_BLOCK_NAME);

      cache_entry *cached_entry = contacts_cache_query(world, world->contacts.values + pair_offset, type);
      if (cached_entry == NULL) {
        PROFILER_BLOCK_END;
        continue;
      }

      float distance_threshold = world->config.advanced.contacts_cache.feature_distance_threshold;
      float distance_threshold_sqr = distance_threshold * distance_threshold;
      float separation_threshold = world->config.advanced.contacts_cache.separation_threshold;

      bnd_v3 position_a = ctx.data_a->positions[ctx.body_a];
      bnd_v3 position_b = ctx.data_b->positions[ctx.body_b];
      bnd_quat rotation_a = ctx.data_a->rotations[ctx.body_a];
      bnd_quat rotation_b = ctx.data_b->rotations[ctx.body_b];

      uint8_t picked_features = 0;
      for (count_t k = 0; k < pair_contacts_count; ++k) {
        if ((cached_contacts_mask & (UINT64_C(1) << k)) == 0) {
          continue;
        }

        contact *c = &world->contacts.values[pair_offset + k];
        contact_features *features = &c->features;

        bnd_quat inv_rotation_a = bnd_quat_invert(rotation_a);
        bnd_quat inv_rotation_b = bnd_quat_invert(rotation_b);
        features->witness_a = bnd_v3_rotate(bnd_v3_sub(features->witness_a, position_a), inv_rotation_a);
        features->witness_b = bnd_v3_rotate(bnd_v3_sub(features->witness_b, position_b), inv_rotation_b);
        features->normal = bnd_v3_rotate(features->normal, inv_rotation_a);

        count_t matched_slot = cached_entry->feature_count;
        for (count_t h = 0; h < cached_entry->feature_count; ++h) {
          const contact_features *cached_features = &cached_entry->features[h];

          float distance_a_sqr = bnd_v3_distancesqr(cached_features->witness_a, features->witness_a);
          float distance_b_sqr = bnd_v3_distancesqr(cached_features->witness_b, features->witness_b);

          if (distance_a_sqr <= distance_threshold_sqr && distance_b_sqr <= distance_threshold_sqr) {
            matched_slot = h;
            break;
          }
        }

        count_t feature_count = cached_entry->feature_count;
        if (matched_slot < feature_count) {
          cached_entry->features[matched_slot] = *features;
          picked_features |= 1 << matched_slot;
        } else if (cached_entry->feature_count < MAX_CONTACTS_PER_PAIR) {
          cached_entry->features[feature_count] = *features;
          cached_entry->feature_count += 1;

          picked_features |= 1 << feature_count;
        }
      }

      count_t fresh_contacts_count = pair_contacts_count;
      count_t contacts_from_cache = 0;
      for (count_t h = 0; h < cached_entry->feature_count; ++h) {
        if (picked_features & (1 << h)) {
          continue;
        }

        const contact_features *cached_features = &cached_entry->features[h];

        bnd_v3 witness_a_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_a, rotation_a), position_a);
        bnd_v3 witness_b_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_b, rotation_b), position_b);
        bnd_v3 normal_world = bnd_v3_rotate(cached_features->normal, rotation_a);

        float separation = bnd_v3_dot(bnd_v3_sub(witness_a_world, witness_b_world), normal_world);
        if (separation > separation_threshold) {
          continue;
        }

        count_t contact_offset = pair_offset + fresh_contacts_count + contacts_from_cache;
        if (IS_ERROR(contacts_ensure_capacity(world, contact_offset, 1))) {
          break;
        }

        contact *c = &world->contacts.values[contact_offset];
        c->index_a = ctx.body_a;
        c->index_b = ctx.body_b;
        c->point = bnd_v3_scale(bnd_v3_add(witness_a_world, witness_b_world), 0.5f);
        c->normal = normal_world;
        c->depth = -separation;
        c->features = *cached_features;
        c->restitution = mix_restitution(&ctx);
        c->friction = mix_friction(&ctx);

        contacts_from_cache += 1;
      }

      pair_contacts_count += contacts_from_cache;

      if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
        contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);

        count_t feature_count = 0;
        for (count_t k = 0; k < MAX_CONTACTS_PER_PAIR; ++k) {
          contact *c = &world->contacts.values[pair_offset + k];
          count_t original_contact_index = filtered_contact_indices[k];

          bool fresh_cashable = original_contact_index < fresh_contacts_count && cached_contacts_mask & ((uint64_t)1 << original_contact_index);
          bool from_cache = original_contact_index >= fresh_contacts_count;
          if (fresh_cashable || from_cache) {
            cached_entry->features[feature_count++] = c->features;
          }
        }

        cached_entry->feature_count = feature_count;
        pair_contacts_count = MAX_CONTACTS_PER_PAIR;
      }

      count += pair_contacts_count;

      PROFILER_BLOCK_END;
    }
  }

  return count;
}

void collision_detection_init(void) {
  memset(collision_detection_table, 0, sizeof(collision_detection_table));

  collision_detection_table[BND_SPHERE][BND_SPHERE] = (collision_detection_entry) { sphere_sphere_collision, true, false };
  collision_detection_table[BND_BOX][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_MESH][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, true, true };

  collision_detection_table[BND_BOX][BND_SPHERE] = (collision_detection_entry) { box_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_BOX] = (collision_detection_entry) { box_sphere_collision, false, false };
  collision_detection_table[BND_BOX][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_CAPSULE][BND_SPHERE] = (collision_detection_entry) { capsule_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_CAPSULE] = (collision_detection_entry) { capsule_sphere_collision, false, false };

  collision_detection_table[BND_MESH][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_BOX][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_MESH][BND_SPHERE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, false };
  collision_detection_table[BND_MESH][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, false };

  collision_detection_table[BND_BOX][BND_PLANE] = (collision_detection_entry) { box_plane_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_PLANE] = (collision_detection_entry) { sphere_plane_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_PLANE] = (collision_detection_entry) { capsule_plane_collision, true, false };
  collision_detection_table[BND_MESH][BND_PLANE] = (collision_detection_entry) { mesh_plane_collision, true, false };
}

void run_broad_phase(bnd_world *world) {
  common_data *dynamics = (common_data *)&world->dynamics;
  common_data *statics  = (common_data *)&world->statics;

  for (count_t i = 0; i < dynamics->count; ++i) {
    for (count_t j = 0; j < i; j++) {
      bnd_collision_mask validation_mask = layer_to_mask(dynamics->collision_layers[i]);
      bnd_collision_mask reference_mask = world->matrix.matrix[dynamics->collision_layers[j]];

      if ((reference_mask & validation_mask) == 0) {
        continue;
      }

      bool potential_overlap = aabb_intersect(dynamics, dynamics, i, j);
    }
  }
}
