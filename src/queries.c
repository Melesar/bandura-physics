#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include <float.h>

typedef bool (*raycast_func)(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit);

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_type type;
  count_t body_index;
  count_t shape_index;
} raycast_context;

static bnd_ray ray_transform(bnd_ray r, bnd_v3 witness, bnd_quat rotation) {
  bnd_quat inv_rotation = bnd_quat_invert(rotation);
  r.origin = bnd_v3_rotate(bnd_v3_sub(r.origin, witness), inv_rotation);
  r.direction = bnd_v3_rotate(r.direction, inv_rotation);

  return r;
}

static bool check_ray_cylinder(bnd_ray local_ray, float half_height, float radius, bnd_raycast_hit *local_hit) {
  const float epsilon = 1e-6f;

  // --- infinite cylinder (XZ plane) ---
  float a = local_ray.direction.x * local_ray.direction.x + local_ray.direction.z * local_ray.direction.z;
  float b = 2.0f * (local_ray.origin.x * local_ray.direction.x + local_ray.origin.z * local_ray.direction.z);
  float c = local_ray.origin.x * local_ray.origin.x + local_ray.origin.z * local_ray.origin.z - radius * radius;

  float t_body_enter = -FLT_MAX;
  float t_body_exit = FLT_MAX;
  bool body_hit = false;

  if (fabsf(a) > epsilon) {
    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
      return false;
    float sq = sqrtf(disc);
    float inv2a = 1.0f / (2.0f * a);
    t_body_enter = (-b - sq) * inv2a;
    t_body_exit = (-b + sq) * inv2a;
    body_hit = true;
  } else {
    // ray parallel to axis — must be inside the infinite cylinder
    if (c > 0.0f)
      return false;
  }

  // --- end caps (Y axis slab) ---
  float t_cap_enter, t_cap_exit;
  bnd_v3 normal_cap_enter, normal_cap_exit;

  if (fabsf(local_ray.direction.y) > epsilon) {
    float inv_dy = 1.0f / local_ray.direction.y;
    float t1 = (-half_height - local_ray.origin.y) * inv_dy;
    float t2 = (half_height - local_ray.origin.y) * inv_dy;
    if (t1 < t2) {
      t_cap_enter = t1;
      normal_cap_enter = (bnd_v3){0, -1, 0};
      t_cap_exit = t2;
      normal_cap_exit = (bnd_v3){0, 1, 0};
    } else {
      t_cap_enter = t2;
      normal_cap_enter = (bnd_v3){0, 1, 0};
      t_cap_exit = t1;
      normal_cap_exit = (bnd_v3){0, -1, 0};
    }
  } else {
    // ray parallel to caps — must be between them
    if (local_ray.origin.y < -half_height || local_ray.origin.y > half_height)
      return false;
    t_cap_enter = -FLT_MAX;
    normal_cap_enter = (bnd_v3){0, -1, 0};
    t_cap_exit = FLT_MAX;

    normal_cap_enter = (bnd_v3){0, -1, 0};
    t_cap_exit = FLT_MAX;
    normal_cap_exit = (bnd_v3){0, 1, 0};
  }

  // --- intersect intervals ---
  float t_enter = (body_hit && t_body_enter > t_cap_enter) ? t_body_enter : t_cap_enter;
  float t_exit = (body_hit && t_body_exit < t_cap_exit) ? t_body_exit : t_cap_exit;

  if (t_enter > t_exit)
    return false;

  float t = t_enter;
  if (t < 0.0f)
    t = t_exit;
  if (t < 0.0f || t > local_ray.max_distance)
    return false;

  // --- normal in local space ---
  bnd_v3 local_normal;
  if (t == t_body_enter || (t_enter < 0.0f && t == t_body_exit)) {
    bnd_v3 p = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
    bnd_v3 radial = (bnd_v3){p.x, 0, p.z};
    local_normal = bnd_v3_normalize(radial);
  } else {
    local_normal = (t == t_cap_enter) ? normal_cap_enter : normal_cap_exit;
  }

  local_hit->distance = t;
  local_hit->point = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
  local_hit->normal = local_normal;

  return true;
}

static bool check_ray_sphere(bnd_ray ray, bnd_v3 center, float radius, bnd_raycast_hit *hit) {
  bnd_v3 offset = bnd_v3_sub(center, ray.origin);
  float o = bnd_v3_lensqr(offset);
  float rr = radius * radius;

  float tc = bnd_v3_dot(offset, ray.direction);
  if (tc < 0.0f && o > rr)
    return false;

  float d2 = o - tc * tc;
  if (d2 > rr)
    return false;

  float delta = sqrtf(rr - d2);
  float t = (o > rr) ? tc - delta : tc + delta;

  if (t < 0.0f || t > ray.max_distance)
    return false;

  hit->distance = t;
  hit->point = bnd_v3_add(ray.origin, bnd_v3_scale(ray.direction, t));
  hit->normal = bnd_v3_normalize(bnd_v3_sub(hit->point, center));

  return true;
}

static bool raycast_sphere(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 position = body_center(ctx);

  return check_ray_sphere(r, position, ctx->shape.value.sphere.radius, hit);
}

static bool raycast_box(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 half = bnd_v3_scale(ctx->shape.value.box.size, 0.5f);
  bnd_v3 position = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  float tmin = -FLT_MAX;
  float tmax = FLT_MAX;
  bnd_v3 near_normal = bnd_v3_zero();
  bnd_v3 far_normal = bnd_v3_zero();

  const float epsilon = 1e-6f;

  for (count_t axis = 0; axis < 3; ++axis) {
    float o = ((float *)&local_ray.origin)[axis];
    float d = ((float *)&local_ray.direction)[axis];
    float h = ((float *)&half)[axis];

    if (fabsf(d) < epsilon) {
      if (o < -h || o > h) {
        return false;
      }
      continue;
    }

    float t1 = (-h - o) / d;
    float t2 = (h - o) / d;

    bnd_v3 n1 = bnd_v3_zero();
    bnd_v3 n2 = bnd_v3_zero();
    ((float *)&n1)[axis] = -1.0f;
    ((float *)&n2)[axis] = 1.0f;

    if (t1 > t2) {
      float temp = t1;
      t1 = t2;
      t2 = temp;

      bnd_v3 ntemp = n1;
      n1 = n2;
      n2 = ntemp;
    }

    if (t1 > tmin) {
      tmin = t1;
      near_normal = n1;
    }

    if (t2 < tmax) {
      tmax = t2;
      far_normal = n2;
    }

    if (tmin > tmax) {
      return false;
    }
  }

  float distance = tmin;
  bnd_v3 local_normal = near_normal;

  if (distance < 0.0f) {
    distance = tmax;
    local_normal = far_normal;
  }

  if (distance < 0.0f || distance > r.max_distance) {
    return false;
  }

  hit->distance = distance;
  hit->point = bnd_v3_add(r.origin, bnd_v3_scale(r.direction, distance));
  hit->normal = bnd_v3_rotate(local_normal, rotation);

  return true;
}

static bool raycast_capsule(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, center, rotation);

  float height = ctx->shape.value.capsule.height;
  float radius = ctx->shape.value.capsule.radius;

  bnd_v3 local_cap_top =    (bnd_v3) { 0,  0.5 * height, 0 };
  bnd_v3 local_cap_bottom = (bnd_v3) { 0, -0.5 * height, 0 };

  bnd_raycast_hit proxy_hit = { 0 };
  if (check_ray_sphere(local_ray, local_cap_top, radius, &proxy_hit) && proxy_hit.point.y > local_cap_top.y) {
    goto hit;
  }

  if (check_ray_sphere(local_ray, local_cap_bottom, radius, &proxy_hit) && proxy_hit.point.y < local_cap_bottom.y) {
    goto hit;
  }

  if (check_ray_cylinder(local_ray, 0.5 * height, radius, &proxy_hit)) {
    goto hit;
  }

  return false;

  hit:
  hit->point = bnd_v3_add(center, bnd_v3_rotate(proxy_hit.point, rotation));
  hit->normal = bnd_v3_rotate(proxy_hit.normal, rotation);
  hit->distance = proxy_hit.distance;
  return true;
}

static bool raycast_plane(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  float dod = bnd_v3_dot(bnd_v3_sub(ctx->data->positions[ctx->index], r.origin), ctx->shape.value.plane.normal);
  float dd = bnd_v3_dot(r.direction, ctx->shape.value.plane.normal);

  if (dd >= 0)
    return false;

  float distance = dod / dd;

  if (distance > r.max_distance)
    return false;

  hit->distance = distance;
  hit->point = bnd_v3_add(r.origin, bnd_v3_scale(r.direction, distance));
  hit->normal = ctx->shape.value.plane.normal;

  return true;
}

static bool raycast_mesh(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 position = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  bool has_hit = false;
  float closest_distance = r.max_distance;
  bnd_v3 closest_point, normal;

  const mesh_storage *meshes = &ctx->world->meshes;
  bnd_mesh m = meshes->meshes[ctx->shape.value.mesh];

  count_t submeshes_start = m.submesh_offset;
  count_t submeshes_end = submeshes_start + m.submesh_count;

  for (count_t i = submeshes_start; i < submeshes_end; ++i) {
    submesh sm = meshes->submeshes[i];

    count_t index_start = sm.index_offset;
    count_t index_end = index_start + sm.index_count;

    for (count_t j = index_start; j + 2 < index_end; j += 3) {
      bnd_v3 v0 = meshes->verticies[meshes->indicies[j + 0]];
      bnd_v3 v1 = meshes->verticies[meshes->indicies[j + 1]];
      bnd_v3 v2 = meshes->verticies[meshes->indicies[j + 2]];

      bnd_v3 n = bnd_v3_cross(bnd_v3_sub(v1, v0), bnd_v3_sub(v2, v0));
      float d = bnd_v3_dot(n, local_ray.direction);
      if (d >= -EPSILON) {
        continue;
      }

      float t = (bnd_v3_dot(n, v0) - bnd_v3_dot(n, local_ray.origin)) / d;
      if (t < 0 || t > closest_distance) {
        continue;
      }

      bnd_v3 p = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
      bnd_v3 bary = bnd_v3_barycentric(p, v0, v1, v2);

      if (bary.x < -EPSILON || bary.y < -EPSILON || bary.z < -EPSILON) {
        continue;
      }

      has_hit = true;
      closest_distance = t;
      closest_point = p;
      normal = n;
    }
  }

  if (!has_hit) {
    return false;
  }

  hit->point = bnd_v3_add(position, bnd_v3_rotate(closest_point, rotation));
  hit->normal = bnd_v3_normalize(bnd_v3_rotate(normal, rotation));
  hit->distance = closest_distance;

  return true;
}

static raycast_func raycasts[] = {
  raycast_box,
  raycast_sphere,
  raycast_capsule,
  raycast_mesh,
  raycast_plane,
};

static raycast_context begin_raycast(const bnd_world *world, bnd_body_type type) {
  const common_data *data = as_common_const(world, type);

  return (raycast_context) {
    .world = world,
    .data = data,
    .type = type,
    .body_index = 0,
    .shape_index = 0,
  };
}

static bool next_raycast(raycast_context *ctx, bnd_ray r, bnd_raycast_hit *hit) {
  if (ctx->body_index >= ctx->data->count) {
    return false;
  }

  body_shapes shapes_info = ctx->data->shapes[ctx->body_index];
  bnd_body_shape *shapes = shapes_get(ctx->world, shapes_info);
  if (ctx->shape_index >= shapes_info.count) {
    ctx->body_index += 1;
    ctx->shape_index = 0;
    return next_raycast(ctx, r, hit);
  }

  bnd_body_shape shape = shapes[ctx->shape_index++];
  shape_context shape_ctx = { ctx->world, ctx->data, shape, ctx->body_index };

  bool is_hit = raycasts[shape.type](r, &shape_ctx, hit);
  if (is_hit) {
    hit->body = make_body_handle(ctx->world, ctx->type, ctx->body_index);
    return true;
  }

  return next_raycast(ctx, r, hit);
}

bool bnd_raycast_closest(const bnd_world *world, bnd_ray ray, bnd_raycast_hit *closest_hit) {
  closest_hit->distance = FLT_MAX;

  bnd_raycast_hit hit;
  raycast_context ctxs[] = { begin_raycast(world, BND_BODY_DYNAMIC), begin_raycast(world, BND_BODY_STATIC) };

  for (count_t i = 0; i < 2; ++i) {
    while(next_raycast(&ctxs[i], ray, &hit)) {
      if (hit.distance < closest_hit->distance) {
        *closest_hit = hit;
      }
    }
  }

  return closest_hit->distance < FLT_MAX;
}

count_t bnd_raycast_multiple(const bnd_world *world, bnd_ray ray, bnd_raycast_hit *hits, count_t max_hits) {
  if (max_hits == 0) {
    return 0;
  }

  count_t num_hits = 0;
  raycast_context ctxs[] = { begin_raycast(world, BND_BODY_DYNAMIC), begin_raycast(world, BND_BODY_STATIC) };

  for (count_t i = 0; i < 2; ++i) {
    while(next_raycast(&ctxs[i], ray, &hits[num_hits])) {
      num_hits += 1;
      if (num_hits >= max_hits) {
        return num_hits;
      }
    }
  }

  return num_hits;
}

static count_t overlap_typed(const bnd_world *world, bnd_v3 origin, float radius, bnd_body_handle *overlaps, count_t max_overlaps, bnd_body_type type) {
  const common_data *data = as_common_const(world, type);

  count_t ephemeral_index = ephemeral_body_index(data);
  data->positions[ephemeral_index] = origin;
  data->aabbs[ephemeral_index] = (bnd_aabb){ origin, (bnd_v3){radius, radius, radius} };

  bnd_body_shape ephemeral_shape = { BND_SPHERE, { .sphere = { radius } }, bnd_v3_zero(), bnd_quat_identity() };

  count_t overlap_count = 0;
  simplex s = { 0 };
  for (count_t i = 0; i < data->count; ++i) {
    if (!aabb_intersect(data, data, ephemeral_index, i)) {
      continue;
    }

    collision_detection_context ctx;
    ctx.world = world;
    ctx.data_a = data;
    ctx.data_b = data;
    ctx.body_a = ephemeral_index;
    ctx.body_b = i;
    ctx.shape_a = ephemeral_shape;

    body_shapes shape_info = data->shapes[i];
    bnd_body_shape *shapes = shapes_get(world, shape_info);
    for (count_t j = 0; j < shape_info.count; ++j) {
      ctx.shape_b = shapes[j];

      if (ctx.shape_b.type == BND_SPHERE) {
        bnd_v3 sphere_center = data->positions[i];
        float r = ctx.shape_b.value.sphere.radius + radius;

        if (bnd_v3_distancesqr(sphere_center, origin) > r * r) {
          continue;
        }
      } else if (ctx.shape_b.type == BND_PLANE) {
        bnd_v3 plane_point = data->positions[i];
        bnd_v3 plane_normal = ctx.shape_b.value.plane.normal;

        if (bnd_v3_dot(plane_normal, bnd_v3_sub(origin, plane_point)) > radius) {
          continue;
        }
      } else if (!gjk_check_intersection(world, &ctx, &s)) {
        continue;
      }

      overlaps[overlap_count++] = make_body_handle(world, type, i);
      if (overlap_count >= max_overlaps) {
        return overlap_count;
      }
    }
  }

  return overlap_count;
}

count_t bnd_overlap(const bnd_world *world, bnd_v3 origin, float radius, bnd_body_handle *overlaps, count_t max_overlaps) {
  if (max_overlaps == 0) {
    return 0;
  }

  count_t overlap_count = overlap_typed(world, origin, radius, overlaps, max_overlaps, BND_BODY_DYNAMIC);
  if (overlap_count == max_overlaps) {
    return overlap_count;
  }

  max_overlaps -= overlap_count;
  overlap_count += overlap_typed(world, origin, radius, overlaps + overlap_count, max_overlaps, BND_BODY_STATIC);

  return overlap_count;
}
