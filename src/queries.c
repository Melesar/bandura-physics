#include "bandura.h"
#include "bnd-core.h"
#include <float.h>
#include <math.h>

typedef bool (*raycast_func)(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit);

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_type type;
  count_t body_index;
  count_t shape_index;
} raycast_context;

static bnd_ray ray_transform(bnd_ray r, v3 witness, quat rotation) {
  quat inv_rotation = qinvert(rotation);
  r.origin = rotate(sub(r.origin, witness), inv_rotation);
  r.direction = rotate(r.direction, inv_rotation);

  return r;
}

static bool raycast_sphere(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  v3 position = body_center(ctx);

  v3 offset = sub(position, r.origin);
  float o = lensq(offset);
  float rr = ctx->shape.sphere.radius * ctx->shape.sphere.radius;

  float tc = dot(offset, r.direction);
  if (tc < 0.0f && o > rr)
    return false;

  float d2 = o - tc * tc;
  if (d2 > rr)
    return false;

  float delta = sqrtf(rr - d2);
  float t = (o > rr) ? tc - delta : tc + delta;

  if (t < 0.0f || t > r.max_distance)
    return false;

  hit->distance = t;
  hit->point = add(r.origin, scale(r.direction, t));
  hit->normal = normalize(sub(hit->point, position));

  return true;
}

static bool raycast_box(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  v3 half = scale(ctx->shape.box.size, 0.5f);
  v3 position = body_center(ctx);
  quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  float tmin = -FLT_MAX;
  float tmax = FLT_MAX;
  v3 near_normal = zero();
  v3 far_normal = zero();

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

    v3 n1 = zero();
    v3 n2 = zero();
    ((float *)&n1)[axis] = -1.0f;
    ((float *)&n2)[axis] = 1.0f;

    if (t1 > t2) {
      float temp = t1;
      t1 = t2;
      t2 = temp;

      v3 ntemp = n1;
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
  v3 local_normal = near_normal;

  if (distance < 0.0f) {
    distance = tmax;
    local_normal = far_normal;
  }

  if (distance < 0.0f || distance > r.max_distance) {
    return false;
  }

  hit->distance = distance;
  hit->point = add(r.origin, scale(r.direction, distance));
  hit->normal = rotate(local_normal, rotation);

  return true;
}

static bool raycast_cylinder(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  v3 position = body_center(ctx);
  quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  float half_h = ctx->shape.cylinder.height * 0.5f;
  const float epsilon = 1e-6f;

  // --- infinite cylinder (XZ plane) ---
  float a = local_ray.direction.x * local_ray.direction.x + local_ray.direction.z * local_ray.direction.z;
  float b = 2.0f * (local_ray.origin.x * local_ray.direction.x + local_ray.origin.z * local_ray.direction.z);
  float c = local_ray.origin.x * local_ray.origin.x + local_ray.origin.z * local_ray.origin.z - ctx->shape.cylinder.radius * ctx->shape.cylinder.radius;

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
  v3 normal_cap_enter, normal_cap_exit;

  if (fabsf(local_ray.direction.y) > epsilon) {
    float inv_dy = 1.0f / local_ray.direction.y;
    float t1 = (-half_h - local_ray.origin.y) * inv_dy;
    float t2 = (half_h - local_ray.origin.y) * inv_dy;
    if (t1 < t2) {
      t_cap_enter = t1;
      normal_cap_enter = (v3){0, -1, 0};
      t_cap_exit = t2;
      normal_cap_exit = (v3){0, 1, 0};
    } else {
      t_cap_enter = t2;
      normal_cap_enter = (v3){0, 1, 0};
      t_cap_exit = t1;
      normal_cap_exit = (v3){0, -1, 0};
    }
  } else {
    // ray parallel to caps — must be between them
    if (local_ray.origin.y < -half_h || local_ray.origin.y > half_h)
      return false;
    t_cap_enter = -FLT_MAX;
    normal_cap_enter = (v3){0, -1, 0};
    t_cap_exit = FLT_MAX;
    normal_cap_exit = (v3){0, 1, 0};
  }

  // --- intersect intervals ---
  float t_enter = (body_hit && t_body_enter > t_cap_enter) ? t_body_enter : t_cap_enter;
  float t_exit = (body_hit && t_body_exit < t_cap_exit) ? t_body_exit : t_cap_exit;

  if (t_enter > t_exit)
    return false;

  float t = t_enter;
  if (t < 0.0f)
    t = t_exit;
  if (t < 0.0f || t > r.max_distance)
    return false;

  // --- normal in local space ---
  v3 local_normal;
  if (t == t_body_enter || (t_enter < 0.0f && t == t_body_exit)) {
    v3 p = add(local_ray.origin, scale(local_ray.direction, t));
    v3 radial = (v3){p.x, 0, p.z};
    local_normal = normalize(radial);
  } else {
    local_normal = (t == t_cap_enter) ? normal_cap_enter : normal_cap_exit;
  }

  hit->distance = t;
  hit->point = add(r.origin, scale(r.direction, t));
  hit->normal = rotate(local_normal, rotation);
  return true;
}

static bool raycast_plane(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  float dod = dot(sub(ctx->data->positions[ctx->index], r.origin), ctx->shape.plane.normal);
  float dd = dot(r.direction, ctx->shape.plane.normal);

  if (dd >= 0)
    return false;

  float distance = dod / dd;

  if (distance > r.max_distance)
    return false;

  hit->distance = distance;
  hit->point = add(r.origin, scale(r.direction, distance));
  hit->normal = ctx->shape.plane.normal;

  return true;
}

static bool raycast_mesh(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  v3 position = body_center(ctx);
  quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  bool has_hit = false;
  float closest_distance = r.max_distance;
  v3 closest_point, normal;

  const mesh_storage *meshes = &ctx->world->meshes;
  bnd_mesh m = meshes->meshes[ctx->shape.mesh];

  count_t submeshes_start = m.submesh_offset;
  count_t submeshes_end = submeshes_start + m.submesh_count;

  for (count_t i = submeshes_start; i < submeshes_end; ++i) {
    submesh sm = meshes->submeshes[i];

    count_t index_start = sm.index_offset;
    count_t index_end = index_start + sm.index_count;

    for (count_t j = index_start; j + 2 < index_end; j += 3) {
      v3 v0 = meshes->verticies[meshes->indicies[j + 0]];
      v3 v1 = meshes->verticies[meshes->indicies[j + 1]];
      v3 v2 = meshes->verticies[meshes->indicies[j + 2]];

      v3 n = cross(sub(v1, v0), sub(v2, v0));
      float d = dot(n, local_ray.direction);
      if (d >= -EPSILON) {
        continue;
      }

      float t = (dot(n, v0) - dot(n, local_ray.origin)) / d;
      if (t < 0 || t > closest_distance) {
        continue;
      }

      v3 p = add(local_ray.origin, scale(local_ray.direction, t));
      v3 bary = barycentric(p, v0, v1, v2);

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

  hit->point = add(position, rotate(closest_point, rotation));
  hit->normal = normalize(rotate(normal, rotation));
  hit->distance = closest_distance;

  return true;
}

static raycast_func raycasts[] = {
  raycast_box,
  raycast_sphere,
  raycast_cylinder,
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
  raycast_context ctxs[] = { begin_raycast(world, BND_DYNAMIC), begin_raycast(world, BND_STATIC) };

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
  raycast_context ctxs[] = { begin_raycast(world, BND_DYNAMIC), begin_raycast(world, BND_STATIC) };

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

static count_t overlap_typed(const bnd_world *world, v3 origin, float radius, bnd_body_handle *overlaps, count_t max_overlaps, bnd_body_type type) {
  const common_data *dynamics = (common_data *) &world->dynamics;
  const common_data *data = as_common_const(world, type);

  count_t ephemeral_index = ephemeral_body_index(dynamics);
  data->positions[ephemeral_index] = origin;
  data->aabbs[ephemeral_index] = (bnd_aabb){ origin, vec3(radius, radius, radius) };

  bnd_body_shape ephemeral_shape = { BND_SPHERE, { .sphere = { radius } }, zero(), qidentity() };

  count_t overlap_count = 0;
  simplex s = { 0 };
  for (count_t i = 0; i < data->count; ++i) {
    if (!aabb_intersect(dynamics, data, ephemeral_index, i)) {
      continue;
    }

    collision_detection_context ctx;
    ctx.world = world;
    ctx.data_a = dynamics;
    ctx.data_b = data;
    ctx.body_a = ephemeral_index;
    ctx.body_b = i;
    ctx.shape_a = ephemeral_shape;

    body_shapes shape_info = data->shapes[i];
    bnd_body_shape *shapes = shapes_get(world, shape_info);
    for (count_t j = 0; j < shape_info.count; ++j) {
      ctx.shape_b = shapes[j];

      if (ctx.shape_b.type == BND_SPHERE) {
        v3 sphere_center = data->positions[i];
        float r = ctx.shape_b.sphere.radius + radius;

        if (distancesqr(sphere_center, origin) > r * r) {
          continue;
        }
      } else if (ctx.shape_b.type == BND_PLANE) {
        v3 plane_point = data->positions[i];
        v3 plane_normal = ctx.shape_b.plane.normal;

        if (dot(plane_normal, sub(origin, plane_point)) > radius) {
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

count_t bnd_overlap(const bnd_world *world, v3 origin, float radius, bnd_body_handle *overlaps, count_t max_overlaps) {
  if (max_overlaps == 0) {
    return 0;
  }

  count_t overlap_count = overlap_typed(world, origin, radius, overlaps, max_overlaps, BND_DYNAMIC);
  if (overlap_count == max_overlaps) {
    return overlap_count;
  }

  max_overlaps -= overlap_count;
  overlap_count += overlap_typed(world, origin, radius, overlaps + overlap_count, max_overlaps, BND_STATIC);

  return overlap_count;
}
