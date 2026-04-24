#include "bnd-core.h"
#include <float.h>
#include <math.h>

static bool raycast_sphere(v3 origin, v3 direction, float max_distance, v3 center, float radius, raycast_hit *hit) {
  v3 offset = sub(center, origin);
  float o = lensq(offset);
  float r = radius * radius;

  float tc = dot(offset, direction);
  if (tc < 0.0f && o > r)
    return false;

  float d2 = o - tc * tc;
  if (d2 > r)
    return false;

  float delta = sqrtf(r - d2);
  float t = (o > r) ? tc - delta : tc + delta;

  if (t < 0.0f || t > max_distance)
    return false;

  hit->distance = t;
  hit->point = add(origin, scale(direction, t));
  hit->normal = normalize(sub(hit->point, center));

  return true;
}

static bool raycast_box(v3 origin, v3 direction, float max_distance, v3 position, v3 size, quat rotation,
                        raycast_hit *hit) {
  v3 half = scale(size, 0.5f);
  quat inv_rotation = qinvert(rotation);
  v3 local_origin = rotate(sub(origin, position), inv_rotation);
  v3 local_direction = rotate(direction, inv_rotation);

  float tmin = -FLT_MAX;
  float tmax = FLT_MAX;
  v3 near_normal = zero();
  v3 far_normal = zero();

  const float epsilon = 1e-6f;

  for (count_t axis = 0; axis < 3; ++axis) {
    float o = ((float *)&local_origin)[axis];
    float d = ((float *)&local_direction)[axis];
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

  if (distance < 0.0f || distance > max_distance) {
    return false;
  }

  hit->distance = distance;
  hit->point = add(origin, scale(direction, distance));
  hit->normal = rotate(local_normal, rotation);

  return true;
}

static bool raycast_cylinder(v3 origin, v3 direction, float max_distance, v3 position, float radius, float height,
                             quat rotation, raycast_hit *hit) {
  quat inv_rotation = qinvert(rotation);
  v3 lo = rotate(sub(origin, position), inv_rotation);
  v3 ld = rotate(direction, inv_rotation);

  float half_h = height * 0.5f;
  const float epsilon = 1e-6f;

  // --- infinite cylinder (XZ plane) ---
  float a = ld.x * ld.x + ld.z * ld.z;
  float b = 2.0f * (lo.x * ld.x + lo.z * ld.z);
  float c = lo.x * lo.x + lo.z * lo.z - radius * radius;

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

  if (fabsf(ld.y) > epsilon) {
    float inv_dy = 1.0f / ld.y;
    float t1 = (-half_h - lo.y) * inv_dy;
    float t2 = (half_h - lo.y) * inv_dy;
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
    if (lo.y < -half_h || lo.y > half_h)
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
  if (t < 0.0f || t > max_distance)
    return false;

  // --- normal in local space ---
  v3 local_normal;
  if (t == t_body_enter || (t_enter < 0.0f && t == t_body_exit)) {
    v3 p = add(lo, scale(ld, t));
    v3 radial = (v3){p.x, 0, p.z};
    local_normal = normalize(radial);
  } else {
    local_normal = (t == t_cap_enter) ? normal_cap_enter : normal_cap_exit;
  }

  hit->distance = t;
  hit->point = add(origin, scale(direction, t));
  hit->normal = rotate(local_normal, rotation);
  return true;
}

static bool raycast_plane(v3 origin, v3 direction, float max_distance, v3 point, v3 normal, raycast_hit *hit) {
  float dod = dot(sub(point, origin), normal);
  float dd = dot(direction, normal);

  if (dd >= 0)
    return false;

  float distance = dod / dd;

  if (distance > max_distance)
    return false;

  hit->distance = distance;
  hit->point = add(origin, scale(direction, distance));
  hit->normal = normal;

  return true;
}

static count_t raycast_bodies(const physics_world *world, body_type type, v3 origin, v3 direction, float max_distance,
                              count_t hit_count, count_t max_hits, raycast_hit *hits) {
  if (hit_count >= max_hits) {
    return 0;
  }

  count_t num_hits = 0;
  const common_data *data = as_common_const(world, type);
  for (count_t i = 0; i < data->count; ++i) {
    body_shape *shapes = shapes_get(world, data->shapes[i]);

    for (count_t j = 0; j < data->shapes[i].count; ++j) {
      raycast_hit *hit = hits + hit_count + num_hits;
      body_shape shape = shapes[j];

      bool is_hit;
      switch (shape.type) {
        case SHAPE_BOX:
          is_hit =
              raycast_box(origin, direction, max_distance, data->positions[i], shape.box.size, data->rotations[i], hit);
          break;

        case SHAPE_SPHERE:
          is_hit = raycast_sphere(origin, direction, max_distance, data->positions[i], shape.sphere.radius, hit);
          break;

        case SHAPE_PLANE:
          is_hit = raycast_plane(origin, direction, max_distance, data->positions[i], shape.plane.normal, hit);
          break;

        case SHAPE_CYLINDER:
          is_hit = raycast_cylinder(origin, direction, max_distance, data->positions[i], shape.cylinder.radius,
                                    shape.cylinder.height, data->rotations[i], hit);
          break;

        default:
          is_hit = false;
          break;
      }

      if (is_hit) {
        hit->body = make_body_handle(world, type, i);
      }

      num_hits += is_hit;
      if (hit_count + num_hits == max_hits) {
        return num_hits;
      }

      if (is_hit) {
        break;
      }
    }
  }

  return num_hits;
}

count_t physics_raycast(const physics_world *world, v3 origin, v3 direction, float max_distance, count_t max_hits,
                        raycast_hit *hits) {
  count_t hit_count = 0;

  hit_count += raycast_bodies(world, BODY_DYNAMIC, origin, direction, max_distance, hit_count, max_hits, hits);
  hit_count += raycast_bodies(world, BODY_STATIC, origin, direction, max_distance, hit_count, max_hits, hits);

  return hit_count;
}
