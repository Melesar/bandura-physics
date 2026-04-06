#include "physics.h"
#include <float.h>
#include <math.h>

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
    body_shape shape = *shapes_get(world, data->shapes[i]);
    raycast_hit *hit = hits + hit_count + num_hits;
    count_t prev_count = num_hits;

    switch (shape.type) {
      case SHAPE_BOX:
        num_hits +=
            raycast_box(origin, direction, max_distance, data->positions[i], shape.box.size, data->rotations[i], hit);
        break;

      case SHAPE_PLANE:
        num_hits += raycast_plane(origin, direction, max_distance, data->positions[i], shape.plane.normal, hit);
        break;

      default:
        break;
    }

    if (num_hits > prev_count) {
      hit->body = make_body_handle(world, type, i);
    }

    if (num_hits >= max_hits) {
      return num_hits;
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
