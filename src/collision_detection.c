#include "bandura.h"
#include "profiler.h"
#include "physics.h"
#include <math.h>
#include <stdlib.h>

#define ARRAY_RESIZE_IF_NEEDED(array, count, capacity, type) \
  while (count >= capacity) { \
    capacity <<= 1; \
    if (count <= capacity) { \
      array = realloc(array, capacity * sizeof(type)); \
      break; \
    } \
  }

typedef struct {
  v3 center;
  v3 size;
  v3 axis[3];
} collision_box;

static contact *new_contact(physics_world *world, const collision_detection_context *ctx) {
  contact *c = &world->collisions.contacts[world->collisions.count++];
  c->index_a = ctx->body_a;
  c->index_b = ctx->body_b;
  c->friction = world->config.friction;
  c->restitution = world->config.restitution;

  return c;
}

typedef count_t(*collision_func)(physics_world *world, const collision_detection_context *ctx);

static m4 body_transform(v3 shape_offset, quat shape_rotation, v3 body_position, quat body_rotation) {
  m4 transform = as_matrix(shape_rotation);
  transform = mul(transform, translate(shape_offset));
  transform = mul(transform, as_matrix(body_rotation));
  transform = mul(transform, translate(body_position));

  return transform;
}

static v3 body_center(v3 shape_offset, quat global_rotation, v3 body_position) {
  v3 center = shape_offset;
  center = rotate(center, global_rotation);
  center = add(center, body_position);

  return center;
}

static v3 body_a_center(const collision_detection_context *ctx) {
  return body_center(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static v3 body_b_center(const collision_detection_context *ctx) {
  return body_center(ctx->shape_b.offset, ctx->data_b->rotations[ctx->body_b], ctx->data_b->positions[ctx->body_b]);
}

static m4 body_a_transform(const collision_detection_context *ctx) {
  return body_transform(ctx->shape_a.offset, ctx->shape_a.rotation, ctx->data_a->positions[ctx->body_a], ctx->data_a->rotations[ctx->body_a]);
}

collision_box collision_box_make(const physics_world *world, const common_data *data, count_t index, body_shape shape) {
  quat rotation = qmul(data->rotations[index], shape.rotation);

  return (collision_box) {
    .center = body_center(shape.offset, data->rotations[index], data->positions[index]),
    .size = shape.box.size,
    .axis = { rotate(right(), rotation), rotate(up(), rotation), rotate(forward(), rotation) }
  };
}

m4 collision_box_transform(const collision_box *box) {
  m4 transform = { 0 };

  transform.m0 = box->axis[0].x;
  transform.m1 = box->axis[0].y;
  transform.m2 = box->axis[0].z;

  transform.m4 = box->axis[1].x;
  transform.m5 = box->axis[1].y;
  transform.m6 = box->axis[1].z;

  transform.m8 = box->axis[2].x;
  transform.m9 = box->axis[2].y;
  transform.m10 = box->axis[2].z;

  transform.m12 = box->center.x;
  transform.m13 = box->center.y;
  transform.m14 = box->center.z;

  transform.m15 = 1;

  return transform;
}

static v3 contact_point(
  v3 point_a, v3 axis_a, float half_size_a,
  v3 point_b, v3 axis_b, float half_size_b,
  bool use_a
) {
  float len_a = lensq(axis_a);
  float len_b = lensq(axis_b);
  float dp = dot(axis_a, axis_b);

  v3 offset = sub(point_a, point_b);

  float dp_a = dot(axis_a, offset);
  float dp_b = dot(axis_b, offset);

  float denom = len_a * len_b - dp * dp;

  if (fabsf(denom) < 0.0001f)
    return use_a ? point_a : point_b;

  float a = (dp * dp_b - len_b * dp_a) / denom;
  float b = (len_a * dp_b - dp * dp_a) / denom;

  if (a > half_size_a || a < -half_size_a || b > half_size_b || b < -half_size_b)
    return use_a ? point_a : point_b;

  v3 c_a = add(point_a, scale(axis_a, a));
  v3 c_b = add(point_b, scale(axis_b, b));

  return add(scale(c_a, 0.5), scale(c_b, 0.5));
}

static void fill_point_face_box_box(
  const collision_box *box_a,
  const collision_box *box_b,
  v3 offset,
  count_t best_axis,
  float penetration,
  collisions *collisions) {

  v3 normal = box_a->axis[best_axis];
  if (dot(box_a->axis[best_axis], offset) > 0)
    normal = scale(normal, -1);

  v3 vertex = scale(box_b->size, 0.5);
  if (dot(box_b->axis[0], normal) < 0) vertex.x = -vertex.x;
  if (dot(box_b->axis[1], normal) < 0) vertex.y = -vertex.y;
  if (dot(box_b->axis[2], normal) < 0) vertex.z = -vertex.z;

  contact *contact = &collisions->contacts[collisions->count - 1];
  contact->point = transform(vertex, collision_box_transform(box_b));
  contact->normal = normal;
  contact->depth = penetration;
}

static float transform_to_axis(const collision_box *box, v3 axis) {
  v3 half_size = scale(box->size, 0.5);
  return
    half_size.x * fabsf(dot(axis, box->axis[0])) +
    half_size.y * fabsf(dot(axis, box->axis[1])) +
    half_size.z * fabsf(dot(axis, box->axis[2]));
}

static bool try_axis(const collision_box *box_a, const collision_box *box_b, v3 axis, count_t index, v3 offset, float *min_penetration, count_t *min_index) {
  if (lensq(axis) < 0.0001)
    return true;

  axis = normalize(axis);

  float project_a = transform_to_axis(box_a, axis);
  float project_b = transform_to_axis(box_b, axis);

  float distance = fabsf(dot(offset, axis));
  float penetration = project_a + project_b - distance;

  if (penetration < 0)
    return false;

  if (penetration < *min_penetration) {
    *min_penetration = penetration;
    *min_index = index;
  }

  return true;
}

#define CHECK_OVERLAP(axis, index) \
  if (!try_axis(&box_a, &box_b, (axis), (index), offset, &penetration, &best_axis)) return 0;

static count_t box_box_collision(physics_world *world, const collision_detection_context *ctx, bool *switched_bodies) {
  *switched_bodies = false;

  collision_box box_a = collision_box_make(world, ctx->data_a, ctx->body_a, ctx->shape_a);
  collision_box box_b = collision_box_make(world, ctx->data_b, ctx->body_b, ctx->shape_b);

  v3 offset = sub(box_b.center, box_a.center);

  float penetration = INFINITY;
  count_t best_axis = -1;

  CHECK_OVERLAP(box_a.axis[0], 0)
  CHECK_OVERLAP(box_a.axis[1], 1)
  CHECK_OVERLAP(box_a.axis[2], 2)

  CHECK_OVERLAP(box_b.axis[0], 3)
  CHECK_OVERLAP(box_b.axis[1], 4)
  CHECK_OVERLAP(box_b.axis[2], 5)

  count_t best_single_axis = best_axis;

  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[0]), 6)
  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[1]), 7)
  CHECK_OVERLAP(cross(box_a.axis[0], box_b.axis[2]), 8)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[0]), 9)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[1]), 10)
  CHECK_OVERLAP(cross(box_a.axis[1], box_b.axis[2]), 11)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[0]), 12)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[1]), 13)
  CHECK_OVERLAP(cross(box_a.axis[2], box_b.axis[2]), 14)

  if (best_axis == (count_t)-1) {
    return 0;
  }

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);

  if (best_axis < 3) {
    // We've got a vertex of box two on a face of box one.
    fill_point_face_box_box(&box_a, &box_b, offset, best_axis, penetration, collisions);
  } else if (best_axis < 6) {
    // We've got a vertex of box one on a face of box two.
    // We use the same algorithm as above, but swap around
    // one and two (and therefore also the vector between their
    // centres).
    fill_point_face_box_box(&box_b, &box_a, scale(offset, -1), best_axis - 3, penetration, collisions);
    *switched_bodies = true;
  } else {
    // We've got an edge-edge contact. Find out which axes
    best_axis -= 6;
    count_t axis_a_index = best_axis / 3;
    count_t axis_b_index = best_axis % 3;
    v3 axis_a = box_a.axis[axis_a_index];
    v3 axis_b = box_b.axis[axis_b_index];
    v3 axis = normalize(cross(axis_a, axis_b));

    if (dot(axis, offset) > 0)
      axis = scale(axis, -1);

    v3 half_size_a = scale(box_a.size, 0.5);
    v3 half_size_b = scale(box_b.size, 0.5);
    v3 edge_point_a = half_size_a;
    v3 edge_point_b = half_size_b;
    for (count_t i = 0; i < 3; ++i) {
      float *p_a = (float*)&edge_point_a;
      float *p_b = (float*)&edge_point_b;

      if (i == axis_a_index)
        p_a[i] = 0;
      else if (dot(box_a.axis[i], axis) > 0)
        p_a[i] = -p_a[i];

      if (i == axis_b_index)
        p_b[i] = 0;
      else if (dot(box_b.axis[i], axis) < 0)
        p_b[i] = -p_b[i];
    }

    edge_point_a = transform(edge_point_a, collision_box_transform(&box_a));
    edge_point_b = transform(edge_point_b, collision_box_transform(&box_b));

    v3 vertex = contact_point(
      edge_point_a, axis_a, *((float*)&half_size_a + axis_a_index),
      edge_point_b, axis_b, *((float*)&half_size_b + axis_b_index),
      best_single_axis > 2);

    contact->point = vertex;
    contact->normal = axis;
    contact->depth = penetration;
  }

  return 1;
}

#undef CHECK_OVERLAP

static count_t box_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  quat shape_rotation = ctx->shape_a.rotation;

  v3 box_center = body_a_center(ctx);
  v3 extents = scale(ctx->shape_a.box.size, 0.5);

  v3 plane_normal = ctx->shape_b.plane.normal;
  v3 plane_point = ctx->data_b->positions[ctx->body_b];

  v3 corners[] = {
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

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + max_contacts, collisions->capacity, contact)

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

count_t box_sphere_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 sphere_center = body_b_center(ctx);
  float sphere_radius = ctx->shape_b.sphere.radius;

  v3 box_half_extents = scale(ctx->shape_a.box.size, 0.5);
  m4 box_transform = body_a_transform(ctx);
  v3 relative_sphere_center = transform(sphere_center, inverse(box_transform));

  if (fabsf(relative_sphere_center.x) > box_half_extents.x + sphere_radius ||
      fabsf(relative_sphere_center.y) > box_half_extents.y + sphere_radius ||
      fabsf(relative_sphere_center.z) > box_half_extents.z + sphere_radius) {
    return 0;
  }

  v3 closest_point;
  float distance = relative_sphere_center.x;
  if (relative_sphere_center.x > box_half_extents.x) distance = box_half_extents.x;
  else if (relative_sphere_center.x < -box_half_extents.x) distance = -box_half_extents.x;
  closest_point.x = distance;

  distance = relative_sphere_center.y;
  if (relative_sphere_center.y > box_half_extents.y) distance = box_half_extents.y;
  else if (relative_sphere_center.y < -box_half_extents.y) distance = -box_half_extents.y;
  closest_point.y = distance;

  distance = relative_sphere_center.z;
  if (relative_sphere_center.z > box_half_extents.z) distance = box_half_extents.z;
  else if (relative_sphere_center.z < -box_half_extents.z) distance = -box_half_extents.z;
  closest_point.z = distance;

  distance = distancesqr(closest_point, relative_sphere_center);
  if (distance > sphere_radius * sphere_radius)
    return 0;

  closest_point = transform(closest_point, box_transform);

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->point = closest_point;
  contact->normal = normalize(sub(closest_point, sphere_center));
  contact->depth = sphere_radius - sqrtf(distance);

  return 1;
}

static v3 closest_point_on_box(const collision_box *box, v3 point) {
  v3 rel = sub(point, box->center);
  v3 half_size = scale(box->size, 0.5f);
  v3 result = box->center;

  for (count_t axis_index = 0; axis_index < 3; ++axis_index) {
    float distance = dot(rel, box->axis[axis_index]);
    float max_distance = *((float *)&half_size + axis_index);

    if (distance > max_distance) distance = max_distance;
    else if (distance < -max_distance) distance = -max_distance;

    result = add(result, scale(box->axis[axis_index], distance));
  }

  return result;
}

static v3 closest_point_on_cylinder(v3 center, v3 axis, float half_height, float radius, v3 point) {
  v3 rel = sub(point, center);
  float y = dot(rel, axis);

  if (y > half_height) y = half_height;
  else if (y < -half_height) y = -half_height;

  v3 radial = sub(rel, scale(axis, y));
  float radial_len_sq = lensq(radial);
  if (radial_len_sq > radius * radius)
    radial = scale(radial, radius / sqrtf(radial_len_sq));

  return add(center, add(scale(axis, y), radial));
}

static float cylinder_transform_to_axis(v3 axis, v3 cylinder_axis, float half_height, float radius) {
  float axis_projection = fabsf(dot(axis, cylinder_axis));
  float radial_projection_sq = 1.0f - axis_projection * axis_projection;
  if (radial_projection_sq < 0.0f)
    radial_projection_sq = 0.0f;

  return half_height * axis_projection + radius * sqrtf(radial_projection_sq);
}

static bool try_axis_box_cylinder(
  const collision_box *box,
  v3 cylinder_axis,
  float cylinder_half_height,
  float cylinder_radius,
  v3 axis,
  v3 offset,
  float *min_penetration,
  v3 *best_axis,
  bool *has_axis) {

  if (lensq(axis) < 0.0001f)
    return true;

  axis = normalize(axis);

  float project_box = transform_to_axis(box, axis);
  float project_cylinder = cylinder_transform_to_axis(axis, cylinder_axis, cylinder_half_height, cylinder_radius);

  float distance = fabsf(dot(offset, axis));
  float penetration = project_box + project_cylinder - distance;

  if (penetration < 0.0f)
    return false;

  if (penetration < *min_penetration) {
    *min_penetration = penetration;
    *best_axis = axis;
    *has_axis = true;
  }

  return true;
}

static bool try_axis_cylinder_cylinder(
  v3 axis_a,
  float half_height_a,
  float radius_a,
  v3 axis_b,
  float half_height_b,
  float radius_b,
  v3 axis,
  v3 offset,
  float *min_penetration,
  v3 *best_axis,
  bool *has_axis) {

  if (lensq(axis) < 0.0001f)
    return true;

  axis = normalize(axis);

  float project_a = cylinder_transform_to_axis(axis, axis_a, half_height_a, radius_a);
  float project_b = cylinder_transform_to_axis(axis, axis_b, half_height_b, radius_b);

  float distance = fabsf(dot(offset, axis));
  float penetration = project_a + project_b - distance;

  if (penetration < 0.0f)
    return false;

  if (penetration < *min_penetration) {
    *min_penetration = penetration;
    *best_axis = axis;
    *has_axis = true;
  }

  return true;
}

static v3 support_point_on_cylinder(v3 center, v3 axis, float half_height, float radius, v3 direction) {
  float axis_sign = dot(direction, axis) >= 0.0f ? 1.0f : -1.0f;
  v3 point = add(center, scale(axis, axis_sign * half_height));

  v3 radial = sub(direction, scale(axis, dot(direction, axis)));
  float radial_len_sq = lensq(radial);
  if (radial_len_sq > 0.000001f)
    point = add(point, scale(radial, radius / sqrtf(radial_len_sq)));

  return point;
}

static void closest_points_on_segment_segment(v3 p1, v3 q1, v3 p2, v3 q2, v3 *c1, v3 *c2) {
  v3 d1 = sub(q1, p1);
  v3 d2 = sub(q2, p2);
  v3 r = sub(p1, p2);
  float a = dot(d1, d1);
  float e = dot(d2, d2);
  float f = dot(d2, r);

  float s;
  float t;
  const float eps = 0.000001f;

  if (a <= eps && e <= eps) {
    *c1 = p1;
    *c2 = p2;
    return;
  }

  if (a <= eps) {
    s = 0.0f;
    t = f / e;
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;
  } else {
    float c = dot(d1, r);

    if (e <= eps) {
      t = 0.0f;
      s = -c / a;
      if (s < 0.0f) s = 0.0f;
      else if (s > 1.0f) s = 1.0f;
    } else {
      float b = dot(d1, d2);
      float denom = a * e - b * b;

      if (denom != 0.0f) {
        s = (b * f - c * e) / denom;
        if (s < 0.0f) s = 0.0f;
        else if (s > 1.0f) s = 1.0f;
      } else {
        s = 0.0f;
      }

      t = (b * s + f) / e;
      if (t < 0.0f) {
        t = 0.0f;
        s = -c / a;
        if (s < 0.0f) s = 0.0f;
        else if (s > 1.0f) s = 1.0f;
      } else if (t > 1.0f) {
        t = 1.0f;
        s = (b - c) / a;
        if (s < 0.0f) s = 0.0f;
        else if (s > 1.0f) s = 1.0f;
      }
    }
  }

  *c1 = add(p1, scale(d1, s));
  *c2 = add(p2, scale(d2, t));
}

count_t cylinder_box_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 cylinder_center = body_a_center(ctx);
  quat cylinder_rotation = ctx->data_a->rotations[ctx->body_a];
  quat shape_rotation = ctx->shape_a.rotation;
  v3 cylinder_axis = rotate(up(), qmul(cylinder_rotation, shape_rotation));
  float cylinder_radius = ctx->shape_a.cylinder.radius;
  float cylinder_half_height = ctx->shape_a.cylinder.height * 0.5f;

  collision_box box = collision_box_make(world, ctx->data_b, ctx->body_b, ctx->shape_b);

  v3 point_on_cylinder = cylinder_center;
  v3 point_on_box = closest_point_on_box(&box, point_on_cylinder);
  for (count_t iteration = 0; iteration < 12; ++iteration) {
    v3 new_point_on_cylinder = closest_point_on_cylinder(cylinder_center, cylinder_axis, cylinder_half_height, cylinder_radius, point_on_box);
    if (distancesqr(new_point_on_cylinder, point_on_cylinder) < 0.00000001f) {
      point_on_cylinder = new_point_on_cylinder;
      break;
    }

    point_on_cylinder = new_point_on_cylinder;
    point_on_box = closest_point_on_box(&box, point_on_cylinder);
  }

  point_on_box = closest_point_on_box(&box, point_on_cylinder);

  v3 delta = sub(point_on_cylinder, point_on_box);
  float distance = len(delta);
  if (distance > 0.0005f)
    return 0;

  v3 offset = sub(cylinder_center, box.center);
  float penetration = INFINITY;
  v3 best_axis = zero();
  bool has_axis = false;

  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, box.axis[0], offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, box.axis[1], offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, box.axis[2], offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, cylinder_axis, offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, cross(cylinder_axis, box.axis[0]), offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, cross(cylinder_axis, box.axis[1]), offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, cross(cylinder_axis, box.axis[2]), offset, &penetration, &best_axis, &has_axis)) return 0;

  v3 radial_axis = sub(point_on_box, add(cylinder_center, scale(cylinder_axis, dot(sub(point_on_box, cylinder_center), cylinder_axis))));
  if (!try_axis_box_cylinder(&box, cylinder_axis, cylinder_half_height, cylinder_radius, radial_axis, offset, &penetration, &best_axis, &has_axis)) return 0;

  if (!has_axis)
    return 0;

  v3 normal;
  float depth;
  if (distance > 0.0001f) {
    normal = scale(delta, 1.0f / distance);
    depth = 0.0f;
  } else {
    normal = best_axis;
    if (dot(normal, offset) < 0.0f)
      normal = scale(normal, -1.0f);

    depth = penetration;
  }

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->point = add(point_on_box, scale(sub(point_on_cylinder, point_on_box), 0.5f));
  contact->normal = normal;
  contact->depth = depth;

  return 1;
}

count_t cylinder_cylinder_collision(physics_world *world, const collision_detection_context *ctx) {
  quat rotation_a = qmul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  quat rotation_b = qmul(ctx->data_b->rotations[ctx->body_b], ctx->shape_b.rotation);

  v3 center_a = body_a_center(ctx);
  v3 center_b = body_b_center(ctx);

  v3 axis_a = rotate(up(), rotation_a);
  v3 axis_b = rotate(up(), rotation_b);

  float radius_a = ctx->shape_a.cylinder.radius;
  float radius_b = ctx->shape_b.cylinder.radius;
  float half_height_a = ctx->shape_a.cylinder.height * 0.5f;
  float half_height_b = ctx->shape_b.cylinder.height * 0.5f;

  v3 offset = sub(center_a, center_b);
  v3 segment_start_a = sub(center_a, scale(axis_a, half_height_a));
  v3 segment_end_a = add(center_a, scale(axis_a, half_height_a));
  v3 segment_start_b = sub(center_b, scale(axis_b, half_height_b));
  v3 segment_end_b = add(center_b, scale(axis_b, half_height_b));
  v3 axis_point_a;
  v3 axis_point_b;
  closest_points_on_segment_segment(segment_start_a, segment_end_a, segment_start_b, segment_end_b, &axis_point_a, &axis_point_b);

  v3 axis_delta = sub(axis_point_a, axis_point_b);
  v3 radial_offset_a = sub(offset, scale(axis_a, dot(offset, axis_a)));
  v3 radial_offset_b = sub(offset, scale(axis_b, dot(offset, axis_b)));

  float penetration = INFINITY;
  v3 best_axis = zero();
  bool has_axis = false;

  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, axis_a, offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, axis_b, offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, cross(axis_a, axis_b), offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, axis_delta, offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, radial_offset_a, offset, &penetration, &best_axis, &has_axis)) return 0;
  if (!try_axis_cylinder_cylinder(axis_a, half_height_a, radius_a, axis_b, half_height_b, radius_b, radial_offset_b, offset, &penetration, &best_axis, &has_axis)) return 0;

  if (!has_axis)
    return 0;

  v3 normal = best_axis;
  if (dot(normal, offset) < 0.0f)
    normal = scale(normal, -1.0f);

  v3 point_on_a = support_point_on_cylinder(center_a, axis_a, half_height_a, radius_a, scale(normal, -1.0f));
  v3 point_on_b = support_point_on_cylinder(center_b, axis_b, half_height_b, radius_b, normal);

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->point = add(point_on_b, scale(sub(point_on_a, point_on_b), 0.5f));
  contact->normal = normal;
  contact->depth = penetration;

  return 1;
}

count_t cylinder_sphere_collision(physics_world *world, const collision_detection_context *ctx) {
  quat cylinder_rotation = qmul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  float cylinder_radius = ctx->shape_a.cylinder.radius;
  float cylinder_half_height = ctx->shape_a.cylinder.height * 0.5f;

  v3 sphere_center = body_b_center(ctx);
  float sphere_radius = ctx->shape_b.sphere.radius;

  m4 cylinder_transform = body_a_transform(ctx);
  v3 local_sphere_center = transform(sphere_center, inverse(cylinder_transform));

  float clamped_y = local_sphere_center.y;
  if (clamped_y > cylinder_half_height) clamped_y = cylinder_half_height;
  else if (clamped_y < -cylinder_half_height) clamped_y = -cylinder_half_height;

  v3 radial = { local_sphere_center.x, 0, local_sphere_center.z };
  float radial_len_sq = lensq(radial);
  float radial_len = sqrtf(radial_len_sq);

  v3 clamped_radial = radial;
  if (radial_len_sq > cylinder_radius * cylinder_radius)
    clamped_radial = scale(radial, cylinder_radius / radial_len);

  v3 closest_local = { clamped_radial.x, clamped_y, clamped_radial.z };
  v3 local_to_cylinder = sub(closest_local, local_sphere_center);
  float distance = len(local_to_cylinder);

  bool inside = fabsf(local_sphere_center.y) <= cylinder_half_height && radial_len_sq <= cylinder_radius * cylinder_radius;
  if (!inside && distance > sphere_radius)
    return 0;

  v3 normal_local;
  v3 contact_point_local;
  float depth;

  const float eps = 0.000001f;
  if (inside) {
    float side_depth = cylinder_radius - radial_len;
    float cap_depth = cylinder_half_height - fabsf(local_sphere_center.y);

    if (side_depth < cap_depth) {
      v3 outward = radial_len > eps ? scale(radial, 1.0f / radial_len) : right();
      contact_point_local = (v3) { outward.x * cylinder_radius, local_sphere_center.y, outward.z * cylinder_radius };
      normal_local = scale(outward, -1.0f);
      depth = sphere_radius + side_depth;
    } else {
      float sign = local_sphere_center.y >= 0.0f ? 1.0f : -1.0f;
      contact_point_local = (v3) { local_sphere_center.x, sign * cylinder_half_height, local_sphere_center.z };
      normal_local = (v3) { 0, -sign, 0 };
      depth = sphere_radius + cap_depth;
    }
  } else {
    contact_point_local = closest_local;
    if (distance > eps)
      normal_local = scale(local_to_cylinder, 1.0f / distance);
    else
      normal_local = (v3) { 0, local_sphere_center.y > 0.0f ? -1.0f : 1.0f, 0 };

    depth = sphere_radius - distance;
  }

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->point = transform(contact_point_local, cylinder_transform);
  contact->normal = normalize(rotate(normal_local, cylinder_rotation));
  contact->depth = depth;

  return 1;
}

count_t sphere_sphere_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 p1 = body_a_center(ctx);
  v3 p2 = body_b_center(ctx);

  float r1 = ctx->shape_a.sphere.radius;
  float r2 = ctx->shape_b.sphere.radius;

  v3 offset = sub(p1, p2);
  float distance = len(offset);
  float radii = r1 + r2;
  if (distance > radii) {
    return 0;
  }

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

  contact *contact = new_contact(world, ctx);
  contact->point = add(p1, scale(offset, 0.5));
  contact->normal = normalize(offset);
  contact->depth = radii - distance;

  return 1;
}

static count_t sphere_plane_collision(physics_world *world, const collision_detection_context *ctx) {
  v3 sphere_center = body_a_center(ctx);
  float sphere_radius = ctx->shape_a.sphere.radius;

  v3 plane_point = ctx->data_b->positions[ctx->body_b];
  v3 plane_normal = ctx->shape_b.plane.normal;

  float plane_sphere_distance = dot(sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius)
    return 0;

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

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
  float min_distance = center_distance - cylinder_half_height * fabsf(axis_projection) - cylinder_radius * radial_projection;
  if (min_distance > 0.0f)
    return 0;

  collisions *collisions = &world->collisions;
  ARRAY_RESIZE_IF_NEEDED(collisions->contacts, collisions->count + 1, collisions->capacity, contact);

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

void collisions_reset(physics_world *world) {
  world->collisions.count = 0;
  world->collisions.dynamic_contacts_count = 0;
}

collisions collisions_init(const physics_config *config) {
  collisions result = {};
  result.capacity = config->collisions_capacity;
  result.count = 0;
  result.contacts = malloc(config->collisions_capacity * sizeof(contact));

  return result;
}

static void invert_static_collision(physics_world *world, collision_func func, const collision_detection_context *ctx) {
  collisions *collisions = &world->collisions;
  if (func(world, ctx)) {
    contact *contact = &collisions->contacts[collisions->count - 1];
    contact->index_a = ctx->body_b;
    contact->index_b = ctx->body_a;
    contact->normal = negate(contact->normal);
  }
}

void collisions_detect(physics_world *world) {
  PROFILE_FUNCTION

  collisions *collisions = &world->collisions;

  const common_data *dynamics =(common_data*) &world->dynamics;
  const common_data *statics = (common_data*)&world->statics;

  collision_detection_context ctx = { .world = world };
  ctx.data_a = dynamics;
  ctx.data_b = dynamics;

  collision_detection_context inv_ctx = ctx;

  count_t dyn_count = collisions->dynamic_contacts_count;
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

          switch(shape_a.type) {
            case SHAPE_BOX:
              switch(shape_b.type) {
                case SHAPE_BOX:
                  bool did_switch_bodies;
                  dyn_count += box_box_collision(world, &ctx, &did_switch_bodies);
                  if (did_switch_bodies) {
                    contact *contact = &collisions->contacts[collisions->count - 1];
                    contact->index_a = j;
                    contact->index_b = i;
                  }
                  break;

                case SHAPE_SPHERE:
                  dyn_count += box_sphere_collision(world, &ctx);
                  break;

                default:
                  break;
              }
              break;

            case SHAPE_SPHERE:
              switch(shape_b.type) {
                case SHAPE_BOX:
                  dyn_count += box_sphere_collision(world, &inv_ctx);
                  break;

                case SHAPE_SPHERE:
                  dyn_count += sphere_sphere_collision(world, &ctx);
                  break;

                default:
                  break;
              }
              break;

            case SHAPE_CYLINDER:
              switch (shape_b.type) {
                case SHAPE_SPHERE:
                  dyn_count += cylinder_sphere_collision(world, &ctx);
                  break;

                case SHAPE_BOX:
                  dyn_count += cylinder_box_collision(world, &ctx);
                  break;

                case SHAPE_CYLINDER:
                  dyn_count += cylinder_cylinder_collision(world, &ctx);
                  break;

                default:
                  break;
              }
              break;

            default:
              break;
          }
        }
      }
    }
  }

  collisions->dynamic_contacts_count = dyn_count;

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

          switch(shape_b.type) {
            case SHAPE_PLANE:
              switch(shape_a.type) {
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

            case SHAPE_BOX:
              switch(shape_a.type) {
                case SHAPE_BOX:
                  bool did_switch_bodies;
                  box_box_collision(world, &ctx, &did_switch_bodies);
                  if (did_switch_bodies) {
                    contact *contact = &collisions->contacts[collisions->count - 1];
                    contact->normal = negate(contact->normal);
                  }
                  break;

                case SHAPE_SPHERE:
                  invert_static_collision(world, &box_sphere_collision, &inv_ctx);
                  break;

                default:
                  break;
              }
              break;

            case SHAPE_CYLINDER:
              switch (shape_a.type) {
                case SHAPE_SPHERE:
                  invert_static_collision(world, &cylinder_sphere_collision, &inv_ctx);
                  break;

                case SHAPE_BOX:
                  invert_static_collision(world, &cylinder_box_collision, &inv_ctx);
                  break;

                default:
                  break;
              }

            default:
              break;
          }
        }
      }
    }
  }
}

void collisions_teardown(collisions collisions) {
  free(collisions.contacts);
}
