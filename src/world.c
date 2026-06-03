#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <float.h>
#include <math.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>

#define INVALID_INDEX ((count_t)~0)
#define INVALID_HANDLE (bnd_body_handle) { 0, 0, INVALID_INDEX }

#define ASSERT_BODY_DYNAMIC(handle) \
  if (handle.type != BND_BODY_DYNAMIC) { \
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID , "Operation is not valid for static bodies" }; \
  }

#define REALLOCATE_IF_NEEDED(data, is_dynamic, allocator) \
  if ((data)->count + 1 > (data)->capacity) { \
    if (allocator.realloc == NULL) { \
      return (bnd_result_handle) { (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Bodies count exceeds capacity and Allocator.realloc is NULL. Increase capacity in bnd_config or provide a realloc function in the allocator" }, INVALID_HANDLE }; \
    } \
    \
    bnd_error e_realloc = realloc_data(data, allocator, is_dynamic); \
    if (e_realloc.type != BND_OK) { \
      return (bnd_result_handle) { e_realloc, INVALID_HANDLE }; \
    } \
  }

extern count_t max_body_index;

static void swap_bodies(bnd_world *world, bnd_body_type type, count_t index_a, count_t index_b);
static void move_body(bnd_world *world, count_t src_index, count_t dst_index);

static bnd_v3 capsule_inertia(float radius, float height, float mass) {
  float r2 = radius * radius;
  float r3 = r2 * radius;
  float h2 = height * height;

  const float pi = 3.14159265358979323846f;
  float mcy = r2 * height * pi;
  float mhs = 2.0 / 3 * r3 * pi;
  float m = mcy + mhs + mhs;
  float scale = mass / m;

  float side = mcy * (h2 / 12.0 + r2 / 4.0) + 2 * mhs * (2 * r2 / 5.0 + h2 / 2 + 3 * height * radius / 8.0);
  float prime = mcy * r2 / 2.0 + 2 * mhs * 2 * r2 / 5.0;
  return (bnd_v3) { scale * side, scale * prime, scale * side };
}

static bnd_v3 sphere_inertia(float radius, float mass) {
  float s = 2.0 * mass * radius * radius / 5.0;
  return bnd_v3_scale(bnd_v3_one(), s);
}

static bnd_v3 box_inertia(bnd_v3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  bnd_v3 i = { yy + zz, xx + zz, xx + yy };
  return bnd_v3_scale(i, m);
}

static bnd_m3 mesh_inertia(const bnd_world *world, bnd_mesh_handle handle, float mass) {
  bnd_m3 base_inertia = world->meshes.inertias[handle];
  float scale = mass / world->meshes.volumes[handle];

  return bnd_m3_scale(base_inertia, scale);
}

static bnd_m3 inertia_matrix(const bnd_world *world, bnd_body_shape shape, float mass) {
  switch (shape.type) {
    case BND_BOX:
      return bnd_m3_initial_inertia(box_inertia(shape.value.box.size, mass));

    case BND_SPHERE:
      return bnd_m3_initial_inertia(sphere_inertia(shape.value.sphere.radius, mass));

    case BND_CAPSULE:
      return bnd_m3_initial_inertia(capsule_inertia(shape.value.capsule.radius, shape.value.capsule.height, mass));

    case BND_MESH:
      return mesh_inertia(world, shape.value.mesh, mass);

    default:
      return bnd_m3_initial_inertia(bnd_v3_one());
  }
}

static bnd_v3 rotated_box_half_extents(bnd_m3 rotation_matrix, bnd_v3 local_half_extends) {
  bnd_v3 half_extents;
  half_extents.x =
    fabsf(rotation_matrix.m0[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m0[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m0[2]) * local_half_extends.z;

  half_extents.y =
    fabsf(rotation_matrix.m1[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m1[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m1[2]) * local_half_extends.z;

  half_extents.z =
    fabsf(rotation_matrix.m2[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m2[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m2[2]) * local_half_extends.z;

  return half_extents;
}

static void calculate_aabb(bnd_world *world, common_data *data, count_t index) {
  bnd_v3 position = data->positions[index];
  bnd_quat rotation = data->rotations[index];
  body_shapes shapes_data = data->shapes[index];

  const bnd_body_shape *shapes = shapes_get(world, shapes_data);

  bnd_v3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
  bnd_v3 max = bnd_v3_negate(min);
  for (count_t i = 0; i < shapes_data.count; ++i) {
    bnd_body_shape shape = shapes[i];

    bnd_quat shape_rotation = bnd_qmul(rotation, shape.rotation);
    bnd_v3 shape_center = bnd_v3_add(position, bnd_v3_rotate(shape.offset, rotation));

    bnd_m3 rotation_matrix;
    bnd_v3 shape_min, shape_max;
    bnd_v3 half_extents;
    bnd_aabb local_aabb;
    switch (shape.type) {
      case BND_BOX:
        rotation_matrix = quat_as_matrix(shape_rotation);
        half_extents = rotated_box_half_extents(rotation_matrix, bnd_v3_scale(shape.value.box.size, 0.5));
        break;

      case BND_SPHERE:
        half_extents = bnd_v3_scale(bnd_v3_one(), shape.value.sphere.radius);
        break;

      case BND_CAPSULE:
        rotation_matrix = quat_as_matrix(shape_rotation);
        local_aabb = (bnd_aabb) {
          .center = shape_center,
          .half_extents = { shape.value.capsule.radius, 0.5 * shape.value.capsule.height + shape.value.capsule.radius, shape.value.capsule.radius }
        };
        half_extents = rotated_box_half_extents(rotation_matrix, local_aabb.half_extents);
        break;

      case BND_MESH:
        rotation_matrix = quat_as_matrix(shape_rotation);
        local_aabb = world->meshes.aabbs[shape.value.mesh];
        half_extents = rotated_box_half_extents(rotation_matrix, local_aabb.half_extents);
        break;

      default:
        half_extents = (bnd_v3){FLT_MAX, FLT_MAX, FLT_MAX};
        break;
    }

    shape_min = bnd_v3_add(shape_center, bnd_v3_negate(half_extents));
    shape_max = bnd_v3_add(shape_center, half_extents);

    min = bnd_v3_min(min, shape_min);
    max = bnd_v3_max(max, shape_max);
  }

  data->aabbs[index] = (bnd_aabb) {
    .center = bnd_v3_scale(bnd_v3_add(min, max), 0.5),
    .half_extents = bnd_v3_scale(bnd_v3_sub(max, min), 0.5),
  };
}

static void clear_forces(bnd_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;

  const count_t size = sizeof(bnd_v3) * dynamics->count;
  memset(dynamics->forces, 0, size);
  memset(dynamics->torques, 0, size);
  memset(dynamics->impulses, 0, size);
  memset(dynamics->angular_impulses, 0, size);
  memset(dynamics->accelerations, 0, size);
}

static void awaken_body(bnd_world *world, count_t index) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count) {
    return;
  }

  dynamics->motion_avgs[index] = 2.0 * world->config.simulation.sleep_threshold;

  swap_bodies(world, BND_BODY_DYNAMIC, index, dynamics->awake_count);
  dynamics->awake_count += 1;
}

static void update_awake_statuses(bnd_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (dynamics->count == 0) {
    return;
  }

  const float sleep_threshold = world->config.simulation.sleep_threshold;
  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    bnd_v3 angular_velocity = bnd_m3_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = bnd_v3_lensqr(dynamics->velocities[i]) + bnd_v3_lensqr(angular_velocity);
    float bias = powf(world->config.simulation.sleep_base_bias, dt);

    float motion = current_motion * bias + new_motion * (1 - bias);
    motion = fminf(motion, 10 * sleep_threshold);

    dynamics->motion_avgs[i] = motion;
  }

  count_t left = 0;
  count_t right = dynamics->count - 1;
  while (left < awake_count && right >= awake_count) {
    while (dynamics->motion_avgs[left] > sleep_threshold) {
      left += 1;
    }

    while (dynamics->motion_avgs[right] <= sleep_threshold && right >= awake_count) {
      right -= 1;
    }

    if (left >= awake_count || right <= awake_count - 1) {
      break;
    }

    swap_bodies(world, BND_BODY_DYNAMIC, left, right);
  }

  for (count_t i = awake_count - 1; i >= left && i != (count_t)-1; --i) {
    if (dynamics->motion_avgs[i] >= sleep_threshold) {
      continue;
    }

    count_t target_index = awake_count - 1;
    if (i != target_index) {
      swap_bodies(world, BND_BODY_DYNAMIC, i, target_index);
    }

    dynamics->velocities[target_index] = dynamics->angular_momenta[target_index] = bnd_v3_zero();
    awake_count -= 1;
  }

  for (count_t i = awake_count; i <= right; ++i) {
    if (dynamics->motion_avgs[i] < sleep_threshold)
      continue;

    count_t target_index = awake_count;
    if (i != target_index)
      swap_bodies(world, BND_BODY_DYNAMIC, i, target_index);

    awake_count += 1;
  }

  dynamics->awake_count = awake_count;
}

static void calculate_compound_shape_dynamic(const bnd_world *world, bnd_body_shape *shapes, float *masses, count_t count, float *total_mass, bnd_m3 *inertia) {
  *total_mass = 0;
  for (count_t i = 0; i < count; ++i) {
    *total_mass += masses[i];
  }

  bnd_v3 center_of_mass = bnd_v3_zero();
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    center_of_mass = bnd_v3_add(center_of_mass, bnd_v3_scale(shape.offset, mass / *total_mass));
  }

  for (count_t i = 0; i < count; ++i) {
    shapes[i].offset = bnd_v3_sub(shapes[i].offset, center_of_mass);
  }

  *inertia = (bnd_m3){ 0 };
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    bnd_m3 body_inertia = inertia_matrix(world, shape, mass);
    body_inertia = bnd_m3_inertia(body_inertia, shape.rotation);
    body_inertia = bnd_m3_displacement_inertia(body_inertia, shape.offset, mass);

    *inertia = bnd_m3_add(*inertia, body_inertia);
  }
}

common_data *as_common(bnd_world *world, bnd_body_type type) {
  switch (type) {
    case BND_BODY_DYNAMIC:
      return (common_data *)&world->dynamics;

    case BND_BODY_STATIC:
      return (common_data *)&world->statics;

    default:
      return NULL;
  }
}

const common_data *as_common_const(const bnd_world *world, bnd_body_type type) {
  return as_common((bnd_world *)world, type);
}

bnd_body_handle make_body_handle(const bnd_world *world, bnd_body_type type, count_t index) {
  const common_data *data = as_common_const(world, type);
  count_t outer_index = data->inner_lookup[index];

  return (bnd_body_handle){
    .type = type,
    .index = outer_index,
    .generation = data->generations[outer_index],
  };
}

count_t handle_to_inner_index(const bnd_world *world, bnd_body_handle handle) {
  return as_common_const(world, handle.type)->outer_lookup[handle.index].index;
}

void new_outer_lookup(common_data *data, outer_lookup_node *target_node, count_t index, count_t value) {
  if (data->first_outer_node == max_body_index) {
    data->first_outer_node = index;

    target_node->index = value;
    target_node->prev = max_body_index;
    target_node->next = max_body_index;
    return;
  }

  count_t current_index = data->first_outer_node;
  outer_lookup_node *node = &data->outer_lookup[current_index];
  while (node->next < index) {
    current_index = node->next;
    node = &data->outer_lookup[node->next];
  }

  if (node->next == max_body_index) {
    target_node->index = value;
    target_node->prev = current_index;
    target_node->next = max_body_index;

    node->next = index;
    return;
  }

  count_t next = node->next;
  data->outer_lookup[next].prev = index;
  node->next = index;

  target_node->index = value;
  target_node->prev = current_index;
  target_node->next = next;
}

static bnd_error realloc_data(common_data *data, bnd_allocator allocator, bool with_dynamics) {
  count_t old_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;
  while (data->count >= data->capacity) {
    data->capacity = data->capacity << 1;
  }

  count_t total_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;
  REALLOC_BUFFER4(data->positions, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->rotations, allocator, sizeof(bnd_quat), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->shapes, allocator, sizeof(body_shapes), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->aabbs, allocator, sizeof(bnd_aabb), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->event_masks, allocator, sizeof(bnd_event_type), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->event_links, allocator, sizeof(event_link), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->free_list, allocator, sizeof(count_t), old_capacity, total_capacity);
  REALLOC_BUFFER1(data->generations, allocator, sizeof(uint8_t), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->outer_lookup, allocator, sizeof(outer_lookup_node), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->inner_lookup, allocator, sizeof(count_t), old_capacity, total_capacity);

  if (with_dynamics) {
    dynamic_bodies *dynamics = (dynamic_bodies *) data;
    REALLOC_BUFFER4(dynamics->forces, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->torques, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->impulses, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->angular_impulses, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->accelerations, allocator, sizeof(bnd_v3), old_capacity, total_capacity);

    REALLOC_BUFFER4(dynamics->inv_masses, allocator, sizeof(float), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->velocities, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->angular_momenta, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->inv_inertia_tensors, allocator, sizeof(bnd_m3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->inv_intertias, allocator, sizeof(bnd_m3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->motion_avgs, allocator, sizeof(float), old_capacity, total_capacity);
  }

  return OK;
}

static shape_dimension_bracket get_shapes_bracket(count_t shapes_count) {
  assert(shapes_count <= (1 << (BRACKET_COUNT - 1)));

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t bracket_capacity = 1 << i;
    if (shapes_count <= bracket_capacity) {
      return i;
    }
  }

  return BRACKET_COUNT;
}

static void init_body_common(bnd_world *world, common_data *data, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t index) {
  data->positions[index] = bnd_v3_zero();
  data->rotations[index] = bnd_qidentity();
  data->shapes[index] = shapes_write(world, bracket, shapes, shapes_count);
  data->event_masks[index] = 0;
  data->event_links[index] = (event_link) { 0 };

  calculate_aabb(world, data, index);
}

static void init_body_dynamic(bnd_world *world, float mass, bnd_m3 inertia_tensor, count_t index) {
  dynamic_bodies *data = &world->dynamics;

  data->inv_masses[index] = 1.0 / mass;
  data->velocities[index] = bnd_v3_zero();
  data->angular_momenta[index] = bnd_v3_zero();
  data->inv_inertia_tensors[index] = bnd_m3_inverse(inertia_tensor);
  data->inv_intertias[index] = data->inv_inertia_tensors[index];
  data->motion_avgs[index] = 2 * world->config.simulation.sleep_threshold;
  data->forces[index] = bnd_v3_zero();
  data->torques[index] = bnd_v3_zero();
  data->impulses[index] = bnd_v3_zero();
  data->angular_impulses[index] = bnd_v3_zero();
  data->accelerations[index] = bnd_v3_zero();
}

static count_t insert_new_dynamic_body(bnd_world *world) {
  dynamic_bodies *data = &world->dynamics;

  count_t prev_count = data->count;
  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : prev_count;

  count_t index;
  if (data->awake_count < prev_count) {
    move_body(world, data->awake_count, prev_count);

    index = data->awake_count;

    count_t prev_outer_index = data->inner_lookup[index];
    data->outer_lookup[prev_outer_index].index = prev_count;
    data->inner_lookup[prev_count] = prev_outer_index;

    new_outer_lookup((common_data *)data, &data->outer_lookup[outer_index], outer_index, index);
    data->inner_lookup[index] = outer_index;
  } else {
    index = prev_count;
    new_outer_lookup((common_data *)data, &data->outer_lookup[outer_index], outer_index, index);
    data->inner_lookup[index] = outer_index;
  }

  return index;
}

static bnd_result_handle add_primitive_body_static(bnd_world *world, bnd_body_shape shape) {
  common_data *data = as_common(world, BND_BODY_STATIC);
  REALLOCATE_IF_NEEDED(data, false, world->allocator)

  count_t index = data->count++;
  init_body_common(world, data, BRACKET_PRIMITIVE, &shape, 1, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_STATIC, index));
}

static bnd_result_handle add_primitive_body_dynamic(bnd_world *world, bnd_body_shape shape, float mass) {
  dynamic_bodies *data = &world->dynamics;
  REALLOCATE_IF_NEEDED((common_data *)data, true, world->allocator)

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data *)data, BRACKET_PRIMITIVE, &shape, 1, index);

  bnd_m3 inertia = inertia_matrix(world, shape, mass);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_DYNAMIC, index));
}

bnd_error bnd_add_plane(bnd_world *world, bnd_v3 point, bnd_v3 normal) {
  bnd_result_handle plane = add_primitive_body_static(world, (bnd_body_shape){ .type = BND_PLANE, .value = {.plane = { .normal = normal } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() });
  if (plane.error.type != BND_OK) {
    world->statics.positions[handle_to_inner_index(world, plane.value)] = point;
  }

  return plane.error;
}

bnd_result_handle bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_BOX, .value = {.box = { .size = size } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() }, mass);
}

bnd_result_handle bnd_add_box_static(bnd_world *world, bnd_v3 size) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_BOX, .value = {.box = { .size = size } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() });
}

bnd_result_handle bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_SPHERE, .value = {.sphere = { .radius = radius } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() }, mass);
}

bnd_result_handle bnd_add_sphere_static(bnd_world *world, float radius) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_SPHERE, .value = {.sphere = { .radius = radius } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() });
}

bnd_result_handle bnd_add_capsule_static(bnd_world *world, float radius, float height) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_CAPSULE, .value = {.capsule = { .radius = radius, .height = height } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() });
}

bnd_result_handle bnd_add_capsule_dynamic(bnd_world *world, float mass, float radius, float height) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_CAPSULE, .value = {.capsule = { .radius = radius, .height = height } }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() }, mass);
}

bnd_result_handle bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  common_data *data = as_common(world, BND_BODY_STATIC);
  REALLOCATE_IF_NEEDED(data, false, world->allocator)

  count_t index = data->count++;
  init_body_common(world, (common_data *)&world->statics, bracket, shapes, shapes_count, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_STATIC, index));
}

bnd_result_handle bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, count_t shapes_count) {
  dynamic_bodies *data = &world->dynamics;
  REALLOCATE_IF_NEEDED((common_data *) data, true, world->allocator)

  count_t index = insert_new_dynamic_body(world);
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);
  init_body_common(world, (common_data *)data, bracket, shapes, shapes_count, index);

  float mass;
  bnd_m3 inertia;
  bnd_body_shape *body_shapes = shapes_get(world, data->shapes[index]);
  calculate_compound_shape_dynamic(world, body_shapes, masses, shapes_count, &mass, &inertia);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_DYNAMIC, index));
}

bnd_result_handle bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_MESH, .value = {.mesh = mesh }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() }, mass);
}

bnd_result_handle bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_MESH, .value = {.mesh = mesh }, .offset = bnd_v3_zero(), .rotation = bnd_qidentity() });
}

bnd_error bnd_remove_body(bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  data->generations[handle.index] += 1;
  data->free_list[data->free_count++] = handle.index; // We keep the outer index in the free list

  count_t index = handle_to_inner_index(world, handle);
  body_shapes shapes = data->shapes[index];
  shapes_clear_slot(world, shapes.bracket, shapes.offset);

  if (handle.type == BND_BODY_DYNAMIC) {
    count_t body_count = data->count;
    count_t awake_count = world->dynamics.awake_count;

    if (index < awake_count) {
      world->dynamics.awake_count -= 1;
      swap_bodies(world, handle.type, index, awake_count - 1);

      if (awake_count < body_count) {
        swap_bodies(world, handle.type, awake_count - 1, body_count - 1);
      }
    } else {
      swap_bodies(world, handle.type, index, body_count - 1);
    }
  } else {
    swap_bodies(world, handle.type, index, data->count - 1);
  }

  data->count -= 1;

  outer_lookup_node *outer_node = &data->outer_lookup[handle.index];
  outer_node->index = max_body_index;

  if (data->first_outer_node == handle.index) {
    data->first_outer_node = outer_node->next;
  }

  if (outer_node->prev != max_body_index) {
    data->outer_lookup[outer_node->prev].next = outer_node->next;
  }

  if (outer_node->next != max_body_index) {
    data->outer_lookup[outer_node->next].prev = outer_node->prev;
  }

  world->generation += 1;

  return OK;
}

bnd_error bnd_apply_force(bnd_world *world, bnd_body_handle handle, bnd_v3 force) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.forces[index];
  world->dynamics.forces[index] = bnd_v3_add(prev_force, force);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, bnd_v3 force, bnd_v3 position) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.forces[index];
  bnd_v3 prev_torque = world->dynamics.torques[index];

  bnd_v3 r = bnd_v3_sub(position, world->dynamics.positions[index]);
  bnd_v3 torque = bnd_v3_cross(r, force);

  world->dynamics.forces[index] = bnd_v3_add(prev_force, force);
  world->dynamics.torques[index] = bnd_v3_add(prev_torque, torque);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_impulse = world->dynamics.impulses[index];
  world->dynamics.impulses[index] = bnd_v3_add(prev_impulse, impulse);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse, bnd_v3 position) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.impulses[index];
  bnd_v3 prev_angular_impulse = world->dynamics.angular_impulses[index];

  bnd_v3 r = bnd_v3_sub(position, world->dynamics.positions[index]);
  bnd_v3 angular_impulse = bnd_v3_cross(r, impulse);

  world->dynamics.impulses[index] = bnd_v3_add(prev_force, impulse);
  world->dynamics.angular_impulses[index] = bnd_v3_add(prev_angular_impulse, angular_impulse);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_handle_valid(const bnd_world *world, bnd_body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  if (handle.index == INVALID_INDEX) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle doesn't belong to an actual body" };
  }

  if (handle.generation != data->generations[handle.index]) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle refers to a body that has been removed" };
  }

  return OK;
}

bnd_config *bnd_edit_config(bnd_world *world) {
  return &world->config;
}

bnd_world_stats bnd_stats(const bnd_world *world) {
  return world->stats;
}

count_t bnd_body_count(const bnd_world *world, bnd_body_type type) {
  return as_common_const(world, type)->count;
}

count_t bnd_awake_count(const bnd_world *world) {
  return world->dynamics.awake_count;
}

count_t bnd_collisions_count(const bnd_world *world) {
  return world->contacts.count;
}

bnd_result_v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, data->positions[index]);
}

bnd_result_quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(quat, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(quat, data->rotations[index]);
}

bnd_result_u32 bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, bnd_body_shape *shapes, uint32_t max_shapes) {
  PROPAGATE_RESULT(u32, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  body_shapes body_shapes = data->shapes[index];
  bnd_body_shape *inner_shapes = shapes_get(world, body_shapes);

  count_t count = max_shapes >= body_shapes.count ? body_shapes.count : max_shapes;
  memcpy(shapes, inner_shapes, count * sizeof(bnd_body_shape));

  return BND_RESULT_OK(u32, count);
}

bnd_result_aabb bnd_get_bounding_box(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(aabb, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(aabb, data->aabbs[index]);
}

bnd_result_v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, world->dynamics.velocities[index]);
}

bnd_result_v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 momentum = dynamics->angular_momenta[index];
  bnd_quat rotation = dynamics->rotations[index];
  bnd_m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return BND_RESULT_OK(v3, bnd_m3_rotate(momentum, bnd_m3_inertia(inv_inertia, rotation)));
}

bnd_result_v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, world->dynamics.angular_momenta[index]);
}

bnd_error bnd_set_position(bnd_world *world, bnd_body_handle handle, bnd_v3 position) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  data->positions[index] = position;

  return OK;
}

bnd_error bnd_set_rotation(bnd_world *world, bnd_body_handle handle, bnd_quat rotation) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  data->rotations[index] = rotation;

  return OK;
}

bnd_error bnd_set_velocity(bnd_world *world, bnd_body_handle handle, bnd_v3 velocity) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  world->dynamics.velocities[index] = velocity;

  return OK;
}

bnd_error bnd_set_angular_momentum(bnd_world *world, bnd_body_handle handle, bnd_v3 angular_momentum) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  world->dynamics.angular_momenta[index] = angular_momentum;

  return OK;
}

count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts) {
  count_t count = world->contacts.count < max_contacts ? world->contacts.count : max_contacts;
  for (count_t i = 0; i < count; ++i) {
    contact full_contact = world->contacts.values[i];
    bnd_body_type type = i < world->contacts.dynamic_count ? BND_BODY_DYNAMIC : BND_BODY_STATIC;

    contacts[i] = (bnd_contact){
      .point = full_contact.point,
      .normal = full_contact.normal,
      .depth = full_contact.depth,
      .body_a = make_body_handle(world, BND_BODY_DYNAMIC, full_contact.index_a),
      .body_b = make_body_handle(world, type, full_contact.index_b)
    };
  }

  return count;
}


void bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator *enumerator) {
  enumerator->handle = (bnd_body_handle){ .type = type, .index = max_body_index & 0x7FFFFF };
  enumerator->generation = world->generation;
}

bool bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator) {
  if (enumerator->generation != world->generation) {
    return false;
  }

  const common_data *data = as_common_const(world, enumerator->handle.type);
  if (enumerator->handle.index == max_body_index) {
    if (data->count == 0) {
      return false;
    }

    enumerator->handle.index = data->first_outer_node;
    enumerator->handle.generation = data->generations[enumerator->handle.index];
    return true;
  }

  outer_lookup_node node = data->outer_lookup[enumerator->handle.index];
  if (node.next == max_body_index) {
    return false;
  }

  enumerator->handle.index = node.next;
  enumerator->handle.generation = data->generations[enumerator->handle.index];
  return true;
}

static void update_aabbs(bnd_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    calculate_aabb(world, (common_data *) dynamics, i);
  }

  if (!world->statics.dirty) {
    return;
  }

  common_data *statics = as_common(world, BND_BODY_STATIC);
  for (count_t i = 0; i < statics->count; ++i) {
    calculate_aabb(world, statics, i);
  }

  world->statics.dirty = false;
}

static void integrate_bodies(bnd_world *world, float dt) {
  PROFILE_FUNCTION

  bnd_v3 gravity_acc = world->config.simulation.gravity;
  float linear_damping = powf(world->config.simulation.linear_drag, dt);
  float angular_damping = powf(world->config.simulation.angular_drag, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    float inv_mass = dynamics->inv_masses[i];

    bnd_v3 acceleration = bnd_v3_scale(dynamics->forces[i], inv_mass);
    acceleration = bnd_v3_add(acceleration, gravity_acc);

    bnd_v3 impulse = bnd_v3_scale(dynamics->impulses[i], inv_mass);

    bnd_v3 velocity = dynamics->velocities[i];
    velocity = bnd_v3_add(velocity, bnd_v3_scale(acceleration, dt));
    velocity = bnd_v3_add(velocity, impulse);
    velocity = bnd_v3_scale(velocity, linear_damping);

    bnd_quat rotation = dynamics->rotations[i];
    bnd_m3 base_inv_inertia = dynamics->inv_inertia_tensors[i];

    bnd_v3 momentum_delta = bnd_v3_scale(dynamics->torques[i], dt);
    momentum_delta = bnd_v3_add(momentum_delta, dynamics->angular_impulses[i]);

    bnd_v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = bnd_v3_add(angular_momentum, momentum_delta);
    angular_momentum = bnd_v3_scale(angular_momentum, angular_damping);

    rotation = integrate_rotation_midpoint(rotation, angular_momentum, base_inv_inertia, dt);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->inv_intertias[i] = bnd_m3_inertia(base_inv_inertia, rotation);
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = bnd_v3_add(dynamics->positions[i], bnd_v3_scale(velocity, dt));
  }
}

void bnd_simulate(bnd_world *world, float dt) {
  PROFILER_START_FRAME;
  {
    PROFILE_FUNCTION

    world->stats.body_count = world->dynamics.count + world->statics.count;

    // TODO before changing the order of integration and collision detection,
    // revisit aabb generation.
    integrate_bodies(world, dt);
    update_aabbs(world);
    contacts_reset(world);
    events_reset(world);
    contacts_generate(world);
    contacts_resolve(world, dt);
    update_awake_statuses(world, dt);
    clear_forces(world);
  }

#ifdef BND_PROFILING
  profiler_frame_metadata metadata = {
    .body_count = world->dynamics.count + world->statics.count,
    .contacts_count = world->contacts.count,
  };
#endif

  PROFILER_END_FRAME(metadata);
}

bnd_error bnd_put_to_sleep(bnd_world *world, bnd_body_handle handle) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  if (index >= dynamics->awake_count) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "The body is already asleep" };
  }

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index) {
    swap_bodies(world, BND_BODY_DYNAMIC, index, target_index);
  }

  dynamics->awake_count -= 1;
  dynamics->motion_avgs[target_index] = 0;
  dynamics->velocities[target_index] = bnd_v3_zero();
  dynamics->angular_momenta[target_index] = bnd_v3_zero();

  return OK;
}

bnd_error bnd_awaken_body(bnd_world *world, bnd_body_handle handle) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "The body is already awake" };
  }

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index) {
    swap_bodies(world, BND_BODY_DYNAMIC, index, target_index);
  }

  dynamics->motion_avgs[target_index] = 2.0 * world->config.simulation.sleep_threshold;
  dynamics->awake_count += 1;

  return OK;
}

void bnd_reset_world(bnd_world *world) {
  world->dynamics.count = 0;
  world->dynamics.free_count = 0;
  world->dynamics.awake_count = 0;
  world->dynamics.first_outer_node = max_body_index;

  world->statics.count = 0;
  world->statics.free_count = 0;
  world->statics.first_outer_node = max_body_index;

  world->stats.incomplete_resolutions = 0;
  world->stats.incomplete_collision_detections = 0;

  contacts_reset(world);
  shapes_reset(world);
  joints_reset(world);
}

static void swap_bodies(bnd_world *world, bnd_body_type type, count_t index_a, count_t index_b) {
  common_data *data = as_common(world, type);

#define SWAP_COMMON(t, arr)                                                                                            \
  t tmp_##arr = data->arr[index_a];                                                                                    \
  data->arr[index_a] = data->arr[index_b];                                                                             \
  data->arr[index_b] = tmp_##arr;

#define SWAP_DYNAMIC(t, arr)                                                                                           \
  t tmp_##arr = world->dynamics.arr[index_a];                                                                          \
  world->dynamics.arr[index_a] = world->dynamics.arr[index_b];                                                         \
  world->dynamics.arr[index_b] = tmp_##arr;

  SWAP_COMMON(bnd_v3, positions)
  SWAP_COMMON(bnd_quat, rotations)
  SWAP_COMMON(body_shapes, shapes)
  SWAP_COMMON(bnd_aabb, aabbs)
  SWAP_COMMON(bnd_event_type, event_masks)
  SWAP_COMMON(event_link, event_links)
  SWAP_COMMON(count_t, inner_lookup)

  if (type == BND_BODY_DYNAMIC) {
    SWAP_DYNAMIC(float, inv_masses)
    SWAP_DYNAMIC(bnd_v3, velocities)
    SWAP_DYNAMIC(bnd_v3, angular_momenta)
    SWAP_DYNAMIC(bnd_m3, inv_inertia_tensors)
    SWAP_DYNAMIC(bnd_m3, inv_intertias)
    SWAP_DYNAMIC(float, motion_avgs)

    SWAP_DYNAMIC(bnd_v3, forces)
    SWAP_DYNAMIC(bnd_v3, torques)
    SWAP_DYNAMIC(bnd_v3, impulses)
    SWAP_DYNAMIC(bnd_v3, angular_impulses)
    SWAP_DYNAMIC(bnd_v3, accelerations)
  }

  data->outer_lookup[data->inner_lookup[index_b]].index = index_b;
  data->outer_lookup[data->inner_lookup[index_a]].index = index_a;

  world->generation += 1;

#undef SWAP_COMMON
#undef SWAP_DYNAMIC
}

static void move_body(bnd_world *world, count_t src_index, count_t dst_index) {
  dynamic_bodies *data = &world->dynamics;

  data->positions[dst_index] = data->positions[src_index];
  data->rotations[dst_index] = data->rotations[src_index];
  data->shapes[dst_index] = data->shapes[src_index];
  data->aabbs[dst_index] = data->aabbs[src_index];
  data->event_masks[dst_index] = data->event_masks[src_index];
  data->event_links[dst_index] = data->event_links[src_index];
  data->inv_masses[dst_index] = data->inv_masses[src_index];
  data->velocities[dst_index] = data->velocities[src_index];
  data->angular_momenta[dst_index] = data->angular_momenta[src_index];
  data->inv_inertia_tensors[dst_index] = data->inv_inertia_tensors[src_index];
  data->inv_intertias[dst_index] = data->inv_intertias[src_index];
  data->motion_avgs[dst_index] = data->motion_avgs[src_index];

  data->forces[dst_index] = data->forces[src_index];
  data->torques[dst_index] = data->torques[src_index];
  data->impulses[dst_index] = data->impulses[src_index];
  data->angular_impulses[dst_index] = data->angular_impulses[src_index];
  data->accelerations[dst_index] = data->accelerations[src_index];
}
