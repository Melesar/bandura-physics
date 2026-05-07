#include "bandura.h"
#include "bnd-core.h"
#include "profiler.h"

#include <float.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

extern count_t max_body_index;

static void swap_bodies(bnd_world *world, bnd_body_type type, count_t index_a, count_t index_b);
static void move_body(bnd_world *world, count_t src_index, count_t dst_index);

static void notify_body_removed(bnd_body_handle handle) {
  raise_error_debug(BND_ERROR_BODY_REMOVED, NULL, "Body %d (%s) has been removed", handle.index, handle.type == BND_DYNAMIC ? "dynamic" : "static");
}

static v3 cylinder_inertia(float radius, float height, float mass) {
  float principal = mass * (3 * radius * radius + height * height) / 12.0;
  return (v3){ principal, mass * radius * radius / 2.0, principal };
}

static v3 sphere_inertia(float radius, float mass) {
  float s = 2.0 * mass * radius * radius / 5.0;
  return scale(one(), s);
}

static v3 box_inertia(v3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  v3 i = { yy + zz, xx + zz, xx + yy };
  return scale(i, m);
}

static m3 mesh_inertia(const bnd_world *world, bnd_mesh_handle handle, float mass) {
  m3 base_inertia = world->meshes.inertias[handle];
  float scale = mass / world->meshes.volumes[handle];

  return matrix_scale(base_inertia, scale);
}

static m3 inertia_matrix(const bnd_world *world, bnd_body_shape shape, float mass) {
  switch (shape.type) {
    case BND_BOX:
      return matrix_initial_inertia(box_inertia(shape.box.size, mass));

    case BND_SPHERE:
      return matrix_initial_inertia(sphere_inertia(shape.sphere.radius, mass));

    case BND_CYLINDER:
      return matrix_initial_inertia(cylinder_inertia(shape.cylinder.radius, shape.cylinder.height, mass));

    case BND_MESH:
      return mesh_inertia(world, shape.mesh, mass);

    default:
      return matrix_initial_inertia(one());
  }
}

static v3 rotated_box_half_extents(m3 rotation_matrix, v3 local_half_extends) {
  v3 half_extents;
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
  v3 position = data->positions[index];
  quat rotation = data->rotations[index];
  body_shapes shapes_data = data->shapes[index];

  const bnd_body_shape *shapes = shapes_get(world, shapes_data);

  v3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
  v3 max = negate(min);
  for (count_t i = 0; i < shapes_data.count; ++i) {
    bnd_body_shape shape = shapes[i];

    quat shape_rotation = qmul(rotation, shape.rotation);
    v3 shape_center = add(position, rotate(shape.offset, rotation));

    m3 rotation_matrix;
    v3 shape_min, shape_max;
    v3 half_extents, axis;
    float half_height, radius;
    switch (shape.type) {
      case BND_BOX:
        rotation_matrix = quat_as_matrix(rotation);
        half_extents = rotated_box_half_extents(rotation_matrix, scale(shape.box.size, 0.5));
        break;

      case BND_SPHERE:
        half_extents = scale(one(), shape.sphere.radius);
        break;

      case BND_CYLINDER:
        half_height = shape.cylinder.height * 0.5;
        axis = rotate(up(), shape_rotation);
        radius = shape.cylinder.radius;

        float ax = fabsf(axis.x) * half_height;
        float ay = fabsf(axis.y) * half_height;
        float az = fabsf(axis.z) * half_height;

        float rx = radius * sqrtf(1.0f - axis.x * axis.x);
        float ry = radius * sqrtf(1.0f - axis.y * axis.y);
        float rz = radius * sqrtf(1.0f - axis.z * axis.z);

        half_extents = vec3(ax + rx, ay + ry, az + rz);
        break;

      case BND_MESH:
        rotation_matrix = quat_as_matrix(rotation);
        aabb local_aabb = world->meshes.aabbs[shape.mesh];
        half_extents = rotated_box_half_extents(rotation_matrix, local_aabb.half_extents);
        break;

      default:
        half_extents = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
        break;
    }

    shape_min = add(shape_center, negate(half_extents));
    shape_max = add(shape_center, half_extents);

    min = v3_min(min, shape_min);
    max = v3_max(max, shape_max);
  }

  data->aabbs[index] = (aabb) {
    .center = scale(add(min, max), 0.5),
    .half_extents = scale(sub(max, min), 0.5),
  };
}

static void clear_forces(bnd_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;

  const count_t size = sizeof(v3) * dynamics->count;
  memset(dynamics->forces, 0, size);
  memset(dynamics->torques, 0, size);
  memset(dynamics->impulses, 0, size);
  memset(dynamics->angular_impulses, 0, size);
  memset(dynamics->accelerations, 0, size);
}

static void update_awake_statuses(bnd_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (dynamics->count == 0) {
    return;
  }

  const float sleep_threshold = world->config.simulation.sleep_threshold;
  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    v3 angular_velocity = matrix_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = lensq(dynamics->velocities[i]) + lensq(angular_velocity);
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

    swap_bodies(world, BND_DYNAMIC, left, right);
  }

  for (count_t i = awake_count - 1; i >= left && i != (count_t)-1; --i) {
    if (dynamics->motion_avgs[i] >= sleep_threshold) {
      continue;
    }

    count_t target_index = awake_count - 1;
    if (i != target_index) {
      swap_bodies(world, BND_DYNAMIC, i, target_index);
    }

    dynamics->velocities[target_index] = dynamics->angular_momenta[target_index] = zero();
    awake_count -= 1;
  }

  for (count_t i = awake_count; i <= right; ++i) {
    if (dynamics->motion_avgs[i] < sleep_threshold)
      continue;

    count_t target_index = awake_count;
    if (i != target_index)
      swap_bodies(world, BND_DYNAMIC, i, target_index);

    awake_count += 1;
  }

  dynamics->awake_count = awake_count;
}

static void calculate_compound_shape_dynamic(const bnd_world *world, bnd_body_shape *shapes, float *masses, count_t count, float *total_mass, m3 *inertia) {
  *total_mass = 0;
  for (count_t i = 0; i < count; ++i) {
    *total_mass += masses[i];
  }

  v3 center_of_mass = zero();
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    center_of_mass = add(center_of_mass, scale(shape.offset, mass / *total_mass));
  }

  for (count_t i = 0; i < count; ++i) {
    shapes[i].offset = sub(shapes[i].offset, center_of_mass);
  }

  *inertia = (m3){ 0 };
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    m3 body_inertia = inertia_matrix(world, shape, mass);
    body_inertia = matrix_inertia(body_inertia, shape.rotation);
    body_inertia = matrix_displacement_inertia(body_inertia, shape.offset, mass);

    *inertia = matrix_add(*inertia, body_inertia);
  }
}

common_data *as_common(bnd_world *world, bnd_body_type type) {
  switch (type) {
    case BND_DYNAMIC:
      return (common_data *)&world->dynamics;

    case BND_STATIC:
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
  return (bnd_body_handle){
    .type = type,
    .generation = data->generations[index],
    .index = data->inner_lookup[index],
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

static void realloc_commons(common_data *data) {
  data->capacity = data->capacity << 1;
  data->positions = realloc(data->positions, sizeof(v3) * data->capacity);
  data->rotations = realloc(data->rotations, sizeof(quat) * data->capacity);
  data->shapes = realloc(data->shapes, sizeof(body_shapes) * data->capacity);
  data->aabbs = realloc(data->aabbs, sizeof(aabb) * data->capacity);
  data->free_list = realloc(data->free_list, sizeof(count_t) * data->capacity);
  data->generations = realloc(data->generations, sizeof(uint8_t) * data->capacity);
  data->outer_lookup = realloc(data->outer_lookup, sizeof(outer_lookup_node) * data->capacity);
  data->inner_lookup = realloc(data->inner_lookup, sizeof(count_t) * data->capacity);
}

static void realloc_dynamics(dynamic_bodies *data) {
  data->forces = realloc(data->forces, sizeof(v3) * data->capacity);
  data->torques = realloc(data->torques, sizeof(v3) * data->capacity);
  data->impulses = realloc(data->impulses, sizeof(v3) * data->capacity);
  data->angular_impulses = realloc(data->angular_impulses, sizeof(v3) * data->capacity);
  data->accelerations = realloc(data->accelerations, sizeof(v3) * data->capacity);

  data->inv_masses = realloc(data->inv_masses, sizeof(float) * data->capacity);
  data->velocities = realloc(data->velocities, sizeof(v3) * data->capacity);
  data->angular_momenta = realloc(data->angular_momenta, sizeof(v3) * data->capacity);
  data->inv_inertia_tensors = realloc(data->inv_inertia_tensors, sizeof(m3) * data->capacity);
  data->inv_intertias = realloc(data->inv_intertias, sizeof(m3) * data->capacity);
  data->motion_avgs = realloc(data->motion_avgs, sizeof(float) * data->capacity);
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
  data->positions[index] = zero();
  data->rotations[index] = qidentity();
  data->shapes[index] = shapes_write(world, bracket, shapes, shapes_count);

  calculate_aabb(world, data, index);
}

static void init_body_dynamic(bnd_world *world, float mass, m3 inertia_tensor, count_t index) {
  dynamic_bodies *data = &world->dynamics;

  data->inv_masses[index] = 1.0 / mass;
  data->velocities[index] = zero();
  data->angular_momenta[index] = zero();
  data->inv_inertia_tensors[index] = matrix_inverse(inertia_tensor);
  data->inv_intertias[index] = data->inv_inertia_tensors[index];
  data->motion_avgs[index] = 2 * world->config.simulation.sleep_threshold;
  data->forces[index] = zero();
  data->torques[index] = zero();
  data->impulses[index] = zero();
  data->angular_impulses[index] = zero();
  data->accelerations[index] = zero();
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

static bnd_body add_primitive_body_static(bnd_world *world, bnd_body_shape shape) {
  common_data *data = as_common(world, BND_STATIC);
  if (data->capacity < data->count + 1) {
    realloc_commons(data);
  }

  count_t index = data->count++;
  init_body_common(world, data, BRACKET_PRIMITIVE, &shape, 1, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return (bnd_body){ .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = NULL,
    .angular_momentum = NULL,
    .handle = make_body_handle(world, BND_STATIC, index) };
}

static bnd_body add_primitive_body_dynamic(bnd_world *world, bnd_body_shape shape, float mass) {
  dynamic_bodies *data = &world->dynamics;
  if (data->capacity < data->count + 1) {
    realloc_commons((common_data *)data);
    realloc_dynamics(data);
  }

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data *)data, BRACKET_PRIMITIVE, &shape, 1, index);

  m3 inertia = inertia_matrix(world, shape, mass);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return (bnd_body){
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = &data->velocities[index],
    .angular_momentum = &data->angular_momenta[index],
    .handle = make_body_handle(world, BND_DYNAMIC, index),
  };
}

void bnd_add_plane(bnd_world *world, v3 point, v3 normal) {
  bnd_body plane = add_primitive_body_static(world, (bnd_body_shape){ .type = BND_PLANE, .plane = { .normal = normal }, .offset = zero(), .rotation = qidentity() });
  world->statics.positions[handle_to_inner_index(world, plane.handle)] = point;
}

bnd_body bnd_add_box_dynamic(bnd_world *world, float mass, v3 size) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() }, mass);
}

bnd_body bnd_add_box_static(bnd_world *world, v3 size) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() });
}

bnd_body bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_SPHERE, .sphere = { .radius = radius }, .offset = zero(), .rotation = qidentity() }, mass);
}

bnd_body bnd_add_cylinder_static(bnd_world *world, float radius, float height) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() });
}

bnd_body bnd_add_cylinder_dynamic(bnd_world *world, float mass, float radius, float height) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() }, mass);
}

bnd_body bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  common_data *data = as_common(world, BND_STATIC);
  if (data->capacity < data->count + 1) {
    realloc_commons(data);
  }

  count_t index = data->count++;
  init_body_common(world, (common_data *)&world->statics, bracket, shapes, shapes_count, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return (bnd_body){ .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = NULL,
    .angular_momentum = NULL,
    .handle = make_body_handle(world, BND_STATIC, index) };
}

bnd_body bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  dynamic_bodies *data = &world->dynamics;
  if (data->capacity < data->count + 1) {
    realloc_commons((common_data *)data);
    realloc_dynamics(data);
  }

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data *)data, bracket, shapes, shapes_count, index);

  float mass;
  m3 inertia;
  bnd_body_shape *body_shapes = shapes_get(world, data->shapes[index]);
  calculate_compound_shape_dynamic(world, body_shapes, masses, shapes_count, &mass, &inertia);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return (bnd_body){ .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = &data->velocities[index],
    .angular_momentum = &data->angular_momenta[index],
    .handle = make_body_handle(world, BND_DYNAMIC, index) };
}

bnd_body bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_MESH, .mesh = mesh, .offset = zero(), .rotation = qidentity() }, mass);
}

bnd_body bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_MESH, .mesh = mesh, .offset = zero(), .rotation = qidentity() });
}

void bnd_remove_body(bnd_world *world, bnd_body_handle handle) {
  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  if (handle.generation != data->generations[index]) {
    return;
  }

  data->generations[index] += 1;
  data->free_list[data->free_count++] = handle.index; // We keep the outer index in the free list

  body_shapes shapes = data->shapes[index];
  shapes_clear_slot(world, shapes.bracket, shapes.offset);

  if (handle.type == BND_DYNAMIC) {
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
}

void bnd_apply_force(bnd_world *world, bnd_body_handle handle, v3 force) {
  if (handle.type != BND_DYNAMIC) {
    return;
  }

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return;
  }

  v3 prev_force = world->dynamics.forces[index];
  world->dynamics.forces[index] = add(prev_force, force);
}

void bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, v3 force, v3 position) {
  if (handle.type != BND_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return;
  }

  v3 prev_force = world->dynamics.forces[index];
  v3 prev_torque = world->dynamics.torques[index];

  v3 r = sub(position, world->dynamics.positions[index]);
  v3 torque = cross(r, force);

  world->dynamics.forces[index] = add(prev_force, force);
  world->dynamics.torques[index] = add(prev_torque, torque);
}

void bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, v3 impulse) {
  if (handle.type != BND_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return;
  }

  v3 prev_impulse = world->dynamics.impulses[index];
  world->dynamics.impulses[index] = add(prev_impulse, impulse);
}

void bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, v3 impulse, v3 position) {
  if (handle.type != BND_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.impulses[index];
  v3 prev_angular_impulse = world->dynamics.angular_impulses[index];

  v3 r = sub(position, world->dynamics.positions[index]);
  v3 angular_impulse = cross(r, impulse);

  world->dynamics.impulses[index] = add(prev_force, impulse);
  world->dynamics.angular_impulses[index] = add(prev_angular_impulse, angular_impulse);
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

v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != data->generations[index]) {
    notify_body_removed(handle);
    return zero();
  }

  return data->positions[index];
}

quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != data->generations[index]) {
    notify_body_removed(handle);
    return qidentity();
  }

  return data->rotations[index];
}

bnd_body_shape *bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, count_t *count) {
  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != data->generations[index]) {
    notify_body_removed(handle);
    return NULL;
  }

  body_shapes shapes = data->shapes[index];
  *count = shapes.count;
  return shapes_get(world, shapes);
}

aabb bnd_get_bounding_box(const bnd_world *world, bnd_body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  if (data->generations[index] != handle.generation) {
    notify_body_removed(handle);
    return (aabb){ 0 };
  }

  return data->aabbs[index];
}

v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return zero();
  }

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return zero();
  }

  return world->dynamics.velocities[index];
}

v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return zero();
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != dynamics->generations[index]) {
    notify_body_removed(handle);
    return zero();
  }

  v3 momentum = dynamics->angular_momenta[index];
  quat rotation = dynamics->rotations[index];
  m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return matrix_rotate(momentum, matrix_inertia(inv_inertia, rotation));
}

v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return zero();
  }

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return zero();
  }

  return world->dynamics.angular_momenta[index];
}

m3 bnd_get_inertia(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return (m3){ 0 };
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != dynamics->generations[index]) {
    notify_body_removed(handle);
    return (m3){ 0 };
  }

  quat rotation = dynamics->rotations[index];
  m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return matrix_inverse(matrix_inertia(inv_inertia, rotation));
}

m3 bnd_get_base_inertia(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return (m3){ 0 };
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != dynamics->generations[index]) {
    notify_body_removed(handle);
    return (m3){ 0 };
  }

  m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return matrix_inverse(inv_inertia);
}

float bnd_get_motion_avg(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC) {
    return 0;
  }

  count_t index = handle_to_inner_index(world, handle);
  if (handle.generation != world->dynamics.generations[index]) {
    notify_body_removed(handle);
    return 0;
  }

  return world->dynamics.motion_avgs[index];
}

count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts) {
  count_t count = world->contacts.count < max_contacts ? world->contacts.count : max_contacts;
  for (count_t i = 0; i < count; ++i) {
    contact full_contact = world->contacts.values[i];
    bnd_body_type type = i < world->contacts.dynamic_count ? BND_DYNAMIC : BND_STATIC;

    contacts[i] = (bnd_contact){
      .point = full_contact.point,
      .normal = full_contact.normal,
      .depth = full_contact.depth,
      .body_a = make_body_handle(world, BND_DYNAMIC, full_contact.index_a),
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
    enumerator->handle.generation = data->generations[data->outer_lookup[enumerator->handle.index].index];
    return true;
  }

  outer_lookup_node node = data->outer_lookup[enumerator->handle.index];
  if (node.next == max_body_index) {
    return false;
  }

  enumerator->handle.index = node.next;
  enumerator->handle.generation = data->generations[data->outer_lookup[enumerator->handle.index].index];
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

  common_data *statics = as_common(world, BND_STATIC);
  for (count_t i = 0; i < statics->count; ++i) {
    calculate_aabb(world, statics, i);
  }

  world->statics.dirty = false;
}

static void integrate_bodies(bnd_world *world, float dt) {
  PROFILE_FUNCTION

  v3 gravity_acc = world->config.simulation.gravity;
  float linear_damping = powf(world->config.simulation.linear_damping, dt);
  float angular_damping = powf(world->config.simulation.angular_damping, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    float inv_mass = dynamics->inv_masses[i];

    v3 acceleration = scale(dynamics->forces[i], inv_mass);
    acceleration = add(acceleration, gravity_acc);

    v3 impulse = scale(dynamics->impulses[i], inv_mass);

    v3 velocity = dynamics->velocities[i];
    velocity = add(velocity, scale(acceleration, dt));
    velocity = add(velocity, impulse);
    velocity = scale(velocity, linear_damping);

    quat rotation = dynamics->rotations[i];
    m3 base_inv_inertia = dynamics->inv_inertia_tensors[i];

    v3 momentum_delta = scale(dynamics->torques[i], dt);
    momentum_delta = add(momentum_delta, dynamics->angular_impulses[i]);

    v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = add(angular_momentum, momentum_delta);
    angular_momentum = scale(angular_momentum, angular_damping);

    rotation = integrate_rotation_midpoint(rotation, angular_momentum, base_inv_inertia, dt);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->inv_intertias[i] = matrix_inertia(base_inv_inertia, rotation);
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = add(dynamics->positions[i], scale(velocity, dt));
  }
}

void bnd_simulate(bnd_world *world, float dt) {
  profiler_start_frame();
  {
    PROFILE_FUNCTION

    world->stats.body_count = world->dynamics.count + world->statics.count;

    integrate_bodies(world, dt);
    update_aabbs(world);
    contacts_reset(world);
    contacts_generate(world);
    contacts_resolve(world, dt);
    update_awake_statuses(world, dt);
    clear_forces(world);
  }

  profiler_frame_metadata metadata = {
    .body_count = world->dynamics.count + world->statics.count,
    .contacts_count = world->contacts.count,
  };

  profiler_end_frame(metadata);
}

void bnd_awaken_body(bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count)
    return;

  if (handle.generation != dynamics->generations[index]) {
    notify_body_removed(handle);
    return;
  }

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index) {
    swap_bodies(world, BND_DYNAMIC, index, target_index);
  }

  dynamics->motion_avgs[target_index] = 2.0 * world->config.simulation.sleep_threshold;
  dynamics->awake_count += 1;
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

  SWAP_COMMON(v3, positions)
  SWAP_COMMON(quat, rotations)
  SWAP_COMMON(body_shapes, shapes)
  SWAP_COMMON(aabb, aabbs)
  SWAP_COMMON(uint8_t, generations);
  SWAP_COMMON(count_t, inner_lookup)

  if (type == BND_DYNAMIC) {
    SWAP_DYNAMIC(float, inv_masses)
    SWAP_DYNAMIC(v3, velocities)
    SWAP_DYNAMIC(v3, angular_momenta)
    SWAP_DYNAMIC(m3, inv_inertia_tensors)
    SWAP_DYNAMIC(m3, inv_intertias)
    SWAP_DYNAMIC(float, motion_avgs)

    SWAP_DYNAMIC(v3, forces)
    SWAP_DYNAMIC(v3, torques)
    SWAP_DYNAMIC(v3, impulses)
    SWAP_DYNAMIC(v3, angular_impulses)
    SWAP_DYNAMIC(v3, accelerations)
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
  data->generations[dst_index] = data->generations[src_index];
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
