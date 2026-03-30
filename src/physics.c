#include "physics.h"
#include "bandura.h"
#include "profiler.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

static void swap_bodies(physics_world *world, count_t index_a, count_t index_b);
static void move_body(physics_world *world, count_t src_index, count_t dst_index);

static v3 cylinder_inertia(float radius, float height, float mass) {
  float principal =  mass * (3 * radius * radius + height * height) / 12.0;
  return (v3){ principal, mass * radius * radius / 2.0, principal };
}

static v3 sphere_inertia(float radius, float mass) {
  float scale = 2.0 * mass * radius * radius / 5.0;
  return scale(one(), scale);
}

static v3 box_inertia(v3 size, float mass) {
  float m = mass / 12;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  v3 i = { yy + zz, xx + zz, xx + yy };
  return scale(i, m);
}

static v3 inertia_vector(body_shape shape, float mass) {
  switch (shape.type) {
    case SHAPE_BOX:
      return box_inertia(shape.box.size, mass);

    case SHAPE_SPHERE:
      return sphere_inertia(shape.sphere.radius, mass);

    case SHAPE_CYLINDER:
      return cylinder_inertia(shape.cylinder.radius, shape.cylinder.height, mass);

    default:
      return one();
  }
}

static void clear_forces(physics_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;

  const count_t size = sizeof(v3) * dynamics->count;
  memset(dynamics->forces, 0, size);
  memset(dynamics->torques, 0, size);
  memset(dynamics->impulses, 0, size);
  memset(dynamics->angular_impulses, 0, size);
  memset(dynamics->accelerations, 0, size);
}

static void update_awake_statuses(physics_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (dynamics->count == 0)
    return;

  const float sleep_threshold = world->config.sleep_threshold;
  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    v3 angular_velocity = matrix_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = lensq(dynamics->velocities[i]) + lensq(angular_velocity);
    float bias = powf(world->config.sleep_base_bias, dt);

    float motion = current_motion * bias + new_motion * (1 - bias);
    motion = fminf(motion, 10 * sleep_threshold);

    dynamics->motion_avgs[i] = motion;
  }

  count_t left = 0;
  count_t right = dynamics->count - 1;
  while(left < awake_count && right >= awake_count) {
    while(dynamics->motion_avgs[left] > sleep_threshold) {
      left += 1;
    }

    while (dynamics->motion_avgs[right] <= sleep_threshold && right >= awake_count) {
      right -= 1;
    }

    if (left >= awake_count || right <= awake_count - 1)
      break;

    swap_bodies(world, left, right);
  }

  for (count_t i = awake_count - 1; i >= left && i != (count_t)-1; --i) {
    if (dynamics->motion_avgs[i] >= sleep_threshold)
      continue;

    count_t target_index = awake_count - 1;
    if (i != target_index)
      swap_bodies(world, i, target_index);

    dynamics->velocities[target_index] = dynamics->angular_momenta[target_index] = zero();
    awake_count -= 1;
  }

  for (count_t i = awake_count; i <= right; ++i) {
    if (dynamics->motion_avgs[i] < sleep_threshold)
      continue;

    count_t target_index = awake_count;
    if (i != target_index)
      swap_bodies(world, i, target_index);

    awake_count += 1;
  }

  dynamics->awake_count = awake_count;
}

static void calculate_compound_shape_static(body_shape *shapes, float *masses, count_t count, float *total_mass) {
  *total_mass = 0;
  for (count_t i = 0; i < count; ++i) {
    *total_mass += masses[i];
  }

  v3 center_of_mass = zero();
  for (count_t i = 0; i < count; ++i) {
    body_shape shape = shapes[i];
    float mass = masses[i];

    center_of_mass = add(center_of_mass, scale(shape.offset, mass / *total_mass));
  }

  for (count_t i = 0; i < count; ++i) {
    shapes[i].offset = sub(shapes[i].offset, center_of_mass);
  }
}

static void calculate_compound_shape_dynamic(body_shape *shapes, float *masses, count_t count, float *total_mass, m3 *inertia) {
  calculate_compound_shape_static(shapes, masses, count, total_mass);

  *inertia = (m3) { 0 };
  for (count_t i = 0; i < count; ++i) {
    body_shape shape = shapes[i];
    float mass = masses[i];

    m3 body_inertia = matrix_initial_inertia(inertia_vector(shape, mass));
    body_inertia = matrix_inertia(body_inertia, shape.rotation);
    body_inertia = matrix_displacement_inertia(body_inertia, shape.offset, mass);

    *inertia = matrix_add(*inertia, body_inertia);
  }
}

common_data* as_common(physics_world *world, body_type type) {
  switch (type) {
    case BODY_DYNAMIC:
      return (common_data*) &world->dynamics;

    case BODY_STATIC:
      return (common_data*) &world->statics;

    default:
      return NULL;
  }
}

const common_data* as_common_const(const physics_world *world, body_type type) {
  return as_common((physics_world*)world, type);
}

static void swap_bodies(physics_world *world, count_t index_a, count_t index_b);

physics_config physics_default_config() {
  return (physics_config) {
    .gravity = (v3) { 0, -9.81f, 0 },
    .dynamics_capacity = 32,
    .statics_capacity = 8,
    .contacts_capacity = 64,
    .joints_capacity = 64,
    .shapes_brackets_capacity = { 64, 1, 1, 1, 1 },
    .linear_damping = 0.95,
    .angular_damping = 0.8,
    .restitution = 0.2,
    .friction = 0.9,
    .max_penentration_iterations = 50,
    .max_velocity_iterations = 50,
    .sleep_base_bias = 0.5,
    .sleep_threshold = 0.3,
    .penetration_epsilon = 0.01,
    .velocity_epsilon = 0.01,
    .restitution_damping_limit = 0.25,
  };
}

body_handle make_body_handle(const physics_world *world, body_type type, count_t index) {
  return (body_handle) {
    .type = type,
    .index = type == BODY_DYNAMIC ? world->dynamics.inner_lookup[index] : index,
  };
}

count_t handle_to_inner_index(const physics_world *world, body_handle handle) {
  return handle.type == BODY_DYNAMIC ? world->dynamics.outer_lookup[handle.index] : handle.index;
}

static void init_commons(common_data *data, count_t capacity) {
  data->capacity = capacity;
  data->count = 0;
  data->positions = malloc(sizeof(v3) * capacity);
  data->rotations = malloc(sizeof(quat) * capacity);
  data->shapes = malloc(sizeof(body_shapes) * capacity);
  data->outer_lookup = malloc(sizeof(count_t) * capacity);
  data->inner_lookup = malloc(sizeof(count_t) * capacity);
}

static void realloc_commons(common_data *data) {
  data->capacity = data->capacity << 1;
  data->positions = realloc(data->positions, sizeof(v3) * data->capacity);
  data->rotations = realloc(data->rotations, sizeof(quat) * data->capacity);
  data->shapes = realloc(data->shapes, sizeof(body_shapes) * data->capacity);
  data->outer_lookup = realloc(data->outer_lookup, sizeof(count_t) * data->capacity);
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

static void teardown_commons(common_data *data) {
  free(data->positions);
  free(data->rotations);
  free(data->shapes);
  free(data->outer_lookup);
  free(data->inner_lookup);
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

static void init_body_common(physics_world *world, common_data *data, shape_dimension_bracket bracket, body_shape* shapes, count_t shapes_count, count_t index) {
  data->positions[index] = zero();
  data->rotations[index] = qidentity();
  data->shapes[index] = shapes_write(world, bracket, shapes, shapes_count);
}

static void init_body_dynamic(physics_world *world, float mass, m3 inertia_tensor, count_t index) {
  dynamic_bodies *data = &world->dynamics;

  data->inv_masses[index] = 1.0 / mass;
  data->velocities[index] = zero();
  data->angular_momenta[index] = zero();
  data->inv_inertia_tensors[index] = matrix_inverse(inertia_tensor);
  data->motion_avgs[index] = 2 * world->config.sleep_threshold;
  data->forces[index] = zero();
  data->torques[index] = zero();
  data->impulses[index] = zero();
  data->angular_impulses[index] = zero();
  data->accelerations[index] = zero();
}

static count_t insert_new_dynamic_body(physics_world *world) {
  dynamic_bodies *data = &world->dynamics;

  count_t prev_count = data->count;
  count_t index;
  if (data->awake_count < prev_count) {
    move_body(world, data->awake_count, prev_count);

    index = data->awake_count;

    count_t prev_outer_index = data->inner_lookup[index];
    data->outer_lookup[prev_outer_index] = prev_count;
    data->inner_lookup[prev_count] = prev_outer_index;

    data->outer_lookup[prev_count] = index;
    data->inner_lookup[index] = prev_count;
  } else {
    index = prev_count;
    data->outer_lookup[index] = index;
    data->inner_lookup[index] = index;
  }

  return index;
}

physics_world* physics_init(const physics_config *config) {
  physics_world* world = malloc(sizeof(physics_world));

  init_commons((common_data*) &world->dynamics, config->dynamics_capacity);
  init_commons((common_data*) &world->statics, config->statics_capacity);

  const count_t vectors = sizeof(v3) * config->dynamics_capacity;
  const count_t floats = sizeof(float) * config->dynamics_capacity;
  const count_t matrices = sizeof(m3) * config->dynamics_capacity;

  world->dynamics.forces = malloc(vectors);
  world->dynamics.torques = malloc(vectors);
  world->dynamics.impulses = malloc(vectors);
  world->dynamics.angular_impulses = malloc(vectors);
  world->dynamics.accelerations = malloc(vectors);

  world->dynamics.inv_masses = malloc(floats);
  world->dynamics.velocities = malloc(vectors);
  world->dynamics.angular_momenta = malloc(vectors);
  world->dynamics.inv_inertia_tensors = malloc(matrices);
  world->dynamics.inv_intertias = malloc(matrices);
  world->dynamics.motion_avgs = malloc(floats);
  world->dynamics.awake_count = 0;
  world->generation = 0;

  world->config = *config;

  contacts_init(world);
  joints_init(world);
  shapes_init(world);
  profiler_init_default();

  return world;
}


static body add_primitive_body_static(physics_world* world, body_shape shape) {
  static_bodies *data = &world->statics;
  if (data->capacity < data->count + 1) {
    realloc_commons(data);
  }

  count_t index = data->count++;
  init_body_common(world, data, BRACKET_PRIMITIVE, &shape, 1, index);

  data->outer_lookup[index] = index;
  data->inner_lookup[index] = index;

  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = NULL,
    .angular_momentum = NULL,
    .handle = make_body_handle(world, BODY_STATIC, index)
  };
}

static body add_primitive_body_dynamic(physics_world* world, body_shape shape, float mass) {
  dynamic_bodies *data = &world->dynamics;
  if (data->capacity < data->count + 1) {
    realloc_commons((common_data*) data);
    realloc_dynamics(data);
  }

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data*) data, BRACKET_PRIMITIVE, &shape, 1, index);

  m3 inertia = matrix_initial_inertia(inertia_vector(shape, mass));
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = &data->velocities[index],
    .angular_momentum = &data->angular_momenta[index],
    .handle = make_body_handle(world, BODY_DYNAMIC, index),
  };
}

void physics_add_plane(physics_world *world, v3 point, v3 normal) {
  body plane = add_primitive_body_static(world, (body_shape) { .type = SHAPE_PLANE, .plane = { .normal = normal }, .offset = zero(), .rotation = qidentity() });
  world->statics.positions[handle_to_inner_index(world, plane.handle)] = point;
}

body physics_add_box_dynamic(physics_world *world, float mass, v3 size) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() }, mass);
}

body physics_add_box_static(physics_world *world, v3 size) {
  return add_primitive_body_static(world, (body_shape) { .type = SHAPE_BOX, .box = { .size = size }, .offset = zero(), .rotation = qidentity() });
}

body physics_add_sphere_dynamic(physics_world *world, float mass, float radius) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = radius }, .offset = zero(), .rotation = qidentity() }, mass);
}

body physics_add_cylinder_static(physics_world *world, float radius, float height) {
  return add_primitive_body_static(world, (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() });
}

body physics_add_cylinder_dynamic(physics_world *world, float mass, float radius, float height) {
  return add_primitive_body_dynamic(world, (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .radius = radius, .height = height }, .offset = zero(), .rotation = qidentity() }, mass);
}

body physics_add_compound_body_static(physics_world *world, body_shape *shapes, float *masses, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  static_bodies *data = &world->statics;
  if (data->capacity < data->count + 1) {
    realloc_commons(data);
  }

  count_t index = data->count++;
  init_body_common(world, (common_data*) &world->statics, bracket, shapes, shapes_count, index);

  data->outer_lookup[index] = index;
  data->inner_lookup[index] = index;

  float mass;
  body_shape* body_shapes = shapes_get(world, data->shapes[index]);
  calculate_compound_shape_static(body_shapes, masses, shapes_count, &mass);

  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = NULL,
    .angular_momentum = NULL,
    .handle = make_body_handle(world, BODY_STATIC, index)
  };
}

body physics_add_compound_body_dynamic(physics_world *world, body_shape *shapes, float *masses, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  dynamic_bodies *data = &world->dynamics;
  if (data->capacity < data->count + 1) {
    realloc_commons((common_data*) data);
    realloc_dynamics(data);
  }

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data*) data, bracket, shapes, shapes_count, index);

  float mass;
  m3 inertia;
  body_shape *body_shapes = shapes_get(world, data->shapes[index]);
  calculate_compound_shape_dynamic(body_shapes, masses, shapes_count, &mass, &inertia);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return (body) {
    .position = &data->positions[index],
    .rotation = &data->rotations[index],
    .velocity = &data->velocities[index],
    .angular_momentum = &data->angular_momenta[index],
    .handle = make_body_handle(world, BODY_DYNAMIC, index)
  };
}

void physics_apply_force(physics_world *world, body_handle handle, v3 force) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.forces[index];

  world->dynamics.forces[index] = add(prev_force, force);
}

void physics_apply_force_at(physics_world *world, body_handle handle, v3 force, v3 position) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.forces[index];
  v3 prev_torque = world->dynamics.torques[index];

  v3 r = sub(position, world->dynamics.positions[index]);
  v3 torque = cross(r, force);

  world->dynamics.forces[index] = add(prev_force, force);
  world->dynamics.torques[index] = add(prev_torque, torque);
}

void physics_apply_impulse(physics_world *world, body_handle handle, v3 impulse) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_impulse = world->dynamics.impulses[index];

  world->dynamics.impulses[index] = add(prev_impulse, impulse);
}

void physics_apply_impulse_at(physics_world *world, body_handle handle, v3 impulse, v3 position) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  v3 prev_force = world->dynamics.impulses[index];
  v3 prev_angular_impulse = world->dynamics.angular_impulses[index];

  v3 r = sub(position, world->dynamics.positions[index]);
  v3 angular_impulse = cross(r, impulse);

  world->dynamics.impulses[index] = add(prev_force, impulse);
  world->dynamics.angular_impulses[index] = add(prev_angular_impulse, angular_impulse);
}

physics_config* physics_edit_config(physics_world *world) {
  return &world->config;
}

count_t physics_body_count(const physics_world *world, body_type type) {
  return as_common_const(world, type)->count;
}

count_t physics_awake_count(const physics_world *world) {
  return world->dynamics.awake_count;
}

count_t physics_collisions_count(const physics_world *world) {
  return world->contacts.count;
}

v3 physics_get_position(const physics_world *world, body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->positions[handle_to_inner_index(world, handle)];
}

quat physics_get_rotation(const physics_world *world, body_handle handle) {
  const common_data *data = as_common_const(world, handle.type);
  return data->rotations[handle_to_inner_index(world, handle)];
}

body_shape* physics_get_shapes(const physics_world *world, body_handle handle, count_t *count) {
  const common_data *data = as_common_const(world, handle.type);
  body_shapes shapes = data->shapes[handle_to_inner_index(world, handle)];
  *count = shapes.count;
  return shapes_get(world, shapes);
}

v3 physics_get_velocity(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  return world->dynamics.velocities[handle_to_inner_index(world, handle)];
}

v3 physics_get_angular_velocity(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  v3 momentum = dynamics->angular_momenta[index];
  quat rotation = dynamics->rotations[index];
  m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return matrix_rotate(momentum, matrix_inertia(inv_inertia, rotation));
}

v3 physics_get_angular_momentum(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return zero();
  }

  return world->dynamics.angular_momenta[handle_to_inner_index(world, handle)];
}

float physics_get_motion_avg(const physics_world *world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC) {
    return 0;
  }

  return world->dynamics.motion_avgs[handle_to_inner_index(world, handle)];
}

count_t physics_get_contacts(const physics_world *world, contact_t *contacts, count_t max_contacts) {
  count_t count = world->contacts.count < max_contacts ? world->contacts.count : max_contacts;
  for(count_t i = 0; i < count; ++i) {
    contact full_contact = world->contacts.values[i];
    body_type type = i < world->contacts.dynamic_count ? BODY_DYNAMIC : BODY_STATIC;

    contacts[i] = (contact_t) {
      .point = full_contact.point,
      .normal = full_contact.normal,
      .depth = full_contact.depth,
      .body_a = make_body_handle(world, BODY_DYNAMIC, full_contact.index_a),
      .body_b = make_body_handle(world, type, full_contact.index_b)
    };
  }

  return count;
}

const count_t sentinel_index = (count_t)~0 >> 1;

void physics_enumerate_bodies_typed(const physics_world *world, body_type type, body_enumerator *enumerator) {
  enumerator->handle = (body_handle) { .type = type, .index = sentinel_index & 0x7FFFFFFF };
  enumerator->generation = world->generation;
}

bool physics_body_next_typed(const physics_world *world, body_enumerator_typed *enumerator) {
  if (enumerator->generation != world->generation) {
    return false;
  }

  const common_data *data = as_common_const(world, enumerator->handle.type);
  if (enumerator->handle.index == sentinel_index) {
    if (data->count == 0) {
      return false;
    }

    enumerator->handle.index = 0;
    return true;
  }

  count_t index = enumerator->handle.index;
  if (index < data->count - 1) {
    enumerator->handle.index = ++index;
    return true;
  }

  return false;
}

void integrate_bodies(physics_world *world, float dt) {
  PROFILE_FUNCTION

  v3 gravity_acc = world->config.gravity;
  float linear_damping = powf(world->config.linear_damping, dt);
  float angular_damping = powf(world->config.angular_damping, dt);

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
    m3 inertia = matrix_inertia(dynamics->inv_inertia_tensors[i], rotation);

    v3 momentum_delta = scale(dynamics->torques[i], dt);
    momentum_delta = add(momentum_delta, dynamics->angular_impulses[i]);

    v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = add(angular_momentum, momentum_delta);
    angular_momentum = scale(angular_momentum, angular_damping);

    v3 omega = matrix_rotate(angular_momentum, inertia);

    quat q_omega = { omega.x, omega.y, omega.z, 0 };
    quat dq = qscale(qmul(q_omega, rotation), 0.5 * dt);
    quat q_orientation = qadd(rotation, dq);
    rotation = qnormalize(q_orientation);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = add(dynamics->positions[i], scale(velocity, dt));
  }
}

void physics_step(physics_world* world, float dt) {
  profiler_start_frame();
  {
    PROFILE_FUNCTION

    integrate_bodies(world, dt);
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

void physics_awaken_body(physics_world* world, body_handle handle) {
  if (handle.type != BODY_DYNAMIC)
    return;

  count_t index = handle_to_inner_index(world, handle);
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count)
    return;

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index)
    swap_bodies(world, index, target_index);

  dynamics->motion_avgs[target_index] = 2.0 * world->config.sleep_threshold;
  dynamics->awake_count += 1;
}

void physics_reset(physics_world *world) {
  world->dynamics.count = 0;
  world->dynamics.awake_count = 0;

  world->statics.count = 0;

  contacts_reset(world);
  shapes_reset(world);
  joints_reset(world);
}

void physics_teardown(physics_world* world) {
  teardown_commons((common_data*) &world->dynamics);
  teardown_commons((common_data*) &world->statics);

  free(world->dynamics.forces);
  free(world->dynamics.torques);
  free(world->dynamics.impulses);
  free(world->dynamics.angular_impulses);
  free(world->dynamics.accelerations);

  free(world->dynamics.inv_masses);
  free(world->dynamics.velocities);
  free(world->dynamics.angular_momenta);
  free(world->dynamics.inv_inertia_tensors);
  free(world->dynamics.inv_intertias);
  free(world->dynamics.motion_avgs);

  shapes_teardown(world);
  joints_teardown(world);
  contacts_teardown(world);

  free(world);

  profiler_teardown();
}

static void swap_bodies(physics_world *world, count_t index_a, count_t index_b) {
  #define SWAP(type, arr) \
    type tmp_##arr = world->dynamics.arr[index_a]; \
    world->dynamics.arr[index_a] = world->dynamics.arr[index_b]; \
    world->dynamics.arr[index_b] = tmp_##arr;

  SWAP(v3, positions)
  SWAP(quat, rotations)
  SWAP(body_shapes, shapes)
  SWAP(float, inv_masses)
  SWAP(v3, velocities)
  SWAP(v3, angular_momenta)
  SWAP(m3, inv_inertia_tensors)
  SWAP(m3, inv_intertias)
  SWAP(float, motion_avgs)

  SWAP(v3, forces)
  SWAP(v3, torques)
  SWAP(v3, impulses)
  SWAP(v3, angular_impulses)
  SWAP(v3, accelerations)

  SWAP(count_t, inner_lookup)

  world->dynamics.outer_lookup[world->dynamics.inner_lookup[index_b]] = index_b;
  world->dynamics.outer_lookup[world->dynamics.inner_lookup[index_a]] = index_a;
  world->generation += 1;

  #undef SWAP
}

static void move_body(physics_world *world, count_t src_index, count_t dst_index) {
  dynamic_bodies *data = &world->dynamics;

  data->positions[dst_index] = data->positions[src_index];
  data->rotations[dst_index] = data->rotations[src_index];
  data->shapes[dst_index] = data->shapes[src_index];
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
