#include "bnd-core.h"
#include "profiler.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#define MAX_MESSAGE_SIZE 512

#define OK (bnd_error){BND_OK, NULL}
#define OOM_ERROR (bnd_error){BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc failed to allocate memory"}

#define ALLOC_BUFFER(buffer, capacity) \
  buffer = allocator.malloc(capacity); \
  if (buffer == NULL) { \
    return OOM_ERROR; \
  } \

const count_t max_body_index = (count_t)~0 >> 9;

char error_message_buffer[MAX_MESSAGE_SIZE];
bnd_error_callback error_callback = NULL;

static void *std_malloc(uint64_t size) {
  return malloc(size);
}

static void *std_realloc(void *ptr, uint64_t old_size, uint64_t new_size) {
  return realloc(ptr, new_size);
}

static void std_free(void *ptr, uint64_t size) {
  free(ptr);
}

static bnd_allocator bnd_default_allocator() {
  return (bnd_allocator){
    .malloc = std_malloc,
    .realloc = std_realloc,
    .free = std_free,
  };
}

void bnd_register_error_callback(bnd_error_callback callback) {
  error_callback = callback;
}

static bnd_error init_commons(common_data *data, count_t capacity, bnd_allocator allocator) {
  data->capacity = capacity;
  data->count = 0;
  data->free_count = 0;
  data->first_outer_node = max_body_index;

  count_t total_capacity = capacity + EPHEMERAL_BODIES_COUNT;
  ALLOC_BUFFER(data->positions, sizeof(bnd_v3) * total_capacity);
  ALLOC_BUFFER(data->rotations, sizeof(bnd_quat) * total_capacity);
  ALLOC_BUFFER(data->shapes, sizeof(body_shapes) * total_capacity);
  ALLOC_BUFFER(data->aabbs, sizeof(bnd_aabb) * total_capacity);
  ALLOC_BUFFER(data->event_masks, sizeof(bnd_event_type) * total_capacity);
  ALLOC_BUFFER(data->event_links, sizeof(event_link) * total_capacity);
  ALLOC_BUFFER(data->free_list, sizeof(count_t) * total_capacity);
  ALLOC_BUFFER(data->generations, sizeof(uint8_t) * total_capacity);
  ALLOC_BUFFER(data->outer_lookup, sizeof(outer_lookup_node) * total_capacity);
  ALLOC_BUFFER(data->inner_lookup, sizeof(count_t) * total_capacity);

  return OK;
}

static void teardown_commons(common_data *data, bnd_allocator allocator) {
  allocator.free(data->positions, data->capacity * sizeof(bnd_v3));
  allocator.free(data->rotations, data->capacity * sizeof(bnd_quat));
  allocator.free(data->shapes, data->capacity * sizeof(body_shapes));
  allocator.free(data->aabbs, data->capacity * sizeof(bnd_aabb));
  allocator.free(data->event_masks, data->capacity * sizeof(bnd_event_type));
  allocator.free(data->event_links, data->capacity * sizeof(event_link));
  allocator.free(data->free_list, data->capacity * sizeof(count_t));
  allocator.free(data->generations, data->capacity * sizeof(uint8_t));
  allocator.free(data->outer_lookup, data->capacity * sizeof(outer_lookup_node));
  allocator.free(data->inner_lookup, data->capacity * sizeof(count_t));
}

bnd_config bnd_default_config() {
  return (bnd_config){
    .simulation = {
      .gravity = (bnd_v3){0, -9.81f, 0},
      .linear_damping = 0.95,
      .angular_damping = 0.8,
      .restitution = 0.2,
      .friction = 0.9,
      .sleep_base_bias = 0.5,
      .sleep_threshold = 0.3,
    },
    .memory = {
      .dynamics_capacity = 32,
      .statics_capacity = 8,
      .contacts_capacity = 64,
      .joints_capacity = 64,
      .epa_max_nodes = 512,
      .meshes_capacity = 32,
      .events_capacity = 128,
      .shapes_brackets_capacity = {64, 1, 1, 1, 1},
    },
    .collision_detection = {
      .max_gjk_iterations = 100,
      .epa_tolerance = 0.01,
    },
    .collision_resolution = {
      .resolution_attempts_factor = 15,
      .penetration_epsilon = 0.01,
      .velocity_epsilon = 0.01,
      .restitution_damping_limit = 0.25,
    },
  };
}

static bnd_error bnd_init_internal(bnd_world *world, bnd_config config, bnd_allocator allocator) {
  world->allocator = allocator;
  world->config = config;

  bnd_error error = init_commons((common_data *)&world->dynamics, config.memory.dynamics_capacity, allocator);
  if (error.type != BND_OK) {
    return error;
  }

  error = init_commons((common_data *)&world->statics, config.memory.statics_capacity, allocator);
  if (error.type != BND_OK) {
    return error;
  }

  const count_t vectors = sizeof(bnd_v3) * config.memory.dynamics_capacity;
  const count_t floats = sizeof(float) * config.memory.dynamics_capacity;
  const count_t matrices = sizeof(bnd_m3) * config.memory.dynamics_capacity;

  world->statics.dirty = false;

  ALLOC_BUFFER(world->dynamics.forces, vectors);
  ALLOC_BUFFER(world->dynamics.torques, vectors);
  ALLOC_BUFFER(world->dynamics.impulses, vectors);
  ALLOC_BUFFER(world->dynamics.angular_impulses, vectors);
  ALLOC_BUFFER(world->dynamics.accelerations, vectors);

  ALLOC_BUFFER(world->dynamics.inv_masses, floats);
  ALLOC_BUFFER(world->dynamics.velocities, vectors);
  ALLOC_BUFFER(world->dynamics.angular_momenta, vectors);
  ALLOC_BUFFER(world->dynamics.inv_inertia_tensors, matrices);
  ALLOC_BUFFER(world->dynamics.inv_intertias, matrices);
  ALLOC_BUFFER(world->dynamics.motion_avgs, floats);

  world->dynamics.awake_count = 0;
  world->generation = 0;

  contacts_init(world);
  joints_init(world);
  shapes_init(world);
  meshes_init(world);
  events_init(world);
  epa_init(config);

  profiler_init_default();

  return OK;
}

bnd_world *bnd_init(bnd_config config) {
  bnd_allocator allocator = bnd_default_allocator();
  bnd_world *world = allocator.malloc(sizeof(bnd_world));

  bnd_init_internal(world, config, allocator);
  return world;
}

bnd_world *bnd_init_with_allocator(bnd_config config, bnd_allocator allocator, bnd_error *error) {
  if (allocator.malloc == NULL) {
    *error = (bnd_error){BND_ERROR_INVALID_ALLOCATOR, "Allocator must define a malloc function"};
    return NULL;
  }

  bnd_world *world = allocator.malloc(sizeof(bnd_world));
  if (world == NULL) {
    *error = OOM_ERROR;
    return NULL;
  }

  *error = bnd_init_internal(world, config, allocator);
  return error->type == BND_OK ? world : NULL;
}

void bnd_teardown(bnd_world *world) {
  if (world->allocator.free == NULL) {
    profiler_teardown();
    return;
  }

  teardown_commons((common_data *)&world->dynamics, world->allocator);
  teardown_commons((common_data *)&world->statics, world->allocator);

  world->allocator.free(world->dynamics.forces, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.torques, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.impulses, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_impulses, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.accelerations, world->config.memory.dynamics_capacity * sizeof(bnd_v3));

  world->allocator.free(world->dynamics.inv_masses, world->config.memory.dynamics_capacity * sizeof(float));
  world->allocator.free(world->dynamics.velocities, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_momenta, world->config.memory.dynamics_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.inv_inertia_tensors, world->config.memory.dynamics_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.inv_intertias, world->config.memory.dynamics_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.motion_avgs, world->config.memory.dynamics_capacity * sizeof(float));

  shapes_teardown(world);
  joints_teardown(world);
  contacts_teardown(world);
  meshes_teardown(world);
  events_teardown(world);

  world->allocator.free(world, sizeof(bnd_world));

  profiler_teardown();
}

count_t ephemeral_body_index(const common_data *data) {
  return data->capacity;
}

void raise_error(bnd_error_type type, void *data, const char *template, ...) {
  if (error_callback == NULL) {
    return;
  }

  va_list list;
  va_start(list, template);
  vsnprintf(error_message_buffer, MAX_MESSAGE_SIZE, template, list);
  va_end(list);

  error_callback(type, error_message_buffer, data);
}

void raise_error_debug(bnd_error_type type, void *data, const char *template, ...) {
#if defined(BND_DEBUG)
  if (error_callback == NULL) {
    return;
  }

  va_list list;
  va_start(list, template);
  vsnprintf(error_message_buffer, MAX_MESSAGE_SIZE, template, list);
  va_end(list);

  error_callback(type, error_message_buffer, data);
#endif
}

void notify_body_removed(bnd_body_handle handle) {
  raise_error_debug(BND_ERROR_BODY_REMOVED, NULL, "Body %d (%s) has been removed", handle.index, handle.type == BND_DYNAMIC ? "dynamic" : "static");
}
