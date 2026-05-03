#include "bnd-core.h"
#include "profiler.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#define MAX_MESSAGE_SIZE 512

const count_t max_body_index = (count_t)~0 >> 9;

char error_message_buffer[MAX_MESSAGE_SIZE];
bnd_error_callback error_callback = NULL;

void bnd_register_error_callback(bnd_error_callback callback) { error_callback = callback; }

static void init_commons(common_data *data, count_t capacity) {
  data->capacity = capacity;
  data->count = 0;
  data->free_count = 0;
  data->first_outer_node = max_body_index;
  data->positions = malloc(sizeof(v3) * capacity);
  data->rotations = malloc(sizeof(quat) * capacity);
  data->shapes = malloc(sizeof(body_shapes) * capacity);
  data->free_list = malloc(sizeof(count_t) * capacity);
  data->generations = malloc(sizeof(uint8_t) * capacity);
  data->outer_lookup = malloc(sizeof(outer_lookup_node) * capacity);
  data->inner_lookup = malloc(sizeof(count_t) * capacity);
}

static void teardown_commons(common_data *data) {
  free(data->positions);
  free(data->rotations);
  free(data->shapes);
  free(data->free_list);
  free(data->generations);
  free(data->outer_lookup);
  free(data->inner_lookup);
}

bnd_config bnd_default_config() {
  return (bnd_config){
    .simulation = {
      .gravity = (v3){0, -9.81f, 0},
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

bnd_world *bnd_init(const bnd_config *config) {
  bnd_world *world = malloc(sizeof(bnd_world));

  init_commons((common_data *)&world->dynamics, config->memory.dynamics_capacity);
  init_commons((common_data *)&world->statics, config->memory.statics_capacity);

  const count_t vectors = sizeof(v3) * config->memory.dynamics_capacity;
  const count_t floats = sizeof(float) * config->memory.dynamics_capacity;
  const count_t matrices = sizeof(m3) * config->memory.dynamics_capacity;

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
  meshes_init(world);
  epa_init(config);

  profiler_init_default();

  return world;
}

void bnd_teardown(bnd_world *world) {
  teardown_commons((common_data *)&world->dynamics);
  teardown_commons((common_data *)&world->statics);

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
  meshes_teardown(world);

  free(world);

  profiler_teardown();
}

void raise_error(bnd_error type, void *data, const char *template, ...) {
  if (error_callback == NULL) {
    return;
  }

  va_list list;
  va_start(list, template);
  vsnprintf(error_message_buffer, MAX_MESSAGE_SIZE, template, list);
  va_end(list);

  error_callback(type, error_message_buffer, data);
}
