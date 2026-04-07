#include "core.h"

void scenario_initialize(program_config *config,
                         physics_config *physics_config) {
  config->window_title = "Collisions";
  config->camera_position = vec3(0, 5, -10);
  config->camera_target = vec3(0, 2, 10);
}

void scenario_setup_scene(physics_world *world) {}

void scenario_handle_input(physics_world *world, Camera *camera) {}

void scenario_simulate(physics_world *world, float dt) {}

void scenario_draw_scene(physics_world *world) {}

void scenario_build_ui(physics_world *world) {}

void scenario_teardown() {}
