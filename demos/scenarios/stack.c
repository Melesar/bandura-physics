#include "scenario-core.h"
#include "bnd-math.h"

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Stack";
  config->camera_position = (bnd_v3){0, 5, -10};
  config->camera_target = (bnd_v3){0, 2, 10};
}

void scenario_initialize(bnd_world *world) { }

void scenario_setup_scene(bnd_world *world) {
  const float floor_thikness = 0.5;
  const float box_size = 1;
  const int stack_height = 5;

  bnd_add_box_static(world, (bnd_v3) { 30, floor_thikness, 30 });

  bnd_v3 center = { 0, 0.5 * (floor_thikness + box_size), 0 };
  for (int i = 0; i < stack_height; ++i) {
    bnd_body_handle box = bnd_add_box_dynamic(world, 3, (bnd_v3) { box_size, box_size, box_size }).value;
    bnd_set_position(world, box, center);
    
    center = bnd_v3_add(center, (bnd_v3) { 0, box_size, 0 });
  }
}
void scenario_handle_input(bnd_world *world, Camera *camera) {
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {
}

void scenario_teardown() {
}
