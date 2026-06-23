#include "scenario-core.h"

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Features";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
}

void scenario_initialize(bnd_world *world) {

}

void scenario_setup_scene(bnd_world *world) {
  bnd_body_handle floor = bnd_add_box_static(world, (bnd_v3) { 10, 1, 10 }).value;
  bnd_body_handle box = bnd_add_box_dynamic(world, 5, (bnd_v3) { 1.5, 1.5, 1.5 }).value;

  bnd_set_position(world, box, (bnd_v3) { 0, 7, 0 });
}

void scenario_handle_input(bnd_world *world, Camera *camera) { }

void scenario_simulate(bnd_world *world, float dt) {
  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) { }

void scenario_build_ui(bnd_world *world) { }

void scenario_teardown() { }
