#include "scenario-core.h"
#include "load-testing.h"

#define NUM_ANCHORS 12
bnd_body_handle *anchors;

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "High load";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
}

void scenario_initialize(bnd_world *world) {
  anchors = malloc(NUM_ANCHORS * sizeof(bnd_body_handle));
}

void scenario_setup_scene(bnd_world *world) {
  // dense_settling_pile(world, 8);
  // compound_crowd(world, 4, 5, 4);
  joints_lattice(world, NUM_ANCHORS, anchors);
}

void scenario_handle_input(bnd_world *world, Camera *camera) { }

void scenario_simulate(bnd_world *world, float dt) {
  static int step = 0;
  drive_joint_lattice(world, step++, anchors, NUM_ANCHORS);
  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) { }

void scenario_build_ui(bnd_world *world) { }

void scenario_teardown() { }
