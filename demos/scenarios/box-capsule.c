#include "scenario-core.h"
#include "bnd-math.h"

static bnd_body_handle capsule;

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Box-capsule collisions";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };

  physics_config->simulation.gravity = bnd_v3_zero();
  physics_config->advanced.resolution_attempts_factor = 0;
}

void scenario_initialize(bnd_world *world) {

}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_box_dynamic(world, 5, (bnd_v3) { 3, 1, 5 });

  capsule = bnd_add_capsule_dynamic(world, 5, 1, 1.5).value;
  bnd_set_position(world, capsule, (bnd_v3) { 0, 3, 0 });

  register_gizmo(world, capsule);
}

void scenario_handle_input(bnd_world *world, Camera *camera) { }

void scenario_simulate(bnd_world *world, float dt) {
  bnd_awaken_body(world, capsule);
  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) { }

void scenario_build_ui(bnd_world *world) { }

void scenario_teardown() { }
