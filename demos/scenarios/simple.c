#include "raylib.h"
#include "scenario-core.h"
#include "bnd-math.h"
#include "raymath.h"

#include <stdlib.h>

ragdoll rgd;

void scenario_configure(program_config *config, bnd_config *bandura_config) {
  config->window_title = "Simple";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
}

void scenario_initialize(bnd_world *world) {
  
}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), (bnd_v3) { 0, 1, 0 });

  bnd_body_handle box = bnd_add_box_static(world, (bnd_v3) { 3, 0.5, 3 }).value;
  bnd_set_position(world, box, (bnd_v3) { 0, 5, 0 });

  bnd_body_handle capsule = bnd_add_capsule_dynamic(world, 5, 0.5, 1).value;
  bnd_set_position(world, capsule, (bnd_v3) { 1.5f, 3.75f, 1.5f });

  bnd_result_joint result = bnd_add_hinge_joint(world, box, capsule, (bnd_hinge_joint_config) {
    .local_anchor_a = { 1.5f, -0.25f, 1.5f },
    .local_anchor_b = { 0, 1, 0 },
  });

  if (result.error.type != BND_OK) {
    TraceLog(LOG_ERROR, "Failed to add joint: %s", result.error.message);
  }
}

void scenario_handle_input(bnd_world *world, Camera *camera) {
  
}

void scenario_simulate(bnd_world *world, float dt) {
  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) {
  
}

void scenario_build_ui(bnd_world *world) {
  
}

void scenario_teardown() {
  
}

