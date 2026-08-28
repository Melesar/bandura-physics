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
  bnd_add_box_static(world, (bnd_v3) { 20, 0.5, 20 });
  
  bnd_body_handle box = bnd_add_box_dynamic(world, 5, bnd_v3_one()).value;
  bnd_set_position(world, box, (bnd_v3){ -3, 5, 0 });
  bnd_set_rotation(world, box, QuaternionFromEuler(PI / 6, 0, 0));

  bnd_body_handle ball = bnd_add_sphere_dynamic(world, 5, 1).value;
  bnd_set_position(world, ball, (bnd_v3) { 0, 5, 0 });

  bnd_body_handle capsule = bnd_add_capsule_dynamic(world, 5, 0.5, 1).value;
  bnd_set_position(world, capsule, (bnd_v3) { 3, 5, 0 });
  bnd_set_rotation(world, capsule, QuaternionFromEuler(PI / 3 , 0, 0));

  if (rgd) {
    free(rgd);
  }

  rgd = ragdoll_create(world, (bnd_v3) { 0, 5, -5 });
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

