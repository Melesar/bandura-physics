#include "core.h"
#include <stdlib.h>

ragdoll normal_doll;

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = vec3(0, 5, -10);
  config->camera_target = vec3(0, 2, 10);
}

void scenario_setup_scene(physics_world *world) {
  if (normal_doll) {
    free(normal_doll);
  }

  normal_doll = ragdoll_create(world, scale(up(), 3));
}

void scenario_handle_input(physics_world *world, Camera *camera) {
  if (IsKeyPressed(KEY_J)) {
    physics_apply_impulse(world, normal_doll[PELVIS], scale(up(), 12));
  }
}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
