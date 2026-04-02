#include "core.h"
#include "raygui.h"
#include <stdlib.h>

ragdoll hanging_doll;
ragdoll normal_doll;

bool simulate_dolls;

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = vec3(0, 5, -10);
  config->camera_target = vec3(0, 2, 10);
}

void scenario_setup_scene(physics_world *world) {
  // if (normal_doll) {
  //   free(normal_doll);
  // }

  // normal_doll = ragdoll_create(world, scale(up(), 3));

  body_shape ramp_shapes[] = {
    (body_shape) { .type = SHAPE_BOX, .box = { .size = vec3(1, 10, 1) }, .offset = vec3(0, 5, 0), .rotation = qidentity() },
    (body_shape) { .type = SHAPE_BOX, .box = { .size = vec3(3, 1, 1) }, .offset = vec3(1, 10, 0), .rotation = qidentity() },
  };

  body ramp = physics_add_compound_body_static(world, ramp_shapes, 2);
  *ramp.position = vec3(5, 0, 5);

  if (hanging_doll) {
    free(hanging_doll);
  }

  hanging_doll = ragdoll_create(world, vec3(8, 5, 5));
  physics_add_joint(world, ramp.handle, hanging_doll[RIGHT_LOWER_ARM], vec3(3, 10, 0), vec3(0, -0.6, 0), 0.05);
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

void scenario_build_ui() {
}
