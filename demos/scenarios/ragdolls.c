#include "core.h"
#include "raymath.h"

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = (v3) { 0, 5, -10 };
  config->camera_target = (v3) { 0, 5, 10 };
}

void scenario_setup_scene(physics_world *world) {
  body a = physics_add_cylinder_dynamic(world, 5, 0.5, 2);
  body b = physics_add_cylinder_dynamic(world, 5, 0.5, 2);

  *a.position = (v3) { 0.95, 3, 0 };
  *b.position = (v3) { -0.95, 3, 0 };

  *a.rotation = QuaternionFromAxisAngle(forward(), PI / 6);
  *b.rotation = QuaternionFromAxisAngle(forward(), -PI / 6);

  v3 offset = {0, 1, 0};
  physics_add_joint(world, a.handle, b.handle, offset, offset, 0.5);

  physics_awaken_body(world, a.handle);
  physics_awaken_body(world, b.handle);
}

void scenario_handle_input(physics_world *world, Camera *camera) {


}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
