#include "scenario-core.h"
#include "raymath.h"
#include "bnd-math.h"

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Features";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
}

void scenario_initialize(bnd_world *world) {

}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_box_static(world, (bnd_v3) { 10, 1, 10 });

  bnd_body_shape shapes[] = {
    (bnd_body_shape) { .type = BND_BOX, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity(), .value = { .box = { .size = (bnd_v3) { 1.5, 1.5, 1.5 } } } },
    (bnd_body_shape) { .type = BND_SPHERE, .offset = (bnd_v3) { 0, 1.5, 0 }, .rotation = bnd_quat_identity(), .value = { .sphere = { .radius = 0.75 } } },
  };
  float masses[] = { 2, 1 };

  bnd_body_handle body = bnd_add_compound_body_dynamic(world, shapes, masses, 2).value;

  bnd_set_position(world, body, (bnd_v3) { 0, 7, 0 });
  bnd_set_rotation(world, body, QuaternionFromEuler(PI * 0.25, 0, 0));
}

void scenario_handle_input(bnd_world *world, Camera *camera) { }

void scenario_simulate(bnd_world *world, float dt) {
  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) { }

void scenario_build_ui(bnd_world *world) { }

void scenario_teardown() { }
