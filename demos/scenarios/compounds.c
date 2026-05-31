#include "scenario-core.h"
#include "bnd-math.h"
#include "raymath.h"

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Compounds";
  config->camera_position = (bnd_v3){ 0, 5, -10 };
  config->camera_target = (bnd_v3){ 0, 5, 10 };

  physics_config->simulation.gravity = bnd_v3_zero();
  physics_config->simulation.angular_drag = 1;
  physics_config->simulation.sleep_base_bias = 1;
}

void scenario_initialize(bnd_world *world) { }

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  bnd_body_shape shapes[] = {
    (bnd_body_shape){ .type = BND_BOX,
      .value = { .box = { .size = (bnd_v3){ .x = 0.3, .y = 3, .z = 0.3 } } },
      .offset = bnd_v3_zero(),
      .rotation = bnd_qidentity() },
    (bnd_body_shape){ .type = BND_CAPSULE,
      .value = { .capsule = { .height = 2, .radius = 0.5 } },
      .offset = (bnd_v3){ .x = 0, .y = 1.75, .z = 0 },
      .rotation = QuaternionFromEuler(PI * 0.5, 0, 0) },
    (bnd_body_shape){ 0 },
  };
  float masses[] = { 5.0, 3.0, 1.0 };

  bnd_body_handle b = bnd_add_compound_body_dynamic(world, shapes, masses, 2).value;
  bnd_set_position(world, b, (bnd_v3){ 0, 10, 0 });
  bnd_set_angular_momentum(world, b, (bnd_v3){ 0, 0, 36 });

  shapes[0] = (bnd_body_shape){ .type = BND_CAPSULE,
    .value = { .capsule = { .radius = 0.3, .height = 3 } },
    .offset = bnd_v3_zero(),
    .rotation = QuaternionFromEuler(PI * 0.5, 0, 0) };
  shapes[1] = (bnd_body_shape){
    .type = BND_SPHERE, .value = { .sphere = { .radius = 0.7 } }, .offset = (bnd_v3){ .x = 0, .y = 0, .z = 1.5 }, .rotation = bnd_qidentity()
  };
  shapes[2] = (bnd_body_shape){ .type = BND_SPHERE,
    .value = { .sphere = { .radius = 0.7 } },
    .offset = (bnd_v3){ .x = 0, .y = 0, .z = -1.5 },
    .rotation = bnd_qidentity() };

  masses[0] = 3;
  masses[1] = 5;
  masses[2] = 5;

  b = bnd_add_compound_body_dynamic(world, shapes, masses, 3).value;
  bnd_set_position(world, b, (bnd_v3){ 15, 10, 0 });
  bnd_set_angular_momentum(world, b, (bnd_v3){ 0, 50, 0 });
}

void scenario_handle_input(bnd_world *world, Camera *camera) {

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    bnd_v3 direction = bnd_v3_normalize(bnd_v3_sub(camera->target, camera->position));

    bnd_body_handle ball = bnd_add_sphere_dynamic(world, 3, 0.7).value;
    bnd_set_position(world, ball, bnd_v3_add(camera->position, direction));

    bnd_apply_impulse(world, ball, bnd_v3_scale(direction, 70));
    bnd_awaken_body(world, ball);
  }
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {}
