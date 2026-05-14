#include "scenario-core.h"
#include "raymath.h"

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Compounds";
  config->camera_position = (v3){ 0, 5, -10 };
  config->camera_target = (v3){ 0, 5, 10 };

  physics_config->simulation.gravity = zero();
  physics_config->simulation.angular_damping = 1;
  physics_config->simulation.sleep_base_bias = 1;
}

void scenario_initialize(bnd_world *world) { }

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, zero(), up());

  bnd_body_shape shapes[] = {
    (bnd_body_shape){ .type = BND_BOX,
      .box = { .size = (v3){ .x = 0.3, .y = 3, .z = 0.3 } },
      .offset = zero(),
      .rotation = qidentity() },
    (bnd_body_shape){ .type = BND_CYLINDER,
      .cylinder = { .height = 2, .radius = 0.5 },
      .offset = (v3){ .x = 0, .y = 1.75, .z = 0 },
      .rotation = ray_quat(QuaternionFromEuler(PI * 0.5, 0, 0)) },
    (bnd_body_shape){ 0 },
  };
  float masses[] = { 5.0, 3.0, 1.0 };

  bnd_body b = bnd_add_compound_body_dynamic(world, shapes, masses, 2);
  *b.position = (v3){ 0, 10, 0 };
  *b.angular_momentum = (v3){ 0, 0, 36 };

  shapes[0] = (bnd_body_shape){ .type = BND_CYLINDER,
    .cylinder = { .radius = 0.3, .height = 3 },
    .offset = zero(),
    .rotation = ray_quat(QuaternionFromEuler(PI * 0.5, 0, 0)) };
  shapes[1] = (bnd_body_shape){
    .type = BND_SPHERE, .sphere = { .radius = 0.7 }, .offset = (v3){ .x = 0, .y = 0, .z = 1.5 }, .rotation = qidentity()
  };
  shapes[2] = (bnd_body_shape){ .type = BND_SPHERE,
    .sphere = { .radius = 0.7 },
    .offset = (v3){ .x = 0, .y = 0, .z = -1.5 },
    .rotation = qidentity() };

  masses[0] = 3;
  masses[1] = 5;
  masses[2] = 5;

  b = bnd_add_compound_body_dynamic(world, shapes, masses, 3);
  *b.position = (v3){ 15, 10, 0 };
  *b.angular_momentum = (v3){ 0, 50, 0 };
}

void scenario_handle_input(bnd_world *world, Camera *camera) {

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    v3 direction = normalize(sub(ray_vec(camera->target), ray_vec(camera->position)));

    bnd_body ball = bnd_add_sphere_dynamic(world, 3, 0.7);
    *ball.position = add(ray_vec(camera->position), direction);

    bnd_apply_impulse(world, ball.handle, scale(direction, 70));
    bnd_awaken_body(world, ball.handle);
  }
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {}
