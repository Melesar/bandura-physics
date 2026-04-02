#include "core.h"
#include "raymath.h"

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Compounds";
  config->camera_position = (v3) { 0, 5, -10 };
  config->camera_target = (v3) { 0, 5, 10 };
}

void scenario_setup_scene(physics_world *world) {
  body_shape shapes[] = {
    (body_shape) { .type = SHAPE_BOX, .box = { .size = (v3) { .x = 0.3, .y = 3, .z = 0.3 } }, .offset = zero(), .rotation = qidentity() },
    (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .height = 2, .radius = 0.5 }, .offset = (v3) { .x = 0, .y = 1.75, .z = 0 }, .rotation = QuaternionFromEuler(PI * 0.5, 0, 0) },
    (body_shape) { 0 },
  };
  float masses[] = { 5.0, 3.0, 1.0 };

  body b = physics_add_compound_body_dynamic(world, shapes, masses, 2);
  *b.position = (v3) { 0, 10, 0 };
  *b.angular_momentum = (v3) { 0, 0, 36 };

  physics_awaken_body(world, b.handle);

  shapes[0] = (body_shape) { .type = SHAPE_CYLINDER, .cylinder = { .radius = 0.3, .height = 3 }, .offset = zero(), .rotation = QuaternionFromEuler(PI * 0.5, 0, 0) };
  shapes[1] = (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = 0.7 }, .offset = (v3) { .x = 0, .y = 0, .z = 1.5 }, .rotation = qidentity() };
  shapes[2] = (body_shape) { .type = SHAPE_SPHERE, .sphere = { .radius = 0.7 }, .offset = (v3) { .x = 0, .y = 0, .z = -1.5 }, .rotation = qidentity() };

  masses[0] = 3;
  masses[1] = 5;
  masses[2] = 5;

  b = physics_add_compound_body_dynamic(world, shapes, masses, 3);
  *b.position = (v3) { 15, 10, 0};
  *b.angular_momentum = (v3) { 0, 50, 0};

  physics_awaken_body(world, b.handle);

}

void scenario_handle_input(physics_world *world, Camera *camera) {

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    v3 direction = normalize(sub(camera->target, camera->position));

    body ball = physics_add_sphere_dynamic(world, 3, 0.7);
    *ball.position = add(camera->position, direction);

    physics_apply_impulse(world, ball.handle, scale(direction, 70));
    physics_awaken_body(world, ball.handle);
  }
}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_build_ui() {

}
