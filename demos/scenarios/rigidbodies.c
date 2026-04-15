#include "core.h"
#include "raylib.h"

#include <unistd.h>

bool is_collision;
raycast_hit hit;

void scenario_initialize(program_config *config, physics_config *physics) {
  config->window_title = "Rigidbodies";
  config->camera_position = (v3){22.542, 11.645, 20.752};
  config->camera_target = (v3){0, 0, 0};
}

void scenario_setup_scene(physics_world *world) {
  body big_box = physics_add_box_static(world, (v3){10, 3, 1});
  *big_box.position = (v3){0, 1.5, -5};

  big_box = physics_add_box_static(world, (v3){10, 3, 1});
  *big_box.position = (v3){0, 1.5, 5};

  big_box = physics_add_box_static(world, (v3){1, 3, 10});
  *big_box.position = (v3){-7, 1.5, 0};

  big_box = physics_add_cylinder_static(world, 1, 3);
  *big_box.position = (v3){0, 1.5, 0};
}

void scenario_simulate(physics_world *world, float dt) {
  physics_step(world, dt);
}

void scenario_handle_input(physics_world *world, Camera *cam) {
  if (IsKeyPressed(KEY_X)) {
    body big_box = physics_add_box_dynamic(world, 10, (v3){1.3, 1.3, 1.3});
    *big_box.position = (v3){0, 7, 0};
    *big_box.angular_momentum = (v3){1, 1, 1};
  }

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    v3 direction = normalize(sub(cam->target, cam->position));

    body ball = physics_add_sphere_dynamic(world, 3, 0.7);
    *ball.position = add(cam->position, direction);

    physics_apply_impulse(world, ball.handle, scale(direction, 70));
  }

  if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
    v3 direction = normalize(sub(cam->target, cam->position));

    raycast_hit raycast_hit;
    if (physics_raycast(world, cam->position, direction, 100.0, 1, &raycast_hit)) {
      physics_remove_body(world, raycast_hit.body);
    }
  }
}

void scenario_draw_scene(physics_world *world) {}

void scenario_build_ui(physics_world *world) {}

void scenario_teardown() {}
