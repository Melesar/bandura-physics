#include "scenario-core.h"
#include "bnd-math.h"
#include "raymath.h"
#include <unistd.h>

bool is_collision;
bnd_raycast_hit hit;

void handle_error(bnd_error_type error_type, char *error_message, void *error_data) {
  if (error_type == BND_ERROR_INVALID_POLYTOPE) {
    TraceLog(LOG_FATAL, error_message);
  }
}

void scenario_configure(program_config *config, bnd_config *physics) {
  config->window_title = "Rigidbodies";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
}

void scenario_initialize(bnd_world *world) {
  bnd_register_error_callback(handle_error);
}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  bnd_body big_box = bnd_add_box_static(world, (bnd_v3){ 10, 3, 1 });
  *big_box.position = (bnd_v3){ 0, 1.5, -5 };

  big_box = bnd_add_box_static(world, (bnd_v3){ 10, 3, 1 });
  *big_box.position = (bnd_v3){ 0, 1.5, 5 };

  big_box = bnd_add_box_static(world, (bnd_v3){ 1, 3, 10 });
  *big_box.position = (bnd_v3){ -7, 1.5, 0 };

  big_box = bnd_add_cylinder_static(world, 1, 3);
  *big_box.position = (bnd_v3){ 0, 1.5, 0 };
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_handle_input(bnd_world *world, Camera *cam) {
  if (IsKeyPressed(KEY_X)) {
    bnd_body big_box = bnd_add_box_dynamic(world, 10, (bnd_v3){ 1.3, 1.3, 1.3 });
    *big_box.position = (bnd_v3){ 0, 7, 0 };
    *big_box.angular_momentum = (bnd_v3){ 1, 1, 1 };
  }

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    bnd_v3 direction = Vector3Normalize(Vector3Subtract(cam->target, cam->position));

    bnd_body ball = bnd_add_sphere_dynamic(world, 3, 0.7);
    *ball.position = bnd_v3_add(cam->position, direction);

    bnd_apply_impulse(world, ball.handle, bnd_v3_scale(direction, 70));
  }

  if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
    Ray r = GetScreenToWorldRay(GetMousePosition(), *cam);

    bnd_ray ray = {
      .origin = r.position,
      .direction = r.direction,
      .max_distance = 100.0,
    };

    bnd_raycast_hit raycast_hit;
    if (bnd_raycast_closest(world, ray, &raycast_hit)) {
      count_t num_shapes;
      bnd_body_shape *shapes = bnd_get_shapes(world, raycast_hit.body, &num_shapes);

      if (num_shapes > 0 && shapes[0].type != BND_PLANE) {
        bnd_remove_body(world, raycast_hit.body);
      }
    }
  }
}

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {}
