#include "scenario-core.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "raylib.h"

bnd_body_handle boxes[6];

bnd_body_shape box_shape = {
  .type = BND_BOX,
  .value = { .box = { .size = (bnd_v3) { 1, 1, 1 } } },
  .offset = { 0 },
  .rotation = (bnd_quat){ 0, 0, 0, 1 },
};

void scenario_configure(program_config *config, bnd_config *physics) {
  config->window_title = "Collisions";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
  config->draw_ground = false;
}

void scenario_initialize(bnd_world *world) {
}

void scenario_setup_scene(bnd_world *world) {
  bnd_body_handle b1 = bnd_add_box_dynamic(world, 1, bnd_v3_one()).value;
  bnd_body_handle b2 = bnd_add_box_dynamic(world, 1, bnd_v3_one()).value;
  bnd_set_position(world, b1, (bnd_v3){0, 5, 0});
  bnd_set_position(world, b2, (bnd_v3){1, 6, 1});

  boxes[0] = b1;
  boxes[1] = b2;
  register_gizmo(world, b1);
  register_gizmo(world, b2);
}

void scenario_simulate(bnd_world *world, float dt) {
}

void scenario_handle_input(bnd_world *world, Camera *cam) {

}

void scenario_draw_scene(bnd_world *world) {
  simplex s;
  const common_data *data = as_common_const(world, BND_BODY_DYNAMIC);

  for (int i = 0; i + 1 < 2; i += 2) {
    collision_detection_context ctx = {
      .world = world,
      .data_a = data,
      .data_b = data,
      .body_a = handle_to_inner_index(world, boxes[i]),
      .body_b = handle_to_inner_index(world, boxes[i + 1]),
      .shape_a = box_shape,
      .shape_b = box_shape,
    };

    if (!gjk_check_intersection(world, &ctx, &s)) {
      continue;
    }

    contact c;
    epa_get_contact(&ctx, &s, world->config.advanced.epa_tolerance, &c);

    bnd_v3 points[6];
    epa_get_final_points(points);

    Color colors[] = { RED, GREEN };
    for (int j = 0; j < 6; j += 3) {
      bnd_v3 p0 = points[j + 0];
      bnd_v3 p1 = points[j + 1];
      bnd_v3 p2 = points[j + 2];

      DrawSphere(p0, 0.05, colors[j / 3]);
      DrawSphere(p1, 0.05, colors[j / 3]);
      DrawSphere(p2, 0.05, colors[j / 3]);

      const float radius = 0.01;
      DrawCylinderEx(p0, p1, radius, radius, 16, BLACK);
      DrawCylinderEx(p1, p2, radius, radius, 16, BLACK);
      DrawCylinderEx(p2, p0, radius, radius, 16, BLACK);
    }

  }
}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {}
