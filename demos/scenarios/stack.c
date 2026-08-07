#include "raylib.h"
#include "scenario-core.h"
#include "bnd-core.h"
#include "bnd-math.h"

bnd_body_handle box_top;
bnd_body_handle box_underneath;

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Stack";
  config->camera_position = (bnd_v3){0, 5, -10};
  config->camera_target = (bnd_v3){0, 2, 10};
}

void scenario_initialize(bnd_world *world) { }

void scenario_setup_scene(bnd_world *world) {
  const float floor_thikness = 0.5;
  const float box_size = 1;
  const int stack_height = 5;

  bnd_add_box_static(world, (bnd_v3) { 30, floor_thikness, 30 });

  bnd_v3 center = { 0, 0.5 * (floor_thikness + box_size), 0 };
  for (int i = 0; i < stack_height; ++i) {
    bnd_body_handle box = bnd_add_box_dynamic(world, 3, (bnd_v3) { box_size, box_size, box_size }).value;
    bnd_set_position(world, box, center);
    
    center = bnd_v3_add(center, (bnd_v3) { 0, box_size, 0 });

    if (i == 4) {
      box_top = box;
    } else if (i == 3) {
      box_underneath = box;
    }
  }
}
void scenario_handle_input(bnd_world *world, Camera *camera) {
}

void scenario_simulate(bnd_world *world, float dt) {
  if (world->age < 121) {
    bnd_simulate(world, dt);
  }

}

void draw_face(bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_debug_epa_flags flags, void *user_data) {
  Color color = flags & DEBUG_EPA_FACE_NEAREST ? GREEN : ORANGE;
  DrawTriangle3D(a, b, c, color);

  DrawLine3D(a, b, BLACK);
  DrawLine3D(b, c, BLACK);
  DrawLine3D(c, a, BLACK);
}

void draw_normal(bnd_v3 origin, bnd_v3 unit_normal, bnd_debug_epa_flags flags, void *user_data) {
  draw_arrow(origin, bnd_v3_scale(unit_normal, 0.1), RED);
}

void draw_support(bnd_v3 point, void *user_data) {
  
}

void scenario_draw_scene(bnd_world *world) {
  if (world->age == 121) {
    debug_epa_begin(world, box_underneath, box_top);
    debug_epa_iteration(world, box_underneath, box_top, 2, (bnd_debug_draw_epa_callbacks) { draw_face, draw_normal, draw_support }, NULL);
  }
}

void scenario_build_ui(bnd_world *world) {
}

void scenario_teardown() {
}
