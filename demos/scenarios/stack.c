#include "raylib.h"
#include "scenario-core.h"
#include "bnd-core.h"
#include "bnd-math.h"

int target_age;
int target_iteration;
uint32_t iterations_count;

epa_debug_status debug_status;
bnd_error error;

bnd_body_handle box_top;
bnd_body_handle box_underneath;

bool ui_collapsed;

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Stack";
  config->camera_position = (bnd_v3){0, 5, -10};
  config->camera_target = (bnd_v3){0, 2, 10};
}

void scenario_initialize(bnd_world *world) {
  target_age = 122;
}

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
#if defined(BND_DEBUG)
  if (world->age > target_age) {
    return;
  }
  if (world->age == target_age) {
    epa_debug_next_frame(world, box_underneath, box_top, &debug_status);
  }
#endif

  bnd_simulate(world, dt);
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
#if defined(BND_DEBUG)
  if (debug_status.initialized) {
    epa_debug_draw(world, &debug_status, world->config.advanced.epa_tolerance, (bnd_debug_draw_epa_callbacks) { draw_face, draw_normal, draw_support }, NULL);
  }
#endif
}

void scenario_build_ui(bnd_world *world) {
  ui_begin_area("Stack", &ui_collapsed);
  ui_value_int("Target age", &target_age, 120, 122);

  if (debug_status.initialized) {
    if (debug_status.iterations_count_result.error.type == BND_OK) {
      ui_label_int("Iterations count", debug_status.iterations_count_result.value);
      ui_value_int("Iteration to render", &debug_status.target_iteration, 0, debug_status.iterations_count_result.value);
    } else {
      ui_label(debug_status.iterations_count_result.error.message);
    }
  }

  ui_end_area();
}

void scenario_teardown() {
}
