#include "scenario-core.h"
#include "raymath.h"
#include "bnd-core.h"

bnd_mesh_handle cone_mesh;
bnd_body_handle floor_body;
bnd_body_handle cone_body;

bool epa_widget_collapsed;
bool epa_intersection;
uint32_t epa_iteration;
uint32_t epa_iterations_count;

static void draw_epa_face(bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_debug_epa_flags flags, void *user_data) {
  Color color = PINK;
  if (flags & (DEBUG_EPA_FACE_REMOVED | DEBUG_EPA_FACE_NEAREST)) {
    color = ORANGE;
  } else if (flags & DEBUG_EPA_FACE_REMOVED) {
    color = RED;
  } else if (flags & DEBUG_EPA_FACE_NEAREST) {
    color = GREEN;
  }
  color.a = 100;

  DrawTriangle3D(a, b, c, color);
  DrawLine3D(a, b, BLACK);
  DrawLine3D(b, c, BLACK);
  DrawLine3D(c, a, BLACK);
}

static void draw_epa_normal(bnd_v3 origin, bnd_v3 normal, bnd_debug_epa_flags flags, void *user_data) {
  DrawSphere(origin, 0.05f, GREEN);

  Color color = flags & DEBUG_EPA_NORMAL_NEAREST ? GREEN : BLUE;
  draw_arrow(origin, Vector3Scale(normal, 0.75f), color);
}

static void draw_epa_support(bnd_v3 point, void *user_data) {
  DrawSphere(point, 0.05f, ORANGE);
  DrawLine3D((bnd_v3) { 0 }, point, ORANGE);
}

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Features";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
  config->draw_ground = false;
}

void scenario_initialize(bnd_world *world) {
  import_raylib_mesh(world, GenMeshCone(2, 3, 16), &cone_mesh);
}

void scenario_setup_scene(bnd_world *world) {
  floor_body = bnd_add_box_static(world, (bnd_v3) { 10, 1, 10 }).value;

  cone_body = bnd_add_mesh_dynamic(world, 5, cone_mesh).value;

  bnd_set_position(world, cone_body, (bnd_v3) { 0, 7, 0 });
  // bnd_set_rotation(world, cone_body, QuaternionFromEuler(PI * 0.25, PI * 0.25, 0));
}

void scenario_handle_input(bnd_world *world, Camera *camera) { }

void scenario_simulate(bnd_world *world, float dt) {
  bnd_simulate(world, dt);

  if (world->age == 258) {
    // simulation_running = false;
    bnd_result_u32 iterations_count = debug_epa_begin(world, cone_body, floor_body);
    if (iterations_count.error.type == BND_OK) {
      epa_iterations_count = iterations_count.value;
      epa_intersection = true;
    }
  }
}

void scenario_draw_scene(bnd_world *world) {
  if (world->age != 258) {
    return;
  }

  bnd_debug_draw_epa_callbacks callbacks = {
    .draw_face = draw_epa_face,
    .draw_normal = draw_epa_normal,
    .draw_support = draw_epa_support,
  };

  BeginBlendMode(BLEND_ALPHA);
  bool result = debug_epa_iteration(world, cone_body, floor_body, epa_iteration, callbacks, NULL);
  EndBlendMode();

  if (!result) {
    epa_intersection = false;
    epa_iterations_count = 0;
    epa_iteration = 0;
    return;
  }

  if (epa_iteration >= epa_iterations_count && epa_iterations_count > 0) {
    epa_iteration = epa_iterations_count - 1;
  }
}

void scenario_build_ui(bnd_world *world) {
  if (world->age != 258) {
    return;
  }

  if (ui_begin_area("EPA debugger", &epa_widget_collapsed)) {
    ui_label_bool("Intersection", epa_intersection);
    ui_label_int("Iteration", epa_iteration);
    ui_label_int("Iterations", epa_iterations_count);

    if (ui_button("Previous") && epa_iteration > 0) {
      epa_iteration -= 1;
    }
    if (ui_button("Next") && epa_iteration + 1 < epa_iterations_count) {
      epa_iteration += 1;
    }
  }
  ui_end_area();
}

void scenario_teardown() { }
