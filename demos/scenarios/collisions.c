#include "scenario-core.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "raylib.h"

collision_test_suite *tests;

bool collapsed;
int pair_index, prev_pair_index;
int case_index, prev_case_index;

static void update_pair(bnd_world *world) {
  bnd_reset_world(world);

  bnd_body_handle handles[2];
  collision_tests_pair_spawn(world, &tests->pairs[pair_index], handles);

  const collision_test_case *test_case = &tests->cases[pair_index * tests->cases_per_pair + case_index];

  bnd_set_position(world, handles[0], test_case->position_a);
  bnd_set_position(world, handles[1], test_case->position_b);

  bnd_set_rotation(world, handles[0], test_case->rotation_a);
  bnd_set_rotation(world, handles[1], test_case->rotation_b);

  prev_case_index = case_index;
  prev_pair_index = pair_index;
}

void scenario_configure(program_config *config, bnd_config *physics) {
  config->window_title = "Collisions";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };

  physics->simulation.gravity = bnd_v3_zero();
}

void scenario_initialize(bnd_world *world) {
  tests = collision_tests_load();
}

void scenario_setup_scene(bnd_world *world) {
  update_pair(world);
}

void scenario_simulate(bnd_world *world, float dt) {
  if (pair_index != prev_pair_index || case_index != prev_case_index) {
    update_pair(world);
  }

  bnd_simulate(world, dt);
}

void scenario_handle_input(bnd_world *world, Camera *cam) {
}

void scenario_draw_scene(bnd_world *world) {
}

void scenario_build_ui(bnd_world *world) {
  ui_begin_area("Collisions", &collapsed);
  ui_value_int("Pair", &pair_index, 0, tests->num_pairs - 1);
  ui_value_int("Case", &case_index, 0, tests->cases_per_pair - 1);

  count_t case_id = pair_index * tests->cases_per_pair + case_index;
  ui_label_v3("Position A", tests->cases[case_id].position_a);
  ui_label_v3("Position B", tests->cases[case_id].position_b);
  ui_end_area();
}

void scenario_teardown() {
  collision_tests_free(tests);
}
