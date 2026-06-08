#include "scenario-core.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>

collision_test_suite *tests;

bool tests_collapsed;
bool collapsed;
int pair_index, prev_pair_index;
int case_index, prev_case_index;

char buffer[1024];
bool *test_results;

static const char *shape_name(bnd_body_shape shape) {
  switch (shape.type) {
    case BND_BOX: return "Box";
    case BND_SPHERE: return "Sphere";
    case BND_CAPSULE: return "Capsule";
    default: return "Unknown";
  }
}

static bnd_body_handle update_pair(bnd_world *world, count_t pair, count_t test_case_index) {
  bnd_reset_world(world);

  bnd_body_handle handles[2];
  collision_tests_pair_spawn(world, &tests->pairs[pair], handles);

  const collision_test_case *test_case = &tests->cases[pair * tests->cases_per_pair + test_case_index];

  bnd_set_position(world, handles[0], test_case->position_a);
  bnd_set_position(world, handles[1], test_case->position_b);

  bnd_set_rotation(world, handles[0], test_case->rotation_a);
  bnd_set_rotation(world, handles[1], test_case->rotation_b);

  return handles[0];
}

void scenario_configure(program_config *config, bnd_config *physics) {
  config->window_title = "Collisions";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
  config->draw_ground = false;

  physics->simulation.gravity = bnd_v3_zero();
  physics->advanced.resolution_attempts_factor = 0;
}

void scenario_initialize(bnd_world *world) {
  tests = collision_tests_load();
  test_results = calloc(tests->num_pairs * tests->cases_per_pair, sizeof(bool));
}

void scenario_setup_scene(bnd_world *world) {
  for (count_t i = 0; i < tests->num_pairs; ++i) {
    for (count_t j = 0; j < tests->cases_per_pair; ++j) {
      bnd_body_handle body = update_pair(world, i, j);

      bnd_event_subscribe(world, body, BND_EVENT_COLLISION);
      bnd_simulate(world, 0.01);

      test_results[i * tests->cases_per_pair + j] = bnd_event_any(world, body).value == tests->cases[i * tests->cases_per_pair + j].intersection;
      bnd_event_unsubscribe_all(world, body);
    }
  }

  pair_index = prev_pair_index = 0;
  case_index = prev_pair_index = 0;

  update_pair(world, 0, 0);
}

void scenario_simulate(bnd_world *world, float dt) {
  if (pair_index >= tests->num_pairs || case_index >= tests->cases_per_pair) {
    return;
  }

  if (pair_index != prev_pair_index || case_index != prev_case_index) {
    update_pair(world, pair_index, case_index);

    prev_case_index = case_index;
    prev_pair_index = pair_index;
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

  ui_begin_area("Tests", &tests_collapsed);

  const collision_test_pair *pair = &tests->pairs[pair_index];

  sprintf(buffer, "%s vs %s", shape_name(pair->a), shape_name(pair->b));
  ui_label(buffer);

  const float item_size = 15;
  count_t results_per_column = tests->cases_per_pair / 3;
  count_t results_per_row = (count_t) sqrt(results_per_column);

  count_t pair_results_start = pair_index * tests->cases_per_pair;
  CLAY(CLAY_ID("Results"), {.layout = { .layoutDirection = CLAY_LEFT_TO_RIGHT, .childGap = 10 }}) {
    for (count_t col = 0; col < 3; ++col) {
      count_t column_results_start = pair_results_start + col * results_per_column;

      CLAY(CLAY_IDI("ResultsColumn", col), {
        .layout = {
          .childGap = 3,
          .layoutDirection = CLAY_TOP_TO_BOTTOM
        }
      }) {
        for (count_t i = 0; i < results_per_row; ++i) {
          CLAY(CLAY_IDI("ResultsRow", col * results_per_column + i), { .layout = { .childGap = 3 } }) {
            for (count_t j = 0; j < results_per_row; ++j) {
              count_t result_index = column_results_start + i * results_per_row + j;

              Clay_Color passed = { 0, 255, 0, 255 };
              Clay_Color failed = { 255, 0, 0, 255 };
              Clay_Color hovered = { 0, 0, 255, 255 };

              CLAY_AUTO_ID({
                .layout = { .sizing = { .width = CLAY_SIZING_FIXED(item_size), .height = CLAY_SIZING_FIXED(item_size) } },
                .backgroundColor = Clay_Hovered() ? hovered : (test_results[result_index] ? passed : failed)
              }) {
                Clay_PointerDataInteractionState pointer_state = Clay_GetPointerState().state;
                bool is_hovering = Clay_Hovered();

                if (is_hovering && pointer_state == CLAY_POINTER_DATA_RELEASED_THIS_FRAME) {
                  case_index = result_index;
                  update_pair(world, pair_index, case_index);
                  prev_case_index = case_index;
                  prev_pair_index = pair_index;
                }
              }
            }
          }
        }
      }
    }
  }

  ui_end_area();
}

void scenario_teardown() {
  collision_tests_free(tests);
  free(test_results);
}
