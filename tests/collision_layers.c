#include "library_testing.h"
#include <CUnit/CUnit.h>

static void test_world_starts_with_one_collision_layer(void) {
  bnd_world *world = test_world();

  CU_ASSERT(bnd_get_layers_count(world) == 1);
  CU_ASSERT(bnd_get_all_layers_mask(world) == UINT64_C(1));
  CU_ASSERT(bnd_get_layers_collision(world, 0, 0));

  test_world_teardown(world);
}

static void test_adding_collision_layers_enables_collisions_with_every_layer(void) {
  bnd_world *world = test_world();
  expect_ok(bnd_set_layers_count(world, 2));
  expect_ok(bnd_set_layers_collision(world, 0, 1, false));

  expect_ok(bnd_set_layers_count(world, 4));

  CU_ASSERT(bnd_get_layers_count(world) == 4);
  CU_ASSERT(bnd_get_all_layers_mask(world) == UINT64_C(0xf));
  CU_ASSERT(!bnd_get_layers_collision(world, 0, 1));
  for (bnd_collision_layer layer = 0; layer < 4; ++layer) {
    CU_ASSERT(bnd_get_layers_collision(world, layer, 2));
    CU_ASSERT(bnd_get_layers_collision(world, 2, layer));
    CU_ASSERT(bnd_get_layers_collision(world, layer, 3));
    CU_ASSERT(bnd_get_layers_collision(world, 3, layer));
  }

  test_world_teardown(world);
}

static void test_setting_collision_between_layers_is_symmetrical(void) {
  bnd_world *world = test_world();
  expect_ok(bnd_set_layers_count(world, 3));

  expect_ok(bnd_set_layers_collision(world, 0, 2, false));
  CU_ASSERT(!bnd_get_layers_collision(world, 0, 2));
  CU_ASSERT(!bnd_get_layers_collision(world, 2, 0));

  expect_ok(bnd_set_layers_collision(world, 2, 0, true));
  CU_ASSERT(bnd_get_layers_collision(world, 0, 2));
  CU_ASSERT(bnd_get_layers_collision(world, 2, 0));

  test_world_teardown(world);
}

static void test_collision_layer_count_rejects_zero_and_values_above_64(void) {
  bnd_world *world = test_world();

  expect_error(bnd_set_layers_count(world, 0), BND_ERROR_INVALID_INPUT);
  expect_error(bnd_set_layers_count(world, 65), BND_ERROR_INVALID_INPUT);
  CU_ASSERT(bnd_get_layers_count(world) == 1);

  test_world_teardown(world);
}

static void test_setting_collision_for_undefined_layers_is_rejected(void) {
  bnd_world *world = test_world();
  expect_ok(bnd_set_layers_count(world, 2));

  expect_error(bnd_set_layers_collision(world, 0, 2, false), BND_ERROR_INVALID_COLLISION_LAYER);
  expect_error(bnd_set_layers_collision(world, 2, 1, false), BND_ERROR_INVALID_COLLISION_LAYER);
  CU_ASSERT(bnd_get_layers_collision(world, 0, 1));
  CU_ASSERT(!bnd_get_layers_collision(world, 0, 2));

  test_world_teardown(world);
}

static void test_collision_response_respects_collision_layers(void) {
  bnd_world *world = test_world();
  expect_ok(bnd_set_layers_count(world, 2));

  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, static_body, (bnd_v3){0, 0, 1.5f}));
  expect_ok(bnd_set_collision_layer(world, static_body, 1));
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_COLLISION));

  expect_ok(bnd_set_layers_collision(world, 0, 1, false));
  bnd_simulate(world, 0.008f);
  CU_ASSERT(bnd_collisions_count(world) == 0);
  CU_ASSERT(!bnd_event_any(world, dynamic).value);
  expect_v3_near(bnd_get_position(world, dynamic).value, bnd_v3_zero());

  expect_ok(bnd_set_layers_collision(world, 0, 1, true));
  bnd_simulate(world, 0.008f);
  CU_ASSERT(bnd_collisions_count(world) > 0);
  CU_ASSERT(bnd_event_any(world, dynamic).value);
  CU_ASSERT(bnd_get_position(world, dynamic).value.z < 0.0f);

  test_world_teardown(world);
}

CU_ErrorCode register_collision_layers_tests(void) {
  CU_TestInfo tests[] = {
    {"test_world_starts_with_one_collision_layer", test_world_starts_with_one_collision_layer},
    {"test_adding_collision_layers_enables_collisions_with_every_layer", test_adding_collision_layers_enables_collisions_with_every_layer},
    {"test_setting_collision_between_layers_is_symmetrical", test_setting_collision_between_layers_is_symmetrical},
    {"test_collision_layer_count_rejects_zero_and_values_above_64", test_collision_layer_count_rejects_zero_and_values_above_64},
    {"test_setting_collision_for_undefined_layers_is_rejected", test_setting_collision_for_undefined_layers_is_rejected},
    {"test_collision_response_respects_collision_layers", test_collision_response_respects_collision_layers},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Collision layers", NULL, NULL, NULL, test_world_cleanup, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
