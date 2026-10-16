#include "library_testing.h"
#include <CUnit/CUnit.h>

static void test_handles_survive_reordering_growth_and_removal(void) {
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 2;
  config.memory.statics_capacity = 1;
  bnd_world *world = test_world_init(config);
  CU_ASSERT_PTR_NOT_NULL_FATAL(world);

  bnd_body_handle first = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle sleeping = add_dynamic_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, first, (bnd_v3){1, 2, 3}));
  expect_ok(bnd_set_velocity(world, first, (bnd_v3){4, 5, 6}));
  expect_ok(bnd_set_position(world, sleeping, (bnd_v3){7, 8, 9}));
  expect_ok(bnd_put_to_sleep(world, sleeping));

  bnd_body_handle inserted = add_dynamic_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, inserted, (bnd_v3){10, 11, 12}));

  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, static_body, (bnd_v3){13, 14, 15}));

  expect_v3_near(bnd_get_position(world, first).value, (bnd_v3){1, 2, 3});
  expect_v3_near(bnd_get_velocity(world, first).value, (bnd_v3){4, 5, 6});
  expect_v3_near(bnd_get_position(world, sleeping).value, (bnd_v3){7, 8, 9});
  expect_v3_near(bnd_get_position(world, inserted).value, (bnd_v3){10, 11, 12});
  expect_v3_near(bnd_get_position(world, static_body).value, (bnd_v3){13, 14, 15});

  expect_ok(bnd_remove_body(world, first));
  expect_error(bnd_handle_valid(world, first), BND_ERROR_BODY_HANDLE_INVALID);
  expect_v3_near(bnd_get_position(world, sleeping).value, (bnd_v3){7, 8, 9});
  expect_v3_near(bnd_get_position(world, inserted).value, (bnd_v3){10, 11, 12});
  expect_v3_near(bnd_get_position(world, static_body).value, (bnd_v3){13, 14, 15});

  bnd_body_handle reused = add_dynamic_sphere(world, 1.0f);
  CU_ASSERT(reused.index == first.index);
  expect_error(bnd_handle_valid(world, first), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_handle_valid(world, reused));

  test_world_teardown(world);
}

static void test_sleep_wake_keeps_the_awake_prefix_consistent(void) {
  bnd_world *world = test_world();
  bnd_body_handle first = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle second = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle third = add_dynamic_sphere(world, 1.0f);

  expect_ok(bnd_put_to_sleep(world, second));
  expect_ok(bnd_put_to_sleep(world, third));
  CU_ASSERT(bnd_awake_count(world) == 1);

  expect_ok(bnd_awaken_body(world, second));
  CU_ASSERT(bnd_awake_count(world) == 2);
  expect_error(bnd_awaken_body(world, first), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_awaken_body(world, third));
  CU_ASSERT(bnd_awake_count(world) == 3);

  test_world_teardown(world);
}

static void test_reset_invalidates_handles_and_enumerators(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);

  bnd_body_enumerator enumerator;
  bnd_enumerate_bodies_typed(world, BND_BODY_DYNAMIC, &enumerator);
  CU_ASSERT_FATAL(bnd_body_next_typed(world, &enumerator));
  bnd_body_handle enumerated = enumerator.handle;
  CU_ASSERT(enumerated.index == dynamic.index);

  add_dynamic_sphere(world, 1.0f);
  CU_ASSERT(!bnd_body_next_typed(world, &enumerator));

  bnd_reset_world(world);
  CU_ASSERT(bnd_body_count(world, BND_BODY_DYNAMIC) == 0);
  CU_ASSERT(bnd_body_count(world, BND_BODY_STATIC) == 0);
  CU_ASSERT(bnd_awake_count(world) == 0);
  expect_error(bnd_handle_valid(world, dynamic), BND_ERROR_BODY_HANDLE_INVALID);
  expect_error(bnd_handle_valid(world, static_body), BND_ERROR_BODY_HANDLE_INVALID);

  bnd_body_handle replacement = add_dynamic_sphere(world, 1.0f);
  CU_ASSERT(replacement.index == dynamic.index);
  expect_error(bnd_handle_valid(world, dynamic), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_handle_valid(world, replacement));

  test_world_teardown(world);
}

static void test_static_trigger_survives_growth_and_reordering(void) {
  bnd_config config = test_config();
  config.memory.statics_capacity = 1;
  bnd_world *world = test_world_init(config);
  CU_ASSERT_PTR_NOT_NULL_FATAL(world);

  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle first = add_static_sphere(world, 1.0f);
  bnd_body_handle second = add_static_sphere(world, 1.0f);
  bnd_body_handle trigger = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, first, (bnd_v3){10, 0, 0}));
  expect_ok(bnd_set_position(world, second, (bnd_v3){20, 0, 0}));
  expect_ok(bnd_set_trigger(world, trigger, true));
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_TRIGGER));

  expect_ok(bnd_remove_body(world, first));
  bnd_simulate(world, 0.0f);

  CU_ASSERT(bnd_collisions_count(world) == 0);
  CU_ASSERT(bnd_event_any(world, dynamic).value);
  bnd_event_enumerator enumerator;
  CU_ASSERT_FATAL(bnd_event_enumerate(world, dynamic, &enumerator).value);
  CU_ASSERT_FATAL(bnd_event_next(world, &enumerator));
  CU_ASSERT(enumerator.e.type == BND_EVENT_TRIGGER);
  expect_handle(enumerator.e.trigger.other, trigger);
  CU_ASSERT(!bnd_event_next(world, &enumerator));

  test_world_teardown(world);
}

CU_ErrorCode register_world_tests(void) {
  CU_TestInfo tests[] = {
    {"test_handles_survive_reordering_growth_and_removal", test_handles_survive_reordering_growth_and_removal},
    {"test_sleep_wake_keeps_the_awake_prefix_consistent", test_sleep_wake_keeps_the_awake_prefix_consistent},
    {"test_reset_invalidates_handles_and_enumerators", test_reset_invalidates_handles_and_enumerators},
    {"test_static_trigger_survives_growth_and_reordering", test_static_trigger_survives_growth_and_reordering},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"World lifecycle", NULL, NULL, NULL, test_world_cleanup, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
