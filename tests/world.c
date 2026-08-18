#include "library_testing.h"
#include "testing.h"

static void test_handles_survive_reordering_growth_and_removal(void) {
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 2;
  config.memory.statics_capacity = 1;
  bnd_world *world = bnd_init(config);
  assert(world != NULL);

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
  assert(reused.index == first.index);
  expect_error(bnd_handle_valid(world, first), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_handle_valid(world, reused));

  bnd_teardown(world);
}

static void test_sleep_wake_keeps_the_awake_prefix_consistent(void) {
  bnd_world *world = test_world();
  bnd_body_handle first = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle second = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle third = add_dynamic_sphere(world, 1.0f);

  expect_ok(bnd_put_to_sleep(world, second));
  expect_ok(bnd_put_to_sleep(world, third));
  assert(bnd_awake_count(world) == 1);

  expect_ok(bnd_awaken_body(world, second));
  assert(bnd_awake_count(world) == 2);
  expect_error(bnd_awaken_body(world, first), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_awaken_body(world, third));
  assert(bnd_awake_count(world) == 3);

  bnd_teardown(world);
}

static void test_reset_invalidates_handles_and_enumerators(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);

  bnd_body_enumerator enumerator;
  bnd_enumerate_bodies_typed(world, BND_BODY_DYNAMIC, &enumerator);
  assert(bnd_body_next_typed(world, &enumerator));
  bnd_body_handle enumerated = enumerator.handle;
  assert(enumerated.index == dynamic.index);

  add_dynamic_sphere(world, 1.0f);
  assert(!bnd_body_next_typed(world, &enumerator));

  bnd_reset_world(world);
  assert(bnd_body_count(world, BND_BODY_DYNAMIC) == 0);
  assert(bnd_body_count(world, BND_BODY_STATIC) == 0);
  assert(bnd_awake_count(world) == 0);
  expect_error(bnd_handle_valid(world, dynamic), BND_ERROR_BODY_HANDLE_INVALID);
  expect_error(bnd_handle_valid(world, static_body), BND_ERROR_BODY_HANDLE_INVALID);

  bnd_body_handle replacement = add_dynamic_sphere(world, 1.0f);
  assert(replacement.index == dynamic.index);
  expect_error(bnd_handle_valid(world, dynamic), BND_ERROR_BODY_HANDLE_INVALID);
  expect_ok(bnd_handle_valid(world, replacement));

  bnd_teardown(world);
}

void world_tests(void) {
  TESTS_BEGIN("World lifecycle")
    TEST(test_handles_survive_reordering_growth_and_removal)
    TEST(test_sleep_wake_keeps_the_awake_prefix_consistent)
    TEST(test_reset_invalidates_handles_and_enumerators)
  TESTS_END;
}
