#include "library_testing.h"
#include "testing.h"

static void test_collision_events_enumerate_for_each_subscribed_body(void) {
  bnd_config config = test_config();
  config.memory.events_capacity = 1;
  bnd_world *world = bnd_init(config);
  assert(world != NULL);

  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle first_static = add_static_sphere(world, 1.0f);
  bnd_body_handle second_static = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, first_static, (bnd_v3){0, 0, 1.5f}));
  expect_ok(bnd_set_position(world, second_static, (bnd_v3){0, 0, -1.5f}));

  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_COLLISION));
  expect_ok(bnd_event_subscribe(world, first_static, BND_EVENT_COLLISION));
  bnd_simulate(world, 0.0f);

  assert(bnd_event_any(world, dynamic).value);
  assert(bnd_event_any(world, first_static).value);
  assert(!bnd_event_any(world, second_static).value);

  bnd_event_enumerator enumerator;
  assert(bnd_event_enumerate(world, dynamic, &enumerator).value);
  uint32_t event_count = 0;
  while (bnd_event_next(world, &enumerator)) {
    assert(enumerator.e.type == BND_EVENT_COLLISION);
    expect_handle(enumerator.e.collision.body_a, dynamic);
    event_count += 1;
  }
  assert(event_count == 2);

  expect_ok(bnd_event_unsubscribe_all(world, dynamic));
  expect_ok(bnd_set_position(world, dynamic, (bnd_v3){10, 0, 0}));
  bnd_simulate(world, 0.0f);
  assert(!bnd_event_any(world, dynamic).value);
  assert(!bnd_event_any(world, first_static).value);

  expect_ok(bnd_remove_body(world, dynamic));
  expect_error(bnd_event_subscribe(world, dynamic, BND_EVENT_COLLISION), BND_ERROR_BODY_HANDLE_INVALID);
  bnd_teardown(world);
}

void events_tests(void) {
  TESTS_BEGIN("Events")
    TEST(test_collision_events_enumerate_for_each_subscribed_body)
  TESTS_END;
}
