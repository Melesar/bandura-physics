#include "library_testing.h"
#include "testing.h"

static void expect_single_trigger_event(bnd_world *world, bnd_body_handle body, bnd_body_handle other) {
  bnd_event_enumerator enumerator;
  assert(bnd_event_enumerate(world, body, &enumerator).value);
  assert(bnd_event_next(world, &enumerator));
  assert(enumerator.e.type == BND_EVENT_TRIGGER);
  expect_handle(enumerator.e.trigger.other, other);
  assert(!bnd_event_next(world, &enumerator));
}

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

static void test_trigger_events_skip_collision_response_and_notify_both_bodies(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle trigger = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, trigger, (bnd_v3){0, 0, 1.5f}));
  expect_ok(bnd_set_trigger(world, trigger, true));

  expect_error(bnd_set_trigger(world, dynamic, true), BND_ERROR_INVALID_BODY_TYPE);
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_TRIGGER | BND_EVENT_COLLISION));
  expect_ok(bnd_event_subscribe(world, trigger, BND_EVENT_TRIGGER | BND_EVENT_COLLISION));

  bnd_simulate(world, 0.0f);

  assert(bnd_collisions_count(world) == 0);
  expect_v3_near(bnd_get_position(world, dynamic).value, bnd_v3_zero());
  expect_single_trigger_event(world, dynamic, trigger);
  expect_single_trigger_event(world, trigger, dynamic);

  bnd_teardown(world);
}

static void test_trigger_events_respect_collision_layers(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle trigger = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, trigger, (bnd_v3){0, 0, 1.5f}));
  expect_ok(bnd_set_trigger(world, trigger, true));
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_TRIGGER));
  expect_ok(bnd_set_layers_count(world, 2));
  expect_ok(bnd_set_collision_layer(world, trigger, 1));
  expect_ok(bnd_set_layers_collision(world, 0, 1, false));

  bnd_simulate(world, 0.0f);
  assert(!bnd_event_any(world, dynamic).value);

  expect_ok(bnd_set_layers_collision(world, 0, 1, true));
  bnd_simulate(world, 0.0f);
  expect_single_trigger_event(world, dynamic, trigger);

  bnd_teardown(world);
}

static void test_compound_trigger_emits_one_event_per_body_pair(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_shape shapes[] = {
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {0, 0, -0.5f}, .rotation = bnd_quat_identity() },
    { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {0, 0, 0.5f}, .rotation = bnd_quat_identity() },
  };
  bnd_result_handle result = bnd_add_compound_body_static(world, shapes, 2);
  expect_ok(result.error);
  bnd_body_handle trigger = result.value;
  expect_ok(bnd_set_trigger(world, trigger, true));
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_TRIGGER));

  bnd_simulate(world, 0.0f);

  assert(bnd_collisions_count(world) == 0);
  expect_single_trigger_event(world, dynamic, trigger);

  bnd_teardown(world);
}

static void test_generic_static_plane_can_be_a_trigger(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, dynamic, (bnd_v3){0, 0.5f, 0}));

  bnd_result_handle result = bnd_add_primitive_body(world, BND_BODY_STATIC, BND_PLANE, (bnd_shape) { .plane = { .normal = bnd_v3_up() } }, 0.0f);
  expect_ok(result.error);
  bnd_body_handle trigger = result.value;
  expect_ok(bnd_set_trigger(world, trigger, true));
  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_TRIGGER));

  bnd_simulate(world, 0.0f);

  assert(bnd_collisions_count(world) == 0);
  expect_single_trigger_event(world, dynamic, trigger);

  bnd_teardown(world);
}

void events_tests(void) {
  TESTS_BEGIN("Events")
    TEST(test_collision_events_enumerate_for_each_subscribed_body)
    TEST(test_trigger_events_skip_collision_response_and_notify_both_bodies)
    TEST(test_trigger_events_respect_collision_layers)
    TEST(test_compound_trigger_emits_one_event_per_body_pair)
    TEST(test_generic_static_plane_can_be_a_trigger)
  TESTS_END;
}
