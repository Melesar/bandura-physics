#include "library_testing.h"
#include "bnd-core.h"
#include <CUnit/CUnit.h>

#define expect_single_trigger_event(world, body, other) \
  expect_single_trigger_event_at((world), (body), (other), #body, __FILE__, __LINE__)

static void expect_single_trigger_event_at(bnd_world *world, bnd_body_handle body, bnd_body_handle other,
                                           const char *body_expression, const char *file, unsigned int line) {
  char message[192];
  bnd_event_enumerator enumerator;
  bool enumerated = bnd_event_enumerate(world, body, &enumerator).value;
  snprintf(message, sizeof(message), "trigger event for %s: enumeration failed", body_expression);
  CU_assertImplementation(enumerated, line, message, file, "", CU_FALSE);
  if (!enumerated) return;
  bool has_event = bnd_event_next(world, &enumerator);
  snprintf(message, sizeof(message), "trigger event for %s: no event", body_expression);
  CU_assertImplementation(has_event, line, message, file, "", CU_FALSE);
  if (!has_event) return;
  snprintf(message, sizeof(message), "trigger event for %s: expected type %d, got %d", body_expression,
           (int)BND_EVENT_TRIGGER, (int)enumerator.e.type);
  bool correct_type = enumerator.e.type == BND_EVENT_TRIGGER;
  CU_assertImplementation(correct_type, line, message, file, "", CU_FALSE);
  if (!correct_type) return;
  expect_handle_at(enumerator.e.trigger.other, other, "trigger.other", file, line);
  bool extra_event = bnd_event_next(world, &enumerator);
  snprintf(message, sizeof(message), "trigger event for %s: more than one event", body_expression);
  CU_assertImplementation(!extra_event, line, message, file, "", CU_FALSE);
}

static void test_collision_events_enumerate_for_each_subscribed_body(void) {
  bnd_config config = test_config();
  config.memory.events_capacity = 1;
  bnd_world *world = test_world_init(config);
  CU_ASSERT_PTR_NOT_NULL_FATAL(world);

  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle first_static = add_static_sphere(world, 1.0f);
  bnd_body_handle second_static = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, first_static, (bnd_v3){0, 0, 1.5f}));
  expect_ok(bnd_set_position(world, second_static, (bnd_v3){0, 0, -1.5f}));

  expect_ok(bnd_event_subscribe(world, dynamic, BND_EVENT_COLLISION));
  expect_ok(bnd_event_subscribe(world, first_static, BND_EVENT_COLLISION));
  bnd_simulate(world, 0.0f);

  CU_ASSERT(bnd_event_any(world, dynamic).value);
  CU_ASSERT(bnd_event_any(world, first_static).value);
  CU_ASSERT(!bnd_event_any(world, second_static).value);

  bnd_event_enumerator enumerator;
  CU_ASSERT_FATAL(bnd_event_enumerate(world, dynamic, &enumerator).value);
  uint32_t event_count = 0;
  while (bnd_event_next(world, &enumerator)) {
    CU_ASSERT(enumerator.e.type == BND_EVENT_COLLISION);
    expect_handle(enumerator.e.collision.body_a, dynamic);
    event_count += 1;
  }
  CU_ASSERT(event_count == 2);

  expect_ok(bnd_event_unsubscribe_all(world, dynamic));
  expect_ok(bnd_set_position(world, dynamic, (bnd_v3){10, 0, 0}));
  bnd_simulate(world, 0.0f);
  CU_ASSERT(!bnd_event_any(world, dynamic).value);
  CU_ASSERT(!bnd_event_any(world, first_static).value);

  expect_ok(bnd_remove_body(world, dynamic));
  expect_error(bnd_event_subscribe(world, dynamic, BND_EVENT_COLLISION), BND_ERROR_BODY_HANDLE_INVALID);
  test_world_teardown(world);
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

  bnd_simulate(world, 0.005f);

  CU_ASSERT(bnd_collisions_count(world) == 0);
  expect_v3_near(bnd_get_position(world, dynamic).value, bnd_v3_zero());
  expect_single_trigger_event(world, dynamic, trigger);
  expect_single_trigger_event(world, trigger, dynamic);

  test_world_teardown(world);
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
  CU_ASSERT(!bnd_event_any(world, dynamic).value);

  expect_ok(bnd_set_layers_collision(world, 0, 1, true));
  bnd_simulate(world, 0.0f);
  expect_single_trigger_event(world, dynamic, trigger);

  test_world_teardown(world);
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

  CU_ASSERT(bnd_collisions_count(world) == 0);
  expect_single_trigger_event(world, dynamic, trigger);

  test_world_teardown(world);
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

  CU_ASSERT(bnd_collisions_count(world) == 0);
  expect_single_trigger_event(world, dynamic, trigger);

  test_world_teardown(world);
}

#define expect_single_event_type(world, body, type) \
  expect_single_event_type_at((world), (body), (type), #body, #type, __FILE__, __LINE__)

static void expect_single_event_type_at(bnd_world *world, bnd_body_handle body, bnd_event_type type,
                                        const char *body_expression, const char *type_expression,
                                        const char *file, unsigned int line) {
  char message[192];
  bnd_event_enumerator enumerator;
  bool enumerated = bnd_event_enumerate(world, body, &enumerator).value;
  snprintf(message, sizeof(message), "%s for %s: event enumeration failed", type_expression, body_expression);
  CU_assertImplementation(enumerated, line, message, file, "", CU_FALSE);
  if (!enumerated) return;
  bool has_event = bnd_event_next(world, &enumerator);
  snprintf(message, sizeof(message), "%s for %s: no event", type_expression, body_expression);
  CU_assertImplementation(has_event, line, message, file, "", CU_FALSE);
  if (!has_event) return;
  snprintf(message, sizeof(message), "%s for %s: expected type %d, got %d", type_expression,
           body_expression, (int)type, (int)enumerator.e.type);
  CU_assertImplementation(enumerator.e.type == type, line, message, file, "", CU_FALSE);
  bool extra_event = bnd_event_next(world, &enumerator);
  snprintf(message, sizeof(message), "%s for %s: more than one event", type_expression, body_expression);
  CU_assertImplementation(!extra_event, line, message, file, "", CU_FALSE);
}

typedef struct {
  bnd_world *world;
  bnd_body_handle dynamic;
  bnd_body_handle other;
} finish_fixture;

static finish_fixture finish_fixture_create(bool compound, bool trigger) {
  finish_fixture fixture = { .world = test_world() };
  fixture.dynamic = add_dynamic_sphere(fixture.world, 1.0f);
  if (compound) {
    bnd_body_shape shapes[] = {
      { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {0, 0, -0.5f}, .rotation = bnd_quat_identity() },
      { .type = BND_SPHERE, .value.sphere = { .radius = 1.0f }, .offset = {0, 0, 0.5f}, .rotation = bnd_quat_identity() },
    };
    bnd_result_handle result = bnd_add_compound_body_static(fixture.world, shapes, 2);
    expect_ok(result.error);
    fixture.other = result.value;
  } else {
    fixture.other = add_static_sphere(fixture.world, 1.0f);
  }
  expect_ok(bnd_set_trigger(fixture.world, fixture.other, trigger));
  bnd_event_type type = trigger ? BND_EVENT_TRIGGER_FINISH : BND_EVENT_COLLISION_FINISH;
  expect_ok(bnd_event_subscribe(fixture.world, fixture.dynamic, type));
  expect_ok(bnd_event_subscribe(fixture.world, fixture.other, type));
  return fixture;
}

#define expect_contact_counts(fixture, broad, narrow) \
  expect_contact_counts_at((fixture), (broad), (narrow), __FILE__, __LINE__)

static void expect_contact_counts_at(finish_fixture fixture, count_t broad, count_t narrow,
                                     const char *file, unsigned int line) {
  const broad_contacts_set *set = &fixture.world->contacts.statics;
  count_t broad_count = 0;
  count_t narrow_count = 0;
  for (count_t root = set->first; root != UINT32_MAX; root = set->contacts[root].next_body) {
    for (count_t index = root; index != UINT32_MAX; index = set->contacts[index].next) {
      const broad_phase_contact *contact = &set->contacts[index];
      broad_count += 1;
      narrow_count += contact->manifold.count > 0;
    }
  }
  char message[192];
  snprintf(message, sizeof(message), "broad contacts: expected %u, got %u", broad, broad_count);
  CU_assertImplementation(broad_count == broad, line, message, file, "", CU_TRUE);
  snprintf(message, sizeof(message), "contacts with manifold.count > 0: expected %u, got %u", narrow, narrow_count);
  CU_assertImplementation(narrow_count == narrow, line, message, file, "", CU_TRUE);
}

#define expect_no_finish_events(fixture) do { \
  CU_ASSERT(!bnd_event_any((fixture).world, (fixture).dynamic).value); \
  CU_ASSERT(!bnd_event_any((fixture).world, (fixture).other).value); \
} while (0)

static void test_simple_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining(void) {
  finish_fixture fixture = finish_fixture_create(false, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 1);
  expect_no_finish_events(fixture);

  // AABBs still overlap, but no sphere pair intersects.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_COLLISION_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_COLLISION_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_no_finish_events(fixture);

  // Removing the remaining empty broad contacts must not finish again.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_simple_collision_finish_when_broad_contacts_disappear_after_touching(void) {
  finish_fixture fixture = finish_fixture_create(false, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 1);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_COLLISION_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_COLLISION_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_simple_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching(void) {
  finish_fixture fixture = finish_fixture_create(false, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining(void) {
  finish_fixture fixture = finish_fixture_create(true, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 2);
  expect_no_finish_events(fixture);

  // One manifold becomes empty while both broad contacts remain.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.0f, 0.5f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 1);
  expect_no_finish_events(fixture);

  // AABBs still overlap, but no sphere pair intersects.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.5f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_COLLISION_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_COLLISION_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_no_finish_events(fixture);

  // Removing the remaining empty broad contacts must not finish again.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_collision_finish_when_broad_contacts_disappear_after_touching(void) {
  finish_fixture fixture = finish_fixture_create(true, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 2);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_COLLISION_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_COLLISION_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching(void) {
  finish_fixture fixture = finish_fixture_create(true, false);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_simple_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining(void) {
  finish_fixture fixture = finish_fixture_create(false, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 1);
  expect_no_finish_events(fixture);

  // AABBs still overlap, but no sphere pair intersects.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_TRIGGER_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_TRIGGER_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_no_finish_events(fixture);

  // Removing the remaining empty broad contacts must not finish again.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_simple_trigger_finish_when_broad_contacts_disappear_after_touching(void) {
  finish_fixture fixture = finish_fixture_create(false, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 1);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_TRIGGER_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_TRIGGER_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_simple_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching(void) {
  finish_fixture fixture = finish_fixture_create(false, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 1, 0);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining(void) {
  finish_fixture fixture = finish_fixture_create(true, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 2);
  expect_no_finish_events(fixture);

  // One manifold becomes empty while both broad contacts remain.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.0f, 0.5f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 1);
  expect_no_finish_events(fixture);

  // AABBs still overlap, but no sphere pair intersects.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.5f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_TRIGGER_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_TRIGGER_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_no_finish_events(fixture);

  // Removing the remaining empty broad contacts must not finish again.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_trigger_finish_when_broad_contacts_disappear_after_touching(void) {
  finish_fixture fixture = finish_fixture_create(true, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 2);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_single_event_type(fixture.world, fixture.dynamic, BND_EVENT_TRIGGER_FINISH);
  expect_single_event_type(fixture.world, fixture.other, BND_EVENT_TRIGGER_FINISH);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

static void test_compound_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching(void) {
  finish_fixture fixture = finish_fixture_create(true, true);
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){1.5f, 1.5f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 2, 0);
  expect_no_finish_events(fixture);

  // Remove the entire body pair from the broad phase.
  expect_ok(bnd_set_position(fixture.world, fixture.other, (bnd_v3){4.0f, 0.0f, 0.0f}));
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);

  // The same separated state must not emit another finish event.
  bnd_simulate(fixture.world, 0.0f);
  expect_contact_counts(fixture, 0, 0);
  expect_no_finish_events(fixture);
  test_world_teardown(fixture.world);
}

CU_ErrorCode register_events_tests(void) {
  CU_TestInfo tests[] = {
    {"test_collision_events_enumerate_for_each_subscribed_body", test_collision_events_enumerate_for_each_subscribed_body},
    {"test_trigger_events_skip_collision_response_and_notify_both_bodies", test_trigger_events_skip_collision_response_and_notify_both_bodies},
    {"test_trigger_events_respect_collision_layers", test_trigger_events_respect_collision_layers},
    {"test_compound_trigger_emits_one_event_per_body_pair", test_compound_trigger_emits_one_event_per_body_pair},
    {"test_generic_static_plane_can_be_a_trigger", test_generic_static_plane_can_be_a_trigger},
    {"test_simple_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining", test_simple_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining},
    {"test_simple_collision_finish_when_broad_contacts_disappear_after_touching", test_simple_collision_finish_when_broad_contacts_disappear_after_touching},
    {"test_simple_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching", test_simple_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching},
    {"test_compound_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining", test_compound_collision_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining},
    {"test_compound_collision_finish_when_broad_contacts_disappear_after_touching", test_compound_collision_finish_when_broad_contacts_disappear_after_touching},
    {"test_compound_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching", test_compound_collision_finish_is_not_emitted_when_broad_contacts_disappear_without_touching},
    {"test_simple_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining", test_simple_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining},
    {"test_simple_trigger_finish_when_broad_contacts_disappear_after_touching", test_simple_trigger_finish_when_broad_contacts_disappear_after_touching},
    {"test_simple_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching", test_simple_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching},
    {"test_compound_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining", test_compound_trigger_finish_when_narrow_contacts_disappear_with_broad_contacts_remaining},
    {"test_compound_trigger_finish_when_broad_contacts_disappear_after_touching", test_compound_trigger_finish_when_broad_contacts_disappear_after_touching},
    {"test_compound_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching", test_compound_trigger_finish_is_not_emitted_when_broad_contacts_disappear_without_touching},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Events", NULL, NULL, NULL, test_world_cleanup, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
