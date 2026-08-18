#include "library_testing.h"
#include "testing.h"

static void test_raycast_closest_hits_primitives_compounds_and_planes(void) {
  bnd_world *world = test_world();
  bnd_ray ray = { .origin = bnd_v3_zero(), .direction = bnd_v3_forward(), .max_distance = 10.0f };

  bnd_body_handle sphere = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, sphere, (bnd_v3){0, 0, 3}));

  bnd_raycast_hit hit;
  assert(bnd_raycast_closest(world, ray, &hit));
  expect_handle(hit.body, sphere);
  expect_float_near(hit.distance, 2.0f);
  expect_v3_near(hit.point, (bnd_v3){0, 0, 2});
  expect_v3_near(hit.normal, (bnd_v3){0, 0, -1});

  bnd_body_shape shape = {
    .type = BND_BOX,
    .value.box = { .size = {2, 2, 2} },
    .offset = {0, 0, 6},
    .rotation = bnd_quat_identity(),
  };
  bnd_result_handle compound = bnd_add_compound_body_static(world, &shape, 1);
  expect_ok(compound.error);

  ray.max_distance = 10.0f;
  assert(bnd_raycast_closest(world, ray, &hit));
  expect_handle(hit.body, sphere);

  expect_ok(bnd_remove_body(world, sphere));
  assert(bnd_raycast_closest(world, ray, &hit));
  expect_handle(hit.body, compound.value);
  expect_float_near(hit.distance, 5.0f);

  bnd_teardown(world);

  world = test_world();
  bnd_result_handle capsule = bnd_add_capsule_static(world, 1.0f, 2.0f);
  expect_ok(capsule.error);
  expect_ok(bnd_set_position(world, capsule.value, (bnd_v3){0, 0, 5}));
  assert(bnd_raycast_closest(world, ray, &hit));
  expect_handle(hit.body, capsule.value);
  expect_float_near(hit.distance, 4.0f);

  bnd_teardown(world);

  world = test_world();
  expect_ok(bnd_add_plane(world, (bnd_v3){0, 3, 0}, bnd_v3_up()));
  ray = (bnd_ray){ .origin = {0, 4, 0}, .direction = {0, -1, 0}, .max_distance = 2.0f };
  assert(bnd_raycast_closest(world, ray, &hit));
  expect_float_near(hit.distance, 1.0f);
  expect_v3_near(hit.point, (bnd_v3){0, 3, 0});
  expect_v3_near(hit.normal, bnd_v3_up());

  bnd_teardown(world);
}

static void test_raycast_multiple_respects_limits_and_body_types(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, dynamic, (bnd_v3){0, 0, 3}));
  expect_ok(bnd_set_position(world, static_body, (bnd_v3){0, 0, 6}));

  bnd_ray ray = { .origin = bnd_v3_zero(), .direction = bnd_v3_forward(), .max_distance = 10.0f };
  bnd_raycast_hit hits[2];
  assert(bnd_raycast_multiple(world, ray, hits, 0) == 0);
  assert(bnd_raycast_multiple(world, ray, hits, 1) == 1);
  expect_handle(hits[0].body, dynamic);
  assert(bnd_raycast_multiple(world, ray, hits, 2) == 2);
  expect_handle(hits[0].body, dynamic);
  expect_handle(hits[1].body, static_body);

  ray.max_distance = 1.0f;
  assert(bnd_raycast_multiple(world, ray, hits, 2) == 0);
  bnd_teardown(world);
}

static void test_overlap_covers_dynamic_and_static_bodies_with_distinct_capacities(void) {
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 8;
  config.memory.statics_capacity = 1;
  bnd_world *world = bnd_init(config);
  assert(world != NULL);

  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  bnd_body_handle overlaps[2];

  assert(bnd_overlap(world, bnd_v3_zero(), 1.0f, overlaps, 0) == 0);
  assert(bnd_overlap(world, bnd_v3_zero(), 1.0f, overlaps, 1) == 1);
  expect_handle(overlaps[0], dynamic);
  assert(bnd_overlap(world, bnd_v3_zero(), 1.0f, overlaps, 2) == 2);
  expect_handle(overlaps[0], dynamic);
  expect_handle(overlaps[1], static_body);

  assert(bnd_overlap(world, (bnd_v3){10, 0, 0}, 0.5f, overlaps, 2) == 0);
  bnd_teardown(world);
}

void queries_tests(void) {
  TESTS_BEGIN("Spatial queries")
    TEST(test_raycast_closest_hits_primitives_compounds_and_planes)
    TEST(test_raycast_multiple_respects_limits_and_body_types)
    TEST(test_overlap_covers_dynamic_and_static_bodies_with_distinct_capacities)
  TESTS_END;
}
