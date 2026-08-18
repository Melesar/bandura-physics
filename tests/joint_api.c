#include "library_testing.h"
#include "testing.h"

static void test_joint_creation_removal_and_capacity_growth(void) {
  bnd_config config = test_config();
  config.memory.joints_capacity = 1;
  bnd_world *world = bnd_init(config);
  assert(world != NULL);

  bnd_body_handle dynamic_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle dynamic_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, dynamic_b, (bnd_v3){10, 0, 0}));
  expect_ok(bnd_set_position(world, static_body, (bnd_v3){20, 0, 0}));

  bnd_result_u32 first = bnd_add_joint(world, static_body, dynamic_a, bnd_v3_zero(), bnd_v3_zero(), 100.0f);
  expect_ok(first.error);
  assert(first.value == 0);

  bnd_result_u32 second = bnd_add_joint(world, dynamic_a, dynamic_b, bnd_v3_zero(), bnd_v3_zero(), 100.0f);
  expect_ok(second.error);
  assert(second.value == 1);

  bnd_remove_joint(world, 9999);
  bnd_remove_joint(world, first.value);
  bnd_result_u32 third = bnd_add_joint(world, dynamic_b, static_body, bnd_v3_zero(), bnd_v3_zero(), 100.0f);
  expect_ok(third.error);
  assert(third.value == 2);

  expect_ok(bnd_remove_body(world, dynamic_a));
  bnd_simulate(world, 0.0f);

  bnd_teardown(world);
}

static void test_joint_validation_rejects_invalid_and_static_pairs(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle first_static = add_static_sphere(world, 1.0f);
  bnd_body_handle second_static = add_static_sphere(world, 1.0f);

  bnd_result_u32 static_pair = bnd_add_joint(world, first_static, second_static, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  expect_error(static_pair.error, BND_ERROR_INVALID_JOINT);

  expect_ok(bnd_remove_body(world, dynamic));
  bnd_result_u32 stale = bnd_add_joint(world, dynamic, first_static, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  expect_error(stale.error, BND_ERROR_BODY_HANDLE_INVALID);

  bnd_world *other_world = test_world();
  bnd_body_handle other_dynamic = add_dynamic_sphere(other_world, 1.0f);
  bnd_result_u32 cross_world = bnd_add_joint(world, other_dynamic, first_static, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  expect_error(cross_world.error, BND_ERROR_BODY_HANDLE_INVALID);

  bnd_teardown(other_world);
  bnd_teardown(world);
}

void joint_api_tests(void) {
  TESTS_BEGIN("Joint management")
    TEST(test_joint_creation_removal_and_capacity_growth)
    TEST(test_joint_validation_rejects_invalid_and_static_pairs)
  TESTS_END;
}
