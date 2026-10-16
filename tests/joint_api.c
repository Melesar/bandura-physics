#include "library_testing.h"
#include <CUnit/CUnit.h>

static void test_joint_creation_removal_and_capacity_growth(void) {
  bnd_config config = test_config();
  config.memory.joints_capacity = 1;
  bnd_world *world = test_world_init(config);
  CU_ASSERT_PTR_NOT_NULL_FATAL(world);

  bnd_body_handle dynamic_a = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle dynamic_b = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle static_body = add_static_sphere(world, 1.0f);
  expect_ok(bnd_set_position(world, dynamic_b, (bnd_v3){10, 0, 0}));
  expect_ok(bnd_set_position(world, static_body, (bnd_v3){20, 0, 0}));

  bnd_distance_joint_config joint_config = {.max_distance = 100.0f};
  bnd_result_joint first = bnd_add_distance_joint(world, static_body, dynamic_a, joint_config);
  expect_ok(first.error);
  CU_ASSERT(first.value.id == 0);

  bnd_result_joint second = bnd_add_distance_joint(world, dynamic_a, dynamic_b, joint_config);
  expect_ok(second.error);
  CU_ASSERT(second.value.id == 1);

  bnd_joint_handle missing = first.value;
  missing.id = 9999;
  bnd_remove_joint(world, missing);
  bnd_remove_joint(world, first.value);
  bnd_result_joint third = bnd_add_distance_joint(world, dynamic_b, static_body, joint_config);
  expect_ok(third.error);
  CU_ASSERT(third.value.id == 2);

  expect_ok(bnd_remove_body(world, dynamic_a));
  bnd_simulate(world, 0.0f);

  test_world_teardown(world);
}

static void test_joint_validation_rejects_invalid_and_static_pairs(void) {
  bnd_world *world = test_world();
  bnd_body_handle dynamic = add_dynamic_sphere(world, 1.0f);
  bnd_body_handle first_static = add_static_sphere(world, 1.0f);
  bnd_body_handle second_static = add_static_sphere(world, 1.0f);

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_result_joint static_pair = bnd_add_distance_joint(world, first_static, second_static, joint_config);
  expect_error(static_pair.error, BND_ERROR_INVALID_JOINT);

  expect_ok(bnd_remove_body(world, dynamic));
  bnd_result_joint stale = bnd_add_distance_joint(world, dynamic, first_static, joint_config);
  expect_error(stale.error, BND_ERROR_BODY_HANDLE_INVALID);

  bnd_world *other_world = test_world();
  bnd_body_handle other_dynamic = add_dynamic_sphere(other_world, 1.0f);
  bnd_result_joint cross_world = bnd_add_distance_joint(world, other_dynamic, first_static, joint_config);
  expect_error(cross_world.error, BND_ERROR_BODY_HANDLE_INVALID);

  test_world_teardown(other_world);
  test_world_teardown(world);
}

CU_ErrorCode register_joint_api_tests(void) {
  CU_TestInfo tests[] = {
    {"test_joint_creation_removal_and_capacity_growth", test_joint_creation_removal_and_capacity_growth},
    {"test_joint_validation_rejects_invalid_and_static_pairs", test_joint_validation_rejects_invalid_and_static_pairs},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Joint management", NULL, NULL, NULL, test_world_cleanup, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
