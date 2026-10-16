#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include <CUnit/CUnit.h>

static bnd_world *joints_test_world;

static void joints_test_setup(void) {
  joints_test_world = bnd_init(bnd_default_config());
}

static void joints_test_teardown(void) {
  if (joints_test_world != NULL) {
    bnd_teardown(joints_test_world);
    joints_test_world = NULL;
  }
}

#define joints_test_begin() joints_test_begin_at(__FILE__, __LINE__)

static bnd_world *joints_test_begin_at(const char *file, unsigned int line) {
  CU_assertImplementation(joints_test_world != NULL, line, "Joint test world allocation failed", file, "", CU_TRUE);
  return joints_test_world;
}

static bool joints_test_has_id(const bnd_world *world, bnd_joint_handle handle) {
  for (count_t i = 0; i < world->joints.count; ++i) {
    if (world->joints.ids[i] == handle.id) {
      return true;
    }
  }

  return false;
}

static void test_joints_removing_unrelated_body_keeps_every_joint(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle a = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle b = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle ground = bnd_add_box_static(world, bnd_v3_one()).value;
  bnd_body_handle unrelated = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_joint_handle dynamic_joint = bnd_add_distance_joint(world, a, b, joint_config).value;
  bnd_joint_handle static_joint = bnd_add_distance_joint(world, a, ground, joint_config).value;

  CU_ASSERT(world->joints.count == 2);
  CU_ASSERT(world->joints.dynamic_count == 1);

  CU_ASSERT(IS_OK(bnd_remove_body(world, unrelated)));

  CU_ASSERT(world->joints.count == 2);
  CU_ASSERT(world->joints.dynamic_count == 1);
  CU_ASSERT(joints_test_has_id(world, dynamic_joint));
  CU_ASSERT(joints_test_has_id(world, static_joint));
}

static void test_joints_removing_body_drops_all_of_its_joints(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle shared = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle b = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle c = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle ground = bnd_add_box_static(world, bnd_v3_one()).value;

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_add_distance_joint(world, shared, b, joint_config);
  bnd_add_distance_joint(world, shared, c, joint_config);
  bnd_add_distance_joint(world, shared, ground, joint_config);

  bnd_joint_handle survivor_dynamic = bnd_add_distance_joint(world, b, c, joint_config).value;
  bnd_joint_handle survivor_static = bnd_add_distance_joint(world, c, ground, joint_config).value;

  CU_ASSERT(world->joints.count == 5);
  CU_ASSERT(world->joints.dynamic_count == 3);

  CU_ASSERT(IS_OK(bnd_remove_body(world, shared)));

  CU_ASSERT(world->joints.count == 2);
  CU_ASSERT(world->joints.dynamic_count == 1);
  CU_ASSERT(joints_test_has_id(world, survivor_dynamic));
  CU_ASSERT(joints_test_has_id(world, survivor_static));

  bnd_simulate(world, 1.0f / 60.0f);
}

static void test_joints_removing_static_body_drops_its_joints(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle a = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle b = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle ground = bnd_add_box_static(world, bnd_v3_one()).value;

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_joint_handle survivor = bnd_add_distance_joint(world, a, b, joint_config).value;
  bnd_add_distance_joint(world, a, ground, joint_config);
  bnd_add_distance_joint(world, b, ground, joint_config);

  CU_ASSERT(world->joints.count == 3);
  CU_ASSERT(world->joints.dynamic_count == 1);

  CU_ASSERT(IS_OK(bnd_remove_body(world, ground)));

  CU_ASSERT(world->joints.count == 1);
  CU_ASSERT(world->joints.dynamic_count == 1);
  CU_ASSERT(joints_test_has_id(world, survivor));

  bnd_simulate(world, 1.0f / 60.0f);
}

static void test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle a = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle doomed = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_add_distance_joint(world, a, doomed, joint_config);
  CU_ASSERT(world->joints.count == 1);

  CU_ASSERT(IS_OK(bnd_remove_body(world, doomed)));
  CU_ASSERT(world->joints.count == 0);

  // The freed outer slot gets recycled here. Had the stale row survived, it would silently
  // re-attach to this brand new body.
  bnd_body_handle recycled = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  CU_ASSERT(recycled.index == doomed.index);

  CU_ASSERT(world->joints.count == 0);
  CU_ASSERT(world->joints.dynamic_count == 0);

  bnd_simulate(world, 1.0f / 60.0f);
}

CU_ErrorCode register_joints_tests(void) {
  CU_TestInfo tests[] = {
    {"test_joints_removing_unrelated_body_keeps_every_joint", test_joints_removing_unrelated_body_keeps_every_joint},
    {"test_joints_removing_body_drops_all_of_its_joints", test_joints_removing_body_drops_all_of_its_joints},
    {"test_joints_removing_static_body_drops_its_joints", test_joints_removing_static_body_drops_its_joints},
    {"test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row", test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Joints", NULL, NULL, joints_test_setup, joints_test_teardown, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
