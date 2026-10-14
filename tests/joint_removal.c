#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "testing.h"

static bnd_world *joints_test_world;

static bnd_world *joints_test_begin(void) {
  bnd_reset_world(joints_test_world);
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

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);

  assert(IS_OK(bnd_remove_body(world, unrelated)));

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);
  assert(joints_test_has_id(world, dynamic_joint));
  assert(joints_test_has_id(world, static_joint));
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

  assert(world->joints.count == 5);
  assert(world->joints.dynamic_count == 3);

  assert(IS_OK(bnd_remove_body(world, shared)));

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);
  assert(joints_test_has_id(world, survivor_dynamic));
  assert(joints_test_has_id(world, survivor_static));

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

  assert(world->joints.count == 3);
  assert(world->joints.dynamic_count == 1);

  assert(IS_OK(bnd_remove_body(world, ground)));

  assert(world->joints.count == 1);
  assert(world->joints.dynamic_count == 1);
  assert(joints_test_has_id(world, survivor));

  bnd_simulate(world, 1.0f / 60.0f);
}

static void test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle a = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle doomed = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;

  bnd_distance_joint_config joint_config = {.max_distance = 1.0f};
  bnd_add_distance_joint(world, a, doomed, joint_config);
  assert(world->joints.count == 1);

  assert(IS_OK(bnd_remove_body(world, doomed)));
  assert(world->joints.count == 0);

  // The freed outer slot gets recycled here. Had the stale row survived, it would silently
  // re-attach to this brand new body.
  bnd_body_handle recycled = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  assert(recycled.index == doomed.index);

  assert(world->joints.count == 0);
  assert(world->joints.dynamic_count == 0);

  bnd_simulate(world, 1.0f / 60.0f);
}

void joints_tests(void) {
  joints_test_world = bnd_init(bnd_default_config());
  assert(joints_test_world != NULL);

  TESTS_BEGIN("Joints")
    TEST(test_joints_removing_unrelated_body_keeps_every_joint)
    TEST(test_joints_removing_body_drops_all_of_its_joints)
    TEST(test_joints_removing_static_body_drops_its_joints)
    TEST(test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row)
  TESTS_END;

  bnd_teardown(joints_test_world);
  joints_test_world = NULL;
}
