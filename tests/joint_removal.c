#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "testing.h"

static bnd_world *joints_test_world;

static bnd_world *joints_test_begin(void) {
  bnd_reset_world(joints_test_world);
  return joints_test_world;
}

static bool joints_test_has_id(const bnd_world *world, count_t id) {
  for (count_t i = 0; i < world->joints.count; ++i) {
    if (world->joints.ids[i] == id) {
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

  count_t dynamic_joint = bnd_add_joint(world, a, b, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;
  count_t static_joint = bnd_add_joint(world, a, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);

  printf("Layer pointer: %p\n", world->dynamics.collision_layers);

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

  bnd_add_joint(world, shared, b, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  bnd_add_joint(world, shared, c, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  bnd_add_joint(world, shared, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f);

  count_t survivor_dynamic = bnd_add_joint(world, b, c, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;
  count_t survivor_static = bnd_add_joint(world, c, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;

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

  count_t survivor = bnd_add_joint(world, a, b, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;
  bnd_add_joint(world, a, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
  bnd_add_joint(world, b, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f);

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

  bnd_add_joint(world, a, doomed, bnd_v3_zero(), bnd_v3_zero(), 1.0f);
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
