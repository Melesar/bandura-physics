#include "bnd-core.h"
#include "bnd-math.h"

#include <string.h>

static bnd_error resize_if_needed(bnd_allocator allocator, joints *joints) {
  if (joints->count < joints->capacity) {
    return OK;
  }

  count_t old_capacity = joints->capacity;
  while (joints->count >= joints->capacity) {
    joints->capacity *= 2;
  }

  REALLOC_BUFFER4(joints->values, allocator , sizeof(bnd_joint), old_capacity, joints->capacity);
  REALLOC_BUFFER4(joints->ids, allocator , sizeof(count_t), old_capacity, joints->capacity);

  return OK;
}

bnd_result_u32 bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a, bnd_v3 contact_offset_b, float max_distance) {
  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_STATIC) {
    return BND_RESULT_ERR(u32, BND_ERROR_INVALID_JOINT, "Two static bodies cannot be bound together");
  }

  // Let body_a always be dynamic - same as with contacts.
  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_DYNAMIC) {
    bnd_body_handle tmp_body = body_b;
    body_b = body_a;
    body_a = tmp_body;

    bnd_v3 tmp_pos = contact_offset_b;
    contact_offset_b = contact_offset_a;
    contact_offset_a = tmp_pos;
  }

  joints *joints = &world->joints;

  PROPAGATE_RESULT(u32, resize_if_needed(world->allocator, joints));

  count_t last_index = joints->count++;
  bool is_dynamic = body_b.type == BND_BODY_DYNAMIC;
  count_t id = joints->next_id++;

  count_t index;
  if (is_dynamic) {
    if (joints->dynamic_count < last_index) {
      // Move first static joint to the end
      joints->values[last_index] = joints->values[joints->dynamic_count];
      joints->ids[last_index] = joints->ids[joints->dynamic_count];
    }

    index = joints->dynamic_count;
    joints->dynamic_count += 1;
  } else {
    index = last_index;
  }

  joints->values[index] = (bnd_joint){
    .bodies = {body_a, body_b},
    .relative_contact_positions = {contact_offset_a, contact_offset_b},
    .max_error = max_distance,
  };
  joints->ids[index] = id;

  return BND_RESULT_OK(u32, id);
}

void bnd_remove_joint(bnd_world *world, count_t id) {
  joints *joints = &world->joints;

  count_t count = joints->count;
  count_t dynamic_count = joints->dynamic_count;
  for (count_t i = 0; i < count; ++i) {
    if (joints->ids[i] != id) {
      continue;
    }

    if (i < dynamic_count) {
      joints->values[i] = joints->values[dynamic_count - 1];
      joints->ids[i] = joints->ids[dynamic_count - 1];

      joints->values[dynamic_count - 1] = joints->values[count - 1];
      joints->ids[dynamic_count - 1] = joints->ids[count - 1];

      joints->dynamic_count -= 1;
    } else {
      joints->values[i] = joints->values[count - 1];
      joints->ids[i] = joints->ids[count - 1];
    }

    joints->count -= 1;

    break;
  }
}

static bool joint_is_stale(const bnd_joint *joint, bnd_body_handle removed_body) {
  for (count_t i = 0; i < 2; ++i) {
    bnd_body_handle body = joint->bodies[i];
    if (body.type == removed_body.type && body.index == removed_body.index) {
      return true;
    }
  }

  return false;
}

void joints_remove_stale_if_needed(bnd_world *world, bnd_body_handle removed_body) {
  joints *joints = &world->joints;

  count_t old_dynamic_count = joints->dynamic_count;
  count_t write = 0;

  for (count_t read = 0; read < old_dynamic_count; ++read) {
    if (joint_is_stale(&joints->values[read], removed_body)) {
      continue;
    }

    if (write != read) {
      joints->values[write] = joints->values[read];
      joints->ids[write] = joints->ids[read];
    }

    write += 1;
  }

  count_t new_dynamic_count = write;

  for (count_t read = old_dynamic_count; read < joints->count; ++read) {
    if (joint_is_stale(&joints->values[read], removed_body)) {
      continue;
    }

    if (write != read) {
      joints->values[write] = joints->values[read];
      joints->ids[write] = joints->ids[read];
    }

    write += 1;
  }

  joints->dynamic_count = new_dynamic_count;
  joints->count = write;
}

count_t joints_generate_contacts(bnd_world *world, count_t contacts_offset, bnd_body_type type) {
  const joints *joints = &world->joints;

  const count_t start = type == BND_BODY_DYNAMIC ? 0 : joints->dynamic_count;
  const count_t end = type == BND_BODY_DYNAMIC ? joints->dynamic_count : joints->count;
  const count_t max_count = end - start;

  if (IS_ERROR(contacts_ensure_capacity(world, contacts_offset, max_count))) {
    return 0;
  }

  contact *contacts = world->contacts.values;

  count_t spawned_count = 0;
  for (count_t i = start; i < end; ++i) {
    bnd_joint j = joints->values[i];

    const common_data *data[2];
    data[0] = as_common(world, BND_BODY_DYNAMIC);
    data[1] = as_common(world, type);

    bnd_v3 world_points[2];
    count_t indices[2];
    for (count_t k = 0; k < 2; ++k) {
      count_t index = handle_to_inner_index(world, j.bodies[k]);
      world_points[k] = bnd_v3_rotate(j.relative_contact_positions[k], data[k]->rotations[index]);
      world_points[k] = bnd_v3_add(world_points[k], data[k]->positions[index]);
      indices[k] = index;
    }

    bnd_v3 offset = bnd_v3_sub(world_points[1], world_points[0]);
    float distance = bnd_v3_len(offset);
    if (distance <= j.max_error) {
      continue;
    }

    contact *contact = contacts + contacts_offset + spawned_count;
    contact->index_a = indices[0];
    contact->index_b = indices[1];
    contact->point = bnd_v3_scale(bnd_v3_add(world_points[0], world_points[1]), 0.5);
    contact->normal = bnd_v3_scale(offset, 1.0 / distance);
    contact->depth = distance - j.max_error;
    contact->friction = 1.0;
    contact->restitution = 0;

    spawned_count += 1;
  }

  return spawned_count;
}

bnd_error joints_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(world->joints.values, world->config.memory.joints_capacity * sizeof(bnd_joint));
  ALLOC_BUFFER4(world->joints.ids, world->config.memory.joints_capacity * sizeof(count_t));

  world->joints.capacity = world->config.memory.joints_capacity;

  joints_reset(world);

  return OK;
}

void joints_reset(bnd_world *world) {
  world->joints.count = 0;
  world->joints.next_id = 0;
  world->joints.dynamic_count = 0;
}

void joints_teardown(bnd_world *world) {
  world->allocator.free(world->joints.values, world->joints.capacity * sizeof(bnd_joint));
  world->allocator.free(world->joints.ids, world->joints.capacity * sizeof(count_t));
}

#ifdef BND_TESTS

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

void test_joints_removing_unrelated_body_keeps_every_joint(void) {
  bnd_world *world = joints_test_begin();

  bnd_body_handle a = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle b = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;
  bnd_body_handle ground = bnd_add_box_static(world, bnd_v3_one()).value;
  bnd_body_handle unrelated = bnd_add_sphere_dynamic(world, 1.0f, 1.0f).value;

  count_t dynamic_joint = bnd_add_joint(world, a, b, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;
  count_t static_joint = bnd_add_joint(world, a, ground, bnd_v3_zero(), bnd_v3_zero(), 1.0f).value;

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);

  assert(IS_OK(bnd_remove_body(world, unrelated)));

  assert(world->joints.count == 2);
  assert(world->joints.dynamic_count == 1);
  assert(joints_test_has_id(world, dynamic_joint));
  assert(joints_test_has_id(world, static_joint));
}

void test_joints_removing_body_drops_all_of_its_joints(void) {
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

void test_joints_removing_static_body_drops_its_joints(void) {
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

void test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row(void) {
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

void joints_tests() {
  joints_test_world = bnd_init(bnd_default_config());

  TESTS_BEGIN("Joints")

  TEST(test_joints_removing_unrelated_body_keeps_every_joint)
  TEST(test_joints_removing_body_drops_all_of_its_joints)
  TEST(test_joints_removing_static_body_drops_its_joints)
  TEST(test_joints_removing_body_reusing_outer_slot_leaves_no_stale_row)

  TESTS_END

  bnd_teardown(joints_test_world);
  joints_test_world = NULL;
}
#endif
