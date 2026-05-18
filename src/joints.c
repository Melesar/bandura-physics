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

  REALLOC_BUFFER(joints->values, allocator , sizeof(bnd_joint), old_capacity, joints->capacity);
  REALLOC_BUFFER(joints->ids, allocator , sizeof(count_t), old_capacity, joints->capacity);

  return OK;
}

count_t bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a, bnd_v3 contact_offset_b, float max_distance) {
  // Two static bodies shouldn't be bound together.
  if (body_a.type == BND_STATIC && body_b.type == BND_STATIC) {
    return ~0;
  }

  // Let body_a always be dynamic - same as with contacts.
  if (body_a.type == BND_STATIC && body_b.type == BND_DYNAMIC) {
    bnd_body_handle tmp_body = body_b;
    body_b = body_a;
    body_a = tmp_body;

    bnd_v3 tmp_pos = contact_offset_b;
    contact_offset_b = contact_offset_a;
    contact_offset_a = tmp_pos;
  }

  joints *joints = &world->joints;

  bnd_error e = resize_if_needed(world->allocator, joints);
  if (e.type != BND_OK) {
    raise_error(e.type, NULL, e.message);
    return ~0;
  }

  count_t last_index = joints->count++;
  bool is_dynamic = body_b.type == BND_DYNAMIC;
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

  return id;
}

void bnd_remove_joint(bnd_world *world, count_t id) {
  joints *joints = &world->joints;

  count_t count = joints->count;
  for (count_t i = 0; i < count; ++i) {
    if (joints->ids[i] != id) {
      continue;
    }

    // TODO Adjust this to account for dynamic-static joint sorting.
    joints->values[i] = joints->values[count - 1];
    joints->ids[i] = joints->ids[count - 1];
    joints->count -= 1;

    break;
  }
}

const bnd_joint *bnd_get_joints(const bnd_world *world, count_t *count) {
  *count = world->joints.count;
  return world->joints.values;
}

static count_t generate_contacts(bnd_world *world, count_t start, count_t end, bool is_dynamic) {
  const joints *joints = &world->joints;
  const dynamic_bodies *dynamics = &world->dynamics;
  const static_bodies *statics = &world->statics;

  count_t count = 0;
  for (count_t i = start; i < end; ++i) {
    bnd_joint j = joints->values[i];

    const common_data *data[2];
    data[0] = (common_data *)dynamics;
    data[1] = is_dynamic ? (common_data *)dynamics : (common_data *)statics;

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

    contact *contact = contacts_new_default(world, indices[0], indices[1]);
    if (contact == NULL) {
      continue;
    }

    contact->point = bnd_v3_scale(bnd_v3_add(world_points[0], world_points[1]), 0.5);
    contact->normal = bnd_v3_scale(offset, 1.0 / distance);
    contact->depth = distance - j.max_error;
    contact->friction = 1.0;
    contact->restitution = 0;

    count += 1;
  }

  return count;
}

count_t joints_generate_dynamic(bnd_world *world) {
  return generate_contacts(world, 0, world->joints.dynamic_count, true);
}

void joints_generate_static(bnd_world *world) {
  generate_contacts(world, world->joints.dynamic_count, world->joints.count, false);
}

bnd_error joints_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER(world->joints.values, world->config.memory.joints_capacity * sizeof(bnd_joint));
  ALLOC_BUFFER(world->joints.ids, world->config.memory.joints_capacity * sizeof(count_t));

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
