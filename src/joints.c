#include "bandura.h"
#include "physics.h"
#include <stdlib.h>

static inline contact *new_contact(physics_world *world) {
  contacts *contacts = &world->contacts;

  if (contacts->count >= contacts->capacity) {
    while (contacts->count >= contacts->capacity) {
      contacts->capacity *= 2;
    }

    contacts->values = realloc(contacts->values, contacts->capacity * sizeof(contact));
  }

  return &contacts->values[contacts->count++];
}

static inline void resize_if_needed(joints *joints) {
  if (joints->count < joints->capacity) {
    return;
  }

  while(joints->count >= joints->capacity) {
    joints->capacity *= 2;
  }

  joints->values = realloc(joints->values, joints->capacity * sizeof(joint));
  joints->ids = realloc(joints->ids, joints->capacity * sizeof(count_t));
}

count_t physics_add_joint(physics_world *world, body_handle body_a, body_handle body_b, v3 contact_offset_a, v3 contact_offset_b, float max_distance) {
  // Two static bodies shouldn't be bound together.
  if (body_a.type ==  BODY_STATIC && body_b.type == BODY_STATIC) {
    return ~0;
  }

  // Let body_a always be dynamic - same as with contacts.
  if (body_a.type == BODY_STATIC && body_b.type == BODY_DYNAMIC) {
    body_handle tmp_body = body_b;
    body_b = body_a;
    body_a = tmp_body;

    v3 tmp_pos = contact_offset_b;
    contact_offset_b = contact_offset_a;
    contact_offset_a = tmp_pos;
  }

  joints *joints = &world->joints;

  resize_if_needed(joints);

  count_t last_index = joints->count++;
  bool is_dynamic = body_b.type == BODY_DYNAMIC;
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

  joints->values[index] = (joint) {
    .bodies = { body_a, body_b },
    .relative_contact_positions = { contact_offset_a, contact_offset_b },
    .max_error = max_distance,
  };
  joints->ids[index] = id;

  return id;
}

void physics_remove_joint(physics_world *world, count_t id) {
  joints *joints = &world->joints;

  count_t count = joints->count;
  for(count_t i = 0; i < count; ++i) {
    if (joints->ids[i] != id) {
      continue;
    }

    joints->values[i] = joints->values[count - 1];
    joints->ids[i] = joints->ids[count - 1];
    joints->count -= 1;

    break;
  }
}

static void generate_contacts(physics_world *world, count_t start, count_t end, bool is_dynamic) {
  const joints *joints = &world->joints;
  const dynamic_bodies *dynamics = &world->dynamics;
  const static_bodies *statics = &world->statics;

  for(count_t i = start; i < end; ++i) {
    joint j = joints->values[i];

    const common_data *data[2];
    data[0] = (common_data*) dynamics;
    data[1] = is_dynamic ? (common_data*) dynamics : statics;

    v3 world_points[2];
    count_t indices[2];
    for(count_t k = 0; k < 2; ++k) {
      count_t index = handle_to_inner_index(world, j.bodies[k]);
      world_points[k] = rotate(j.relative_contact_positions[k], data[k]->rotations[index]);
      world_points[k] = add(world_points[k], data[k]->positions[index]);
      indices[k] = index;
    }

    v3 offset = sub(world_points[1], world_points[0]);
    float distance = len(offset);
    if (distance <= j.max_error) {
      continue;
    }

    contact *contact = new_contact(world);
    contact->index_a = indices[0];
    contact->index_b = indices[1];
    contact->point = scale(add(world_points[0], world_points[1]), 0.5);
    contact->normal = scale(offset, 1.0 / distance);
    contact->depth = distance - j.max_error;
    contact->friction = 1.0;
    contact->restitution = 0;
  }
}

count_t joints_generate_dynamic(physics_world *world) {
  generate_contacts(world, 0, world->joints.dynamic_count, true);
  return world->joints.dynamic_count;
}

void joints_generate_static(physics_world *world) {
  generate_contacts(world, world->joints.dynamic_count, world->joints.count, false);
}

void joints_init(physics_world *world) {
  world->joints.values = malloc(world->config.joints_capacity * sizeof(joint));
  world->joints.ids = malloc(world->config.joints_capacity * sizeof(count_t));
  world->joints.capacity = world->config.joints_capacity;

  joints_reset(world);
}

void joints_reset(physics_world *world) {
  world->joints.count = 0;
  world->joints.next_id = 0;
  world->joints.dynamic_count = 0;
}

void joints_teardown(physics_world *world) {
  free(world->joints.values);
  free(world->joints.ids);
}
