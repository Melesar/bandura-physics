#include "physics.h"
#include <stdlib.h>

static inline contact *new_contact(physics_world *world) {
  collisions *collisions = &world->collisions;

  if (collisions->count >= collisions->capacity) {
    while (collisions->count >= collisions->capacity) {
      collisions->capacity *= 2;
    }

    collisions->contacts = realloc(collisions->contacts, collisions->capacity * sizeof(contact));
  }

  return &collisions->contacts[collisions->count++];
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

  count_t index = joints->count++;
  count_t id = joints->next_id++;

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

void joints_produce_contacts(physics_world *world) {
  const joints *joints = &world->joints;
  const dynamic_bodies *dynamics = &world->dynamics;
  const static_bodies *statics = &world->statics;

  collisions *collisions = &world->collisions;

  for(count_t i = 0; i < joints->count; ++i) {
    joint j = joints->values[i];
    bool is_dynamic = j.bodies[1].type == BODY_DYNAMIC;

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

    // TODO: must preserve the order: dynamic contacts -> static contacts. Both across joints and collisions.
    if (is_dynamic) {
      collisions->dynamic_contacts_count += 1;
    }
  }
}

void joints_init(physics_world *world) {
  world->joints.values = malloc(world->config.joints_capacity * sizeof(joint));
  world->joints.ids = malloc(world->config.joints_capacity * sizeof(count_t));
  world->joints.capacity = world->config.joints_capacity;
  world->joints.count = 0;
  world->joints.next_id = 0;
}

void joints_teardown(physics_world *world) {
  free(world->joints.values);
  free(world->joints.ids);
}
