#include "bandura.h"
#include "bnd-core.h"
#include "profiler.h"

#include <string.h>

static bnd_event make_collision_event(const bnd_world *world, bnd_body_type type, const contact *c) {
  return (bnd_event) { .type = BND_EVENT_COLLISION, .collision = { .contact = (bnd_contact) {
    .point = c->point,
    .normal = c->normal,
    .depth = c->depth,
    .body_a = make_body_handle(world, BND_BODY_DYNAMIC, c->index_a),
    .body_b = make_body_handle(world, type, c->index_b),
  }}};
}

static void emit_collision_events(bnd_world *world, bnd_body_type type, count_t start, count_t end) {
  for (count_t i = start; i < end; ++i) {
    const contact *c = &world->contacts.values[i];
    common_data *data_a = (common_data *)&world->dynamics;
    common_data *data_b = as_common(world, type);

    if (events_subscribed(data_a, c->index_a, BND_EVENT_COLLISION)) {
      events_push(world, data_a, c->index_a, make_collision_event(world, type, c));
    }

    if (events_subscribed(data_b, c->index_b, BND_EVENT_COLLISION)) {
      events_push(world, data_b, c->index_b, make_collision_event(world, type, c));
    }
  }
}

void contacts_generate(bnd_world *world) {
  PROFILE_FUNCTION

  count_t dynamic_count = 0;
  dynamic_count += collisions_detect_dynamic(world);
  emit_collision_events(world, BND_BODY_DYNAMIC, 0, dynamic_count);

  dynamic_count += joints_generate_dynamic(world);

  world->contacts.dynamic_count = dynamic_count;

  collisions_detect_static(world);
  emit_collision_events(world, BND_BODY_STATIC, world->contacts.dynamic_count, world->contacts.count);

  joints_generate_static(world);

  world->stats.contacts_count = world->contacts.count;
}

void contacts_reset(bnd_world *world) {
  world->contacts.count = 0;
  world->contacts.dynamic_count = 0;
}

bnd_error contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(contacts->values, world->config.memory.contacts_capacity * sizeof(contact));

  contacts->capacity = world->config.memory.contacts_capacity;
  contacts->count = 0;
  contacts->dynamic_count = 0;

  return OK;
}

void contacts_teardown(bnd_world *world) {
  world->allocator.free(world->contacts.values, world->contacts.capacity * sizeof(contact));
}

bnd_error contacts_ensure_capacity(bnd_world *world, count_t additional_count) {
  contacts *contacts = &world->contacts;

  count_t count_needed = contacts->count + additional_count;
  if (count_needed < contacts->capacity) {
    return OK;
  }

  if (world->allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Contacts buffer is full and Allocator.realloc is NULL" };
  }

  count_t old_capacity = contacts->capacity;
  while (count_needed >= contacts->capacity) {
    contacts->capacity <<= 1;

    if (contacts->capacity >= count_needed) {
      REALLOC_BUFFER4(contacts->values, world->allocator, sizeof(contact), old_capacity, contacts->capacity);
      break;
    }
  }

  return OK;
}

contact *contacts_new_default(bnd_world *world, count_t body_a, count_t body_b) {
  bnd_error e = contacts_ensure_capacity(world, 1);
  if (e.type != BND_OK) {
    return NULL;
  }

  contact *c = &world->contacts.values[world->contacts.count++];
  c->index_a = body_a;
  c->index_b = body_b;
  c->friction = world->config.simulation.friction;
  c->restitution = world->config.simulation.restitution;

  return c;
}
