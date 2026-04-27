#include "bnd-core.h"
#include "profiler.h"
#include <stdlib.h>

void contacts_generate(physics_world *world) {
  PROFILE_FUNCTION

  count_t dynamic_count = 0;
  dynamic_count += collisions_detect_dynamic(world);
  dynamic_count += joints_generate_dynamic(world);

  world->contacts.dynamic_count = dynamic_count;

  collisions_detect_static(world);
  joints_generate_static(world);

  world->stats.contacts_count = world->contacts.count;
}

void contacts_reset(physics_world *world) {
  world->contacts.count = 0;
  world->contacts.dynamic_count = 0;
}

void contacts_init(physics_world *world) {
  contacts *contacts = &world->contacts;
  contacts->values = malloc(world->config.contacts_capacity * sizeof(contact));
  contacts->capacity = world->config.contacts_capacity;
  contacts->count = 0;
  contacts->dynamic_count = 0;
}

void contacts_teardown(physics_world *world) { free(world->contacts.values); }

void contacts_ensure_capacity(physics_world *world, count_t additional_count) {
  contacts *contacts = &world->contacts;

  count_t count_needed = contacts->count + additional_count;
  while (count_needed >= contacts->capacity) {
    contacts->capacity <<= 1;

    if (contacts->capacity >= count_needed) {
      contacts->values = realloc(contacts->values, contacts->capacity * sizeof(contact));
      break;
    }
  }
}

contact *contacts_new_default(physics_world *world, count_t body_a, count_t body_b) {
  contacts_ensure_capacity(world, 1);

  contact *c = &world->contacts.values[world->contacts.count++];
  c->index_a = body_a;
  c->index_b = body_b;
  c->friction = world->config.friction;
  c->restitution = world->config.restitution;

  return c;
}
