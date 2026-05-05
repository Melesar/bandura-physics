#include "bnd-core.h"
#include "profiler.h"
#include <stdlib.h>

void contacts_generate(bnd_world *world) {
  PROFILE_FUNCTION

  count_t dynamic_count = 0;
  dynamic_count += collisions_detect_dynamic(world);
  dynamic_count += joints_generate_dynamic(world);

  world->contacts.dynamic_count = dynamic_count;

  collisions_detect_static(world);
  joints_generate_static(world);

  world->stats.contacts_count = world->contacts.count;
}

void contacts_reset(bnd_world *world) {
  world->contacts.count = 0;
  world->contacts.dynamic_count = 0;
}

void contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  contacts->values = malloc(world->config.memory.contacts_capacity * sizeof(contact));
  contacts->capacity = world->config.memory.contacts_capacity;
  contacts->count = 0;
  contacts->dynamic_count = 0;
}

void contacts_teardown(bnd_world *world) { free(world->contacts.values); }

void contacts_ensure_capacity(bnd_world *world, count_t additional_count) {
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

contact *contacts_new_default(bnd_world *world, count_t body_a, count_t body_b) {
  contacts_ensure_capacity(world, 1);

  contact *c = &world->contacts.values[world->contacts.count++];
  c->index_a = body_a;
  c->index_b = body_b;
  c->friction = world->config.simulation.friction;
  c->restitution = world->config.simulation.restitution;

  return c;
}
