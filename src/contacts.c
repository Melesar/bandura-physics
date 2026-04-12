#include "physics.h"
#include "profiler.h"
#include "trace.h"
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
