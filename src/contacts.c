#include "physics.h"
#include <stdlib.h>

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

void contacts_teardown(physics_world *world) {
  free(world->contacts.values);
}
