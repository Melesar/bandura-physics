#include "bandura.h"
#include "bnd-core.h"
#include <string.h>

static bnd_error new_event_index(bnd_world *world, count_t *event_index) {
  count_t new_count = world->events.count + 1;
  if (new_count >= world->events.capacity) {
    if (world->allocator.realloc == NULL) {
      return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Events memory buffer is full and Allocator.realloc is NULL" };
    }

    count_t old_capacity = world->events.capacity;
    while (new_count >= world->events.capacity)  {
      world->events.capacity *= 2;
    }

    REALLOC_BUFFER(world->events.events, world->allocator, sizeof(bnd_event), old_capacity, world->events.capacity);
    REALLOC_BUFFER(world->events.links, world->allocator, sizeof(count_t), old_capacity, world->events.capacity);
  }

  *event_index = world->events.count;
  return OK;
}

void bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  common_data *data = as_common(world, body.type);
  if (!bnd_handle_valid(world, body)) {
    notify_handle_invalid(body);
    return;
  }

  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] |= type;
}

void bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  common_data *data = as_common(world, body.type);
  if (!bnd_handle_valid(world, body)) {
    notify_handle_invalid(body);
    return;
  }

  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] &= ~type;
}

void bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body) {
  common_data *data = as_common(world, body.type);
  if (!bnd_handle_valid(world, body)) {
    notify_handle_invalid(body);
    return;
  }

  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] = 0;
}

bool bnd_event_any(bnd_world *world, bnd_body_handle body) {
  const common_data *data = as_common_const(world, body.type);
  if (!bnd_handle_valid(world, body)) {
    notify_handle_invalid(body);
    return false;
  }

  count_t index = handle_to_inner_index(world, body);
  return data->event_links[index].count != 0;
}

bool bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator) {
  const common_data *data = as_common_const(world, body.type);
  if (!bnd_handle_valid(world, body)) {
    notify_handle_invalid(body);
    return false;
  }

  count_t index = handle_to_inner_index(world, body);
  if (data->event_links[index].count == 0) {
    return false;
  }

  enumerator->index = data->event_links[index].first;
  return true;
}

bool bnd_event_next(bnd_world *world, bnd_event_enumerator *enumerator) {
  if (enumerator->index == 0xFFFFFFFF) {
    return false;
  }

  enumerator->e = world->events.events[enumerator->index];
  enumerator->index = world->events.links[enumerator->index];

  return true;
}

bnd_error events_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER(world->events.events, world->config.memory.events_capacity * sizeof(bnd_event));
  ALLOC_BUFFER(world->events.links, world->config.memory.events_capacity * sizeof(count_t));

  world->events.capacity = world->config.memory.events_capacity;
  world->events.count = 0;

  return OK;
}

void events_teardown(bnd_world *world) {
  world->allocator.free(world->events.events, world->events.capacity * sizeof(bnd_event));
  world->allocator.free(world->events.links, world->events.capacity * sizeof(count_t));
}

void events_reset(bnd_world *world) {
  world->events.count = 0;
  memset(world->dynamics.event_links, 0, world->dynamics.count * sizeof(event_link));
  memset(world->statics.event_links, 0, world->statics.count * sizeof(event_link));
}

bool events_subscribed(const common_data *data, count_t index, bnd_event_type event_type) {
  return data->event_masks[index] & event_type;
}

void events_push(bnd_world *world, common_data *data, count_t index, bnd_event event) {
  event_link *link = &data->event_links[index];

  count_t event_index;
  bnd_error e = new_event_index(world, &event_index);
  if (e.type != BND_OK) {
    raise_error(e.type, NULL, e.message);
    return;
  }

  world->events.events[event_index] = event;
  if (link->count > 0) {
    world->events.links[link->last] = event_index;
  }
  world->events.links[event_index] = 0xFFFFFFFF;
  world->events.count += 1;

  if (link->count == 0) {
    link->first = event_index;
  }
  link->last = event_index;
  link->count += 1;
}
