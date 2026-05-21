#include "bandura.h"
#include "bnd-core.h"
#include <string.h>

#define TRY_REALLOC(buffer, size, old_capacity, new_capacity) \
  world->events.buffer = world->allocator.realloc(world->events.buffer, 4, size * old_capacity, size * new_capacity); \
  if (world->events.buffer == NULL) { \
    return BND_RESULT_ERR(u32, BND_ERROR_OUT_OF_MEMORY, "Allocator.realloc failed to re-allocate the events memory buffer"); \
  }

static bnd_result_u32 new_event_index(bnd_world *world) {
  count_t new_count = world->events.count + 1;
  if (new_count >= world->events.capacity) {
    if (world->allocator.realloc == NULL) {
      return BND_RESULT_ERR(u32, BND_ERROR_NO_SPACE_AVAILABLE, "Events memory buffer is full and Allocator.realloc is NULL");
    }

    count_t old_capacity = world->events.capacity;
    while (new_count >= world->events.capacity)  {
      world->events.capacity *= 2;
    }

    TRY_REALLOC(events, sizeof(bnd_event), old_capacity, world->events.capacity)
    TRY_REALLOC(links, sizeof(count_t), old_capacity, world->events.capacity)
  }

  return BND_RESULT_OK(u32, world->events.count);
}

bnd_error bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  common_data *data = as_common(world, body.type);
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] |= type;

  return OK;
}

bnd_error bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  common_data *data = as_common(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] &= ~type;

  return OK;
}

bnd_error bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body) {
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  common_data *data = as_common(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] = 0;

  return OK;
}

bnd_result_bool bnd_event_any(bnd_world *world, bnd_body_handle body) {
  bnd_error e = bnd_handle_valid(world, body);
  if (e.type != BND_OK) {
    return BND_RESULT_ERR2(bool, e);
  }

  const common_data *data = as_common_const(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  return BND_RESULT_OK(bool, data->event_links[index].count != 0);
}

bnd_result_bool bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator) {
  bnd_error e = bnd_handle_valid(world, body);
  if (e.type != BND_OK) {
    return BND_RESULT_ERR2(bool, e);
  }

  const common_data *data = as_common_const(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  if (data->event_links[index].count == 0) {
    enumerator->index = 0xFFFFFFFF;
    return BND_RESULT_OK(bool, true);
  }

  enumerator->index = data->event_links[index].first;
  return BND_RESULT_OK(bool, true);
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

  ALLOC_BUFFER4(world->events.events, world->config.memory.events_capacity * sizeof(bnd_event));
  ALLOC_BUFFER4(world->events.links, world->config.memory.events_capacity * sizeof(count_t));

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

bnd_error events_push(bnd_world *world, common_data *data, count_t index, bnd_event event) {
  event_link *link = &data->event_links[index];

  bnd_result_u32 event_index = new_event_index(world);
  if (event_index.error.type != BND_OK) {
    return event_index.error;
  }

  world->events.events[event_index.value] = event;
  if (link->count > 0) {
    world->events.links[link->last] = event_index.value;
  }
  world->events.links[event_index.value] = 0xFFFFFFFF;
  world->events.count += 1;

  if (link->count == 0) {
    link->first = event_index.value;
  }
  link->last = event_index.value;
  link->count += 1;

  return OK;
}
