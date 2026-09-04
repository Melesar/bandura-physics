#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <string.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <assert.h>

typedef enum {
  SLOT_EMPTY      = 1,
  SLOT_SAME_KEY   = 2,

  SLOT_ANY        = SLOT_EMPTY | SLOT_SAME_KEY,
} hash_table_slot_flags;

static bnd_event make_collision_event(const bnd_world *world, bnd_body_type type, const contact *c) {
  return (bnd_event) { .type = BND_EVENT_COLLISION, .collision =  (bnd_contact) {
    .point = c->point,
    .normal = c->normal,
    .depth = c->depth,
    .body_a = make_body_handle(world, BND_BODY_DYNAMIC, c->index_a),
    .body_b = make_body_handle(world, type, c->index_b),
  }};
}

static void emit_collision_events(bnd_world *world, const contact *contacts, count_t count, bnd_body_type type) {
  common_data *data_a = as_common(world, BND_BODY_DYNAMIC);
  common_data *data_b = as_common(world, type);

  for (count_t i = 0; i < count; ++i) {
    const contact *c = &contacts[i];

    if (events_subscribed(data_a, c->index_a, BND_EVENT_COLLISION)) {
      events_push(world, data_a, c->index_a, make_collision_event(world, type, c));
    }

    if (events_subscribed(data_b, c->index_b, BND_EVENT_COLLISION)) {
      events_push(world, data_b, c->index_b, make_collision_event(world, type, c));
    }
  }
}

static inline uint64_t cache_key_hash(uint64_t key) {
  key ^= key >> 30; key *= 0xbf58476d1ce4e5b9ULL;
  key ^= key >> 27; key *= 0x94d049bb133111ebULL;
  key ^= key >> 31;

  return key;
}

static bnd_result_u32 cache_table_find_slot(bnd_world *world, uint64_t key, hash_table_slot_flags flags) {
  contacts_cache *cache = &world->contacts_cache;

  uint64_t hash = cache_key_hash(key);
  count_t hash_table_index = hash & (cache->hash_table_capacity - 1);

  bool search_empty = flags & SLOT_EMPTY;
  bool search_same_key = flags & SLOT_SAME_KEY;
  int32_t first_tombstone = -1;

  count_t i = hash_table_index;
  do {
    count_t index = cache->hash_table[i];

    if (index == HASH_TABLE_EMPTY) {
      if (search_same_key && search_empty && first_tombstone >= 0) {
        return BND_RESULT_OK(u32, first_tombstone);
      }

      if (search_empty) {
        return BND_RESULT_OK(u32, i);
      }

      return BND_RESULT_ERR(u32, BND_ERROR_NOT_FOUND, "");
    }

    if (index == (count_t) HASH_TABLE_TOMBSTONE) {
      if (search_empty && search_same_key) {
        if (first_tombstone < 0) {
          first_tombstone = i;
        }
      } else if (search_empty) {
        return BND_RESULT_OK(u32, i);
      }

      goto next;
    }

    cache_entry *entry = &cache->entries[index];
    if ((flags & SLOT_SAME_KEY) && entry->key == key) {
      return BND_RESULT_OK(u32, i);
    }

    next:
    i = (i + 1) & (cache->hash_table_capacity - 1);
  } while(i != hash_table_index);

  if (search_same_key && search_empty && first_tombstone >= 0) {
    return BND_RESULT_OK(u32, first_tombstone);
  }

  return BND_RESULT_ERR(u32, BND_ERROR_OUT_OF_MEMORY, "Hash table is full");
}

static bnd_error cache_table_realloc_if_needed(bnd_world *world) {
  contacts_cache *cache = &world->contacts_cache;

  if (cache->hash_table_capacity * 0.75f < cache->entry_count) {
    count_t new_capacity = cache->hash_table_capacity * 2;
    REALLOC_BUFFER4(cache->hash_table, world->allocator, sizeof(count_t), cache->hash_table_capacity, new_capacity)
    memset(cache->hash_table, 0, new_capacity * sizeof(count_t));

    cache->hash_table_capacity = new_capacity;

    for (count_t i = 1; i <= cache->entry_count; ++i) {
      uint64_t key = cache->entries[i].key;
      bnd_result_u32 index = cache_table_find_slot(world, key, SLOT_ANY);
      if (index.error.type != BND_OK) {
        return index.error;
      }

      cache->hash_table[index.value] = i;
    }
  }

  if (cache->entry_count + 1 >= cache->buffer_capacity) {
    count_t new_capacity = cache->buffer_capacity * 2;

    REALLOC_BUFFER8(cache->entries, world->allocator, sizeof(cache_entry), cache->buffer_capacity, new_capacity);
    memset(cache->entries + cache->buffer_capacity, 0, cache->buffer_capacity * sizeof(cache_entry));

    cache->buffer_capacity = new_capacity;
  }

  return OK;
}

static bnd_result_u32 cache_table_insert(bnd_world *world, uint64_t key) {
  contacts_cache *cache = &world->contacts_cache;

  PROPAGATE_RESULT(u32, cache_table_realloc_if_needed(world));

  bnd_result_u32 hash_table_slot = cache_table_find_slot(world, key, SLOT_ANY);
  if (hash_table_slot.error.type != BND_OK) {
    return hash_table_slot;
  }

  cache_entry *entry;
  count_t entry_index = cache->hash_table[hash_table_slot.value];
  if (entry_index == HASH_TABLE_EMPTY || entry_index == (count_t)HASH_TABLE_TOMBSTONE) {
    entry_index = ++cache->entry_count; // Prefix-increment because we want to skip 0 index

    entry = &cache->entries[entry_index];
    entry->key = key;
    entry->access_time = world->age;
    entry->feature_count = 0;

    cache->hash_table[hash_table_slot.value] = entry_index;
  } else if (cache->entries[entry_index].key == key) {
    entry = &cache->entries[entry_index];
    entry->access_time = world->age;
  }

  return BND_RESULT_OK(u32, entry_index);
}

void contacts_reset(bnd_world *world) {
  contacts *contacts = &world->contacts;

  contacts->dynamics.count = 0;
  contacts->statics.count = 0;

  memset(contacts->keys, 0, sizeof(broad_phase_contact) * contacts->hash_table_capacity);
}

void contacts_cache_reset(bnd_world *world) {
  contacts_cache *cache = &world->contacts_cache;
  cache->entry_count = 0;
  memset(cache->hash_table, 0, cache->hash_table_capacity * sizeof(uint32_t));
}

void contacts_generate(bnd_world *world) {
  PROFILER_FUNCTION_START

  count_t dynamic_count = collisions_detect(world, 0, BND_BODY_DYNAMIC);
  emit_collision_events(world, world->contacts.values, dynamic_count, BND_BODY_DYNAMIC);

  dynamic_count += joints_generate_contacts(world, dynamic_count, BND_BODY_DYNAMIC);

  const count_t static_offset = dynamic_count;
  count_t static_count = collisions_detect(world, static_offset, BND_BODY_STATIC);
  emit_collision_events(world, world->contacts.values + static_offset, static_count, BND_BODY_STATIC);

  static_count += joints_generate_contacts(world, static_offset + static_count, BND_BODY_STATIC);

  world->contacts.count = dynamic_count + static_count;
  world->contacts.dynamics.count = dynamic_count;
  world->stats.contacts_count = world->contacts.count;

  PROFILER_FUNCTION_END
}

bnd_error contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  bnd_allocator allocator = world->allocator;

  count_t hash_table_capacity = world->config.memory.hash_table_capacity;
  ALLOC_BUFFER8(contacts->keys, sizeof(uint64_t) * hash_table_capacity);
  ALLOC_BUFFER4(contacts->indices, sizeof(count_t) * hash_table_capacity);

  count_t dynamic_capacity, static_capacity;
  count_t contacts_capacity = world->config.memory.contacts_capacity;
  if (contacts_capacity & 1) {
    dynamic_capacity = (contacts_capacity >> 1) + 2;
    static_capacity = (contacts_capacity >> 1) + 1;
  } else {
    dynamic_capacity = (contacts_capacity >> 1) + 1;
    static_capacity = dynamic_capacity + 1;
  }

  ALLOC_BUFFER8(contacts->dynamics.contacts, sizeof(broad_phase_contact) * dynamic_capacity);
  ALLOC_BUFFER8(contacts->statics.contacts,  sizeof(broad_phase_contact) * static_capacity);

  contacts->hash_table_capacity = hash_table_capacity;
  contacts->dynamics.capacity= dynamic_capacity;
  contacts->statics.capacity= static_capacity;

  contacts->dynamics.count = 0;
  contacts->statics.count = 0;

  collision_detection_init();

  return OK;
}

void contacts_teardown(bnd_world *world) {
  contacts *contacts = &world->contacts;
  world->allocator.free(contacts->keys, contacts->hash_table_capacity * sizeof(uint64_t));
  world->allocator.free(contacts->indices, contacts->hash_table_capacity * sizeof(count_t));

  world->allocator.free(contacts->dynamics.contacts, contacts->dynamics.capacity * sizeof(broad_phase_contact));
  world->allocator.free(contacts->statics.contacts, contacts->statics.capacity * sizeof(broad_phase_contact));
}

bnd_error contacts_ensure_capacity(bnd_world *world, count_t contacts_offset, count_t additional_count) {
  count_t desired_count = contacts_offset + additional_count;
  if (desired_count <= world->contacts.capacity) {
    return OK;
  }

  if (world->allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Contacts buffer is full and Allocator.realloc is NULL" };
  }

  count_t old_capacity = world->contacts.capacity;
  while (desired_count > world->contacts.capacity) {
    world->contacts.capacity <<= 1;

    if (world->contacts.capacity >= desired_count) {
      REALLOC_BUFFER4(world->contacts.values, world->allocator, sizeof(contact), old_capacity, world->contacts.capacity);
      break;
    }
  }

  return OK;
}

bnd_error contacts_cache_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;
  contacts_cache *cache = &world->contacts_cache;

  cache->entry_count = 0;
  cache->buffer_capacity = world->config.advanced.contacts_cache.buffer_capacity;

  count_t desired_capacity = world->config.advanced.contacts_cache.hash_table_capacity;
  cache->hash_table_capacity = 1;
  while (cache->hash_table_capacity < desired_capacity) {
    cache->hash_table_capacity *= 2;
  }

  ALLOC_BUFFER4(cache->hash_table, cache->hash_table_capacity * sizeof(uint32_t));
  ALLOC_BUFFER8(cache->entries, cache->buffer_capacity * sizeof(cache_entry));

  memset(cache->hash_table, 0, cache->hash_table_capacity * sizeof(uint32_t));
  memset(cache->entries, 0, cache->buffer_capacity * sizeof(cache_entry));

  return OK;
}

uint64_t hash_table_create_key(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b, bnd_body_type type) {
  uint64_t outer_index_a = data_a->inner_lookup[index_a];
  uint64_t outer_index_b = data_b->inner_lookup[index_b];
  uint64_t gen_a = data_a->generations[outer_index_a];
  uint64_t gen_b = data_b->generations[outer_index_b];

  if (type == BND_BODY_DYNAMIC && outer_index_b > outer_index_a) {
    uint64_t tmp = outer_index_a;
    outer_index_a = outer_index_b;
    outer_index_b = tmp;

    tmp = gen_a;
    gen_a = gen_b;
    gen_b = tmp;
  }

  const uint64_t mask_23bit = 0x7FFFFF;

  uint64_t key = (uint64_t)type << 62;
  key |= gen_a << 54;
  key |= (outer_index_a & mask_23bit) << 31;
  key |= gen_b << 23;
  key |= outer_index_b & mask_23bit;
  
  return key;
}

bool hash_table_find_slot_for_key(const contacts *contacts, uint64_t key, count_t *slot) {
  uint64_t hash = cache_key_hash(key);
  count_t bucket_index = hash & (contacts->hash_table_capacity - 1);

  count_t i = bucket_index;
  do {
    uint64_t stored_key = contacts->keys[i];
    if (stored_key == key) {
      *slot = i;
      return true;
    }

    if (stored_key == HASH_TABLE_EMPTY) {
      return false;
    }

    i = (i + 1) & (contacts->hash_table_capacity - 1);
  } while (i != bucket_index);

  return false;
}

bool hash_table_find_empty_slot(const contacts *contacts, uint64_t key, count_t *slot) {
  uint64_t hash = cache_key_hash(key);
  count_t bucket_index = hash & (contacts->hash_table_capacity - 1);

  int32_t first_tombstone = -1;
  count_t i = bucket_index;
  do {
    uint64_t stored_key = contacts->keys[i];
    if (stored_key == key) {
      return false;
    }

    if (stored_key == HASH_TABLE_EMPTY) {
      if (first_tombstone >= 0) {
        *slot = (count_t) first_tombstone;
      } else {
        *slot = i;
      }

      return true;
    }

    if (stored_key == HASH_TABLE_TOMBSTONE && first_tombstone < 0) {
      first_tombstone = i;
    }

    i = (i + 1) & (contacts->hash_table_capacity - 1);
  } while (i != bucket_index);

  if (first_tombstone >= 0) {
    *slot = first_tombstone;
    return true;
  }

  return false;
}

bnd_error hash_table_resize_if_needed(bnd_world *world, count_t additional_count) {
  contacts *contacts = &world->contacts;

  const count_t initial_capacity = contacts->hash_table_capacity;
  const float threshold_factor = 0.75f;

  float threshold_capacity = initial_capacity * threshold_factor;
  float intended_count = (float)(contacts->hash_table_entry_count + additional_count);

  if (threshold_capacity > intended_count) {
    return OK;
  } 

  if (world->allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Not enough space for contacts and Allocator.realloc is null" };
  }

  count_t new_capacity = contacts->hash_table_capacity;
  while (new_capacity * threshold_factor < intended_count) {
    new_capacity <<= 1;
  }

  REALLOC_BUFFER8(contacts->keys, world->allocator, sizeof(uint64_t), initial_capacity, new_capacity);
  REALLOC_BUFFER4(contacts->indices, world->allocator, sizeof(count_t), initial_capacity, new_capacity);

  contacts->hash_table_capacity = new_capacity;

  memset(contacts->keys, 0, sizeof(uint64_t) * new_capacity);
  memset(contacts->indices, 0, sizeof(count_t) * new_capacity);

  count_t slot;
  count_t counts[] = { contacts->dynamics.count, contacts->statics.count };
  broad_phase_contact *arrays[] = { contacts->dynamics.contacts, contacts->statics.contacts };

  for (count_t k = 0; k < 2; ++k) {
    for (count_t i = 1; i <= counts[k]; ++i) {
      const broad_phase_contact *c = &arrays[k][i];

      if (hash_table_find_empty_slot(contacts, c->key, &slot)) {
        contacts->keys[slot] = c->key;
        contacts->indices[slot] = i;
      } else {
        // Should not be the case, since we've just cleared and resized the table
        assert(false);
      }
    }
  }

  return OK;
}

cache_entry *contacts_cache_query(bnd_world *world, contact *contact, bnd_body_type type) {
  const common_data *data_a = as_common_const(world, BND_BODY_DYNAMIC);
  const common_data *data_b = as_common_const(world, type);

  uint64_t key = hash_table_create_key(data_a, data_b, contact->index_a, contact->index_b, type);
  bnd_result_u32 index = cache_table_insert(world, key);
  if (index.error.type != BND_OK) {
    return NULL;
  }

  return &world->contacts_cache.entries[index.value];
}

static float cross_2d(bnd_v3 a, bnd_v3 b, bnd_v3 c) {
  bnd_v3 ab = { b.x - a.x, b.y - a.y, 0 };
  bnd_v3 ac = { c.x - a.x, c.y - a.y, 0 };
  return ab.x * ac.y - ab.y * ac.x;
}

static void sort_points(bnd_v3 *points) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 value = points[i];
    count_t j = i;
    while (j > 0) {
      bnd_v3 previous = points[j - 1];
      if (previous.x < value.x || (previous.x == value.x && previous.y <= value.y)) {
        break;
      }

      points[j] = previous;
      --j;
    }
    points[j] = value;
  }
}

static float contact_set_area(contact *contacts, const count_t *indices, bnd_v3 origin, bnd_v3 tangent_x, bnd_v3 tangent_y) {
  bnd_v3 points[MAX_CONTACTS_PER_PAIR];
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 offset = bnd_v3_sub(contacts[indices[i]].point, origin);
    points[i] = (bnd_v3){
      .x = bnd_v3_dot(offset, tangent_x),
      .y = bnd_v3_dot(offset, tangent_y),
    };
  }

  sort_points(points);

  bnd_v3 hull[MAX_CONTACTS_PER_PAIR * 2];
  count_t hull_count = 0;

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    while (hull_count >= 2 && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  count_t lower_count = hull_count;
  for (count_t i = MAX_CONTACTS_PER_PAIR - 1; i < MAX_CONTACTS_PER_PAIR; --i) {
    while (hull_count > lower_count && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  if (hull_count <= 3) {
    return 0;
  }

  --hull_count;

  float area = 0;
  for (count_t i = 0; i < hull_count; ++i) {
    bnd_v3 a = hull[i];
    bnd_v3 b = hull[(i + 1) % hull_count];
    area += a.x * b.y - a.y * b.x;
  }

  return fabsf(area) * 0.5f;
}

static float contact_set_depth(contact *contacts, const count_t *indices) {
  float depth = 0;
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    depth += contacts[indices[i]].depth;
  }

  return depth;
}

static bool better_contact_set(float area, float depth, float best_area, float best_depth) {
  if (area > best_area + EPSILON) {
    return true;
  }

  if (fabsf(area - best_area) <= EPSILON && depth > best_depth + EPSILON) {
    return true;
  }

  return false;
}

static void sort_indices(count_t *indices) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    count_t value = indices[i];
    count_t j = i;
    while (j > 0 && indices[j - 1] > value) {
      indices[j] = indices[j - 1];
      --j;
    }
    indices[j] = value;
  }
}

void contacts_filter_largest_surface_area(contact *contacts, count_t contact_count, count_t *selected_indices) {
  count_t deepest = 0;
  for (count_t i = 1; i < contact_count; ++i) {
    if (contacts[i].depth > contacts[deepest].depth) {
      deepest = i;
    }
  }

  bnd_v3 normal = contacts[deepest].normal;
  bnd_v3 tangent_seed = fabsf(normal.y) < 0.70710678f ? bnd_v3_up() : bnd_v3_right();
  bnd_v3 tangent_x = bnd_v3_cross(tangent_seed, normal);
  if (bnd_v3_lensqr(tangent_x) <= EPSILON * EPSILON) {
    tangent_x = bnd_v3_cross(bnd_v3_forward(), normal);
  }
  tangent_x = bnd_v3_normalize(tangent_x);
  bnd_v3 tangent_y = bnd_v3_normalize(bnd_v3_cross(normal, tangent_x));
  bnd_v3 origin = contacts[deepest].point;

  float best_area = -FLT_MAX;
  float best_depth = -FLT_MAX;

  for (count_t i = 0; i < contact_count; ++i) {
    if (i == deepest) {
      continue;
    }

    for (count_t j = i + 1; j < contact_count; ++j) {
      if (j == deepest) {
        continue;
      }

      for (count_t k = j + 1; k < contact_count; ++k) {
        if (k == deepest) {
          continue;
        }

        count_t indices[MAX_CONTACTS_PER_PAIR] = { deepest, i, j, k };
        float area = contact_set_area(contacts, indices, origin, tangent_x, tangent_y);
        float depth = contact_set_depth(contacts, indices);

        if (better_contact_set(area, depth, best_area, best_depth)) {
          memcpy(selected_indices, indices, sizeof(indices));
          best_area = area;
          best_depth = depth;
        }
      }
    }
  }

  // Since will move the elements within the same buffer, having the indices in ascending order will prevent data corruption.
  sort_indices(selected_indices);

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    contacts[i] = contacts[selected_indices[i]];
  }
}
