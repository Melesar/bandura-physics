#include "bandura.h"
#include "bnd-core.h"
#include "profiler.h"

#include <string.h>

#define MAX_CACHE_ENTRIES_PER_PAIR 4
#define HASH_TABLE_TOMBSTONE UINT32_MAX
#define HASH_TABLE_EMPTY 0

static bnd_event make_collision_event(const bnd_world *world, bnd_body_type type, const contact *c) {
  return (bnd_event) { .type = BND_EVENT_COLLISION, .collision =  (bnd_contact) {
    .point = c->point,
    .normal = c->normal,
    .depth = c->depth,
    .body_a = make_body_handle(world, BND_BODY_DYNAMIC, c->index_a),
    .body_b = make_body_handle(world, type, c->index_b),
  }};
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

static uint64_t cache_key_make(bnd_world *world, contact *c, bool is_dynamic) {
  const common_data *data_a = as_common_const(world, BND_BODY_DYNAMIC);
  const common_data *data_b = as_common_const(world, is_dynamic ? BND_BODY_DYNAMIC : BND_BODY_STATIC);

  uint64_t index_a = data_a->inner_lookup[c->index_a];
  uint64_t index_b = data_b->inner_lookup[c->index_b];
  uint64_t gen_a = data_a->generations[index_a];
  uint64_t gen_b = data_b->generations[index_b];

  if (index_a > index_b) {
    uint64_t tmp = index_a;
    index_a = index_b;
    index_b = tmp;

    tmp = gen_a;
    gen_a = gen_b;
    gen_b = tmp;
  }

  const uint64_t mask_23bit = 0x7FFFFF;

  uint64_t key = (uint64_t)is_dynamic << 62;
  key |= gen_a << 53;
  key |= (index_a & mask_23bit) << 31;
  key |= gen_b << 23;
  key |= index_b & mask_23bit;

  return key;
}

static inline uint64_t cache_key_hash(uint64_t key) {
  key ^= key >> 30; key *= 0xbf58476d1ce4e5b9ULL;
  key ^= key >> 27; key *= 0x94d049bb133111ebULL;
  key ^= key >> 31;

  return key;
}

static bnd_error cache_table_realloc_if_needed(bnd_world *world) {
  contacts_cache *cache = &world->contacts_cache;

  if (cache->hash_table_capacity * 0.75f < cache->entry_count) {
    count_t new_capacity = cache->hash_table_capacity * 2;
    REALLOC_BUFFER4(cache->hash_table, world->allocator, sizeof(count_t), cache->hash_table_capacity, new_capacity)
    memset(cache->hash_table, 0, new_capacity * sizeof(count_t));

    for (count_t i = 0; i < cache->entry_count; ++i) {
      uint64_t key = cache->entries[i].key;
      uint64_t hash = cache_key_hash(key);
      uint64_t new_index = hash & (new_capacity - 1);

      for (count_t k = new_index; k < new_capacity; ++k) {
        if (cache->hash_table[k] == HASH_TABLE_EMPTY) {
          cache->hash_table[k] = i;
          break;
        }
      }
    }

    cache->hash_table_capacity = new_capacity;
  }

  if (cache->entry_count >= cache->buffer_capacity) {
    count_t new_capacity = cache->buffer_capacity * 2;

    REALLOC_BUFFER8(cache->entries, world->allocator, sizeof(cache_entry), cache->buffer_capacity, new_capacity);

    cache->buffer_capacity = new_capacity;
  }

  return OK;
}

static bnd_error cache_table_insert(bnd_world *world, uint64_t key, const contact *c, count_t *entry_index) {
  contacts_cache *cache = &world->contacts_cache;

  PROPAGATE_ERROR(cache_table_realloc_if_needed(world));

  uint64_t hash = cache_key_hash(key);
  count_t hash_table_index = hash & (cache->hash_table_capacity - 1);

  for (count_t i = hash_table_index; i < cache->hash_table_capacity; ++i) {
    count_t index = cache->hash_table[i];
    cache_entry *entry = &cache->entries[index];

    if (index == HASH_TABLE_EMPTY || index == HASH_TABLE_TOMBSTONE) {
      entry->key = key;
      entry->access_time = world->age;
      entry->feature_count = 1;
      entry->features[0] = c->features;

      cache->hash_table[i] = ++cache->entry_count; // Prefix-increment because we want to skip 0 index
    } else if (entry->key == key) {
      entry->access_time = world->age;
      if (entry->feature_count < MAX_CACHE_ENTRIES_PER_PAIR) {
        entry->features[entry->feature_count++] = c->features;
      }
    }
  }

  return OK;
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

  collision_detection_init(world);
  PROPAGATE_ERROR(contacts_cache_init(world));

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
  c->restitution = world->config.simulation.bounciness;

  return c;
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

  return OK;
}

const cache_entry *contacts_cache_spawn_and_update(bnd_world *world, count_t contact_index, bool is_dynamic) {
  contact *c = &world->contacts.values[contact_index];
  uint64_t key = cache_key_make(world, c, is_dynamic);

  count_t buffer_index;
  bnd_error e = cache_table_insert(world, key, c, &buffer_index);
  if (e.type != BND_OK) {
    return NULL;
  }

    /*
     * TODO:
     *  - Audit 4 cache entries in the bucket
     *  - Prune stale entries
     *  - Transfer the valid ones to the contacts buffer
     *  - Update the ages
     */

  return NULL;
}

void contacts_cache_prune(bnd_world *world) {

}
