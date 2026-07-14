#include "bandura.h"
#include "bnd-core.h"
#include "profiler.h"

#include <string.h>

#define HASH_TABLE_TOMBSTONE UINT32_MAX
#define HASH_TABLE_EMPTY 0

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

    if (index == HASH_TABLE_TOMBSTONE) {
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

static bnd_result_u32 cache_table_insert(bnd_world *world, uint64_t key, const contact *c) {
  contacts_cache *cache = &world->contacts_cache;

  PROPAGATE_RESULT(u32, cache_table_realloc_if_needed(world));

  bnd_result_u32 hash_table_slot = cache_table_find_slot(world, key, SLOT_ANY);
  if (hash_table_slot.error.type != BND_OK) {
    return hash_table_slot;
  }

  cache_entry *entry;
  count_t entry_index = cache->hash_table[hash_table_slot.value];
  if (entry_index == HASH_TABLE_EMPTY || entry_index == HASH_TABLE_TOMBSTONE) {
    entry_index = ++cache->entry_count; // Prefix-increment because we want to skip 0 index

    entry = &cache->entries[entry_index];
    entry->key = key;
    entry->access_time = world->age;

    cache->hash_table[hash_table_slot.value] = entry_index;
  } else if (cache->entries[entry_index].key == key) {
    entry = &cache->entries[entry_index];
    entry->access_time = world->age;
  }

  return BND_RESULT_OK(u32, entry_index);
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

  contacts_cache *cache = &world->contacts_cache;
  world->allocator.free(cache->hash_table, cache->hash_table_capacity * sizeof(count_t));
  world->allocator.free(cache->entries, cache->buffer_capacity * sizeof(cache_entry));
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
  memset(cache->entries, 0, cache->buffer_capacity * sizeof(cache_entry));

  return OK;
}

cache_entry *contacts_cache_query(bnd_world *world, count_t contact_index, bool is_dynamic) {
  contact *c = &world->contacts.values[contact_index];

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

  bnd_result_u32 index = cache_table_insert(world, key, c);
  if (index.error.type != BND_OK) {
    return NULL;
  }

  return &world->contacts_cache.entries[index.value];
}

void contacts_cache_prune(bnd_world *world) {
  PROFILE_FUNCTION

  contacts_cache *cache = &world->contacts_cache;
  cache_entry *entries = cache->entries;
  int32_t entry_count = (int32_t) cache->entry_count;

  for (int32_t i = entry_count; i > 0; --i) {
    count_t age = world->age - entries[i].access_time;

    if (age < world->config.advanced.contacts_cache.max_age) {
      continue;
    }

    bnd_result_u32 current_slot_index = cache_table_find_slot(world, entries[i].key, SLOT_SAME_KEY);
    if (current_slot_index.error.type == BND_OK) {
      cache->hash_table[current_slot_index.value] = HASH_TABLE_TOMBSTONE;
    }

    bnd_result_u32 replacement_slot_index = cache_table_find_slot(world, entries[entry_count].key, SLOT_SAME_KEY);
    if (replacement_slot_index.error.type == BND_OK) {
      cache->hash_table[replacement_slot_index.value] = i;
    }

    entries[i] = entries[entry_count--];
  }

  world->contacts_cache.entry_count = entry_count;
}
