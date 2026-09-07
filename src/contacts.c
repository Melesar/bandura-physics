#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

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

static inline uint64_t cache_key_hash(uint64_t key) {
  key ^= key >> 30; key *= 0xbf58476d1ce4e5b9ULL;
  key ^= key >> 27; key *= 0x94d049bb133111ebULL;
  key ^= key >> 31;

  return key;
}


void contacts_reset(bnd_world *world) {
  contacts *contacts = &world->contacts;

  broad_contacts_set *sets[] = { &contacts->dynamics, &contacts->statics };
  for (count_t k = 0; k < 2; ++k) {
    sets[k]->first = sets[k]->last = sets[k]->free_list = UINT32_MAX;
    sets[k]->next = 0;
  }

  memset(contacts->keys, 0, sizeof(uint64_t) * contacts->hash_table_capacity);
  contacts->hash_table_entry_count = 0;
}

bnd_error contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  bnd_allocator allocator = world->allocator;

  count_t hash_table_capacity = world->config.memory.hash_table_capacity;
  if (hash_table_capacity == 0 || hash_table_capacity & (hash_table_capacity - 1)) {
    // TODO make sure these conditions are false before proceeding. Watch out for reported memory value though.
  }

  ALLOC_BUFFER8(contacts->keys, sizeof(uint64_t) * hash_table_capacity);
  ALLOC_BUFFER4(contacts->indices, sizeof(count_t) * hash_table_capacity);

  memset(contacts->keys, 0, sizeof(uint64_t) * hash_table_capacity);
  contacts->hash_table_entry_count = 0;

  count_t dynamic_capacity, static_capacity;
  count_t contacts_capacity = world->config.memory.contacts_capacity;
  if (contacts_capacity & 1) {
    dynamic_capacity = (contacts_capacity >> 1) + 1;
    static_capacity = (contacts_capacity >> 1);
  } else {
    dynamic_capacity = contacts_capacity >> 1;
    static_capacity = dynamic_capacity;
  }

  ALLOC_BUFFER8(contacts->dynamics.contacts, sizeof(broad_phase_contact) * dynamic_capacity);
  ALLOC_BUFFER8(contacts->statics.contacts,  sizeof(broad_phase_contact) * static_capacity);

  contacts->hash_table_capacity = hash_table_capacity;
  contacts->dynamics.capacity = dynamic_capacity;
  contacts->statics.capacity = static_capacity;

  broad_contacts_set *sets[] = { &contacts->dynamics, &contacts->statics };
  for (count_t k = 0; k < 2; ++k) {
    sets[k]->first = sets[k]->last = sets[k]->free_list = UINT32_MAX;
    sets[k]->next = 0;
  }


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

  uint64_t key = 0x8000000000000000; // 63-rd bit set. This indicates a valid (alive) key.
  key |= (uint64_t)type << 62;
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
  broad_contacts_set *contact_sets[] = { &contacts->dynamics, &contacts->statics };

  for (count_t k = 0; k < 2; ++k) {
    broad_contacts_set *set = contact_sets[k];
    count_t index = set->first;

    while (index != UINT32_MAX) {
      const broad_phase_contact *c = &set->contacts[index];

      assert(hash_table_find_empty_slot(contacts, c->key, &slot));
      contacts->keys[slot] = c->key;
      contacts->indices[slot] = index;

      index = c->next_body;
    }
  }

  return OK;
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
