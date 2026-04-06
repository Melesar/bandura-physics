#if defined(MSC_VER)
#pragma message("Profiling is not supported with MSVC")
#endif

#include "profiler.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static inline uint32_t hash(label l) {
  uint32_t h = 2166136261u;
  for (uint8_t i = 0; i < l.len; i++) {
    h ^= (uint8_t)l.s[i];
    h *= 16777619u;
  }
  return h;
}

static inline uint64_t slot_pack(uint32_t offset, uint32_t length) { return ((uint64_t)offset << 32) | length; }

static inline void slot_unpack(uint64_t value, uint32_t *offset, uint32_t *length) {
  *offset = (uint32_t)(value >> 32);
  *length = (uint32_t)(value & 0xFFFFFFFF);
}

static inline bool slot_free(labels_slot slot) { return slot.value == (uint64_t)~0; }

static void slot_write(labels *labels, labels_slot *slot, label l) {
  uint64_t new_value = slot_pack(labels->storage_ptr, l.len);

  memcpy(labels->storage + labels->storage_ptr, l.s, l.len);
  labels->storage_ptr += l.len;

  __atomic_store_n(&slot->value, new_value, __ATOMIC_RELEASE);
}

static label slot_read(const labels *labels, labels_slot slot) {
  if (slot_free(slot)) {
    return INVALID_LABEL;
  }

  uint32_t offset, len;
  slot_unpack(slot.value, &offset, &len);

  return (label){labels->storage + offset, (uint8_t)len};
}

bool label_is_valid(label l) { return l.s != NULL && l.len != 0; }

bool label_is_equal(label l, const char *string) { return strncmp(l.s, string, l.len) == 0; }

labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity) {
  labels self = {0};
  self.storage = malloc(storage_capacity);

  uint32_t slots_count = 1;
  while (slots_count < slots_capacity) {
    slots_count <<= 1;
  }

  uint32_t slots_memory_size = slots_count * sizeof(labels_slot);
  self.slots = malloc(slots_memory_size);
  memset(self.slots, 0xFF, slots_memory_size);

  self.capacity = slots_count;
  self.mask = slots_count - 1;

  return self;
}

uint32_t labels_store(labels *self, label l) {
  uint32_t h = hash(l);
  uint32_t index = h & self->mask;
  uint32_t initial_index = index;

  if (slot_free(self->slots[index])) {
    slot_write(self, &self->slots[index], l);
    return index;
  }

  do {
    label stored_label = slot_read(self, self->slots[index]);

    if (stored_label.len == l.len && strncmp(stored_label.s, l.s, l.len) == 0)
      return index;

    index = (index + 1) & self->mask;

    if (index == initial_index) {
      return LABELS_STORAGE_FULL;
    }
  } while (!slot_free(self->slots[index]));

  slot_write(self, &self->slots[index], l);

  return index;
}

label labels_get(labels *self, uint32_t id) {
  if (id >= self->capacity) {
    return INVALID_LABEL;
  }

  return slot_read(self, self->slots[id]);
}

void labels_teardown(labels self) {
  free(self.storage);
  free(self.slots);
}

// ========= TESTS ============

#ifdef BND_TESTS
#include "testing.h"

#define LABEL(s)                                                                                                       \
  (label) { s, strlen(s) }

static bool label_equal(const label l, char *s) { return strncmp(l.s, s, l.len) == 0; }

static void storing_label_allows_to_retreive_it() {
  labels ls = labels_init(32, 32);
  uint32_t id = labels_store(&ls, LABEL("Hello"));
  label l = labels_get(&ls, id);

  assert(label_equal(l, "Hello"));

  labels_teardown(ls);
}

static void requesting_invalid_id_gives_null_pointer() {
  labels ls = labels_init(32, 32);
  label l = labels_get(&ls, 200);

  assert(!label_is_valid(l));

  labels_teardown(ls);
}

static void storing_same_label_multiple_times_gives_the_same_id() {
  labels ls = labels_init(32, 32);
  uint32_t id = labels_store(&ls, LABEL("Hello"));

  for (int i = 0; i < 10; ++i) {
    uint32_t another_id = labels_store(&ls, LABEL("Hello"));
    assert(id == another_id);
  }

  labels_teardown(ls);
}

static void different_labels_produce_different_ids() {
  labels ls = labels_init(1000, 32);
  uint32_t ids[] = {labels_store(&ls, LABEL("Hello")), labels_store(&ls, LABEL("foo")), labels_store(&ls, LABEL("bas")),
                    labels_store(&ls, LABEL("42")), labels_store(&ls, LABEL("Starship Millenium Falcon"))};

  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i != j)
        assert(ids[i] != ids[j]);
    }
  }

  labels_teardown(ls);
}

extern void labels_tests() {
  TESTS_BEGIN("Profiling labels")
  TEST(storing_label_allows_to_retreive_it)
  TEST(requesting_invalid_id_gives_null_pointer)
  TEST(storing_same_label_multiple_times_gives_the_same_id)
  TEST(different_labels_produce_different_ids)
  TESTS_END
}

#endif
