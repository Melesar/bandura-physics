#ifndef PROFILER_H
#define PROFILER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t labels_storage_capacity;
  uint32_t labels_slots_capacity;
  uint32_t stack_capacity;
  uint32_t samples_memory_size;
  uint32_t frame_headers_capacity;
  bool auto_enable_monitors;
} profiler_config;

typedef struct {
  uint16_t body_count;
  uint16_t contacts_count;
} profiler_frame_metadata;

#ifndef BND_PROFILING

#define PROFILE_BLOCK(name)
#define PROFILE_FUNCTION

static profiler_config profiler_default_config() { return (profiler_config){0}; }

static void profiler_init_default() {}

static void profiler_init(profiler_config config) {}

static void profiler_teardown() {}

static void profiler_start_frame() {}

static void profiler_end_frame(profiler_frame_metadata meta) {}

#else

typedef struct {
  char *label;
  uint64_t start_time;
  uint32_t sample_index;
} profiler_marker;

typedef struct {
  uint32_t label_id;
  uint32_t parent_index;
  uint64_t time;
} profiler_sample;

typedef struct {
  uint32_t offset;
  uint16_t count;
  uint8_t mask;
} profiler_frame_header;

typedef struct {
  profiler_sample *framebuffer;
  profiler_frame_metadata frame_metadata;
  uint32_t framebuffer_capacity;
  uint32_t frame_index;
  uint32_t samples_available;
  uint8_t id;
} profiler_monitor;

typedef struct {
  char *s;
  uint8_t len;
} label;

typedef struct {
  uint64_t value;
} labels_slot;

typedef struct {
  char *storage;
  labels_slot *slots;
  uint32_t capacity;
  uint32_t mask;
  uint32_t storage_ptr;
} labels;

#define LABELS_STORAGE_FULL 0xFFFFFFFF
#define INVALID_LABEL (label){NULL, 0}

#define CONCAT(a, b) a##b
#define MARKER_NAME(a, b) CONCAT(a, b)

#define PROFILE_BLOCK(name)                                                                                            \
  profiler_marker MARKER_NAME(marker_, __LINE__) __attribute__((__cleanup__(profiler_end_block))) =                    \
      profiler_start_block(name);

#define PROFILE_FUNCTION PROFILE_BLOCK(__func__)

profiler_config profiler_default_config();
void profiler_init_default();
void profiler_init(profiler_config config);
void profiler_teardown();

void profiler_start_frame();
void profiler_end_frame(profiler_frame_metadata meta);

profiler_marker profiler_start_block(const char *name);
void profiler_end_block(profiler_marker *marker);

bool profiler_get_label(uint32_t label_id, label *label);

bool profiler_monitor_start(profiler_monitor *monitor);
bool profiler_monitor_should_run(profiler_monitor *monitor);
bool profiler_monitor_read_next_frame(profiler_monitor *monitor);
void profiler_monitor_wait_for_frame(const profiler_monitor *monitor);

labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity);
void labels_teardown(labels self);

bool label_is_valid(label l);
bool label_is_equal(label l, const char *string);
uint32_t labels_store(labels *self, label l);
label labels_get(labels *self, uint32_t id);

#endif

#endif
