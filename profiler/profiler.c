#ifdef __linux__
#define _POSIX_C_SOURCE 199309L // This is to have clock_gettime, which is otherwise not available under -std=c99
#endif

#include "profiler.h"
#include "semaphores.h"
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_MONITORS_COUNT 1

labels labels_storage;

profiler_marker *markers_stack;
uint32_t markers_count;
uint32_t markers_capacity;

profiler_sample *samples;
uint32_t samples_capacity;
uint32_t frame_start;
uint32_t frame_offset;
uint32_t max_frame_size;

profiler_frame_header *frame_headers;
profiler_frame_metadata *metadata;
uint32_t frame_header_index;
uint32_t frame_headers_capacity;
uint32_t frame_headers_mask;

const uint32_t monitors_framebuffer_capacity = 256;

pthread_t monitor_threads[MAX_MONITORS_COUNT];
semaphore monitor_semaphores[MAX_MONITORS_COUNT];

struct monitors_t {
  uint8_t count;
  uint8_t mask;
  bool running;
} monitors;

void *csv_file_monitor_run();

static void notify_monitors() {
  uint8_t count = monitors.count;
  for (uint8_t i = 0; i < count; ++i) {
    semaphore_post(&monitor_semaphores[i]);
  }
}

static void update_header_atomic(uint32_t offset, uint16_t count) {
  profiler_frame_header updated_header = {
      .offset = offset,
      .count = count,
      .mask = monitors.mask,
  };

  uint64_t new_header;
  memcpy(&new_header, &updated_header, sizeof(uint64_t));

  uint64_t *current_header = (uint64_t *)&frame_headers[frame_header_index];
  __atomic_store_n(current_header, new_header, __ATOMIC_RELEASE);
}

static label marker_label(profiler_marker marker) { return (label){marker.label, strlen(marker.label)}; }

static uint64_t get_time() {
  struct timespec time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
  return time.tv_nsec;
}

static void end_block() {
  if (markers_count == 0)
    return;

  profiler_marker last_marker = markers_stack[--markers_count];

  uint64_t elapsed_ns = get_time() - last_marker.start_time;
  samples[frame_start + last_marker.sample_index].time = elapsed_ns;
}

void profiler_init_default() { profiler_init(profiler_default_config()); }

profiler_config profiler_default_config() {
  return (profiler_config){
      .samples_memory_size = 1 << 20, // 1 Mb
      .labels_slots_capacity = 128,
      .labels_storage_capacity = 1 << 16, // 32 Kb
      .stack_capacity = 512,
      .frame_headers_capacity = 64,
      .auto_enable_monitors = true,
  };
}

void profiler_init(profiler_config config) {
  labels_storage = labels_init(config.labels_storage_capacity, config.labels_slots_capacity);
  markers_stack = calloc(config.stack_capacity, sizeof(profiler_marker));
  markers_count = 0;
  markers_capacity = config.stack_capacity;

  uint32_t desired_samples_capacity = config.samples_memory_size / sizeof(profiler_sample);
  samples_capacity = 1;
  while (samples_capacity < desired_samples_capacity)
    samples_capacity <<= 1;

  samples = calloc(samples_capacity, sizeof(profiler_sample));
  frame_start = frame_offset = 0;
  max_frame_size = 0;

  frame_headers_capacity = 1;
  while (frame_headers_capacity < config.frame_headers_capacity)
    frame_headers_capacity <<= 1;
  frame_headers_mask = frame_headers_capacity - 1;
  frame_headers = calloc(frame_headers_capacity, sizeof(profiler_frame_header));
  metadata = calloc(frame_headers_capacity, sizeof(profiler_frame_metadata));

  monitors.count = 0;
  monitors.mask = 0;
  monitors.running = true;

  for (uint32_t i = 0; i < MAX_MONITORS_COUNT; ++i) {
    semaphore_init(&monitor_semaphores[i], 0);
  }

  if (config.auto_enable_monitors) {
    pthread_create(&monitor_threads[0], NULL, &csv_file_monitor_run, NULL);
  }
}

void profiler_teardown() {
  monitors.running = false;

  notify_monitors();

  for (uint32_t i = 0; i < monitors.count; ++i) {
    pthread_join(monitor_threads[i], NULL);
  }

  for (uint32_t i = 0; i < MAX_MONITORS_COUNT; ++i) {
    semaphore_destroy(&monitor_semaphores[i]);
  }

  labels_teardown(labels_storage);
  free(markers_stack);
  free(samples);
  free(frame_headers);
  free(metadata);
}

void profiler_start_frame() {
  frame_offset = 0;
  markers_count = 0;
}

void profiler_end_frame(profiler_frame_metadata frame_metadata) {
  max_frame_size = frame_offset > max_frame_size ? frame_offset : max_frame_size;

  metadata[frame_header_index] = frame_metadata;
  update_header_atomic(frame_start, frame_offset);
  notify_monitors();

  frame_start += frame_offset;
  frame_header_index = (frame_header_index + 1) & frame_headers_mask;

  if (samples_capacity < frame_start || samples_capacity - frame_start < max_frame_size) {
    frame_start = 0;
  }
}

profiler_marker profiler_start_block(const char *name) {
  assert(markers_count < markers_capacity);

  profiler_marker marker = {(char *)name, get_time(), frame_offset};
  profiler_marker parent_marker =
      markers_count > 0 ? markers_stack[markers_count - 1] : (profiler_marker){.sample_index = 0xFFFFFFFF};

  uint32_t sample_index = frame_start + frame_offset;
  samples[sample_index] = (profiler_sample){.label_id = labels_store(&labels_storage, marker_label(marker)),
                                            .parent_index = parent_marker.sample_index,
                                            .time = 0};

  markers_stack[markers_count++] = marker;
  frame_offset += 1;

  return marker;
}

void profiler_end_block(profiler_marker *marker) { end_block(); }

bool profiler_get_label(uint32_t label_id, label *label) {
  *label = labels_get(&labels_storage, label_id);
  return label_is_valid(*label);
}

bool profiler_monitor_start(profiler_monitor *monitor) {
  if (monitors.count >= MAX_MONITORS_COUNT)
    return false;

  /**
   *  With current implementation there might be a situation when:
   *  - Monitor A increments the count
   *  - Monitor B increments the count
   *  - Reader A updates the mask
   *  - Profiler submits another frame, but uses mask only with A.
   *  - Therefore monitor B looses one frame, even though it's in the process of registering itself.
   *
   *  This is an okay tradeoff for simplicity sake.
   */

  uint8_t monitor_id = __atomic_fetch_add(&monitors.count, (uint8_t)1, __ATOMIC_RELAXED);
  uint8_t new_monitor_mask = 1 << monitor_id;

  __atomic_fetch_or(&monitors.mask, new_monitor_mask, __ATOMIC_RELAXED);

  monitor->framebuffer = calloc(monitors_framebuffer_capacity, sizeof(profiler_sample));
  monitor->id = monitor_id;
  monitor->frame_index = frame_header_index;

  return true;
}

bool profiler_monitor_should_run(profiler_monitor *monitor) {
  bool running = monitors.running;
  if (!running) {
    free(monitor->framebuffer);
  }

  return running;
}

bool profiler_monitor_read_next_frame(profiler_monitor *monitor) {
  profiler_frame_header *frame = &frame_headers[monitor->frame_index];
  uint8_t monitor_mask = 1 << monitor->id;
  uint8_t disable_mask = ~monitor_mask;
  uint8_t frame_mask = frame->mask;

  if ((frame_mask & monitor_mask) == 0) {
    return false;
  }

  if (frame->count > monitors_framebuffer_capacity) {
    // TODO resize the buffer?
    assert(false); // Temporary for catching these issues should they appear.
    return false;
  }

  monitor->samples_available = frame->count;
  monitor->frame_metadata = metadata[monitor->frame_index];
  monitor->frame_index = (monitor->frame_index + 1) & frame_headers_mask;

  memcpy(monitor->framebuffer, &samples[frame->offset], frame->count * sizeof(profiler_sample));

  uint8_t new_frame_mask;
  do {
    new_frame_mask = frame_mask & disable_mask;
  } while (!__atomic_compare_exchange_n(&frame->mask, &frame_mask, new_frame_mask, true, __ATOMIC_RELEASE,
                                        __ATOMIC_RELAXED));

  return true;
}

void profiler_monitor_wait_for_frame(const profiler_monitor *monitor) {
  semaphore_wait(&monitor_semaphores[monitor->id]);
}
