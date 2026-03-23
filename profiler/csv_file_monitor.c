#ifdef __linux__
  #define _POSIX_C_SOURCE 199309L // This is to have clock_gettime, which is otherwise not available under -std=c99
  #define _XOPEN_SOURCE 500
#endif

#include "profiler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_BUFFER_SIZE 100
#define CALL_TREE_CAPACITY 1024

typedef struct {
  uint32_t label_id;
  uint32_t parent_index;
  uint64_t total_time;
  uint32_t call_count;
} final_sample;

char buffer[MAX_BUFFER_SIZE + 1];

final_sample *call_tree;

static char *read_label(uint32_t label_id) {
  label label;
  if (profiler_get_label(label_id, &label)) {
    uint32_t size = label.len < MAX_BUFFER_SIZE ? label.len : MAX_BUFFER_SIZE;
    memcpy(buffer, label.s, size);
    buffer[size] = 0;

    return buffer;
  }

  return "unknown";
}



static int32_t find_direct_child(final_sample *samples, uint32_t samples_count, uint32_t parent_index, uint32_t label_id) {
  for(uint32_t i = parent_index + 1; i < samples_count; ++i) {
    final_sample sample = samples[i];
    if (sample.label_id != label_id) {
      continue;
    }

    if (sample.parent_index == parent_index) {
      return i;
    }

    if (sample.parent_index > parent_index) {
      // Indirect child - search futher.
      continue;
    }

    if (sample.parent_index < parent_index) {
      // Parent's sibling or further up. There will be no direct children from this point on.
      break;
    }
  }

  return -1;
}

static uint32_t process_samples(profiler_sample *samples, uint32_t samples_count, final_sample *output, uint32_t available_capacity) {
  if (samples_count == 0 || available_capacity == 0) {
    return 0;
  }

  uint32_t final_count = 1;
  profiler_sample root = samples[0];

  output[0] = (final_sample) {
    .label_id = root.label_id,
    .parent_index = root.parent_index,
    .call_count = 1,
    .total_time = root.time,
  };

  uint32_t src_index = 1;
  uint32_t dst_current_index = 0;
  uint32_t dst_free_slot = 1;
  while(src_index < samples_count) {
    if (dst_free_slot >= available_capacity) {
      return final_count;
    }

    profiler_sample src_sample = samples[src_index];
    uint32_t src_parent_label = samples[src_sample.parent_index].label_id;
    uint32_t dst_current_label = output[dst_current_index].label_id;

    if (dst_current_label == src_parent_label) {
      // src_sample is the direct child of the current sample in the output.
      int32_t child_index = find_direct_child(output, final_count, dst_current_index, src_sample.label_id);
      if (child_index < 0) {
        output[dst_free_slot] = (final_sample) { .label_id = src_sample.label_id, .parent_index = dst_current_index, .total_time = src_sample.time, .call_count = 1 };

        dst_current_index = dst_free_slot;
        dst_free_slot += 1;
        final_count += 1;
      } else {
        output[child_index].total_time += src_sample.time;
        output[child_index].call_count += 1;
        dst_current_index = child_index;
      }

      src_index += 1;
      continue;
    }

    /**
     *  Now there are two options:
     *    1) Src item can be a sibling of the dst item.
     *    2) Src item can be a sibling of the dst item's parent.
     *  Do figure this out we will walk them both to the root of the tree. If they reach it simultaneosly - they are siblings.
     */
    uint32_t src_parent_index = src_sample.parent_index;
    uint32_t dst_parent_index = output[dst_current_index].parent_index;
    while(src_parent_index != 0 && dst_parent_index != 0) {
      src_parent_index = samples[src_parent_index].parent_index;
      dst_parent_index = output[dst_parent_index].parent_index;
    }

    if (src_parent_index == 0 && dst_parent_index == 0) {
      // Current items from src and dst are siblings
      dst_current_index = output[dst_current_index].parent_index;
      continue;
    }

    // Src item is a sibling of the dst item's parent
    dst_current_index = output[output[dst_current_index].parent_index].parent_index;
  }

  return final_count;
}

void* csv_file_monitor_run(void *data) {
  FILE *f = fopen("bandura.prof.csv", "w");
  if (!f)
    return NULL;

  call_tree = calloc(CALL_TREE_CAPACITY, sizeof(final_sample));

  profiler_monitor monitor;
  if (!profiler_monitor_start(&monitor)) {
    fclose(f);
    free(call_tree);
    return NULL;
  }

  fprintf(f, "Frame index,Label,Call count,Total time,Body count,Contacts count\n");

  uint32_t running_count = 0;
  while(profiler_monitor_should_run(&monitor)) {
    profiler_monitor_wait_for_frame(&monitor);

    while (profiler_monitor_read_next_frame(&monitor)) {
      uint32_t processed_count = process_samples(monitor.framebuffer, monitor.samples_available, call_tree, CALL_TREE_CAPACITY);

      for(uint32_t sample_index = 0; sample_index < processed_count; ++sample_index) {
        final_sample sample = call_tree[sample_index];
        char *label = read_label(sample.label_id);

        uint64_t time_ns = sample.total_time;
        double time_ms = time_ns / 1000000.0;

        fprintf(f, "%d,%s,%d,%.5f,%d,%d\n", running_count, label, sample.call_count, time_ms, monitor.frame_metadata.body_count, monitor.frame_metadata.contacts_count);
      }

      running_count += 1;
    }
  }

  fclose(f);
  free(call_tree);
  return NULL;
}

// ========== TESTS =================

#ifdef BND_TESTS

#include "testing.h"

static void print_call_tree(final_sample *samples, uint32_t count) {
  printf("\n");

  for(uint32_t i = 0; i < count; ++i) {
    final_sample sample = samples[i];
    uint32_t parent = sample.parent_index;
    while(parent != (uint32_t)~0) {
      printf(" ");
      parent = samples[parent].parent_index;
    }

    double time = sample.total_time / 1000000.0;
    printf("%d x %s: %.5f ms\n", sample.call_count, read_label(sample.label_id), time);
  }
}

static bool label_equals(final_sample sample, const char *title) {
  label l;
  if (!profiler_get_label(sample.label_id, &l)) {
    return false;
  }

  return label_is_equal(l, title);
}

static void work() {
  usleep(1000);
}

static void simulate_frame() {
  PROFILE_FUNCTION

  {
    for(int i = 0; i < 10; ++i)
    {
      PROFILE_BLOCK("water_the_plant")
      work();

      if (i == 5 || i == 7) {
        PROFILE_BLOCK("change_soil")
        work();
      }
    }

    {
      PROFILE_BLOCK("clean_dishes")
      work();
    }

    for(int i = 0; i < 3; ++i) {
      PROFILE_BLOCK("clean_the_room")

      if (i == 0) {
        PROFILE_BLOCK("collect_toys")
        work();
      }

      {
        PROFILE_BLOCK("wipe_dust")

        for (int j = 0; j < 4; ++j) {
          PROFILE_BLOCK("sort_shelf")
          {
            PROFILE_BLOCK("vaccuum")
            work();
          }

          work();
        }

        work();
      }

      {
        PROFILE_BLOCK("vaccuum")
        work();
      }
    }
  }

  work();
}

void total_count_and_time_is_calculated_correctly() {
  call_tree = calloc(CALL_TREE_CAPACITY, sizeof(final_sample));

  profiler_config config = profiler_default_config();
  config.auto_enable_monitors = false;

  profiler_init(config);

  profiler_monitor monitor;
  profiler_monitor_start(&monitor);

  profiler_start_frame();

  simulate_frame();

  profiler_end_frame();

  assert(profiler_monitor_read_next_frame(&monitor));
  assert(monitor.samples_available == 48);

  uint32_t processed_count = process_samples(monitor.framebuffer, monitor.samples_available, call_tree, CALL_TREE_CAPACITY);

  print_call_tree(call_tree, processed_count);

  assert(processed_count == 10);
  assert(label_equals(call_tree[0], "simulate_frame"));
  assert(label_equals(call_tree[1], "water_the_plant"));
  assert(label_equals(call_tree[2], "change_soil"));
  assert(label_equals(call_tree[3], "clean_dishes"));
  assert(label_equals(call_tree[4], "clean_the_room"));
  assert(label_equals(call_tree[5], "collect_toys"));
  assert(label_equals(call_tree[6], "wipe_dust"));
  assert(label_equals(call_tree[7], "sort_shelf"));
  assert(label_equals(call_tree[8], "vaccuum"));
  assert(label_equals(call_tree[9], "vaccuum"));

  assert(call_tree[0].call_count == 1);
  assert(call_tree[1].call_count == 10);
  assert(call_tree[2].call_count == 2);
  assert(call_tree[3].call_count == 1);
  assert(call_tree[4].call_count == 3);
  assert(call_tree[5].call_count == 1);
  assert(call_tree[6].call_count == 3);
  assert(call_tree[7].call_count == 12);
  assert(call_tree[8].call_count == 12);
  assert(call_tree[9].call_count == 3);

  // TODO check total times

  assert(!profiler_monitor_read_next_frame(&monitor));

  profiler_teardown();
  free(call_tree);
}

void csv_file_monitor_tests() {
  TESTS_BEGIN("Text file monitor")
    TEST(total_count_and_time_is_calculated_correctly)
  TESTS_END
}

#endif
