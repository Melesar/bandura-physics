#include "library_testing.h"
#include <CUnit/CUnit.h>

#include <stdlib.h>

typedef struct {
  void *pointer;
  uint64_t size;
} allocation_record;

typedef struct {
  allocation_record records[256];
  uint32_t count;
  uint64_t active_bytes;
  uint64_t peak_bytes;
} allocation_tracker;

static allocation_tracker tracker;

static void tracker_reset(void) {
  CU_ASSERT_EQUAL_FATAL(tracker.count, 0);
  tracker = (allocation_tracker){0};
}

static uint32_t tracker_find(void *pointer) {
  for (uint32_t i = 0; i < tracker.count; ++i) {
    if (tracker.records[i].pointer == pointer) {
      return i;
    }
  }

  CU_FAIL("Allocator pointer was not tracked");
  return UINT32_MAX;
}

static void tracker_store(void *pointer, uint64_t size) {
  CU_ASSERT(pointer != NULL);
  CU_ASSERT(tracker.count < 256);
  tracker.records[tracker.count++] = (allocation_record){pointer, size};
  tracker.active_bytes += size;
  if (tracker.active_bytes > tracker.peak_bytes) {
    tracker.peak_bytes = tracker.active_bytes;
  }
}

static void *tracking_malloc(uint64_t alignment, uint64_t size) {
  CU_ASSERT(alignment == 1 || alignment == 2 || alignment == 4 || alignment == 8);
  if (tracker.count == 256) {
    CU_FAIL("Allocator tracker is full");
    return NULL;
  }
  void *pointer = malloc(size);
  if (pointer != NULL) {
    tracker_store(pointer, size);
  }
  return pointer;
}

static void *tracking_realloc(void *pointer, uint64_t alignment, uint64_t old_size, uint64_t new_size) {
  CU_ASSERT(alignment == 1 || alignment == 2 || alignment == 4 || alignment == 8);
  uint32_t index = tracker_find(pointer);
  if (index == UINT32_MAX) return NULL;
  CU_ASSERT(tracker.records[index].size == old_size);

  void *replacement = realloc(pointer, new_size);
  if (replacement == NULL) {
    return NULL;
  }

  tracker.active_bytes -= old_size;
  tracker.active_bytes += new_size;
  if (tracker.active_bytes > tracker.peak_bytes) {
    tracker.peak_bytes = tracker.active_bytes;
  }
  tracker.records[index] = (allocation_record){replacement, new_size};
  return replacement;
}

static void tracking_free(void *pointer, uint64_t size) {
  uint32_t index = tracker_find(pointer);
  if (index == UINT32_MAX) return;
  CU_ASSERT(tracker.records[index].size == size);
  tracker.active_bytes -= size;
  tracker.records[index] = tracker.records[--tracker.count];
  free(pointer);
}

static bnd_allocator tracking_allocator(bool allow_realloc) {
  return (bnd_allocator){
    .malloc = tracking_malloc,
    .realloc = allow_realloc ? tracking_realloc : NULL,
    .free = tracking_free,
  };
}

static void test_required_memory_covers_initial_allocation(void) {
  tracker_reset();
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 2;
  config.memory.statics_capacity = 1;
  config.memory.contacts_capacity = 2;
  config.memory.joints_capacity = 1;
  config.memory.meshes_capacity = 1;
  config.memory.events_capacity = 1;

  uint32_t required = bnd_required_memory(&config);
  bnd_result_world result = bnd_init_with_allocator(config, tracking_allocator(true));
  test_world_track(result.value);
  expect_ok(result.error);
  CU_ASSERT(required >= tracker.peak_bytes);

  test_world_teardown(result.value);
  CU_ASSERT(tracker.count == 0);
  CU_ASSERT(tracker.active_bytes == 0);
}

static void test_allocator_tracks_growth_and_exact_teardown_sizes(void) {
  tracker_reset();
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 1;
  config.memory.statics_capacity = 1;
  config.memory.joints_capacity = 1;

  bnd_result_world result = bnd_init_with_allocator(config, tracking_allocator(true));
  test_world_track(result.value);
  expect_ok(result.error);
  bnd_world *world = result.value;

  add_dynamic_sphere(world, 1.0f);
  add_dynamic_sphere(world, 1.0f);
  add_static_sphere(world, 1.0f);
  add_static_sphere(world, 1.0f);

  test_world_teardown(world);
  CU_ASSERT(tracker.count == 0);
  CU_ASSERT(tracker.active_bytes == 0);
}

static void test_fixed_capacity_allocator_reports_no_space(void) {
  tracker_reset();
  bnd_config config = test_config();
  config.memory.dynamics_capacity = 1;

  bnd_result_world result = bnd_init_with_allocator(config, tracking_allocator(false));
  test_world_track(result.value);
  expect_ok(result.error);
  add_dynamic_sphere(result.value, 1.0f);

  bnd_result_handle extra = bnd_add_sphere_dynamic(result.value, 1.0f, 1.0f);
  expect_error(extra.error, BND_ERROR_NO_SPACE_AVAILABLE);

  test_world_teardown(result.value);
  CU_ASSERT(tracker.count == 0);
}

static void test_allocator_requires_malloc(void) {
  bnd_result_world result = bnd_init_with_allocator(test_config(), (bnd_allocator){0});
  expect_error(result.error, BND_ERROR_INVALID_ALLOCATOR);
}

CU_ErrorCode register_allocator_tests(void) {
  CU_TestInfo tests[] = {
    {"test_required_memory_covers_initial_allocation", test_required_memory_covers_initial_allocation},
    {"test_allocator_tracks_growth_and_exact_teardown_sizes", test_allocator_tracks_growth_and_exact_teardown_sizes},
    {"test_fixed_capacity_allocator_reports_no_space", test_fixed_capacity_allocator_reports_no_space},
    {"test_allocator_requires_malloc", test_allocator_requires_malloc},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Allocator and memory", NULL, NULL, NULL, test_world_cleanup, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
