#ifndef LIBRARY_TESTING_H
#define LIBRARY_TESTING_H

#include "bandura.h"
#include "bnd-math.h"

#include <CUnit/CUnit.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define TEST_EPSILON 0.0001f

/* CUnit runs tearDown even after a fatal assertion. Track worlds so failures
 * cannot strand allocations or contaminate the next test. */
static bnd_world *test_worlds[8];
static size_t test_worlds_count;

#define test_world_track(world) test_world_track_at((world), __FILE__, __LINE__)
#define test_world_init(config) test_world_init_at((config), __FILE__, __LINE__)
#define test_world_teardown(world) test_world_teardown_at((world), __FILE__, __LINE__)

static inline bnd_world *test_world_track_at(bnd_world *world, const char *file, unsigned int line) {
  if (world == NULL) return NULL;
  if (test_worlds_count == sizeof(test_worlds) / sizeof(test_worlds[0])) {
    bnd_teardown(world);
    CU_assertImplementation(CU_FALSE, line, "Too many test worlds", file, "", CU_TRUE);
    return NULL;
  }
  test_worlds[test_worlds_count++] = world;
  return world;
}

static inline bnd_world *test_world_init_at(bnd_config config, const char *file, unsigned int line) {
  return test_world_track_at(bnd_init(config), file, line);
}

static inline void test_world_teardown_at(bnd_world *world, const char *file, unsigned int line) {
  if (world == NULL) return;
  for (size_t i = 0; i < test_worlds_count; ++i) {
    if (test_worlds[i] == world) {
      test_worlds[i] = test_worlds[--test_worlds_count];
      bnd_teardown(world);
      return;
    }
  }
  CU_assertImplementation(CU_FALSE, line, "Attempted to tear down an untracked world", file, "", CU_FALSE);
}

static void test_world_cleanup(void) {
  while (test_worlds_count != 0) {
    bnd_teardown(test_worlds[--test_worlds_count]);
  }
}

#define expect_ok(error) expect_ok_at((error), #error, __FILE__, __LINE__)
#define expect_error(error, type) expect_error_at((error), (type), #error, __FILE__, __LINE__)
#define expect_float_near(actual, expected) expect_float_near_at((actual), (expected), #actual, __FILE__, __LINE__)
/* Variadic to accept compound literals such as (bnd_v3){1, 2, 3}. */
#define expect_v3_near(actual, ...) expect_v3_near_at((actual), (__VA_ARGS__), #actual, __FILE__, __LINE__)
#define expect_handle(actual, expected) expect_handle_at((actual), (expected), #actual, __FILE__, __LINE__)

static inline void expect_ok_at(bnd_error error, const char *expression, const char *file, unsigned int line) {
  char message[192];
  snprintf(message, sizeof(message), "%s: expected BND_OK, got error %d", expression, (int)error.type);
  CU_assertImplementation(error.type == BND_OK, line, message, file, "", CU_TRUE);
}

static inline void expect_error_at(bnd_error error, bnd_error_type type, const char *expression, const char *file, unsigned int line) {
  char message[192];
  snprintf(message, sizeof(message), "%s: expected error %d, got %d", expression, (int)type, (int)error.type);
  CU_assertImplementation(error.type == type, line, message, file, "", CU_FALSE);
}

static inline void expect_float_near_at(float actual, float expected, const char *expression, const char *file, unsigned int line) {
  char message[192];
  snprintf(message, sizeof(message), "%s: expected %.6g +/- %.6g, got %.6g", expression, (double)expected, (double)TEST_EPSILON, (double)actual);
  CU_assertImplementation(fabsf(actual - expected) <= TEST_EPSILON, line, message, file, "", CU_FALSE);
}

static inline void expect_v3_near_at(bnd_v3 actual, bnd_v3 expected, const char *expression, const char *file, unsigned int line) {
  char component[160];
  snprintf(component, sizeof(component), "%s.x", expression);
  expect_float_near_at(actual.x, expected.x, component, file, line);
  snprintf(component, sizeof(component), "%s.y", expression);
  expect_float_near_at(actual.y, expected.y, component, file, line);
  snprintf(component, sizeof(component), "%s.z", expression);
  expect_float_near_at(actual.z, expected.z, component, file, line);
}

static inline void expect_handle_at(bnd_body_handle actual, bnd_body_handle expected, const char *expression, const char *file, unsigned int line) {
  char message[192];
  snprintf(message, sizeof(message), "%s.type: expected %d, got %d", expression, (int)expected.type, (int)actual.type);
  CU_assertImplementation(actual.type == expected.type, line, message, file, "", CU_FALSE);
  snprintf(message, sizeof(message), "%s.world_id: expected %u, got %u", expression, expected.world_id, actual.world_id);
  CU_assertImplementation(actual.world_id == expected.world_id, line, message, file, "", CU_FALSE);
  snprintf(message, sizeof(message), "%s.index: expected %u, got %u", expression, expected.index, actual.index);
  CU_assertImplementation(actual.index == expected.index, line, message, file, "", CU_FALSE);
  snprintf(message, sizeof(message), "%s.generation: expected %u, got %u", expression, (unsigned int)expected.generation, (unsigned int)actual.generation);
  CU_assertImplementation(actual.generation == expected.generation, line, message, file, "", CU_FALSE);
}

static inline bnd_config test_config(void) {
  bnd_config config = bnd_default_config();
  config.simulation.gravity = bnd_v3_zero();
  config.simulation.linear_drag = 1.0f;
  config.simulation.angular_drag = 1.0f;
  return config;
}

#define test_world() test_world_at(__FILE__, __LINE__)
#define add_dynamic_sphere(world, radius) add_dynamic_sphere_at((world), (radius), __FILE__, __LINE__)
#define add_static_sphere(world, radius) add_static_sphere_at((world), (radius), __FILE__, __LINE__)

static inline bnd_world *test_world_at(const char *file, unsigned int line) {
  bnd_world *world = test_world_init_at(test_config(), file, line);
  CU_assertImplementation(world != NULL, line, "test_world(): world allocation failed", file, "", CU_TRUE);
  return world;
}

static inline bnd_body_handle add_dynamic_sphere_at(bnd_world *world, float radius, const char *file, unsigned int line) {
  bnd_result_handle result = bnd_add_sphere_dynamic(world, 1.0f, radius);
  CU_assertImplementation(result.error.type == BND_OK, line, "add_dynamic_sphere(): body creation failed", file, "", CU_TRUE);
  return result.value;
}

static inline bnd_body_handle add_static_sphere_at(bnd_world *world, float radius, const char *file, unsigned int line) {
  bnd_result_handle result = bnd_add_sphere_static(world, radius);
  CU_assertImplementation(result.error.type == BND_OK, line, "add_static_sphere(): body creation failed", file, "", CU_TRUE);
  return result.value;
}

#endif
