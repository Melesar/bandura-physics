#ifndef LIBRARY_TESTING_H
#define LIBRARY_TESTING_H

#include "bandura.h"
#include "bnd-math.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define TEST_EPSILON 0.0001f

static inline void expect_ok(bnd_error error) {
  assert(error.type == BND_OK);
}

static inline void expect_error(bnd_error error, bnd_error_type type) {
  assert(error.type == type);
}

static inline void expect_float_near(float actual, float expected) {
  if (fabsf(actual - expected) > TEST_EPSILON) {
    fprintf(stderr, "Expected %.6f, got %.6f\n", expected, actual);
  }
  assert(fabsf(actual - expected) <= TEST_EPSILON);
}

static inline void expect_v3_near(bnd_v3 actual, bnd_v3 expected) {
  expect_float_near(actual.x, expected.x);
  expect_float_near(actual.y, expected.y);
  expect_float_near(actual.z, expected.z);
}

static inline void expect_handle(bnd_body_handle actual, bnd_body_handle expected) {
  assert(actual.type == expected.type);
  assert(actual.world_id == expected.world_id);
  assert(actual.index == expected.index);
  assert(actual.generation == expected.generation);
}

static inline bnd_config test_config(void) {
  bnd_config config = bnd_default_config();
  config.simulation.gravity = bnd_v3_zero();
  config.simulation.linear_drag = 1.0f;
  config.simulation.angular_drag = 1.0f;
  return config;
}

static inline bnd_world *test_world(void) {
  bnd_world *world = bnd_init(test_config());
  assert(world != NULL);
  return world;
}

static inline bnd_body_handle add_dynamic_sphere(bnd_world *world, float radius) {
  bnd_result_handle result = bnd_add_sphere_dynamic(world, 1.0f, radius);
  expect_ok(result.error);
  return result.value;
}

static inline bnd_body_handle add_static_sphere(bnd_world *world, float radius) {
  bnd_result_handle result = bnd_add_sphere_static(world, radius);
  expect_ok(result.error);
  return result.value;
}

#endif
