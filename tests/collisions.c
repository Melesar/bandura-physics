#include "bandura.h"
#include <CUnit/CUnit.h>

#include "bnd-core.h"
#include "bnd-math.h"

#include <stdio.h>

static void collisions_are_detected_correctly(void) {
  collision_test_suite *tests = collision_tests_load();
  if (tests == NULL) {
    CU_FAIL("Failed to load collision tests");
    return;
  }

  bnd_config config = bnd_default_config();
  config.simulation.gravity = bnd_v3_zero();

  bnd_world *world = bnd_init(config);
  if (world == NULL) {
    CU_FAIL("Failed to initialize collision test world");
    collision_tests_free(tests);
    return;
  }

  for (count_t i = 0; i < tests->num_pairs; i++) {
    const collision_test_pair *pair = &tests->pairs[i];

    bnd_body_handle handles[2];
    collision_tests_pair_spawn(world, pair, handles);

    bnd_event_subscribe(world, handles[0], BND_EVENT_COLLISION);

    for (count_t j = 0; j < tests->cases_per_pair; ++j) {
      const collision_test_case *test_case = &tests->cases[i * tests->cases_per_pair + j];

      bnd_set_position(world, handles[0], test_case->position_a);
      bnd_set_position(world, handles[1], test_case->position_b);

      bnd_set_rotation(world, handles[0], test_case->rotation_a);
      bnd_set_rotation(world, handles[1], test_case->rotation_b);

      bnd_simulate(world, 0.01);

      bool expected = test_case->intersection;
      bool got = bnd_event_any(world, handles[0]).value;
      if (expected != got) {
        char message[128];
        snprintf(message, sizeof(message), "Pair %u, case %u: expected %d, got %d", i, j, expected, got);
        CU_assertImplementation(CU_FALSE, __LINE__, message, __FILE__, "", CU_FALSE);
        goto cleanup;
      }
    }

    bnd_reset_world(world);
  }

cleanup:
  bnd_teardown(world);
  collision_tests_free(tests);
}

CU_ErrorCode register_collisions_tests(void) {
  CU_TestInfo tests[] = {
    {"collisions_are_detected_correctly", collisions_are_detected_correctly},
    CU_TEST_INFO_NULL
  };
  CU_SuiteInfo suites[] = {
    {"Collisions", NULL, NULL, NULL, NULL, tests},
    CU_SUITE_INFO_NULL
  };
  return CU_register_suites(suites);
}
