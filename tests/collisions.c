#include "bandura.h"
#include "testing.h"

#include "bnd-core.h"
#include "bnd-math.h"

#include <stdlib.h>

void collisions_are_detected_correctly() {
  collision_test_suite *tests = collision_tests_load();
  if (tests == NULL) {
    printf("Failed to load collision tests");
    exit(1);
    return;
  }

  bnd_config config = bnd_default_config();
  config.simulation.gravity = bnd_v3_zero();

  bnd_world *world = bnd_init(config);

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
        fprintf(stderr, "Pair %u, case %u failed. Expected %d, got %d\n", i, j, expected, got);
        exit(1);
      }
    }

    bnd_reset_world(world);
  }

  bnd_teardown(world);
  collision_tests_free(tests);
}

void collisions_tests() {
  TESTS_BEGIN("Collisions")
    TEST(collisions_are_detected_correctly)
  TESTS_END;
}
