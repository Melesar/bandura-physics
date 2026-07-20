#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void draw_contacts(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  for (count_t i = 0; i < world->contacts.count; i++) {
    const contact *contact = &world->contacts.values[i];
    callbacks.draw_contact(contact->point, contact->normal, contact->depth, user_data);
  }
}

static void draw_shapes(const bnd_world *world, bnd_body_type type, bnd_debug_draw_callbacks callbacks, void *user_data) {
  const common_data *data = as_common_const(world, type);
  for (count_t i = 0; i < data->count; i++) {
    body_shapes body_shapes = data->shapes[i];
    bnd_body_shape *shapes = shapes_get(world, body_shapes);

    for (count_t j = 0; j < body_shapes.count; ++j) {
      bnd_body_shape shape = shapes[j];

      shape_context ctx = {
        .world = world,
        .data = data,
        .shape = shape,
        .index = i
      };

      bnd_v3 center = body_center(&ctx);
      bnd_quat rotation = body_rotation(&ctx);
      callbacks.draw_shape(center, rotation, make_body_handle(world, type, i), shape, user_data);
    }
  }
}

void draw_aabbs(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  for (bnd_body_type type = BND_BODY_DYNAMIC; type <= BND_BODY_STATIC; type++) {
    const common_data *data = as_common_const(world, type);
    for (count_t i = 0; i < data->count; i++) {
      bnd_aabb aabb = data->aabbs[i];
      callbacks.draw_aabb(aabb.center, aabb.half_extents, make_body_handle(world, type, i), user_data);
    }
  }
}

void bnd_debug_draw(const bnd_world *world, bnd_debug_draw_flags flags, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (flags & BND_DEBUG_DRAW_CONTACTS) {
    draw_contacts(world, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_SHAPES_DYNAMIC) {
    draw_shapes(world, BND_BODY_DYNAMIC, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_SHAPES_STATIC) {
    draw_shapes(world, BND_BODY_STATIC, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_AABBS) {
    draw_aabbs(world, callbacks, user_data);
  }
}

bnd_result_u32 debug_epa_begin(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b) {
  collision_detection_context ctx;
  bnd_error error = collision_detection_epa_context(world, body_a, body_b, &ctx);
  if (IS_ERROR(error)) {
    return BND_RESULT_ERR2(u32, error);
  }

  simplex simplex;
  if (!gjk_check_intersection(world, &ctx, &simplex)) {
    return BND_RESULT_OK(u32, 0);
  }

  contact c;
  count_t iterations_count = epa_get_contact(&ctx, &simplex, world->config.advanced.epa_tolerance, &c);

  return BND_RESULT_OK(u32, iterations_count);
}

bool debug_epa_iteration(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, uint32_t iteration, bnd_debug_draw_epa_callbacks callbacks, void *user_data) {
  collision_detection_context ctx;
  bnd_error error = collision_detection_epa_context(world, body_a, body_b, &ctx);
  if (IS_ERROR(error)) {
    return false;
  }

  simplex simplex;
  if (!gjk_check_intersection(world, &ctx, &simplex)) {
    return false;
  }

  return epa_debug_draw(&ctx, &simplex, world->config.advanced.epa_tolerance, iteration, callbacks, user_data);
}

collision_test_suite *collision_tests_load() {
#ifdef COLLISION_TEST_SUITE_PATH
  char *path = COLLISION_TEST_SUITE_PATH;
  FILE *f = fopen(path, "r");
  if (!f) {
    return NULL;
  }

  count_t num_pairs;
  count_t cases_per_pair;
  collision_test_suite *suite = NULL;
  if (fscanf(f, "num_pairs: %u\n", &num_pairs) != 1) {
    fprintf(stderr, "Failed to read pairs count");
    goto fail;
  }

  if (fscanf(f, "cases_per_pair: %u\n", &cases_per_pair) != 1) {
    fprintf(stderr, "Failed to read number of cases per pair");
    goto fail;
  }

  suite = malloc(sizeof(collision_test_suite));
  if (suite == NULL) {
    goto fail;
  }

  suite->num_pairs = num_pairs;
  suite->cases_per_pair = cases_per_pair;
  suite->pairs = malloc(num_pairs * sizeof(collision_test_pair));
  suite->cases = malloc(num_pairs * cases_per_pair * sizeof(collision_test_case));

  if (suite->pairs == NULL || suite->cases == NULL) {
    goto fail;
  }

  count_t num;
  char buffer[64];
  for (count_t i = 0; i < num_pairs; ++i) {
    collision_test_pair *pair = &suite->pairs[i];

    fscanf(f, "---\n");
    for (count_t j = 0; j < 2; ++j) {
      bnd_body_shape *shape = j == 0 ? &pair->a : &pair->b;

      fscanf(f, "shape%u:\n", &num);
      fscanf(f, " type: %s\n", buffer);

      if (!strncmp(buffer, "sphere", 6)) {
        shape->type = BND_SPHERE;
        fscanf(f, " radius: %f\n", &shape->value.sphere.radius);
      } else if (!strncmp(buffer, "box", 3)) {
        shape->type = BND_BOX;

        bnd_v3 half_size;
        fscanf(f, " half_extents: (%f, %f, %f)\n", &half_size.x, &half_size.y, &half_size.z);

        shape->value.box.size = bnd_v3_scale(half_size, 2.0);
      } else if (!strncmp(buffer, "capsule", 7)) {
        shape->type = BND_CAPSULE;
        fscanf(f, " height: %f\n radius: %f\n", &shape->value.capsule.height, &shape->value.capsule.radius);
      }
    }

    fscanf(f, "cases:\n");

    for (count_t j = 0; j < cases_per_pair; ++j) {
      collision_test_case *test_case = &suite->cases[i * cases_per_pair + j];

      fscanf(f, " - case%d:\n", &num);
      fscanf(f, " positionA: (%f, %f, %f)\n", &test_case->position_a.x, &test_case->position_a.y, &test_case->position_a.z);
      fscanf(f, " positionB: (%f, %f, %f)\n", &test_case->position_b.x, &test_case->position_b.y, &test_case->position_b.z);
      fscanf(f, " orientationA: (%f, %f, %f, %f)\n", &test_case->rotation_a.x, &test_case->rotation_a.y, &test_case->rotation_a.z, &test_case->rotation_a.w);
      fscanf(f, " orientationB: (%f, %f, %f, %f)\n", &test_case->rotation_b.x, &test_case->rotation_b.y, &test_case->rotation_b.z, &test_case->rotation_b.w);
      fscanf(f, " intersection: %s\n", buffer);

      test_case->intersection = !strncmp(buffer, "true", 4);
      if (test_case->intersection) {
        fscanf(f, " point: (%f, %f, %f)\n", &test_case->point.x, &test_case->point.y, &test_case->point.z);
        fscanf(f, " normal:  (%f, %f, %f)\n", &test_case->normal.x, &test_case->normal.y, &test_case->normal.z);
        fscanf(f, " depth: %f\n", &test_case->depth);
      }
    }

    fscanf(f, "\n");
  }

  fclose(f);
  return suite;

  fail:
  collision_tests_free(suite);
  fclose(f);
  return NULL;
#else
  return NULL;
#endif
}

void collision_tests_pair_spawn(bnd_world *world, const collision_test_pair *pair, bnd_body_handle *pair_handles) {
  bnd_body_shape shapes[] = { pair->a, pair->b };

  for (count_t j = 0; j < 2; j++) {
    const bnd_body_shape *shape = &shapes[j];

    switch(shape->type)  {
      case BND_SPHERE:
        pair_handles[j] = bnd_add_sphere_dynamic(world, 5, shape->value.sphere.radius).value;
        break;

      case BND_BOX:
        pair_handles[j] = bnd_add_box_dynamic(world, 5, shape->value.box.size).value;
        break;

      case BND_CAPSULE:
        pair_handles[j] = bnd_add_capsule_dynamic(world, 5, shape->value.capsule.radius, shape->value.capsule.height).value;
        break;

      default:
        break;
    }
  }
}

void collision_tests_free(collision_test_suite *tests) {
  if (tests == NULL) {
    return;
  }

  free(tests->pairs);
  free(tests->cases);
  free(tests);
}
