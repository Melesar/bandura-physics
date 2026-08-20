#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void draw_contacts(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_contact == NULL) {
    return;
  }

  for (count_t i = 0; i < world->contacts.count; i++) {
    const contact *contact = &world->contacts.values[i];
    callbacks.draw_contact(contact->point, contact->normal, contact->depth, user_data);
  }
}

static void draw_shapes(const bnd_world *world, bnd_body_type type, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_shape == NULL) {
    return;
  }

  const common_data *data = as_common_const(world, type);
  for (count_t i = 0; i < data->count; i++) {
    body_shapes body_shapes = data->shapes[i];
    bool is_trigger = data->flags[i] & BODY_FLAG_TRIGGER;
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
      callbacks.draw_shape(center, rotation, make_body_handle(world, type, i), shape.type, shape.value, is_trigger, user_data);
    }
  }
}

void draw_aabbs(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_aabb == NULL) {
    return;
  }

  for (bnd_body_type type = BND_BODY_DYNAMIC; type <= BND_BODY_STATIC; type++) {
    const common_data *data = as_common_const(world, type);
    for (count_t i = 0; i < data->count; i++) {
      bnd_aabb aabb = data->aabbs[i];
      callbacks.draw_aabb(aabb.center, aabb.half_extents, make_body_handle(world, type, i), user_data);
    }
  }
}

void draw_joints(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_joint == NULL) {
    return;
  }

  const joints *joints = &world->joints;
  for (count_t i = 0; i < joints->count; ++i) {
    const bnd_joint *j = &joints->values[i];

    bnd_v3 points[2];
    for (count_t k = 0; k < 2; ++k) {
      count_t body_index = handle_to_inner_index(world, j->bodies[k]);
      const common_data *data = as_common_const(world, j->bodies[k].type);

      points[k] = bnd_v3_rotate(j->relative_contact_positions[k], data->rotations[body_index]);
      points[k] = bnd_v3_add(points[k], data->positions[body_index]);
    }
   
    callbacks.draw_joint(j->bodies[0], j->bodies[1], points[0], points[1], user_data);
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
  if (flags & BND_DEBUG_DRAW_JOINTS) {
    draw_joints(world, callbacks, user_data);
  }
}

#if defined(BND_DEBUG)

void epa_debug_next_frame(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, epa_debug_status *status) {
  world->epa_debug = status;
  world->epa_debug->src_body_a = body_a;
  world->epa_debug->src_body_b = body_b;
  world->epa_debug->initialized = true;
}

void epa_debug_capture(bnd_world *world) {
  if (world->epa_debug == NULL) {
    return;
  }

  bnd_body_handle src[] = { world->epa_debug->src_body_a, world->epa_debug->src_body_b };
  bnd_body_handle *dst[] = { &world->epa_debug->dst_body_a, &world->epa_debug->dst_body_b };

  for (count_t i = 0; i < 2; ++i) {
    count_t index = handle_to_inner_index(world, src[i]);
    common_data *data = as_common(world, src[i].type);
    
    count_t ephemeral = ephemeral_body_index(data) + i + 2;
    data->positions[ephemeral] = data->positions[index];
    data->rotations[ephemeral] = data->rotations[index];
    data->shapes[ephemeral] = data->shapes[index];
    data->inner_lookup[ephemeral] = ephemeral;
    data->outer_lookup[ephemeral].index = ephemeral;
    data->generations[ephemeral] = 0;

    *dst[i] = make_body_handle(world, src[i].type, ephemeral);
  }

  bnd_error error = collision_detection_epa_context(world, *dst[0], *dst[1], &world->epa_debug->ctx);
  if (IS_ERROR(error)) {
    world->epa_debug->iterations_count_result = (bnd_result_u32) { error, 0 };
    world->epa_debug = NULL;
    return;
  }

  if (!gjk_check_intersection(world, &world->epa_debug->ctx, &world->epa_debug->s)) {
    world->epa_debug->iterations_count_result = (bnd_result_u32) { { BND_ERROR_EPA_NO_INTERSECTION, "Bodies do not intersect" }, 0 };
    world->epa_debug = NULL;
    return;
  }

  contact c;
  count_t iterations_count = epa_get_contact(
    world,
    &world->epa_debug->ctx,
    &world->epa_debug->s,
    world->config.advanced.epa_tolerance,
    &c);

  world->epa_debug->target_iteration = 0;
  world->epa_debug->iterations_count_result = (bnd_result_u32) { OK, iterations_count };
  world->epa_debug = NULL;
}

#endif

collision_test_suite *collision_tests_load(void) {
#ifdef COLLISION_TEST_SUITE_PATH
  char *path = COLLISION_TEST_SUITE_PATH;
  FILE *f = fopen(path, "r");
  if (!f) {
    printf("Failed to open file %s\n", path);
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
