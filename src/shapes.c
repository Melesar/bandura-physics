#include "bnd-core.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SHAPE_BRACKET_BLOCK_CAPACITY 64

static count_t bracket_block_count(const bnd_world *world, shape_dimension_bracket bracket) {
  return world->shape_brackets[bracket].capacity / SHAPE_BRACKET_BLOCK_CAPACITY;
}

static shapes_bracket allocate_bracket(count_t capacity, count_t block_count, count_t shapes_count) {
  uint64_t *slots = malloc(block_count * sizeof(uint64_t));
  bnd_body_shape *shapes = malloc(shapes_count * sizeof(bnd_body_shape));

  memset(slots, 0, block_count * sizeof(uint64_t));

  return (shapes_bracket){slots, shapes, capacity};
}

void shapes_init(bnd_world *world) {
  const bnd_config *config = &world->config;

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count = config->memory.shapes_brackets_capacity[i] / SHAPE_BRACKET_BLOCK_CAPACITY +
                           ((config->memory.shapes_brackets_capacity[i] & (SHAPE_BRACKET_BLOCK_CAPACITY - 1)) > 0);
    count_t bracket_capacity = blocks_count * SHAPE_BRACKET_BLOCK_CAPACITY;

    count_t bracket_dimension = 1 << i;
    count_t shapes_count = bracket_capacity * bracket_dimension;

    world->shape_brackets[i] = allocate_bracket(bracket_capacity, blocks_count, shapes_count);
  }
}

void shapes_teardown(bnd_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    free(world->shape_brackets[i].slots);
    free(world->shape_brackets[i].shapes);
  }
}

void shapes_reset(bnd_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    shapes_bracket *bracket = &world->shape_brackets[i];
    count_t blocks_count = bracket_block_count(world, i);

    bracket->capacity = 0;
    memset(bracket->slots, 0, blocks_count * sizeof(uint64_t));
  }
}

bool shapes_any_slot_available(const bnd_world *world, shape_dimension_bracket bracket) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] < (uint64_t)~0) {
      return true;
    }
  }

  return false;
}

void shapes_expand_bracket(bnd_world *world, shape_dimension_bracket bracket) {
  count_t bracket_capacity = 1 << bracket;

  count_t current_capacity = world->shape_brackets[bracket].capacity;
  count_t current_block_count = bracket_block_count(world, bracket);
  uint64_t *current_slots = world->shape_brackets[bracket].slots;
  bnd_body_shape *current_shapes = world->shape_brackets[bracket].shapes;

  count_t new_capacity = current_capacity + SHAPE_BRACKET_BLOCK_CAPACITY;
  count_t new_block_count = current_block_count + 1;
  count_t shapes_count = bracket_capacity * new_block_count * SHAPE_BRACKET_BLOCK_CAPACITY;

  shapes_bracket new_bracket = allocate_bracket(new_capacity, new_block_count, shapes_count);
  memcpy(new_bracket.slots, current_slots, current_block_count * sizeof(uint64_t));
  memcpy(new_bracket.shapes, current_shapes, current_capacity * bracket_capacity * sizeof(bnd_body_shape));

  world->shape_brackets[bracket] = new_bracket;

  free(current_shapes);
  free(current_slots);
}

bool shapes_put_into_empty_slot(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t *slot_number) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;
  bnd_body_shape *shapes_buffer = world->shape_brackets[bracket].shapes;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] == (uint64_t)~0)
      continue;

    for (count_t k = 0; k < SHAPE_BRACKET_BLOCK_CAPACITY; ++k) {
      uint64_t mask = (uint64_t)1 << k;
      if ((slots[i] & mask) != 0)
        continue;

      count_t bracket_capacity = 1 << bracket;
      count_t shape_offset = (i * SHAPE_BRACKET_BLOCK_CAPACITY + k) * bracket_capacity;

      bnd_body_shape *slot = shapes_buffer + shape_offset;
      memcpy(slot, shapes, shapes_count * sizeof(bnd_body_shape));

      slots[i] |= mask;
      *slot_number = shape_offset;

      return true;
    }
  }

  return false;
}

void shapes_clear_slot(bnd_world *world, shape_dimension_bracket bracket, count_t slot) {
  count_t block_count = bracket_block_count(world, bracket);
  count_t bracket_capacity = 1 << bracket;

  uint64_t *slots = world->shape_brackets[bracket].slots;
  count_t block_index = slot / (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity);
  count_t bit_index = slot % (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity);
  if (block_index < block_count) {
    slots[block_index] &= ~((uint64_t)1 << bit_index);
  }
}

body_shapes shapes_write(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t count) {
  const count_t max_count = 1 << (BRACKET_COUNT - 1);
  assert(count <= max_count);

  if (!shapes_any_slot_available(world, bracket)) {
    shapes_expand_bracket(world, bracket);
  }

  count_t shape_slot;
  shapes_put_into_empty_slot(world, bracket, shapes, count, &shape_slot);

  return (body_shapes){.bracket = bracket, .offset = shape_slot, .count = count};
}

bnd_body_shape *shapes_get(const bnd_world *world, body_shapes shapes) {
  return world->shape_brackets[shapes.bracket].shapes + shapes.offset;
}

#ifdef BND_TESTS
void test_shapes_write_primitive_bracket_uses_second_block(void) {
  physics_world world = {0};
  world.config.shapes_brackets_capacity[BRACKET_PRIMITIVE] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 65; ++i) {
    body_shape shape = {0};
    shape.type = BND_SPHERE;
    shape.sphere.radius = (float)i + 0.5f;

    body_shapes written = shapes_write(&world, BRACKET_PRIMITIVE, &shape, 1);
    body_shape *stored = shapes_get(&world, written);

    assert(written.bracket == BRACKET_PRIMITIVE);
    assert(written.count == 1);
    assert(written.offset == i);
    assert(memcmp(stored, &shape, sizeof(body_shape)) == 0);
  }

  assert(world.shape_brackets[BRACKET_PRIMITIVE].capacity == 128);
  assert(world.shape_brackets[BRACKET_PRIMITIVE].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_PRIMITIVE].slots[1] == 1);
  assert(shapes_any_slot_available(&world, BRACKET_PRIMITIVE));

  shapes_teardown(&world);
}

void test_shapes_write_four_bracket_keeps_alignment_across_blocks(void) {
  physics_world world = {0};
  world.config.shapes_brackets_capacity[BRACKET_FOUR] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 65; ++i) {
    body_shape shapes[4] = {0};
    count_t count = (i & 1) == 0 ? 3 : 4;

    for (count_t k = 0; k < count; ++k) {
      shapes[k].type = BND_CYLINDER;
      shapes[k].cylinder.radius = (float)(i * 10 + k + 1);
      shapes[k].cylinder.height = (float)(100 + i * 10 + k);
    }

    body_shapes written = shapes_write(&world, BRACKET_FOUR, shapes, count);
    body_shape *stored = shapes_get(&world, written);

    assert(written.bracket == BRACKET_FOUR);
    assert(written.count == count);
    assert(written.offset == i * 4);
    assert(memcmp(stored, shapes, count * sizeof(body_shape)) == 0);
  }

  assert(world.shape_brackets[BRACKET_FOUR].capacity == 128);
  assert(world.shape_brackets[BRACKET_FOUR].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_FOUR].slots[1] == 1);

  shapes_teardown(&world);
}

void test_shapes_expand_bracket_preserves_existing_data_after_two_blocks(void) {
  physics_world world = {0};
  world.config.shapes_brackets_capacity[BRACKET_TWO] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 128; ++i) {
    body_shape shapes[2] = {0};
    shapes[0].type = BND_BOX;
    shapes[0].box.size.x = (float)(i + 1);
    shapes[1].type = BND_SPHERE;
    shapes[1].sphere.radius = (float)(i + 200);

    body_shapes written = shapes_write(&world, BRACKET_TWO, shapes, 2);
    assert(written.offset == i * 2);
  }

  assert(world.shape_brackets[BRACKET_TWO].capacity == 128);
  assert(world.shape_brackets[BRACKET_TWO].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[1] == (uint64_t)~0);
  assert(!shapes_any_slot_available(&world, BRACKET_TWO));

  body_shape extra_shapes[2] = {0};
  extra_shapes[0].type = BND_CYLINDER;
  extra_shapes[0].cylinder.radius = 7.0f;
  extra_shapes[0].cylinder.height = 9.0f;
  extra_shapes[1].type = BND_SPHERE;
  extra_shapes[1].sphere.radius = 11.0f;

  body_shapes extra = shapes_write(&world, BRACKET_TWO, extra_shapes, 2);
  body_shape
  *first = world.shape_brackets[BRACKET_TWO].shapes;
  body_shape *middle = world.shape_brackets[BRACKET_TWO].shapes + 64 * 2;
  body_shape *stored_extra = shapes_get(&world, extra);

  assert(world.shape_brackets[BRACKET_TWO].capacity == 192);
  assert(extra.offset == 128 * 2);
  assert(world.shape_brackets[BRACKET_TWO].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[1] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[2] == 1);

  assert(first[0].type == BND_BOX);
  assert(first[0].box.size.x == 1.0f);
  assert(first[1].type == BND_SPHERE);
  assert(first[1].sphere.radius == 200.0f);

  assert(middle[0].type == BND_BOX);
  assert(middle[0].box.size.x == 65.0f);
  assert(middle[1].type == BND_SPHERE);
  assert(middle[1].sphere.radius == 264.0f);

  assert(memcmp(stored_extra, extra_shapes, sizeof(extra_shapes)) == 0);

  shapes_teardown(&world);
}

void test_shapes_clear_slot_reuses_second_block_slot_with_bracket_alignment(void) {
  physics_world world = {0};
  world.config.shapes_brackets_capacity[BRACKET_EIGHT] = 65;

  shapes_init(&world);

  body_shapes entries[66] = {0};
  for (count_t i = 0; i < 66; ++i) {
    body_shape shapes[8] = {0};

    for (count_t k = 0; k < 6; ++k) {
      shapes[k].type = BND_SPHERE;
      shapes[k].sphere.radius = (float)(i * 10 + k + 1);
    }

    entries[i] = shapes_write(&world, BRACKET_EIGHT, shapes, 6);
    assert(entries[i].offset == i * 8);
    assert(entries[i].count == 6);
  }

  assert(entries[64].offset == 64 * 8);
  assert(entries[65].offset == 65 * 8);
  assert(world.shape_brackets[BRACKET_EIGHT].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_EIGHT].slots[1] == 3);

  shapes_clear_slot(&world, BRACKET_EIGHT, entries[65].offset);

  body_shape replacement_shapes[8] = {0};
  for (count_t i = 0; i < 5; ++i) {
    replacement_shapes[i].type = BND_BOX;
    replacement_shapes[i].box.size.x = (float)(300 + i);
  }

  body_shapes replacement = shapes_write(&world, BRACKET_EIGHT, replacement_shapes, 5);
  body_shape *stored_replacement = shapes_get(&world, replacement);
  body_shape *preserved_neighbor = world.shape_brackets[BRACKET_EIGHT].shapes + entries[64].offset;

  assert(replacement.offset == entries[65].offset);
  assert(replacement.count == 5);
  assert(memcmp(stored_replacement, replacement_shapes, 5 * sizeof(body_shape)) == 0);
  assert(preserved_neighbor[0].type == BND_SPHERE);
  assert(preserved_neighbor[0].sphere.radius == 641.0f);

  shapes_teardown(&world);
}
#endif
