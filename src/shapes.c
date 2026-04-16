#include "physics.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SHAPE_BRACKET_BLOCK_CAPACITY 64

static count_t bracket_block_count(const physics_world *world, shape_dimension_bracket bracket) {
  return world->shape_brackets[bracket].capacity / SHAPE_BRACKET_BLOCK_CAPACITY;
}

static void *allocate_bracket(count_t block_count, count_t shapes_count) {
  void *bracket = malloc(block_count * sizeof(uint64_t) + shapes_count * sizeof(body_shape));

  memset(bracket, 0, block_count * sizeof(uint64_t));

  return bracket;
}

void shapes_init(physics_world *world) {
  const physics_config *config = &world->config;

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count = config->shapes_brackets_capacity[i] / SHAPE_BRACKET_BLOCK_CAPACITY +
                           ((config->shapes_brackets_capacity[i] & (SHAPE_BRACKET_BLOCK_CAPACITY - 1)) > 0);
    count_t bracket_capacity = blocks_count * SHAPE_BRACKET_BLOCK_CAPACITY;

    count_t bracket_dimension = 1 << i;
    count_t shapes_count = bracket_capacity * bracket_dimension;

    world->shape_brackets[i] =
        (shapes_bracket){.capacity = bracket_capacity, .bracket = allocate_bracket(blocks_count, shapes_count)};
  }
}

void shapes_teardown(physics_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    free(world->shape_brackets[i].bracket);
  }
}

void shapes_reset(physics_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    shapes_bracket *bracket = &world->shape_brackets[i];
    count_t blocks_count = bracket_block_count(world, i);

    bracket->capacity = 0;
    memset(bracket->bracket, 0, blocks_count * sizeof(uint64_t));
  }
}

bool shapes_any_slot_available(const physics_world *world, shape_dimension_bracket bracket) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *header = (uint64_t *)world->shape_brackets[bracket].bracket;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (header[i] < (uint64_t)~0) {
      return true;
    }
  }

  return false;
}

void shapes_expand_bracket(physics_world *world, shape_dimension_bracket bracket) {
  count_t bracket_capacity = 1 << bracket;

  count_t current_capacity = world->shape_brackets[bracket].capacity;
  count_t current_block_count = bracket_block_count(world, bracket);
  body_shape *current_bracket = world->shape_brackets[bracket].bracket;

  count_t new_capacity = current_capacity + SHAPE_BRACKET_BLOCK_CAPACITY;
  count_t new_block_count = current_block_count + 1;
  count_t shapes_count = bracket_capacity * new_block_count * SHAPE_BRACKET_BLOCK_CAPACITY;

  uint64_t *new_bracket = allocate_bracket(new_block_count, shapes_count);
  memcpy(new_bracket, current_bracket, current_block_count * sizeof(uint64_t));

  body_shape *shapes = (body_shape *)&new_bracket[new_block_count];
  body_shape *old_shapes = (body_shape *)((uint64_t *)current_bracket + current_block_count);
  memcpy(shapes, old_shapes, current_capacity * bracket_capacity * sizeof(body_shape));

  world->shape_brackets[bracket] = (shapes_bracket){.capacity = new_capacity, .bracket = (body_shape *)new_bracket};

  free(current_bracket);
}

bool shapes_put_into_empty_slot(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes,
                                count_t shapes_count, count_t *slot_number) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *header = (uint64_t *)world->shape_brackets[bracket].bracket;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (header[i] == (uint64_t)~0)
      continue;

    for (count_t k = 0; k < SHAPE_BRACKET_BLOCK_CAPACITY; ++k) {
      uint64_t mask = (uint64_t)1 << k;
      if ((header[i] & mask) != 0)
        continue;

      count_t bracket_capacity = 1 << bracket;
      body_shape *shapes_buffer = (body_shape *)(header + blocks_count);

      body_shape *slot = shapes_buffer + i * SHAPE_BRACKET_BLOCK_CAPACITY + k * bracket_capacity;
      memcpy(slot, shapes, shapes_count * sizeof(body_shape));

      header[i] |= mask;
      *slot_number = i * SHAPE_BRACKET_BLOCK_CAPACITY + k;

      return true;
    }
  }

  return false;
}

void shapes_clear_slot(physics_world *world, shape_dimension_bracket bracket, count_t slot) {
  count_t block_count = bracket_block_count(world, bracket);

  uint64_t *header = (uint64_t *)world->shape_brackets[bracket].bracket;
  count_t block_index = slot / SHAPE_BRACKET_BLOCK_CAPACITY;
  if (block_index < block_count) {
    header[block_index] &= ~((uint64_t)1 << (slot % SHAPE_BRACKET_BLOCK_CAPACITY));
  }
}

body_shapes shapes_write(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes, count_t count) {
  const count_t max_count = 1 << (BRACKET_COUNT - 1);
  assert(count <= max_count);

  if (!shapes_any_slot_available(world, bracket)) {
    shapes_expand_bracket(world, bracket);
  }

  count_t shape_slot;
  shapes_put_into_empty_slot(world, bracket, shapes, count, &shape_slot);

  return (body_shapes){.bracket = bracket, .offset = shape_slot, .count = count};
}

body_shape *shapes_get(const physics_world *world, body_shapes shapes) {
  count_t block_count = bracket_block_count(world, shapes.bracket);
  body_shape *bracket = (body_shape *)((uint64_t *)world->shape_brackets[shapes.bracket].bracket + block_count);
  count_t bracket_capacity = 1 << shapes.bracket;

  return bracket + shapes.offset * bracket_capacity;
}
