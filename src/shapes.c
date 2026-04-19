#include "physics.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SHAPE_BRACKET_BLOCK_CAPACITY 64

static count_t bracket_block_count(const physics_world *world, shape_dimension_bracket bracket) {
  return world->shape_brackets[bracket].capacity / SHAPE_BRACKET_BLOCK_CAPACITY;
}

static shapes_bracket allocate_bracket(count_t capacity, count_t block_count, count_t shapes_count) {
  uint64_t *slots = malloc(block_count * sizeof(uint64_t));
  body_shape *shapes = malloc(shapes_count * sizeof(body_shape));

  memset(slots, 0, block_count * sizeof(uint64_t));

  return (shapes_bracket){slots, shapes, capacity};
}

void shapes_init(physics_world *world) {
  const physics_config *config = &world->config;

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count = config->shapes_brackets_capacity[i] / SHAPE_BRACKET_BLOCK_CAPACITY +
                           ((config->shapes_brackets_capacity[i] & (SHAPE_BRACKET_BLOCK_CAPACITY - 1)) > 0);
    count_t bracket_capacity = blocks_count * SHAPE_BRACKET_BLOCK_CAPACITY;

    count_t bracket_dimension = 1 << i;
    count_t shapes_count = bracket_capacity * bracket_dimension;

    world->shape_brackets[i] = allocate_bracket(bracket_capacity, blocks_count, shapes_count);
  }
}

void shapes_teardown(physics_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    free(world->shape_brackets[i].slots);
    free(world->shape_brackets[i].shapes);
  }
}

void shapes_reset(physics_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    shapes_bracket *bracket = &world->shape_brackets[i];
    count_t blocks_count = bracket_block_count(world, i);

    bracket->capacity = 0;
    memset(bracket->slots, 0, blocks_count * sizeof(uint64_t));
  }
}

bool shapes_any_slot_available(const physics_world *world, shape_dimension_bracket bracket) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] < (uint64_t)~0) {
      return true;
    }
  }

  return false;
}

void shapes_expand_bracket(physics_world *world, shape_dimension_bracket bracket) {
  count_t bracket_capacity = 1 << bracket;

  count_t current_capacity = world->shape_brackets[bracket].capacity;
  count_t current_block_count = bracket_block_count(world, bracket);
  uint64_t *current_slots = world->shape_brackets[bracket].slots;
  body_shape *current_shapes = world->shape_brackets[bracket].shapes;

  count_t new_capacity = current_capacity + SHAPE_BRACKET_BLOCK_CAPACITY;
  count_t new_block_count = current_block_count + 1;
  count_t shapes_count = bracket_capacity * new_block_count * SHAPE_BRACKET_BLOCK_CAPACITY;

  shapes_bracket new_bracket = allocate_bracket(new_capacity, new_block_count, shapes_count);
  memcpy(new_bracket.slots, current_slots, current_block_count * sizeof(uint64_t));
  memcpy(new_bracket.shapes, current_shapes, current_capacity * bracket_capacity * sizeof(body_shape));

  world->shape_brackets[bracket] = new_bracket;

  free(current_shapes);
  free(current_slots);
}

bool shapes_put_into_empty_slot(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes,
                                count_t shapes_count, count_t *slot_number) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;
  body_shape *shapes_buffer = world->shape_brackets[bracket].shapes;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] == (uint64_t)~0)
      continue;

    for (count_t k = 0; k < SHAPE_BRACKET_BLOCK_CAPACITY; ++k) {
      uint64_t mask = (uint64_t)1 << k;
      if ((slots[i] & mask) != 0)
        continue;

      count_t bracket_capacity = 1 << bracket;
      count_t shape_offset = (i * SHAPE_BRACKET_BLOCK_CAPACITY + k) * bracket_capacity;

      body_shape *slot = shapes_buffer + shape_offset;
      memcpy(slot, shapes, shapes_count * sizeof(body_shape));

      slots[i] |= mask;
      *slot_number = shape_offset;

      return true;
    }
  }

  return false;
}

void shapes_clear_slot(physics_world *world, shape_dimension_bracket bracket, count_t slot) {
  count_t block_count = bracket_block_count(world, bracket);
  count_t bracket_capacity = 1 << bracket;

  uint64_t *slots = world->shape_brackets[bracket].slots;
  count_t block_index = slot / (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity);
  count_t bit_index = slot % (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity);
  if (block_index < block_count) {
    slots[block_index] &= ~((uint64_t)1 << bit_index);
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
  return world->shape_brackets[shapes.bracket].shapes + shapes.offset;
}
