#include "bandura.h"
#include "bnd-core.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define MAX_MESSAGE_SIZE 512

#define INVOKE(invocation) \
  e = invocation; \
  if (e.type != BND_OK) { \
    bnd_teardown(world); \
    return e; \
  }

#define ALLOC(buffer, size) \
  buffer = allocator.malloc(4, size); \
  if (buffer == NULL) { \
    bnd_teardown(world); \
    return (bnd_error){ BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc  to allocate memory" }; \
  }

BND_RESULT_FUNC_DECL(world, bnd_world*)
BND_RESULT_FUNC_DECL(v3, bnd_v3)
BND_RESULT_FUNC_DECL(quat, bnd_quat)
BND_RESULT_FUNC_DECL(aabb, bnd_aabb)
BND_RESULT_FUNC_DECL(u32, uint32_t)
BND_RESULT_FUNC_DECL(bool, bool)
BND_RESULT_FUNC_DECL(handle, bnd_body_handle)
BND_RESULT_FUNC_DECL(material, bnd_material_handle)
BND_RESULT_FUNC_DECL(layer, bnd_collision_layer)
BND_RESULT_FUNC_DECL(ptr, void*)

const count_t max_body_index = (count_t)~0 >> 9;
count_t next_world_id;

static void *std_malloc(uint64_t alignment, uint64_t size) {
  // malloc aligns its memory at 16-bytes boundary, which is sufficient for all allocations inside the engine.
  (void) alignment;
  return malloc(size);
}

static void *std_realloc(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size) {
  (void) alignment;
  (void) old_size;
  return realloc(ptr, new_size);
}

static void std_free(void *ptr, uint64_t size) {
  (void) size;
  free(ptr);
}

bnd_allocator bnd_default_allocator(void) {
  return (bnd_allocator){
    .malloc = std_malloc,
    .realloc = std_realloc,
    .free = std_free,
  };
}

uint64_t bnd_required_memory(const bnd_config *config) {
  uint64_t size = sizeof(bnd_world);

  uint64_t common_size = sizeof(bnd_v3)
    + sizeof(bnd_quat)
    + sizeof(body_shapes)
    + sizeof(bnd_aabb)
    + sizeof(bnd_material_handle)
    + sizeof(void*)
    + sizeof(bnd_collision_layer)
    + sizeof(bnd_event_type)
    + sizeof(event_link)
    + sizeof(uint8_t) * 2
    + sizeof(count_t)
    + sizeof(outer_lookup_node)
    + sizeof(count_t);

  uint64_t dynamic_size = common_size
    + 4 * sizeof(bnd_v3)
    + sizeof(float)
    + 2 * sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(float);

  uint64_t contact_size = sizeof(contact);
  uint64_t joint_size = sizeof(bnd_joint) + sizeof(count_t);
  uint64_t mesh_size = sizeof(bnd_v3) * DEFAULT_VERTEX_PER_MESH
    + sizeof(uint32_t) * DEFAULT_FACE_PER_MESH * 3
    + sizeof(submesh)
    + sizeof(bnd_mesh)
    + sizeof(bnd_m3)
    + sizeof(float)
    + sizeof(bnd_aabb);
  uint64_t material_size = sizeof(body_material);

  uint64_t event_size = sizeof(bnd_event) + sizeof(count_t);
  uint64_t shapes_size = 0;
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count, shapes_count, bracket_capacity;
    shapes_get_bracket_properties(config, i, &blocks_count, &shapes_count, &bracket_capacity);

    shapes_size += shapes_count * sizeof(bnd_body_shape)
      + blocks_count * sizeof(uint64_t);
  }

  uint64_t arena_size = config->memory.internal_allocation_budget;

  uint64_t cache_hash_table_capacity = 1;
  while (cache_hash_table_capacity < config->advanced.contacts_cache.hash_table_capacity) {
    cache_hash_table_capacity *= 2;
  }

  uint64_t contacts_cache_size = cache_hash_table_capacity * sizeof(uint32_t)
    + config->advanced.contacts_cache.buffer_capacity * sizeof(cache_entry);

  size += (config->memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT) * dynamic_size
    + (config->memory.statics_capacity + EPHEMERAL_BODIES_COUNT) * common_size
    + config->memory.contacts_capacity * contact_size
    + config->memory.joints_capacity * joint_size
    + config->memory.meshes_capacity * mesh_size
    + config->memory.events_capacity * event_size
    + config->memory.materials_capacity * material_size
    + shapes_size
    + arena_size
    + contacts_cache_size;

  // Alignment
  size += 9 * 7; // 8-bytes for world, shapes slots, EPA polytope, custom data and the cache entries buffer
  size += 46 * 3; // 4-bytes for the rest of the buffers

  return size;
}

static bnd_error init_commons(common_data *data, count_t capacity, bnd_allocator allocator) {
  data->capacity = capacity;
  data->count = 0;
  data->free_count = 0;
  data->first_outer_node = max_body_index;

  count_t total_capacity = capacity + EPHEMERAL_BODIES_COUNT;
  ALLOC_BUFFER4(data->positions, sizeof(bnd_v3) * total_capacity);
  ALLOC_BUFFER4(data->rotations, sizeof(bnd_quat) * total_capacity);
  ALLOC_BUFFER4(data->shapes, sizeof(body_shapes) * total_capacity);
  ALLOC_BUFFER4(data->aabbs, sizeof(bnd_aabb) * total_capacity);
  ALLOC_BUFFER4(data->materials, sizeof(bnd_material_handle) * total_capacity);
  ALLOC_BUFFER8(data->custom_data, sizeof(void*) * total_capacity);
  ALLOC_BUFFER1(data->flags, sizeof(uint8_t) * total_capacity);
  ALLOC_BUFFER1(data->collision_layers, sizeof(bnd_collision_layer) * total_capacity);
  ALLOC_BUFFER4(data->event_masks, sizeof(bnd_event_type) * total_capacity);
  ALLOC_BUFFER4(data->event_links, sizeof(event_link) * total_capacity);
  ALLOC_BUFFER4(data->free_list, sizeof(count_t) * total_capacity);
  ALLOC_BUFFER1(data->generations, sizeof(uint8_t) * total_capacity);
  ALLOC_BUFFER4(data->outer_lookup, sizeof(outer_lookup_node) * total_capacity);
  ALLOC_BUFFER4(data->inner_lookup, sizeof(count_t) * total_capacity);

  return OK;
}

static void teardown_commons(common_data *data, bnd_allocator allocator) {
  count_t total_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;

  allocator.free(data->positions, total_capacity * sizeof(bnd_v3));
  allocator.free(data->rotations, total_capacity * sizeof(bnd_quat));
  allocator.free(data->shapes, total_capacity * sizeof(body_shapes));
  allocator.free(data->aabbs, total_capacity * sizeof(bnd_aabb));
  allocator.free(data->materials, total_capacity * sizeof(bnd_material_handle));
  allocator.free(data->flags, total_capacity * sizeof(uint8_t));
  allocator.free(data->custom_data, total_capacity * sizeof(void*));
  allocator.free(data->collision_layers, total_capacity * sizeof(bnd_collision_layer));
  allocator.free(data->event_masks, total_capacity * sizeof(bnd_event_type));
  allocator.free(data->event_links, total_capacity * sizeof(event_link));
  allocator.free(data->free_list, total_capacity * sizeof(count_t));
  allocator.free(data->generations, total_capacity * sizeof(uint8_t));
  allocator.free(data->outer_lookup, total_capacity * sizeof(outer_lookup_node));
  allocator.free(data->inner_lookup, total_capacity * sizeof(count_t));
}

bnd_config bnd_default_config(void) {
  return (bnd_config){
    .simulation = {
      .gravity = (bnd_v3){0, -9.81f, 0},
      .linear_drag = 0.95f,
      .angular_drag = 0.8f,
      .bounciness = 0.2f,
      .friction = 0.9f,
      .sleep_base_bias = 0.5f,
      .sleep_threshold = 0.3f,
      .min_bounce_velocity = 0.25f,
    },
    .memory = {
      .dynamics_capacity = 32,
      .statics_capacity = 8,
      .contacts_capacity = 64,
      .joints_capacity = 64,
      .meshes_capacity = 32,
      .events_capacity = 128,
      .materials_capacity = 8,
      .internal_allocation_budget = 8 << 10, // 8 Kb
    },
    .advanced = {
      .max_gjk_iterations = 100,
      .epa_tolerance = 0.01f,
      .epa_max_nodes = 128,
      .resolution_attempts_factor = 15,
      .penetration_epsilon = 0.01f,
      .velocity_epsilon = 0.01f,
      .shapes_brackets_capacity = {64, 1, 1, 1, 1},
      .contacts_cache = {
        .max_age = 3,
        .hash_table_capacity = 256,
        .buffer_capacity = 64,
        .feature_distance_threshold = 0.02f,
        .separation_threshold = 0.05f,
      }
    },
  };
}

static bnd_error bnd_init_internal(bnd_world *world, bnd_config config, bnd_allocator allocator) {
  world->allocator = allocator;
  world->config = config;
  world->id = next_world_id++; // TODO: make this thread-safe.

  bnd_error e;
  INVOKE(init_commons((common_data *)&world->dynamics, config.memory.dynamics_capacity, allocator))
  INVOKE(init_commons((common_data *)&world->statics, config.memory.statics_capacity, allocator))

  const count_t vectors = sizeof(bnd_v3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t floats = sizeof(float) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t matrices = sizeof(bnd_m3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);

  world->statics.dirty = false;

  ALLOC(world->dynamics.forces, vectors);
  ALLOC(world->dynamics.torques, vectors);
  ALLOC(world->dynamics.impulses, vectors);
  ALLOC(world->dynamics.angular_impulses, vectors);
  ALLOC(world->dynamics.accelerations, vectors);

  ALLOC(world->dynamics.inv_masses, floats);
  ALLOC(world->dynamics.velocities, vectors);
  ALLOC(world->dynamics.angular_momenta, vectors);
  ALLOC(world->dynamics.inv_inertia_tensors, matrices);
  ALLOC(world->dynamics.inv_intertias, matrices);
  ALLOC(world->dynamics.motion_avgs, floats);

  INVOKE(arena_init(allocator, config.memory.internal_allocation_budget, &world->arena))

  world->matrix.matrix[0] = 1;
  for(count_t i = 1; i < MAX_COLLISION_LAYERS; ++i) {
    world->matrix.matrix[i] = 0;
  }

  world->matrix.layers_available = 1;

  world->dynamics.awake_count = 0;
  world->generation = 0;
  world->age = 0;

  INVOKE(contacts_init(world))
  INVOKE(joints_init(world))
  INVOKE(shapes_init(world))
  INVOKE(meshes_init(world))
  INVOKE(events_init(world))
  INVOKE(materials_init(world))

  world->epa_debug = NULL;

  return OK;
}

bnd_world *bnd_init(bnd_config config) {
  bnd_allocator allocator = bnd_default_allocator();
  bnd_world *world = allocator.malloc(8, sizeof(bnd_world));

  // The error is ignored here intentionally. With default allocator it *should not* fail, so we'd rather
  // provide a cleaner API by betting on a happy path.
  bnd_init_internal(world, config, allocator);
  return world;
}

bnd_result_world bnd_init_with_allocator(bnd_config config, bnd_allocator allocator) {
  if (allocator.malloc == NULL) {
    return BND_RESULT_ERR(world, BND_ERROR_INVALID_ALLOCATOR, "Allocator must define a malloc function");
  }

  bnd_world *world = allocator.malloc(8, sizeof(bnd_world));
  if (world == NULL) {
    return BND_RESULT_ERR(world, OOM_ERROR.type, OOM_ERROR.message);
  }

  memset(world, 0, sizeof(bnd_world));

  bnd_error e = bnd_init_internal(world, config, allocator);
  return (bnd_result_world) { e, world };
}

void bnd_teardown(bnd_world *world) {
  if (world->allocator.free == NULL) {
    return;
  }

  teardown_commons((common_data *)&world->dynamics, world->allocator);
  teardown_commons((common_data *)&world->statics, world->allocator);

  count_t dynamics_total_capacity = world->dynamics.capacity + EPHEMERAL_BODIES_COUNT;

  world->allocator.free(world->dynamics.forces, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.torques, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.impulses, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_impulses, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.accelerations, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.inv_masses, dynamics_total_capacity * sizeof(float));
  world->allocator.free(world->dynamics.velocities, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_momenta, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.inv_inertia_tensors, dynamics_total_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.inv_intertias, dynamics_total_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.motion_avgs, dynamics_total_capacity * sizeof(float));

  shapes_teardown(world);
  joints_teardown(world);
  contacts_teardown(world);
  meshes_teardown(world);
  events_teardown(world);

  world->allocator.free(world->arena.buffer, world->arena.capacity);
  world->allocator.free(world->materials.values, sizeof(body_material) * world->materials.capacity);
  world->allocator.free(world, sizeof(bnd_world));
}

count_t ephemeral_body_index(const common_data *data) {
  return data->capacity;
}

bnd_error arena_init(bnd_allocator allocator, uint64_t capacity, bnd_arena *arena) {
  ALLOC_BUFFER1(arena->buffer, capacity);

  arena->capacity = capacity;
  arena->allocator = allocator;
  arena->offset = 0;
  arena->max_offset = 0;

  return OK;
}

bnd_result_ptr arena_alloc(bnd_arena *arena, uint64_t alignment, uint64_t size) {
  uint64_t new_offset = AlignTo(arena->offset, alignment);
  if (new_offset + size > arena->capacity) {
    if (arena->allocator.realloc == NULL) {
      return (bnd_result_ptr)  { (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Internal allocation buffer is full" }, NULL };
    }

    uint64_t new_capacity = arena->capacity;
    while(new_offset + size > new_capacity) {
      new_capacity <<= 1;
    }

    uint8_t *buffer = arena->allocator.realloc(arena->buffer, 1, arena->capacity, new_capacity);
    if (buffer == NULL) {
      return (bnd_result_ptr)  { (bnd_error) { BND_ERROR_OUT_OF_MEMORY, "Allocator.realloc returned null" }, NULL };
    }

    arena->buffer = buffer;
    arena->capacity = new_capacity;
  }

  uint8_t *value = arena->buffer + new_offset;
  arena->offset = new_offset + size;
  if (arena->offset > arena->max_offset) {
    arena->max_offset = arena->offset;
  }
  
  return BND_RESULT_OK(ptr, value);
}

static void arena_reset_to(bnd_arena *arena, uint64_t offset) {
  arena->offset = offset;
}

bnd_arena_stack_frame arena_new_stack_frame(bnd_arena *arena) {
  return (bnd_arena_stack_frame) {
    .offset = arena->offset,
    .arena = arena,
  };
}

void arena_release_stack_frame(bnd_arena_stack_frame frame) {
  arena_reset_to(frame.arena, frame.offset);
}

void arena_reset(bnd_arena *arena) {
  arena_reset_to(arena, 0);
}

