#include "raylib.h"
#include "scenario-core.h"
#include "bnd-math.h"
#include "raymath.h"
#include <unistd.h>
#include <stdlib.h>

bool is_collision;
bnd_raycast_hit hit;

uint8_t *memory;
uint64_t offset, size;

static void *memory_alloc(uint64_t alignment, uint64_t bytes) {
  offset = (offset + alignment - 1) & ~(alignment - 1);
  if (offset + bytes > size) {
    TraceLog(LOG_FATAL, "Memory exceeded. Available: %lu bytes, requested: %lu bytes", size, offset + bytes);
    return NULL;
  }

  void *ptr = memory + offset;
  offset += bytes;
  return ptr;
}

void scenario_configure(program_config *config, bnd_config *physics) {
  config->window_title = "Rigidbodies";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };

  physics->memory.dynamics_capacity = 64;
  physics->memory.statics_capacity = 5;
  physics->memory.contacts_capacity = 128;

  size = bnd_required_memory(physics);
  memory = malloc(size);
  offset = 0;

  config->custom_malloc = memory_alloc;
}

void scenario_initialize(bnd_world *world) {
  TraceLog(LOG_INFO, "Allocated %lu, used %lu bytes", size, offset);
}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  bnd_body_handle big_box = bnd_add_box_static(world, (bnd_v3){ 10, 3, 1 }).value;
  bnd_set_position(world, big_box, (bnd_v3){ 0, 1.5, -5 });

  big_box = bnd_add_box_static(world, (bnd_v3){ 10, 3, 1 }).value;
  bnd_set_position(world, big_box, (bnd_v3){ 0, 1.5, 5 });

  big_box = bnd_add_box_static(world, (bnd_v3){ 1, 3, 10 }).value;
  bnd_set_position(world, big_box, (bnd_v3){ -7, 1.5, 0 });

  big_box = bnd_add_cylinder_static(world, 1, 3).value;
  bnd_set_position(world, big_box, (bnd_v3){ 0, 1.5, 0 });
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_handle_input(bnd_world *world, Camera *cam) {
  if (IsKeyPressed(KEY_X)) {
    bnd_body_handle big_box = bnd_add_box_dynamic(world, 10, (bnd_v3){ 1.3, 1.3, 1.3 }).value;
    bnd_set_position(world, big_box, (bnd_v3){ 0, 7, 0 });
    bnd_set_angular_momentum(world, big_box, (bnd_v3){ 1, 1, 1 });
  }

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    bnd_v3 direction = Vector3Normalize(Vector3Subtract(cam->target, cam->position));

    bnd_body_handle ball = bnd_add_sphere_dynamic(world, 3, 0.7).value;
    bnd_set_position(world, ball, bnd_v3_add(cam->position, direction));
    bnd_apply_impulse(world, ball, bnd_v3_scale(direction, 70));
  }

  if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
    Ray r = GetScreenToWorldRay(GetMousePosition(), *cam);

    bnd_ray ray = {
      .origin = r.position,
      .direction = r.direction,
      .max_distance = 100.0,
    };

    bnd_raycast_hit raycast_hit;
    if (bnd_raycast_closest(world, ray, &raycast_hit)) {
      bnd_body_shape shapes[1];
      bnd_result_u32 num_shapes = bnd_get_shapes(world, raycast_hit.body, shapes, 1);

      if (num_shapes.value > 0 && shapes[0].type != BND_PLANE) {
        bnd_remove_body(world, raycast_hit.body);
      }
    }
  }
}

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {
  free(memory);
}
