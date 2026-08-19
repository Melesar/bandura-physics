#ifndef LOAD_TESTING_H
#define LOAD_TESTING_H

#include "bandura.h"
#include "bnd-math.h"
#include <stdlib.h>

static void add_box_static_at(bnd_world *world, bnd_v3 size, bnd_v3 position) {
  bnd_body_handle handle = bnd_add_box_static(world, size).value;
  bnd_set_position(world, handle, position);
}

static void add_static_enclosure(bnd_world *world, float half_extent, float height) {
  const float wall_thickness = 0.5f;
  const float wall_center = half_extent + wall_thickness * 0.5f;

  add_box_static_at(
       world,
      (bnd_v3){2.0f * (half_extent + wall_thickness), wall_thickness, 2.0f * (half_extent + wall_thickness)},
      (bnd_v3){0.0f, -wall_thickness * 0.5f, 0.0f});
  add_box_static_at(
       world,
       (bnd_v3){wall_thickness, height, 2.0f * (half_extent + wall_thickness)},
       (bnd_v3){wall_center, height * 0.5f, 0.0f});
  add_box_static_at(
       world,
       (bnd_v3){wall_thickness, height, 2.0f * (half_extent + wall_thickness)},
       (bnd_v3){-wall_center, height * 0.5f, 0.0f});
  add_box_static_at(
       world,
       (bnd_v3){2.0f * (half_extent + wall_thickness), height, wall_thickness},
       (bnd_v3){0.0f, height * 0.5f, wall_center});
  add_box_static_at(
       world,
       (bnd_v3){2.0f * (half_extent + wall_thickness), height, wall_thickness},
       (bnd_v3){0.0f, height * 0.5f, -wall_center});
}

static void sparse_awake_grid(bnd_world *world, uint32_t size_x, uint32_t size_y, uint32_t size_z) {
  for (uint32_t  y = 0; y < size_y; ++y) {
    for (uint32_t  z = 0; z < size_z; ++z) {
      for (uint32_t  x = 0; x < size_x; ++x) {
        bnd_body_handle body = bnd_add_box_dynamic(world, 1.0f, (bnd_v3){1.0f, 1.0f, 1.0f}).value;

        bnd_set_position(world, body, (bnd_v3) {4.0f * (float)x, 4.0f * (float)y, 4.0f * (float)z});
        bnd_set_velocity(world, body, (bnd_v3){0.70f, 0.20f, -0.30f});
        bnd_set_angular_momentum(world, body, (bnd_v3){0.20f, -0.10f, 0.15f});
      }
    }
  }
}

static void dense_settling_pile(bnd_world *world, uint32_t size) {
  const float box_size = 1.0f;
  const float spacing = 0.92f;

  const float half_extent =
      0.5f * (float)(size - 1U) * spacing + box_size;

  add_static_enclosure(world, half_extent, (float)size * spacing + 2.0f);

  for (uint32_t y = 0; y < size; ++y) {
    for (uint32_t z = 0; z < size; ++z) {
      for (uint32_t x = 0; x < size; ++x) {
        const bnd_v3 position = {
            ((float)x - 0.5f * (float)(size - 1U)) * spacing,
            0.5f * box_size + (float)y * spacing,
            ((float)z - 0.5f * (float)(size - 1U)) * spacing,
        };
        bnd_body_handle body = bnd_add_box_dynamic(world, 1.0f, (bnd_v3){box_size, box_size, box_size}).value;
        bnd_set_position(world, body, position);
      }
    }
  }
}

static void compound_crowd(bnd_world *world, uint32_t grid_x, uint32_t grid_y, uint32_t grid_z) {
  const float box_size = 0.60f;
  const float shape_offset = 0.35f;
  const float spacing = 1.10f;
  const float half_extent = 0.5f * (float)(grid_x - 1U) * spacing + 1.0f;

  add_static_enclosure(world, half_extent, (float)(grid_y) * spacing + 2.0f);

  for (uint32_t y = 0; y < grid_y; ++y) {
    for (uint32_t z = 0; z < grid_z; ++z) {
      for (uint32_t x = 0; x < grid_x; ++x) {
        bnd_body_shape shapes[4] = {
          {BND_BOX, (bnd_shape){ .box = { .size = { box_size, box_size, box_size } } }, (bnd_v3){shape_offset, 0.0f, 0.0f}, bnd_quat_identity()},
          {BND_BOX, (bnd_shape){ .box = { .size = { box_size, box_size, box_size } } }, (bnd_v3){-shape_offset, 0.0f, 0.0f}, bnd_quat_identity()},
          {BND_BOX, (bnd_shape){ .box = { .size = { box_size, box_size, box_size } } }, (bnd_v3){0.0f, 0.0f, shape_offset}, bnd_quat_identity()},
          {BND_BOX, (bnd_shape){ .box = { .size = { box_size, box_size, box_size } } }, (bnd_v3){0.0f, 0.0f, -shape_offset}, bnd_quat_identity()},
        };
        float masses[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        const bnd_v3 position = {
            ((float)x - 0.5f * (float)(grid_x - 1U)) * spacing,
            0.5f * box_size + (float)y * spacing,
            ((float)z - 0.5f * (float)(grid_z - 1U)) * spacing
        };
        bnd_body_handle body = bnd_add_compound_body_dynamic(world, shapes, masses, 4).value;
        bnd_set_position(world, body, position);
      }
    }
  }
}

static void joints_lattice(bnd_world *world, uint32_t side, bnd_body_handle *anchors) {
  const float radius = 0.15f;
  const float spacing = 1.20f;
  const float max_joint_distance = 1.00f;
  const float anchor_distance = 0.40f;
  bnd_body_handle *bodies = (bnd_body_handle*)malloc(side * side * sizeof(bnd_body_handle));

  for (uint32_t y = 0; y < side; ++y) {
    for (uint32_t x = 0; x < side; ++x) {
      bnd_body_handle body = bnd_add_sphere_dynamic(world, 1.0f, radius).value;
      bnd_set_position(world, body, (bnd_v3){(float)x * spacing, (float)y * spacing, 0.0f});
      bodies[y * side + x] = body;
    }
  }

  for (uint32_t x = 0; x < side; ++x) {
    bnd_body_handle anchor = bnd_add_sphere_static(world, 0.01f).value;
    bnd_set_position(world, anchor, (bnd_v3){(float)x * spacing, (float)(side - 1U) * spacing + anchor_distance, 0.0f});
    anchors[x] = anchor;
  }

  for (unsigned int y = 0; y < side; ++y) {
    for (unsigned int x = 0; x < side; ++x) {
      const bnd_body_handle body = bodies[y * side + x];
      if (x + 1U < side) {
        bnd_add_joint(world, body, bodies[y * side + x + 1U], (bnd_v3){0.0f, 0.0f, 0.0f}, (bnd_v3){0.0f, 0.0f, 0.0f}, max_joint_distance);
      }
      if (y + 1U < side) {
        bnd_add_joint(world, body,
          bodies[(y + 1U) * side + x], (bnd_v3){0.0f, 0.0f, 0.0f}, (bnd_v3){0.0f, 0.0f, 0.0f}, max_joint_distance);
      }
    }
  }

  for (unsigned int x = 0; x < side; ++x) {
    bnd_add_joint(world, bodies[(side - 1U) * side + x], anchors[x], (bnd_v3){0.0f, 0.0f, 0.0f}, (bnd_v3){0.0f, 0.0f, 0.0f}, 0.20f);
  }

  free(bodies);
}

static void drive_joint_lattice(bnd_world *world, uint32_t step, bnd_body_handle *anchors, uint32_t side) {

  const float spacing = 1.20f;
  const float anchor_distance = 0.40f;
  const float phase = (float)step * 0.19f;

  for (uint32_t x = 0; x < side; ++x) {
    const float anchor_phase = phase + (float)x * 0.37f;
    const bnd_v3 position = {
      (float)(x) * spacing + 0.12f * sinf(anchor_phase),
      (float)(side - 1U) * spacing + anchor_distance + 0.10f * sinf(anchor_phase * 0.7f),
      0.10f * cosf(anchor_phase),
    };
    bnd_set_position(world, anchors[x], position);
  }
}

#endif
