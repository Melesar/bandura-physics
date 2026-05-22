#include "bandura.h"
#include "bnd-core.h"

static void draw_contacts(const bnd_world *world, bnd_debug_draw_callbacks callbacks) {
  for (count_t i = 0; i < world->contacts.count; i++) {
    const contact *contact = &world->contacts.values[i];
    callbacks.draw_contact(contact->point, contact->normal, contact->depth);
  }
}

static void draw_shapes(const bnd_world *world, bnd_body_type type, bnd_debug_draw_callbacks callbacks) {
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
      callbacks.draw_shape(center, rotation, shape.type);
    }
  }
}

void bnd_debug_draw(const bnd_world *world, bnd_debug_draw_flags flags, bnd_debug_draw_callbacks callbacks) {
  if (flags & BND_DEBUG_DRAW_CONTACTS) {
    draw_contacts(world, callbacks);
  }
  if (flags & BND_DEBUG_DRAW_SHAPES_DYNAMIC) {
    draw_shapes(world, BND_BODY_DYNAMIC, callbacks);
  }
  if (flags & BND_BODY_STATIC) {
    draw_shapes(world, BND_BODY_STATIC, callbacks);
  }
}
