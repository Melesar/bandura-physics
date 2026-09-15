#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <string.h>

typedef struct {
  bnd_v3 relative_position[2];
  bnd_v3 local_velocity;
  float separation;
} constraint_point;

typedef struct {
  bnd_body_type type;
  count_t contact_index;

  count_t points_count;
  count_t body_a, body_b;
  bnd_m3 basis;

  float friction, restitution;

  constraint_point points[MAX_CONTACTS_PER_PAIR];
} contact_constraint;

static bnd_m3 contact_space_transform(const broad_phase_contact *contact) {
  bnd_v3 y_axis = contact->manifold.normal;
  bnd_v3 x_axis, z_axis;

  if (fabsf(y_axis.z) > fabsf(y_axis.x)) {
    // Take (1, 0, 0) as initial guess
    const float s = 1.0f / sqrtf(y_axis.y * y_axis.y + y_axis.z * y_axis.z);

    z_axis.x = 0.0f;
    z_axis.y = s * y_axis.z;
    z_axis.z = -s * y_axis.y;

    x_axis.x = z_axis.y * y_axis.z - y_axis.y * z_axis.z;
    x_axis.y = y_axis.x * z_axis.z;
    x_axis.z = y_axis.x * z_axis.y;
  } else {
    // Take (0, 0, 1) as initial guess
    const float s = 1.0f / sqrtf(y_axis.x * y_axis.x + y_axis.y * y_axis.y);

    x_axis.x = -s * y_axis.y;
    x_axis.y = s * y_axis.x;
    x_axis.z = 0.0f;

    z_axis.x = -y_axis.z * x_axis.y;
    z_axis.y = x_axis.x * y_axis.z;
    z_axis.z = y_axis.x * x_axis.y - x_axis.x * y_axis.y;
  }

  return bnd_m3_from_basis(x_axis, y_axis, z_axis);
}


static bnd_error constraints_from_contacts(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data) {
  if (contact->manifold.count == 0) {
    return OK;
  }

  bnd_result_ptr constraint_ptr = arena_alloc(&world->arena, 4, sizeof(contact_constraint));
  PROPAGATE_ERROR(constraint_ptr.error)

  dynamic_bodies *dynamics = &world->dynamics;
  const common_data *data_b = as_common_const(world, type);

  // TODO body_a and body_b should be stable indices, now they are not
  count_t body_ids[] = {
    contact->body_a,
    contact->body_b
  };
  count_t body_count = type == BND_BODY_DYNAMIC ? 2 : 1;
  bnd_v3 angular_velocity[2];

  for (count_t k = 0; k < body_count; ++k) {
    bnd_m3 inv_inertia = bnd_m3_inertia(dynamics->inv_inertia_tensors[body_ids[k]], dynamics->rotations[body_ids[k]]);
    angular_velocity[k] = bnd_m3_rotate(dynamics->angular_momenta[body_ids[k]], inv_inertia);

    dynamics->inv_intertias[body_ids[k]] = inv_inertia;
  }

  contact_constraint *constraint = constraint_ptr.value;
  constraint->type = type;
  constraint->contact_index = index;
  constraint->body_a = body_ids[0];
  constraint->body_b = body_ids[1];
  constraint->friction = contact->friction;
  constraint->restitution = contact->restitution;
  constraint->points_count = contact->manifold.count;
  constraint->basis = contact_space_transform(contact);

  count_t *count = (count_t *) custom_data;
  *count += 1;

  return OK;
}

bnd_error resolve_constraints(bnd_world *world, float dt) {
  (void) dt;

  PROFILER_FUNCTION_START
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame(&world->arena);

  count_t total_count;
  contact_constraint *constraints = (contact_constraint *)stack_frame.arena->buffer;

  bnd_error e = for_each_broad_contact(world, constraints_from_contacts, &total_count);
  if (IS_ERROR(e)) {
    arena_release_stack_frame(stack_frame);
    return e;
  }


  arena_release_stack_frame(stack_frame);
  PROFILER_FUNCTION_END

  return OK;
}


