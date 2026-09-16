#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <string.h>

#define ALIGNMENT_CONSTRAINT 4

typedef struct {
  bnd_v3 relative_position[2];
  bnd_v3 local_velocity;
  float normal_mass, tangent_mass[4];
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

typedef struct {
  count_t *constraint_count;
  float dt;
} constraint_creation_context;

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

  constraint_creation_context *cx = (constraint_creation_context *) custom_data;
  bnd_result_ptr constraint_ptr = arena_alloc(&world->arena, ALIGNMENT_CONSTRAINT, sizeof(contact_constraint));
  PROPAGATE_ERROR(constraint_ptr.error)

  dynamic_bodies *dynamics = &world->dynamics;
  const common_data *data_b = as_common_const(world, type);

  count_t body_ids[] = {
    dynamics->outer_lookup[contact->body_a].index,
    data_b->outer_lookup[contact->body_b].index,
  };
  count_t body_count = type == BND_BODY_DYNAMIC ? 2 : 1;

  contact_constraint *constraint = constraint_ptr.value;
  constraint->type = type;
  constraint->contact_index = index;
  constraint->body_a = body_ids[0];
  constraint->body_b = body_ids[1];
  constraint->friction = contact->friction;
  constraint->restitution = contact->restitution;
  constraint->points_count = contact->manifold.count;
  constraint->basis = contact_space_transform(contact);

  bnd_m3 world_to_contact = bnd_m3_transpose(constraint->basis);

  bnd_v3 position[2];
  bnd_quat rotation[2];
  bnd_v3 velocity[2];
  bnd_v3 angular_momentum[2];
  bnd_m3 inv_inertia_tensor[2];
  bnd_m3 inv_inertia[2];
  bnd_v3 angular_velocity[2];
  float inv_mass[2];
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];

    inv_mass[k] = dynamics->inv_masses[body_index];
    position[k] = dynamics->positions[body_index];
    rotation[k] = dynamics->rotations[body_index];
    velocity[k] = dynamics->velocities[body_index];
    angular_momentum[k] = dynamics->angular_momenta[body_index];
    inv_inertia_tensor[k] = dynamics->inv_inertia_tensors[body_index];
    inv_inertia[k] = bnd_m3_inertia(inv_inertia_tensor[k], rotation[k]);

    angular_velocity[k] = bnd_m3_rotate(angular_momentum[k], inv_inertia[k]);
    dynamics->inv_intertias[body_index] = inv_inertia[k];
  }

  for (count_t i = 0; i < contact->manifold.count; ++i) {
    contact_point *mp = &contact->manifold.points[i];
    constraint_point *cp = &constraint->points[i];

    cp->separation = mp->depth;

    bnd_v3 local_velocity[2] = {0};
    bnd_m3 effective_mass = {0};
    for (count_t k = 0; k < 2; ++k) {
      cp->relative_position[k] = bnd_v3_sub(mp->point, position[k]);

      bnd_v3 vel = bnd_v3_add(velocity[k], bnd_v3_cross(angular_velocity[k], cp->relative_position[k]));
      local_velocity[k] = bnd_m3_rotate(vel, world_to_contact);

      bnd_m3 r_cross = bnd_m3_skew_symmetric(cp->relative_position[k]);

      bnd_m3 body_effective_mass = bnd_m3_multiply(r_cross, inv_inertia[k]);
      body_effective_mass = bnd_m3_multiply(body_effective_mass, r_cross);
      body_effective_mass = bnd_m3_negate(body_effective_mass);

      effective_mass = bnd_m3_add(effective_mass, body_effective_mass);
    }

    cp->local_velocity = bnd_v3_sub(local_velocity[0], local_velocity[1]);

    effective_mass = bnd_m3_multiply(world_to_contact, effective_mass);
    effective_mass = bnd_m3_multiply(effective_mass, constraint->basis);
    effective_mass.m0[0] += inv_mass[0] + inv_mass[1];
    effective_mass.m1[1] += inv_mass[0] + inv_mass[1];
    effective_mass.m2[2] += inv_mass[0] + inv_mass[1];

    float normal_mass = effective_mass.m1[1];
    cp->normal_mass = normal_mass > EPSILON ? 1.0f / normal_mass : 0.0f;

    float tan_00 = effective_mass.m0[0];
    float tan_01 = effective_mass.m0[2]; 
    float tan_10 = effective_mass.m2[0];
    float tan_11 = effective_mass.m2[2];
    float tan_det = tan_00 * tan_11 - tan_01 * tan_10;
    if (tan_det > EPSILON) {
      tan_det = 1.0f / tan_det;

      cp->tangent_mass[0] = tan_11 * tan_det;
      cp->tangent_mass[1] = -tan_01 * tan_det;
      cp->tangent_mass[2] = -tan_10 * tan_det;
      cp->tangent_mass[3] = tan_00 * tan_det;
    } else {
      memset(cp->tangent_mass, 0, sizeof(cp->tangent_mass));
    }
  }

  *cx->constraint_count += 1;

  return OK;
}

bnd_error resolve_constraints(bnd_world *world, float dt) {
  (void) dt;

  PROFILER_FUNCTION_START
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame(&world->arena);

  count_t total_count;
  contact_constraint *constraints = (contact_constraint *)stack_frame.arena->buffer;

  constraint_creation_context cx = { &total_count, dt };
  bnd_error e = for_each_broad_contact(world, constraints_from_contacts, &cx);
  if (IS_ERROR(e)) {
    arena_release_stack_frame(stack_frame);
    return e;
  }

  for (count_t i = 0; i < total_count; ++i) {
    contact_constraint *c = &constraints[i];
  }


  arena_release_stack_frame(stack_frame);
  PROFILER_FUNCTION_END

  return OK;
}


