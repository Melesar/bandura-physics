#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <math.h>
#include <string.h>

#define ALIGNMENT_CONTACT_CONSTRAINT 4
#define ALIGNMENT_JOINT_CONSTRAINT   4

typedef struct {
  bnd_v3 relative_position[2];
  float normal_mass, tangent_mass[4];
  float normal_impulse;
  float tangent_impulse[2];
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
  bnd_body_type type;
  count_t joint_index;

  count_t body_a, body_b;
  bnd_v3 axis;

  bnd_v3 Jav;
  bnd_v3 Jbv;

  bnd_v3 Jaw;
  bnd_v3 Jbw;

  bnd_v3 relative_positions[2];

  float effective_mass;
  float bias;

  float lambda;
  float min_lambda;
  float max_lambda;
} joint_constraint;

typedef struct {
  float inv_dt;
  dynamic_bodies *dynamics;

  count_t current_iteration;

  contact_constraint *contacts;
  joint_constraint   *joints;

  count_t contacts_count;
  count_t joints_count;
  
  bnd_config_solver config;
} solver_context;


typedef struct {
  count_t body_count;
  count_t body_ids[2];

  bnd_v3 velocities[2];
  bnd_v3 momenta[2];
  bnd_v3 angular_velocities[2];
  float inv_masses[2];

  bnd_v3 normal;
  bnd_v3 tangent_a;
  bnd_v3 tangent_b;
} constraint_state;

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
  (void)contacts;

  if (contact->manifold.count == 0 || contact->status & CONTACT_TRIGGER_BOTH) {
    return OK;
  }

  count_t *constraints_count = custom_data;
  bnd_result_ptr constraint_ptr = arena_alloc(&world->arena, ALIGNMENT_CONTACT_CONSTRAINT, sizeof(contact_constraint));
  PROPAGATE_ERROR(constraint_ptr.error)

  dynamic_bodies *dynamics = &world->dynamics;
  const common_data *data_b = as_common_const(world, type);

  count_t body_ids[] = {
    dynamics->outer_lookup[contact->body_a].index,
    data_b->outer_lookup[contact->body_b].index,
  };
  count_t body_count = type == BND_BODY_DYNAMIC ? 2 : 1;

  for (count_t k = 0; k < body_count; ++k) {
    dynamics->flags[body_ids[k]] |= BODY_FLAG_IMPULSE_APPLIED;
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

  bnd_m3 world_to_contact = bnd_m3_transpose(constraint->basis);

  bnd_v3 position[2];
  bnd_quat rotation[2];
  bnd_v3 velocity[2];
  bnd_v3 angular_momentum[2];
  bnd_m3 inv_inertia_tensor[2];
  bnd_m3 inv_inertia[2];
  float inv_mass[2] = {0};
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];

    inv_mass[k] = dynamics->inv_masses[body_index];
    position[k] = dynamics->positions[body_index];
    rotation[k] = dynamics->rotations[body_index];
    velocity[k] = dynamics->velocities[body_index];
    angular_momentum[k] = dynamics->angular_momenta[body_index];
    inv_inertia_tensor[k] = dynamics->inv_inertia_tensors[body_index];
    inv_inertia[k] = bnd_m3_inertia(inv_inertia_tensor[k], rotation[k]);

    dynamics->inv_intertias[body_index] = inv_inertia[k];
  }

  for (count_t i = 0; i < contact->manifold.count; ++i) {
    contact_point *mp = &contact->manifold.points[i];
    constraint_point *cp = &constraint->points[i];

    cp->separation = -mp->depth;
    cp->normal_impulse = mp->normal_impulse;
    cp->tangent_impulse[0] = mp->tangential_impulse[0];
    cp->tangent_impulse[1] = mp->tangential_impulse[1];

    bnd_m3 effective_mass = {0};
    for (count_t k = 0; k < body_count; ++k) {
      cp->relative_position[k] = bnd_v3_sub(mp->point, position[k]);

      bnd_m3 r_cross = bnd_m3_skew_symmetric(cp->relative_position[k]);

      bnd_m3 body_effective_mass = bnd_m3_multiply(r_cross, inv_inertia[k]);
      body_effective_mass = bnd_m3_multiply(body_effective_mass, r_cross);
      body_effective_mass = bnd_m3_negate(body_effective_mass);

      effective_mass = bnd_m3_add(effective_mass, body_effective_mass);
    }

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

  *constraints_count += 1;

  return OK;
}

static bnd_result_u32 constraints_from_joints(bnd_world *world) {
  joints *joints = &world->joints;

  const bnd_config_solver config = world->config.solver;
  const dynamic_bodies *dynamics = &world->dynamics;
  const common_data *statics  = (common_data *)&world->statics;

  count_t count = 0;
  for (count_t i = 0; i < joints->count; ++i) {
    const bnd_joint *j = &joints->values[i];

    const bool is_dynamic = i < joints->dynamic_count;
    const common_data *data_a = (common_data *)dynamics;
    const common_data *data_b = is_dynamic ? (common_data *)dynamics : statics;

    const count_t index_a = data_a->outer_lookup[j->bodies[0].index].index;
    const count_t index_b = data_b->outer_lookup[j->bodies[1].index].index;

    bnd_v3 position_a = data_a->positions[index_a];
    bnd_v3 position_b = data_b->positions[index_b];

    bnd_quat rotation_a = data_a->rotations[index_a];
    bnd_quat rotation_b = data_b->rotations[index_b];

    bnd_v3 pa = bnd_v3_add(position_a, bnd_v3_rotate(j->anchors[0], rotation_a));
    bnd_v3 pb = bnd_v3_add(position_b, bnd_v3_rotate(j->anchors[1], rotation_b));

    bnd_v3 offset_a = bnd_v3_sub(pa, position_a);
    bnd_v3 offset_b = bnd_v3_sub(pb, position_b);

    switch (j->type) {
      case BND_JOINT_TYPE_DISTANCE: {
        bnd_v3 offset = bnd_v3_sub(pa, pb);
        float distance_sqr = bnd_v3_lensqr(offset);
        float threshold_min = j->min_distance * j->min_distance;
        float threshold_max = j->max_distance * j->max_distance;

        if (distance_sqr <= threshold_max && distance_sqr >= threshold_min) {
          continue;
        }

        count += 1;

        float distance = distance_sqr > EPSILON * EPSILON ? sqrtf(distance_sqr) : 0.0f;

        bnd_v3 normal = distance_sqr > EPSILON * EPSILON
          ? bnd_v3_scale(offset, 1.0f / distance)
          : bnd_v3_normalize(bnd_v3_sub(position_a, position_b));

        bnd_v3 ra_n = bnd_v3_cross(offset_a, normal);
        bnd_v3 rb_n = bnd_v3_cross(offset_b, normal);

        bnd_result_ptr allocation = arena_alloc(&world->arena, ALIGNMENT_JOINT_CONSTRAINT, sizeof(joint_constraint));
        PROPAGATE_RESULT(u32, allocation.error);

        joint_constraint *constraint = allocation.value;
        constraint->type = is_dynamic ? BND_BODY_DYNAMIC : BND_BODY_STATIC;
        constraint->joint_index = i;
        constraint->body_a = index_a;
        constraint->body_b = index_b;
        constraint->axis = normal;

        constraint->relative_positions[0] = offset_a;
        constraint->relative_positions[1] = offset_b;

        constraint->Jav = normal;
        constraint->Jaw = ra_n;

        if (is_dynamic) {
          constraint->Jbv = bnd_v3_negate(normal);
          constraint->Jbw = bnd_v3_negate(rb_n);
        } else {
          constraint->Jbv = constraint->Jbw = bnd_v3_zero();
        }

        bnd_m3 inv_inertia_a = bnd_m3_inertia(dynamics->inv_inertia_tensors[index_a], rotation_a);

        float K = dynamics->inv_masses[index_a];
        K += bnd_v3_dot(ra_n, bnd_m3_rotate(ra_n, inv_inertia_a));

        if (is_dynamic) {
          bnd_m3 inv_inertia_b = bnd_m3_inertia(dynamics->inv_inertia_tensors[index_b], rotation_b);

          K += dynamics->inv_masses[index_b];
          K += bnd_v3_dot(rb_n, bnd_m3_rotate(rb_n, inv_inertia_b));
        }

        constraint->effective_mass = K > EPSILON ? 1.0f / K : 0.0f;
        constraint->lambda = j->impulse;

        float error;
        if (distance_sqr < threshold_min - config.linear_slop) {
          error = distance - j->min_distance;
          constraint->min_lambda = 0.0f;
          constraint->max_lambda = INFINITY;
        } else if (distance_sqr > threshold_max + config.linear_slop) {
          error = j->max_distance - distance;
          constraint->min_lambda = -INFINITY;
          constraint->max_lambda = 0.0f;
        } else {
          error = constraint->max_lambda = constraint->min_lambda = 0.0f;
        }

        constraint->bias = config.baumgarde_coefficient * error;
        constraint->bias = error > 0.0f
         ? MIN(constraint->bias, config.max_baumgarde_velocity)
         : MAX(constraint->bias, -config.max_baumgarde_velocity);

      } break;

      default:
        continue;
    }
  }

  return BND_RESULT_OK(u32, count);
}

static void apply_impulse(bnd_v3 impulse, const bnd_v3 *relative_positions, constraint_state *state) {
  float sign = 1.0;
  for (count_t k = 0; k < state->body_count; ++k) {
    state->velocities[k] = bnd_v3_add(state->velocities[k], bnd_v3_scale(impulse, state->inv_masses[k] * sign));
    state->momenta[k] = bnd_v3_add(state->momenta[k], bnd_v3_scale(bnd_v3_cross(relative_positions[k], impulse), sign));

    sign = -1;
  }
}

static constraint_state joint_constraint_state(const dynamic_bodies *dynamics, const joint_constraint *constraint) {
  constraint_state s = {
    .body_count = 1 + (constraint->type == BND_BODY_DYNAMIC),
    .body_ids = { constraint->body_a, constraint->body_b },

    .normal = constraint->axis,
  };
  
  for (count_t k = 0; k < 2; ++k) {
    count_t id = s.body_ids[k];
    s.velocities[k] = dynamics->velocities[id];
    s.momenta[k] = dynamics->angular_momenta[id];
    s.inv_masses[k] = dynamics->inv_masses[id];

    bnd_m3 inertia = bnd_m3_inertia(dynamics->inv_inertia_tensors[id], dynamics->rotations[id]);
    s.angular_velocities[k] = bnd_m3_rotate(s.momenta[k], inertia);
  }

  return s;
}

static constraint_state contact_constraint_state(const dynamic_bodies *dynamics, const contact_constraint *constraint) {
  constraint_state s = {
    .body_count = 1 + (constraint->type == BND_BODY_DYNAMIC),
    .body_ids = { constraint->body_a, constraint->body_b },

    .normal    = { constraint->basis.m0[1], constraint->basis.m1[1], constraint->basis.m2[1] },
    .tangent_a = { constraint->basis.m0[0], constraint->basis.m1[0], constraint->basis.m2[0] },
    .tangent_b = { constraint->basis.m0[2], constraint->basis.m1[2], constraint->basis.m2[2] },
  };

  for (count_t k = 0; k < 2; ++k) {
    s.velocities[k] = dynamics->velocities[s.body_ids[k]];
    s.momenta[k] = dynamics->angular_momenta[s.body_ids[k]];
    s.inv_masses[k] = dynamics->inv_masses[s.body_ids[k]];
  }

  return s;
}

static void write_back_constraint_state(dynamic_bodies *dynamics, const constraint_state *state) {
  for (count_t k = 0; k < state->body_count; ++k) {
    dynamics->velocities[state->body_ids[k]] = state->velocities[k];
    dynamics->angular_momenta[state->body_ids[k]] = state->momenta[k];
  }
}

static void write_back_impulses(bnd_world *world, solver_context *cx) {
  for (count_t i = 0; i < cx->contacts_count; ++i) {
    const contact_constraint * constraint = &cx->contacts[i];
    broad_contacts_set *contacts = constraint->type == BND_BODY_DYNAMIC ? &world->contacts.dynamics : &world->contacts.statics;

    contact_manifold *manifold = &contacts->contacts[constraint->contact_index].manifold;
    for (count_t j = 0; j < manifold->count; ++j) {
      manifold->points[j].normal_impulse = constraint->points[j].normal_impulse;
      manifold->points[j].tangential_impulse[0] = constraint->points[j].tangent_impulse[0];
      manifold->points[j].tangential_impulse[1] = constraint->points[j].tangent_impulse[1];
    }
  }

  for (count_t i = 0; i < cx->joints_count; ++i) {
    const joint_constraint *constraint = &cx->joints[i];
    bnd_joint *j = &world->joints.values[constraint->joint_index];

    j->impulse = constraint->lambda;
  }

}

static void warm_start_solver(solver_context *cx) {
  for (count_t i = 0; i < cx->contacts_count; ++i) {
    contact_constraint *constraint = &cx->contacts[i];
    constraint_state state = contact_constraint_state(cx->dynamics, constraint);

    for (count_t p = 0; p < constraint->points_count; ++p) {
      constraint_point *point = &constraint->points[p];
      bnd_v3 tangent_impulse = bnd_v3_add(bnd_v3_scale(state.tangent_a, point->tangent_impulse[0]), bnd_v3_scale(state.tangent_b, point->tangent_impulse[1]));
      bnd_v3 impulse = bnd_v3_add(bnd_v3_scale(state.normal, point->normal_impulse), tangent_impulse);

      apply_impulse(impulse, point->relative_position, &state);
    }

    write_back_constraint_state(cx->dynamics, &state);
  }

  for (count_t i = 0; i < cx->joints_count; ++i) {
    joint_constraint *constraint = &cx->joints[i];
    constraint_state state = joint_constraint_state(cx->dynamics, constraint);

    bnd_v3 impulse = bnd_v3_scale(state.normal, constraint->lambda);
    apply_impulse(impulse, constraint->relative_positions, &state);
    write_back_constraint_state(cx->dynamics, &state);
  }
}

static bnd_v3 contact_point_local_velocity(
  const dynamic_bodies *dynamics,
  const contact_constraint *constraint,
  const constraint_point *point,
  const constraint_state *state
) {
  bnd_v3 local_velocity[2] = {0};
  for (count_t k = 0; k < state->body_count; ++k) {
    bnd_v3 angular_velocity = bnd_m3_rotate(state->momenta[k], dynamics->inv_intertias[state->body_ids[k]]);
    bnd_v3 vel = bnd_v3_add(state->velocities[k], bnd_v3_cross(angular_velocity, point->relative_position[k]));
    local_velocity[k] = bnd_m3_rotate(vel, bnd_m3_transpose(constraint->basis));
  }

  return bnd_v3_sub(local_velocity[0], local_velocity[1]);
}

static void solve_contact(solver_context *cx, count_t index) {
  contact_constraint *constraint = &cx->contacts[index];
  constraint_state interim_state = contact_constraint_state(cx->dynamics, constraint);

  for (count_t p = 0; p < constraint->points_count; ++p) {
    constraint_point *point = &constraint->points[p];
    bnd_v3 local_velocity = contact_point_local_velocity(cx->dynamics, constraint, point, &interim_state);

    float bias = MAX(cx->config.baumgarde_coefficient * cx->inv_dt * MIN(0.0f, point->separation + cx->config.linear_slop), -cx->config.max_baumgarde_velocity);
    float vn = local_velocity.y;
    float normal_impulse = -point->normal_mass * (vn + bias) * (1 + constraint->restitution);
    float new_impulse = MAX(point->normal_impulse + normal_impulse, 0.0f);
    normal_impulse = new_impulse - point->normal_impulse;
    point->normal_impulse = new_impulse;

    bnd_v3 impulse = bnd_v3_scale(interim_state.normal, normal_impulse);
    apply_impulse(impulse, point->relative_position, &interim_state);
  }

  for (count_t p = 0; p < constraint->points_count; ++p) {
    constraint_point *point = &constraint->points[p];
    bnd_v3 local_velocity = contact_point_local_velocity(cx->dynamics, constraint, point, &interim_state);

    float delta_lambda[] = {
      -(local_velocity.x * point->tangent_mass[0] + local_velocity.z * point->tangent_mass[1]),
      -(local_velocity.x * point->tangent_mass[2] + local_velocity.z * point->tangent_mass[3]),
    };

    float candidate_lambda[] = {
      point->tangent_impulse[0] + delta_lambda[0],
      point->tangent_impulse[1] + delta_lambda[1]
    };

    float max_friction = constraint->friction * point->normal_impulse;
    float friction_impulse = sqrtf(candidate_lambda[0] * candidate_lambda[0] + candidate_lambda[1] * candidate_lambda[1]);
    if (friction_impulse > max_friction) {
      candidate_lambda[0] = candidate_lambda[0] / friction_impulse * max_friction;
      candidate_lambda[1] = candidate_lambda[1] / friction_impulse * max_friction;
    }

    float lambda[] = {
      candidate_lambda[0] - point->tangent_impulse[0],
      candidate_lambda[1] - point->tangent_impulse[1],
    };

    memcpy(point->tangent_impulse, candidate_lambda, sizeof(candidate_lambda));

    bnd_v3 impulse = bnd_v3_add(bnd_v3_scale(interim_state.tangent_a, lambda[0]), bnd_v3_scale(interim_state.tangent_b, lambda[1]));

    apply_impulse(impulse, point->relative_position, &interim_state);
  }

  write_back_constraint_state(cx->dynamics, &interim_state);
}

static void solve_joint(solver_context *cx, count_t index) {
  joint_constraint *constraint = &cx->joints[index];
  constraint_state state = joint_constraint_state(cx->dynamics, constraint);

  float bias = constraint->bias * cx->inv_dt;
  float JV = bnd_v3_dot(constraint->Jav, state.velocities[0])
    + bnd_v3_dot(constraint->Jbv, state.velocities[1])
    + bnd_v3_dot(constraint->Jaw, state.angular_velocities[0])
    + bnd_v3_dot(constraint->Jbw, state.angular_velocities[1]);

  float lambda_delta = -constraint->effective_mass * (JV + bias);
  float lambda_old = constraint->lambda;

  constraint->lambda = MAX(constraint->min_lambda, MIN(lambda_old + lambda_delta, constraint->max_lambda));

  float impulse_lambda = constraint->lambda - lambda_old;
  bnd_v3 impulse = bnd_v3_scale(constraint->axis, impulse_lambda);

  apply_impulse(impulse, constraint->relative_positions, &state);
  write_back_constraint_state(cx->dynamics, &state);
}

bnd_error resolve_constraints(bnd_world *world, float dt) {
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame(&world->arena);

  count_t contacts_count = 0;
  count_t joints_count = 0;

  uint64_t contacts_arena_offset = AlignTo(stack_frame.arena->offset, ALIGNMENT_CONTACT_CONSTRAINT);
  uint64_t joints_arena_offset = 0;

  {
    PROFILER_BLOCK_START("prepare_constraints");

    bnd_error e = for_each_broad_contact(world, constraints_from_contacts, &contacts_count);
    if (IS_ERROR(e)) {
      arena_release_stack_frame(stack_frame);
      PROFILER_BLOCK_END;
      return e;
    }

    joints_arena_offset = AlignTo(stack_frame.arena->offset, ALIGNMENT_JOINT_CONSTRAINT);

    bnd_result_u32 result_joints_count = constraints_from_joints(world);
    if (IS_ERROR(result_joints_count.error)) {
      arena_release_stack_frame(stack_frame);
      PROFILER_BLOCK_END;
      return result_joints_count.error;
    }
    joints_count = result_joints_count.value;

    PROFILER_BLOCK_END;
  }

  world->stats.contacts_count = contacts_count;

  if (dt <= 0.0f) {
    arena_release_stack_frame(stack_frame);
    return OK;
  }
  
  PROFILER_BLOCK_START("resolve_constraints");

  contact_constraint *contact_constraints = (contact_constraint *)(stack_frame.arena->buffer + contacts_arena_offset);
  joint_constraint *joint_constraints     = (joint_constraint *)(stack_frame.arena->buffer + joints_arena_offset);

  solver_context scx = {
    .inv_dt            = 1.0f / dt,
    .current_iteration = 0,
    .dynamics          = &world->dynamics,
    .config            = world->config.solver,
    .contacts          = contact_constraints,
    .joints            = joint_constraints,
    .contacts_count    = contacts_count,
    .joints_count      = joints_count
  };

  if (scx.config.warm_start) {
    warm_start_solver(&scx);
  }

  for (count_t i = 0; i < scx.config.iterations_count; ++i) {
    scx.current_iteration = i;

    for (count_t j = 0; j < scx.contacts_count; ++j) {
      solve_contact(&scx, j);
    }

    for (count_t j = 0; j < scx.joints_count; ++j) {
      solve_joint(&scx, j);
    }
  }

  if (scx.config.warm_start) {
    write_back_impulses(world, &scx);
  }

  PROFILER_BLOCK_END;

  arena_release_stack_frame(stack_frame);

  return OK;
}


