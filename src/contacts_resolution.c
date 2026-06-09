#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

static void update_desired_velocity_delta(bnd_world *world, count_t contact_index, float dt) {
  count_t awake_count = world->dynamics.awake_count;
  contact *contact = &world->contacts.values[contact_index];
  count_t body_count = contact_index < world->contacts.dynamic_count ? 2 : 1;
  count_t body_ids[2] = {contact->index_a, contact->index_b};

  bnd_v3 accelerations[2] = {0};
  for (count_t k = 0; k < body_count; k++) {
    if (body_ids[k] < awake_count) {
      accelerations[k] = world->dynamics.accelerations[body_ids[k]];
    }
  }

  float acceleration_velocity = bnd_v3_dot(bnd_v3_sub(accelerations[0], accelerations[1]), contact->normal) * dt;
  float restitution = fabsf(contact->local_velocity.y) >= world->config.simulation.min_bounce_velocity ? contact->restitution : 0.0f;
  float desired_delta = -contact->local_velocity.y - restitution * (contact->local_velocity.y - acceleration_velocity);

  contact->desired_delta_velocity = desired_delta;
}

static bnd_m3 contact_space_transform(const contact *contact) {
  bnd_v3 y_axis = contact->normal;
  bnd_v3 x_axis, z_axis;

  if (fabsf(y_axis.z) > fabsf(y_axis.x)) {
    // Take (1, 0, 0) as initial guess
    const float s = 1.0 / sqrtf(y_axis.y * y_axis.y + y_axis.z * y_axis.z);

    z_axis.x = 0;
    z_axis.y = s * y_axis.z;
    z_axis.z = -s * y_axis.y;

    x_axis.x = z_axis.y * y_axis.z - y_axis.y * z_axis.z;
    x_axis.y = y_axis.x * z_axis.z;
    x_axis.z = y_axis.x * z_axis.y;
  } else {
    // Take (0, 0, 1) as initial guess
    const float s = 1.0 / sqrtf(y_axis.x * y_axis.x + y_axis.y * y_axis.y);

    x_axis.x = -s * y_axis.y;
    x_axis.y = s * y_axis.x;
    x_axis.z = 0;

    z_axis.x = -y_axis.z * x_axis.y;
    z_axis.y = x_axis.x * y_axis.z;
    z_axis.z = y_axis.x * x_axis.y - x_axis.x * y_axis.y;
  }

  return bnd_m3_from_basis(x_axis, y_axis, z_axis);
}

static void prepare_contacts(bnd_world *world, float dt) {
  PROFILE_FUNCTION

  dynamic_bodies *dynamics = &world->dynamics;

  for (count_t i = 0; i < world->contacts.count; ++i) {
    contact *contact = &world->contacts.values[i];
    count_t body_ids[] = {contact->index_a, contact->index_b};
    count_t body_count = i < world->contacts.dynamic_count ? 2 : 1;
    bnd_v3 angular_velocity[2];

    for (count_t k = 0; k < body_count; ++k) {
      bnd_m3 inv_inertia = bnd_m3_inertia(dynamics->inv_inertia_tensors[body_ids[k]], dynamics->rotations[body_ids[k]]);
      angular_velocity[k] = bnd_m3_rotate(dynamics->angular_momenta[body_ids[k]], inv_inertia);

      dynamics->inv_intertias[body_ids[k]] = inv_inertia;
    }

    contact->basis = contact_space_transform(contact);
    bnd_m3 world_to_contact = bnd_m3_transpose(contact->basis);

    for (count_t k = 0; k < body_count; ++k) {
      contact->relative_position[k] = bnd_v3_sub(contact->point, dynamics->positions[body_ids[k]]);
    }

    bnd_v3 local_velocity[2] = {0};
    for (count_t k = 0; k < body_count; ++k) {
      bnd_v3 acceleration_velocity = bnd_v3_scale(dynamics->accelerations[body_ids[k]], dt);
      acceleration_velocity = bnd_m3_rotate(acceleration_velocity, world_to_contact);
      acceleration_velocity.y = 0;

      bnd_v3 vel = bnd_v3_add(dynamics->velocities[body_ids[k]], bnd_v3_cross(angular_velocity[k], contact->relative_position[k]));
      vel = bnd_m3_rotate(vel, world_to_contact);
      local_velocity[k] = bnd_v3_add(vel, acceleration_velocity);
    }

    contact->local_velocity = bnd_v3_sub(local_velocity[0], local_velocity[1]);

    update_desired_velocity_delta(world, i, dt);
  }
}

static void resolve_interpenetration_contact(bnd_world *world, count_t contact_index, bnd_v3 *deltas) {
  PROFILE_FUNCTION

  contact *contact = &world->contacts.values[contact_index];
  count_t body_count = contact_index < world->contacts.dynamic_count ? 2 : 1;
  count_t body_ids[] = {contact->index_a, contact->index_b};

  float total_inertia = 0;
  float linear_inertia[2];
  float angular_inertia_contact[2];
  bnd_v3 torque_per_impulse[2];
  bnd_v3 position[2];
  bnd_m3 inv_inertia_tensor[2];
  bnd_quat rotation[2];
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];

    position[k] = world->dynamics.positions[body_index];
    inv_inertia_tensor[k] = world->dynamics.inv_intertias[body_index];
    rotation[k] = world->dynamics.rotations[body_index];
    float inv_mass = world->dynamics.inv_masses[body_index];

    torque_per_impulse[k] = bnd_v3_cross(contact->relative_position[k], contact->normal);

    bnd_v3 angular_inertia_world = torque_per_impulse[k];
    angular_inertia_world = bnd_m3_rotate(angular_inertia_world, inv_inertia_tensor[k]);
    angular_inertia_world = bnd_v3_cross(angular_inertia_world, contact->relative_position[k]);

    angular_inertia_contact[k] = bnd_v3_dot(angular_inertia_world, contact->normal);
    linear_inertia[k] = inv_mass;
    total_inertia += linear_inertia[k] + angular_inertia_contact[k];
  }

  const float angular_limit = 0.2f;
  float inv_inertia = 1 / total_inertia;
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    float sign = k ? -1 : 1;
    float linear_move = sign * contact->depth * linear_inertia[k] * inv_inertia;
    float angular_move = sign * contact->depth * angular_inertia_contact[k] * inv_inertia;

    float projection_len = -bnd_v3_dot(contact->normal, contact->relative_position[k]);
    bnd_v3 projection = contact->relative_position[k];
    projection = bnd_v3_add(projection, bnd_v3_scale(contact->normal, projection_len));

    float max_magnitude = angular_limit * bnd_v3_len(projection);
    if (angular_move < -max_magnitude) {
      float total_move = angular_move + linear_move;
      angular_move = -max_magnitude;
      linear_move = total_move - angular_move;
    } else if (angular_move > max_magnitude) {
      float total_move = angular_move + linear_move;
      angular_move = max_magnitude;
      linear_move = total_move - angular_move;
    }

    if (fabsf(angular_move) < 0.001) {
      deltas[2 * k + 1] = bnd_v3_zero();
    } else {
      bnd_v3 target_angular_direction = bnd_m3_rotate(torque_per_impulse[k], inv_inertia_tensor[k]);
      deltas[2 * k + 1] = bnd_v3_scale(target_angular_direction, angular_move / angular_inertia_contact[k]);
    }

    bnd_v3 linear_delta = bnd_v3_scale(contact->normal, linear_move);
    deltas[2 * k] = linear_delta;
    world->dynamics.positions[body_index] = bnd_v3_add(position[k], linear_delta);

    bnd_v3 rotation_delta = deltas[2 * k + 1];
    bnd_quat q_omega = {rotation_delta.x, rotation_delta.y, rotation_delta.z, 0};
    bnd_quat dq = bnd_quat_scale(bnd_quat_mul(q_omega, rotation[k]), 0.5);
    world->dynamics.rotations[body_index] = bnd_quat_normalize(bnd_quat_add(rotation[k], dq));

    world->dynamics.inv_intertias[body_index] = bnd_m3_inertia(world->dynamics.inv_inertia_tensors[body_index], world->dynamics.rotations[body_index]);
  }

  for (count_t k = 0; k < body_count; ++k) {
    contact->relative_position[k] = bnd_v3_sub(contact->point, world->dynamics.positions[body_ids[k]]);
  }
}

static void update_penetration_depths(bnd_world *world, count_t contact_index, const bnd_v3 *deltas) {
  contact *worst_contact = &world->contacts.values[contact_index];

  count_t worst_body_ids[] = {worst_contact->index_a, worst_contact->index_b};
  count_t worst_body_count = contact_index < world->contacts.dynamic_count ? 2 : 1;

  count_t count = world->contacts.count;
  for (count_t i = 0; i < count; ++i) {
    contact *contact = &world->contacts.values[i];
    count_t body_count = i < world->contacts.dynamic_count ? 2 : 1;
    count_t body_ids[] = {contact->index_a, contact->index_b};

    for (count_t k = 0; k < body_count; ++k) {
      count_t body_index = body_ids[k];

      for (count_t m = 0; m < worst_body_count; ++m) {
        count_t worst_body_index = worst_body_ids[m];

        if (body_index == worst_body_index) {
          bnd_v3 delta_position = bnd_v3_add(deltas[2 * m], bnd_v3_cross(deltas[2 * m + 1], contact->relative_position[k]));
          contact->depth += (k ? 1 : -1) * bnd_v3_dot(delta_position, contact->normal);
        }
      }
    }
  }
}

static void resolve_velocity_contact(bnd_world *world, count_t contact_index, bnd_v3 *deltas) {
  PROFILE_FUNCTION

  contact *contact = &world->contacts.values[contact_index];
  count_t body_count = contact_index < world->contacts.dynamic_count ? 2 : 1;
  count_t body_ids[] = {contact->index_a, contact->index_b};

  bnd_m3 contact_to_world = contact->basis;
  bnd_m3 world_to_contact = bnd_m3_transpose(contact_to_world);

  bnd_m3 delta_velocity = {0};
  float inv_mass = 0;
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    bnd_m3 r_cross = bnd_m3_skew_symmetric(contact->relative_position[k]);

    bnd_m3 delta_velocity_world = bnd_m3_multiply(r_cross, world->dynamics.inv_intertias[body_index]);
    delta_velocity_world = bnd_m3_multiply(delta_velocity_world, r_cross);
    delta_velocity_world = bnd_m3_negate(delta_velocity_world);

    inv_mass += world->dynamics.inv_masses[body_index];
    delta_velocity = bnd_m3_add(delta_velocity, delta_velocity_world);
  }

  delta_velocity = bnd_m3_multiply(world_to_contact, delta_velocity);
  delta_velocity = bnd_m3_multiply(delta_velocity, contact_to_world);
  delta_velocity.m0[0] += inv_mass;
  delta_velocity.m1[1] += inv_mass;
  delta_velocity.m2[2] += inv_mass;

  bnd_m3 impulse_matrix = bnd_m3_inverse(delta_velocity);
  bnd_v3 velocity_to_kill = {-contact->local_velocity.x, contact->desired_delta_velocity, -contact->local_velocity.z};
  bnd_v3 contact_space_impulse = bnd_m3_rotate(velocity_to_kill, impulse_matrix);
  float planar_impulse = sqrtf(contact_space_impulse.x * contact_space_impulse.x + contact_space_impulse.z * contact_space_impulse.z);

  if (planar_impulse > contact_space_impulse.y * contact->friction) {
    contact_space_impulse.x /= planar_impulse;
    contact_space_impulse.z /= planar_impulse;

    float desired_delta_velocity = contact->desired_delta_velocity;
    contact_space_impulse.y = delta_velocity.m1[0] * contact->friction * contact_space_impulse.x +
                              delta_velocity.m1[1] + delta_velocity.m1[2] * contact->friction * contact_space_impulse.z;
    contact_space_impulse.y = desired_delta_velocity / contact_space_impulse.y;
    contact_space_impulse.x *= contact->friction * contact_space_impulse.y;
    contact_space_impulse.z *= contact->friction * contact_space_impulse.y;
  }

  bnd_v3 world_space_impulse = bnd_m3_rotate(contact_space_impulse, contact->basis);

  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    inv_mass = world->dynamics.inv_masses[body_index];

    bnd_v3 linear_impulse_delta = bnd_v3_scale(world_space_impulse, inv_mass);
    bnd_v3 angular_impulse_delta = bnd_v3_cross(contact->relative_position[k], world_space_impulse);

    bnd_v3 *velocity = &world->dynamics.velocities[body_index];
    bnd_v3 *angular_momentum = &world->dynamics.angular_momenta[body_index];

    *velocity = bnd_v3_add(*velocity, linear_impulse_delta);
    *angular_momentum = bnd_v3_add(*angular_momentum, angular_impulse_delta);

    deltas[2 * k] = linear_impulse_delta;
    deltas[2 * k + 1] = angular_impulse_delta;

    world_space_impulse = bnd_v3_scale(world_space_impulse, -1);
  }
}

// Find the worst penetration contact. Returns false if none above threshold.
static bool find_worst_penetration(bnd_world *world, count_t *out_contact_index) {
  float max_penetration = world->config.advanced.penetration_epsilon;
  count_t best_contact = (count_t)-1;

  for (count_t i = 0; i < world->contacts.count; ++i) {
    contact *contact = &world->contacts.values[i];

    if (contact->depth > max_penetration) {
      max_penetration = contact->depth;
      best_contact = i;
    }
  }

  if (best_contact == (count_t)-1)
    return false;

  *out_contact_index = best_contact;
  return true;
}

// Find the worst velocity contact. Returns false if none above threshold.
static bool find_worst_velocity(bnd_world *world, count_t *out_contact_index) {
  float max_velocity = world->config.advanced.velocity_epsilon;
  count_t best_contact = (count_t)-1;

  for (count_t i = 0; i < world->contacts.count; ++i) {
    contact *contact = &world->contacts.values[i];

    if (contact->desired_delta_velocity > max_velocity) {
      max_velocity = contact->desired_delta_velocity;
      best_contact = i;
    }
  }

  if (best_contact == (count_t)-1) {
    return false;
  }

  *out_contact_index = best_contact;
  return true;
}

static void update_awake_status_for_collision(bnd_world *world, count_t contact_index) {
  if (contact_index >= world->contacts.dynamic_count) {
    return;
  }

  contact *contact = &world->contacts.values[contact_index];

  bool body_a_awake = contact->index_a < world->dynamics.awake_count;
  bool body_b_awake = contact->index_b < world->dynamics.awake_count;
  if (body_a_awake == body_b_awake) {
    return;
  }

  const float sleep_threshold = world->config.simulation.sleep_threshold;
  if (!body_a_awake) {
    world->dynamics.motion_avgs[contact->index_a] = 2.0 * sleep_threshold;
  }

  if (!body_b_awake) {
    world->dynamics.motion_avgs[contact->index_b] = 2.0 * sleep_threshold;
  }
}

static void resolve_interpenetrations(bnd_world *world) {
  PROFILE_FUNCTION

  const count_t count = world->contacts.count;
  const count_t max_iterations = count * world->config.advanced.resolution_attempts_factor;

  if (count == 0) {
    return;
  }

  count_t iterations = 0;
  count_t max_penetration_index = -1;
  while (iterations < max_iterations) {
    if (!find_worst_penetration(world, &max_penetration_index)) {
      break;
    }

    update_awake_status_for_collision(world, max_penetration_index);

    bnd_v3 deltas[4];
    resolve_interpenetration_contact(world, max_penetration_index, deltas);
    update_penetration_depths(world, max_penetration_index, deltas);

    iterations += 1;
  }

  world->stats.incomplete_resolutions += iterations >= max_iterations;
}

static void update_velocity_deltas(bnd_world *world, count_t contact_index, const bnd_v3 *deltas, float dt) {
  contact *worst_contact = &world->contacts.values[contact_index];
  count_t worst_body_ids[] = {worst_contact->index_a, worst_contact->index_b};
  count_t worst_body_count = contact_index < world->contacts.dynamic_count ? 2 : 1;

  count_t count = world->contacts.count;
  for (count_t i = 0; i < count; ++i) {
    contact *contact = &world->contacts.values[i];
    count_t body_ids[] = {contact->index_a, contact->index_b};
    count_t body_count = i < world->contacts.dynamic_count ? 2 : 1;

    for (count_t k = 0; k < body_count; ++k) {
      count_t body_index = body_ids[k];

      for (count_t m = 0; m < worst_body_count; ++m) {
        count_t worst_body_index = worst_body_ids[m];

        if (body_index == worst_body_index) {
          bnd_v3 angular_velocity_delta = bnd_m3_rotate(deltas[2 * m + 1], world->dynamics.inv_intertias[worst_body_index]);
          bnd_v3 delta_velocity = bnd_v3_add(deltas[2 * m], bnd_v3_cross(angular_velocity_delta, contact->relative_position[k]));
          delta_velocity = bnd_m3_rotate_inverse(delta_velocity, contact->basis);

          contact->local_velocity = bnd_v3_add(contact->local_velocity, bnd_v3_scale(delta_velocity, (k ? -1 : 1)));

          update_desired_velocity_delta(world, i, dt);
        }
      }
    }
  }
}

static void resolve_velocities(bnd_world *world, float dt) {
  PROFILE_FUNCTION

  const count_t count = world->contacts.count;
  const count_t max_iterations = count * world->config.advanced.resolution_attempts_factor;
  if (count == 0) {
    return;
  }

  count_t iterations = 0;
  count_t worst_contact_index = -1;
  while (iterations < max_iterations) {
    if (!find_worst_velocity(world, &worst_contact_index)) {
      break;
    }

    update_awake_status_for_collision(world, worst_contact_index);

    bnd_v3 deltas[4];
    resolve_velocity_contact(world, worst_contact_index, deltas);
    update_velocity_deltas(world, worst_contact_index, deltas, dt);

    iterations += 1;
  }

  world->stats.incomplete_resolutions += iterations >= max_iterations;
}

void contacts_resolve(bnd_world *world, float dt) {
  PROFILE_FUNCTION

  prepare_contacts(world, dt);
  resolve_interpenetrations(world);
  resolve_velocities(world, dt);
}
