#ifndef PHYSICS_H
#define PHYSICS_H

#include "bandura.h"
#include <stddef.h>

typedef struct {
  v3 point;
  v3 normal;
  float depth;
  count_t index_a, index_b;
  float friction, restitution;

  m3 basis;
  v3 relative_position[2];
  v3 local_velocity;
  float desired_delta_velocity;
} contact;

typedef struct {
  joint *values;
  count_t *ids;

  count_t capacity;
  count_t count;

  count_t next_id;
} joints;

typedef struct {
  count_t capacity;
  count_t count;
  contact *contacts;

  count_t dynamic_contacts_count;
} collisions;

#define COMMON_FIELDS \
  count_t capacity; \
  count_t count; \
  v3* positions; \
  quat* rotations; \
  body_shapes *shapes; \
  count_t *outer_lookup; \
  count_t *inner_lookup;

typedef enum {
  BRACKET_PRIMITIVE,
  BRACKET_TWO,
  BRACKET_FOUR,
  BRACKET_EIGHT,
  BRACKET_SIXTEEN,

  BRACKET_COUNT
} shape_dimension_bracket;

typedef struct {
  shape_dimension_bracket bracket : 3;
  count_t offset : 24;
  count_t count : 5;
} body_shapes;

typedef struct {
  body_shape *bracket;
  count_t capacity;
} shapes_bracket;

typedef struct {
  COMMON_FIELDS
} common_data;

typedef struct {
  const physics_world *world;
  const common_data *data_a;
  const common_data *data_b;
  count_t body_a, body_b;
  body_shape shape_a, shape_b;
} collision_detection_context;

typedef struct {
  COMMON_FIELDS

  // Forces
  v3 *forces;
  v3 *torques;
  v3 *impulses;
  v3 *angular_impulses;

  // Dynamics
  float *inv_masses;
  v3 *velocities;
  v3 *angular_momenta;
  m3 *inv_inertia_tensors;

  // Derived values.
  v3 *accelerations;
  m3 *inv_intertias;

  // Sleeping
  count_t awake_count;
  float *motion_avgs;
} dynamic_bodies;

typedef common_data static_bodies;

struct physics_world_t {
  dynamic_bodies dynamics;
  static_bodies statics;

  collisions collisions;
  joints joints;

  shapes_bracket shape_brackets[BRACKET_COUNT];

  physics_config config;

  count_t generation;
};

#define CDBG_MAX_CONTACTS 64

typedef enum {
  CDBG_IDLE,
  CDBG_PENETRATION_RESOLVE,
  CDBG_DEPTH_UPDATE,
  CDBG_VELOCITY_RESOLVE,
  CDBG_VELOCITY_UPDATE,
  CDBG_DONE,
} collision_debug_phase;

typedef struct {
  count_t index;
  float before;
  float after;
} depth_update_record;

typedef struct {
  count_t index;
  v3 local_vel_before;
  v3 local_vel_after;
  float ddv_before;
  float ddv_after;
} velocity_update_record;

struct collision_debug_state_t {
  bool active;
  collision_debug_phase prev_phase;
  collision_debug_phase phase;
  count_t iteration;
  bool is_dynamic;

  count_t current_collision_index;
  count_t current_contact_index;

  // Deltas from resolve steps (body1 linear, body1 angular, body2 linear, body2 angular)
  v3 deltas[4];

  // Depth update records
  count_t depth_update_count;
  depth_update_record depth_updates[CDBG_MAX_CONTACTS];

  // Velocity update records
  count_t velocity_update_count;
  velocity_update_record velocity_updates[CDBG_MAX_CONTACTS];

  // Internal state carried between steps
  float dt;
};

body_handle make_body_handle(const physics_world *world, body_type type, count_t index);
count_t handle_to_inner_index(const physics_world *world, body_handle handle);

common_data* as_common(physics_world *world, body_type type);
const common_data* as_common_const(const physics_world *world, body_type type);

collisions collisions_init(const physics_config *config);
void collisions_teardown(collisions collisions);

void clear_forces(physics_world *world);
void integrate_bodies(physics_world *world, float dt);
void prepare_contacts(physics_world *world, float dt);
void resolve_interpenetration_contact(physics_world *world, count_t contact_index, v3 *deltas);
void update_penetration_depths_ex(physics_world *world, count_t contact_index, const v3 *deltas, depth_update_record *records, count_t *record_count);
void resolve_velocity_contact(physics_world *world, count_t contact_index, v3 *deltas);
bool find_worst_penetration(physics_world *world, count_t *out_contact_index);
bool find_worst_velocity(physics_world *world, count_t *out_contact_index);
void update_awake_status_for_collision(physics_world *world, count_t collision_index);
void update_velocity_deltas_ex(physics_world *world, count_t worst_collision_index, const v3 *deltas, float dt, velocity_update_record *records, count_t *record_count);
void resolve_collisions(physics_world *world, float dt);
void collisions_detect(physics_world *world);
void update_awake_statuses(physics_world *world, float dt);

contact *new_contact(physics_world *world, const collision_detection_context *ctx);

void joints_init(physics_world *world);
void joints_teardown(physics_world *world);
void joints_produce_contacts(physics_world *world);

void shapes_init(physics_world *world);
void shapes_teardown(physics_world *world);
void shapes_reset(physics_world *world);
bool shapes_any_slot_available(const physics_world *world, shape_dimension_bracket bracket);
void shapes_expand_bracket(physics_world *world, shape_dimension_bracket bracket);
bool shapes_put_into_empty_slot(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes, count_t shapes_count, count_t *slot_number);
body_shapes shapes_write(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes, count_t count);
body_shape* shapes_get(const physics_world *world, body_shapes shapes);

#endif
