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
  count_t dynamic_count;
} joints;

typedef struct {
  contact *values;

  count_t capacity;
  count_t count;

  count_t dynamic_count;
} contacts;

typedef struct {
  count_t index;
  count_t prev;
  count_t next;
} outer_lookup_node;

#define COMMON_FIELDS                                                                                                  \
  count_t capacity;                                                                                                    \
  count_t count;                                                                                                       \
  count_t free_count;                                                                                                  \
  count_t first_outer_node;                                                                                            \
  v3 *positions;                                                                                                       \
  quat *rotations;                                                                                                     \
  body_shapes *shapes;                                                                                                 \
  uint8_t *generations;                                                                                                \
  count_t *free_list;                                                                                                  \
  outer_lookup_node *outer_lookup;                                                                                     \
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

  contacts contacts;
  joints joints;

  shapes_bracket shape_brackets[BRACKET_COUNT];

  physics_config config;
  physics_world_stats stats;

  count_t generation;
};

typedef struct {
  v3 v;
  v3 v1;
  v3 v2;
} support_point;

typedef struct {
  support_point points[4];
  uint8_t size;
} simplex;

body_handle make_body_handle(const physics_world *world, body_type type, count_t index);
count_t handle_to_inner_index(const physics_world *world, body_handle handle);

common_data *as_common(physics_world *world, body_type type);
const common_data *as_common_const(const physics_world *world, body_type type);

void contacts_init(physics_world *world);
void contacts_teardown(physics_world *world);
void contacts_reset(physics_world *world);
void contacts_generate(physics_world *world);
void contacts_resolve(physics_world *world, float dt);

count_t collisions_detect_dynamic(physics_world *world);
void collisions_detect_static(physics_world *world);

void joints_init(physics_world *world);
void joints_teardown(physics_world *world);
void joints_reset(physics_world *world);
count_t joints_generate_dynamic(physics_world *world);
void joints_generate_static(physics_world *world);

void shapes_init(physics_world *world);
void shapes_teardown(physics_world *world);
void shapes_reset(physics_world *world);
bool shapes_any_slot_available(const physics_world *world, shape_dimension_bracket bracket);
void shapes_expand_bracket(physics_world *world, shape_dimension_bracket bracket);
bool shapes_put_into_empty_slot(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes,
                                count_t shapes_count, count_t *slot_number);
void shapes_clear_slot(physics_world *world, shape_dimension_bracket bracket, count_t slot);
body_shapes shapes_write(physics_world *world, shape_dimension_bracket bracket, body_shape *shapes, count_t count);
body_shape *shapes_get(const physics_world *world, body_shapes shapes);

quat integrate_rotation_midpoint(quat rotation, v3 angular_momentum, m3 base_inv_inertia, float dt);
bool gjk_check_intersection(physics_world *world, const collision_detection_context *ctx, simplex *simplex);

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c, v3 *closest);
float distance_to_line_segment(v3 from, v3 a, v3 b, v3 *closest);


#endif
