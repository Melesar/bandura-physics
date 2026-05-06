#ifndef BND_CORE_H
#define BND_CORE_H

#include "bandura.h"

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
  bnd_joint *values;
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

typedef struct {
  count_t vertex_offset;
  count_t vertex_count;
  count_t index_offset;
  count_t index_count;
} submesh;

typedef struct {
  count_t submesh_offset;
  count_t submesh_count;
} bnd_mesh;

typedef struct {
  v3 *verticies;
  uint32_t *indicies;

  submesh *submeshes;
  bnd_mesh *meshes;

  m3 *inertias;
  float *volumes;

  count_t vertex_count;
  count_t vertex_capacity;
  count_t index_count;
  count_t index_capacity;
  count_t submesh_count;
  count_t submesh_capacity;
  count_t mesh_count;
  count_t mesh_capacity;
} mesh_storage;

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
  uint64_t *slots;
  bnd_body_shape *shapes;
  count_t capacity;
} shapes_bracket;

typedef struct {
  COMMON_FIELDS
} common_data;

typedef struct {
  const bnd_world *world;
  const common_data *data_a;
  const common_data *data_b;
  count_t body_a, body_b;
  bnd_body_shape shape_a, shape_b;
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

struct bnd_world_t {
  dynamic_bodies dynamics;
  static_bodies statics;

  contacts contacts;
  joints joints;
  mesh_storage meshes;

  shapes_bracket shape_brackets[BRACKET_COUNT];

  bnd_config config;
  bnd_world_stats stats;

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

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_shape shape;
  count_t index;
} shape_context;

typedef v3 (*support_func)(const shape_context *, v3);

void raise_error(bnd_error type, void *data, const char *template, ...);
void raise_error_debug(bnd_error type, void *data, const char *template, ...);

bnd_body_handle make_body_handle(const bnd_world *world, bnd_body_type type, count_t index);
count_t handle_to_inner_index(const bnd_world *world, bnd_body_handle handle);

common_data *as_common(bnd_world *world, bnd_body_type type);
const common_data *as_common_const(const bnd_world *world, bnd_body_type type);

void contacts_init(bnd_world *world);
void contacts_teardown(bnd_world *world);
void contacts_reset(bnd_world *world);
void contacts_ensure_capacity(bnd_world *world, count_t additional_count);
contact *contacts_new_default(bnd_world *world, count_t body_a, count_t body_b);
void contacts_generate(bnd_world *world);
void contacts_resolve(bnd_world *world, float dt);

count_t collisions_detect_dynamic(bnd_world *world);
void collisions_detect_static(bnd_world *world);

void joints_init(bnd_world *world);
void joints_teardown(bnd_world *world);
void joints_reset(bnd_world *world);
count_t joints_generate_dynamic(bnd_world *world);
void joints_generate_static(bnd_world *world);

void meshes_init(bnd_world *world);
void meshes_teardown(bnd_world *world);

void shapes_init(bnd_world *world);
void shapes_teardown(bnd_world *world);
void shapes_reset(bnd_world *world);
bool shapes_any_slot_available(const bnd_world *world, shape_dimension_bracket bracket);
void shapes_expand_bracket(bnd_world *world, shape_dimension_bracket bracket);
bool shapes_put_into_empty_slot(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t *slot_number);
void shapes_clear_slot(bnd_world *world, shape_dimension_bracket bracket, count_t slot);
body_shapes shapes_write(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t count);
bnd_body_shape *shapes_get(const bnd_world *world, body_shapes shapes);

quat integrate_rotation_midpoint(quat rotation, v3 angular_momentum, m3 base_inv_inertia, float dt);
bool gjk_check_intersection(bnd_world *world, const collision_detection_context *ctx, simplex *simplex);
void epa_init(const bnd_config *config);
void epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact);
support_point support(const collision_detection_context *ctx, v3 direction);

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c, v3 *closest);
float distance_to_line_segment(v3 from, v3 a, v3 b, v3 *closest);

v3 body_center(const shape_context *ctx);
quat body_rotation(const shape_context *ctx);

#endif
