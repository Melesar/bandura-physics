#ifndef BND_CORE_H
#define BND_CORE_H

#include "bandura.h"

#define EPSILON 0.000001f
#define EPHEMERAL_BODIES_COUNT 4
#define DEFAULT_VERTEX_PER_MESH 512
#define DEFAULT_FACE_PER_MESH 256

#define OK (bnd_error){BND_OK, NULL}
#define OOM_ERROR (bnd_error){BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc failed to allocate memory"}

#define PROPAGATE_ERROR3(error, suffix) \
  bnd_error e_##suffix = error; \
  if (e_##suffix.type != BND_OK) { \
    return e_##suffix; \
  }
#define PROPAGATE_ERROR2(error, suffix) PROPAGATE_ERROR3(error, suffix)
#define PROPAGATE_ERROR(error) PROPAGATE_ERROR2(error, __LINE__)

#define PROPAGATE_RESULT3(suffix, error, error_suffix) \
  bnd_error e_##error_suffix = error; \
  if (e_##error_suffix.type != BND_OK) { \
    return BND_RESULT_ERR2(suffix, e_##error_suffix); \
  }
#define PROPAGATE_RESULT2(suffix, error, error_suffix) PROPAGATE_RESULT3(suffix, error, error_suffix)
#define PROPAGATE_RESULT(suffix, error) PROPAGATE_RESULT2(suffix, error, __LINE__)

#define BND_RESULT_OK(suffix, value) (bnd_result_##suffix) { OK, value }
#define BND_RESULT_ERR(suffix, error_type, message) (bnd_result_##suffix) { (bnd_error) { error_type, message }, { 0 } }
#define BND_RESULT_ERR2(suffix, error) (bnd_result_##suffix) { error, { 0 } }

#define ALLOC_BUFFER1(buffer, capacity) ALLOC_BUFFER(buffer, 1, capacity)
#define ALLOC_BUFFER2(buffer, capacity) ALLOC_BUFFER(buffer, 2, capacity)
#define ALLOC_BUFFER4(buffer, capacity) ALLOC_BUFFER(buffer, 4, capacity)
#define ALLOC_BUFFER8(buffer, capacity) ALLOC_BUFFER(buffer, 8, capacity)

#define REALLOC_BUFFER1(buffer, allocator, element_size, old_size, new_size) REALLOC_BUFFER(buffer, allocator, 1, element_size, old_size, new_size)
#define REALLOC_BUFFER2(buffer, allocator, element_size, old_size, new_size) REALLOC_BUFFER(buffer, allocator, 2, element_size, old_size, new_size)
#define REALLOC_BUFFER4(buffer, allocator, element_size, old_size, new_size) REALLOC_BUFFER(buffer, allocator, 4, element_size, old_size, new_size)
#define REALLOC_BUFFER8(buffer, allocator, element_size, old_size, new_size) REALLOC_BUFFER(buffer, allocator, 8, element_size, old_size, new_size)

#define ALLOC_BUFFER(buffer, alingment, capacity) \
  buffer = allocator.malloc(alingment, capacity); \
  if (buffer == NULL) { \
    return OOM_ERROR; \
  } \

#define REALLOC_BUFFER(buffer, allocator, alignment, element_size, old_size, new_size) \
  buffer = allocator.realloc(buffer, alignment, old_size * element_size, element_size * new_size); \
  if (buffer == NULL) { \
    return (bnd_error) { BND_ERROR_OUT_OF_MEMORY, "Allocator.realloc failed to re-allocate buffer" }; \
  }

typedef uint32_t count_t;

typedef struct {
  bnd_v3 point;
  bnd_v3 normal;
  float depth;
  count_t index_a, index_b;
  float friction, restitution;

  bnd_m3 basis;
  bnd_v3 relative_position[2];
  bnd_v3 local_velocity;
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
  bnd_v3 *verticies;
  uint32_t *indicies;

  submesh *submeshes;
  bnd_mesh *meshes;

  bnd_m3 *inertias;
  float *volumes;
  bnd_aabb *aabbs;

  count_t vertex_count;
  count_t vertex_capacity;
  count_t index_count;
  count_t index_capacity;
  count_t submesh_count;
  count_t submesh_capacity;
  count_t mesh_count;
  count_t mesh_capacity;
} mesh_storage;

typedef struct {
  bnd_event *events;
  count_t *links;
  count_t capacity;
  count_t count;
} events_storage;

typedef struct {
  count_t first;
  count_t last;
  uint8_t count;
} event_link;

#define COMMON_FIELDS                                                                                                  \
  count_t capacity;                                                                                                    \
  count_t count;                                                                                                       \
  count_t free_count;                                                                                                  \
  count_t first_outer_node;                                                                                            \
  bnd_v3 *positions;                                                                                                       \
  bnd_quat *rotations;                                                                                                     \
  body_shapes *shapes;                                                                                                 \
  bnd_aabb *aabbs;                                                                                                     \
  bnd_event_type *event_masks;                                                                                         \
  event_link *event_links;                                                                                             \
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
  bnd_v3 *forces;
  bnd_v3 *torques;
  bnd_v3 *impulses;
  bnd_v3 *angular_impulses;

  // Dynamics
  float *inv_masses;
  bnd_v3 *velocities;
  bnd_v3 *angular_momenta;
  bnd_m3 *inv_inertia_tensors;

  // Derived values.
  bnd_v3 *accelerations;
  bnd_m3 *inv_intertias;

  // Sleeping
  count_t awake_count;
  float *motion_avgs;
} dynamic_bodies;

typedef struct {
  COMMON_FIELDS

  bool dirty;
}static_bodies;

struct bnd_world_t {
  dynamic_bodies dynamics;
  static_bodies statics;

  contacts contacts;
  joints joints;
  mesh_storage meshes;
  events_storage events;

  shapes_bracket shape_brackets[BRACKET_COUNT];

  bnd_config config;
  bnd_world_stats stats;
  bnd_allocator allocator;

  count_t generation;
};

typedef struct {
  bnd_v3 point;
  uint16_t id;
} support_point;

typedef struct {
  bnd_v3 p;
  support_point p1;
  support_point p2;
} body_support;

typedef struct {
  body_support points[4];
  uint8_t size;
} simplex;

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_shape shape;
  count_t index;
} shape_context;

typedef support_point (*support_func)(const shape_context *, bnd_v3);

bnd_allocator bnd_default_allocator();

bnd_body_handle make_body_handle(const bnd_world *world, bnd_body_type type, count_t index);
count_t handle_to_inner_index(const bnd_world *world, bnd_body_handle handle);

common_data *as_common(bnd_world *world, bnd_body_type type);
const common_data *as_common_const(const bnd_world *world, bnd_body_type type);

bnd_error contacts_init(bnd_world *world);
void contacts_teardown(bnd_world *world);
void contacts_reset(bnd_world *world);
bnd_error contacts_ensure_capacity(bnd_world *world, count_t additional_count);
contact *contacts_new_default(bnd_world *world, count_t body_a, count_t body_b);
void contacts_generate(bnd_world *world);
void contacts_resolve(bnd_world *world, float dt);

void collision_detection_init(bnd_world *world);
count_t collisions_detect_dynamic(bnd_world *world);
void collisions_detect_static(bnd_world *world);

bnd_error joints_init(bnd_world *world);
void joints_teardown(bnd_world *world);
void joints_reset(bnd_world *world);
count_t joints_generate_dynamic(bnd_world *world);
void joints_generate_static(bnd_world *world);

bnd_error meshes_init(bnd_world *world);
void meshes_teardown(bnd_world *world);

bnd_error shapes_init(bnd_world *world);
void shapes_teardown(bnd_world *world);
void shapes_reset(bnd_world *world);
void shapes_get_bracket_properties(const bnd_config *config, count_t bracket_index, count_t *blocks, count_t *shapes, count_t *capacity);
bool shapes_any_slot_available(const bnd_world *world, shape_dimension_bracket bracket);
bnd_error shapes_expand_bracket(bnd_world *world, shape_dimension_bracket bracket);
bool shapes_put_into_empty_slot(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t *slot_number);
void shapes_clear_slot(bnd_world *world, shape_dimension_bracket bracket, count_t slot);
body_shapes shapes_write(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t count);
bnd_body_shape *shapes_get(const bnd_world *world, body_shapes shapes);

count_t ephemeral_body_index(const common_data *data);

bnd_error events_init(bnd_world *world);
void events_teardown(bnd_world *world);
void events_reset(bnd_world *world);
bool events_subscribed(const common_data *data, count_t index, bnd_event_type event_type);
bnd_error events_push(bnd_world *world, common_data *data, count_t index, bnd_event event);

bnd_quat integrate_rotation_midpoint(bnd_quat rotation, bnd_v3 angular_momentum, bnd_m3 base_inv_inertia, float dt);
bool gjk_check_intersection(const bnd_world *world, const collision_detection_context *ctx, simplex *simplex);
bnd_error epa_init(bnd_world *world);
void epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact);
void epa_get_final_points(bnd_v3 *points);
body_support support(const collision_detection_context *ctx, bnd_v3 direction);
uint32_t polytope_memory_size(uint16_t max_nodes);

float distance_to_triangle(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_v3 *closest);
float distance_to_line_segment(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 *closest);
bool aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b);

bnd_v3 body_center(const shape_context *ctx);
bnd_quat body_rotation(const shape_context *ctx);

bnd_m3 quat_as_matrix(bnd_quat q);

bnd_v3 bnd_m3_rotate_inverse(bnd_v3 v, bnd_m3 m);
bnd_m3 bnd_m3_from_basis(bnd_v3 x, bnd_v3 y, bnd_v3 z);
bnd_m3 bnd_m3_skew_symmetric(bnd_v3 v);
bnd_m3 bnd_m3_initial_inertia(bnd_v3 inertia);
bnd_m3 bnd_m3_inertia(bnd_m3 initial_inertia, bnd_quat rotation);
bnd_m3 bnd_m3_displacement_inertia(bnd_m3 i0, bnd_v3 offset, float mass);
#endif
