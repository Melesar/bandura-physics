#ifndef BND_CORE_H
#define BND_CORE_H

#include "bandura.h"
#include <stdbool.h>

#define EPSILON 0.000001f
#define EPHEMERAL_BODIES_COUNT 4
#define DEFAULT_VERTEX_PER_MESH 512
#define DEFAULT_FACE_PER_MESH 256
#define MAX_CONTACTS_PER_PAIR 4
#define MAX_COLLISION_LAYERS 64

#define HASH_TABLE_TOMBSTONE UINT64_MAX
#define HASH_TABLE_EMPTY 0

#define OK (bnd_error){BND_OK, NULL}
#define OOM_ERROR (bnd_error){BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc failed to allocate memory"}

#define IS_ERROR(e) ((e).type != BND_OK)
#define IS_OK(e) ((e).type == BND_OK)

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
#define BND_RESULT_ERR(suffix, error_type, message) bnd_result_##suffix##_error((bnd_error) { error_type, message })
#define BND_RESULT_ERR2(suffix, error) bnd_result_##suffix##_error(error)

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

#define BND_RESULT_FUNC_DECL(suffix, type) \
  bnd_result_##suffix bnd_result_##suffix##_error(bnd_error e) { \
    type dummy_value = {0}; \
    return (bnd_result_##suffix) { e, dummy_value }; \
  }\

typedef uint32_t count_t;

typedef enum {
  CONTACT_NONE,
  CONTACT_BEGAN_TOUCHING,
  CONTACT_TOUCHING,
  CONTACT_FINISHED_TOUCHING,
} broad_contact_status;

typedef struct {
  bnd_v3 witness_a, witness_b;
  bnd_v3 normal;
} contact_features;

typedef struct {
  bnd_v3 point;
  float depth;
} contact_point;

typedef struct {
  count_t count;
  bnd_v3 normal;
  contact_point points[MAX_CONTACTS_PER_PAIR];
} contact_manifold;

typedef struct {
  uint64_t key;
  count_t outer_index_a, outer_index_b;
  float friction, restitution;
  broad_contact_status status;
  contact_manifold manifold;
} broad_phase_contact;

typedef struct {
  bnd_v3 point;
  bnd_v3 normal;
  float depth;
  count_t index_a, index_b;
  float friction, restitution;
  contact_features features;

  bnd_m3 basis;
  bnd_v3 relative_position[2];
  bnd_v3 local_velocity;
  float desired_delta_velocity;

  bool from_cache;
} contact;

typedef struct {
  bnd_joint *values;
  count_t *ids;

  count_t capacity;
  count_t count;

  count_t next_id;
  count_t dynamic_count;
} joints;

typedef struct cache_entry cache_entry;

struct cache_entry {
  uint64_t key;
  count_t feature_count;
  count_t access_time;
  contact_features features[MAX_CONTACTS_PER_PAIR];
};

typedef struct {
  float restitution;
  float friction;
} body_material;

typedef struct {
  body_material *values;

  count_t count;
  count_t capacity;
} body_materials;

typedef struct {
  count_t *hash_table;
  count_t hash_table_capacity;

  cache_entry *entries;

  count_t entry_count;
  count_t buffer_capacity;
} contacts_cache;

typedef struct {
  uint64_t *keys;
  count_t  *values;

  count_t capacity;
  count_t entry_count;
} hash_table;

typedef struct {
  hash_table table;
  broad_phase_contact *dynamic_broad_contacts;
  broad_phase_contact *static_broad_contacts;

  count_t dynamic_count, dynamic_capacity;
  count_t static_count, static_capacity;

  // Obsolete
  contact *values;
  count_t capacity;
  count_t count;
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

typedef struct {
  bnd_collision_mask matrix[MAX_COLLISION_LAYERS];
  uint8_t layers_available;
} collision_matrix;

typedef enum {
  HASH_TABLE_SLOT_EMPTY,
  HASH_TABLE_SLOT_TAKEN,
  HASH_TABLE_SLOT_UNAVAILABLE,
} hash_table_slot_status;

typedef enum {
  ALIGNMENT_BROAD_CONTACT = 8,
  ALIGNMENT_BODY_MATERIAL = 4,
} common_alignments;

typedef enum {
  BODY_FLAG_NONE = 0,
  BODY_FLAG_TRIGGER = 1,
  BODY_FLAG_DIRTY = 2,
} body_flags;


#define COMMON_FIELDS                                                                                                  \
  count_t capacity;                                                                                                    \
  count_t count;                                                                                                       \
  count_t free_count;                                                                                                  \
  count_t first_outer_node;                                                                                            \
  bnd_v3 *positions;                                                                                                       \
  bnd_quat *rotations;                                                                                                     \
  body_shapes *shapes;                                                                                                 \
  bnd_aabb *aabbs;                                                                                                     \
  bnd_collision_layer *collision_layers; \
  bnd_material_handle *materials;    \
  uint8_t *flags; \
  void **custom_data;  \
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

typedef enum {
  EPA_NODE_VERTEX,
  EPA_NODE_EDGE,
  EPA_NODE_FACE,

  EPA_NODE_TYPE_COUNT,
} epa_polytope_node_type;

typedef enum {
  EPA_FLAG_FOR_REMOVAL = 1,
  EPA_FLAG_BORDER_EDGE = 2,
} epa_polytope_node_flags;

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
  body_support v;
  uint16_t first_attached_edge;
} epa_vertex;

typedef struct {
  uint16_t verticies[2];
  uint16_t next_attached_edges[2];
  uint16_t attached_faces[2];
} epa_edge;

typedef struct {
  uint16_t edges[3];
} epa_face;

typedef union {
  epa_vertex vertex;
  epa_edge edge;
  epa_face face;
} epa_polytope_node_value;

typedef struct {
  epa_polytope_node_type type;
  epa_polytope_node_value value;
  bnd_v3 normal;
  float distance;

  uint16_t prev;
} epa_polytope_node;

typedef struct {
  epa_polytope_node *nodes;
  uint8_t *flags;
  uint16_t *free_list;

  uint16_t last_nodes[EPA_NODE_TYPE_COUNT];

  uint16_t node_count;
  uint16_t free_count;
  uint16_t max_nodes;

  uint16_t nearest;
  float nearest_distance;
} epa_polytope;

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

  count_t contacts_offset;

  count_t body_a, body_b;
  bnd_body_shape shape_a, shape_b;
} collision_detection_context;

typedef struct {
  body_support points[4];
  uint8_t size;
} simplex;

typedef struct {
  uint8_t *buffer;
  uint64_t capacity;
  uint64_t offset;
  uint64_t max_offset;

  bnd_allocator allocator;
} bnd_arena;

typedef struct {
  bnd_arena *arena;
  uint64_t offset;
} bnd_arena_stack_frame;

typedef struct {
  bnd_body_handle src_body_a, src_body_b;
  bnd_body_handle dst_body_a, dst_body_b;
  bnd_result_u32 iterations_count_result;
  collision_detection_context ctx;
  simplex s;
  int target_iteration;

  bool initialized;
} epa_debug_status;

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
}static_bodies;

struct bnd_world_t {
  dynamic_bodies dynamics;
  static_bodies statics;

  contacts contacts;
  joints joints;
  mesh_storage meshes;
  events_storage events;
  contacts_cache contacts_cache;
  body_materials materials;
  collision_matrix matrix;

  bnd_arena arena;
  epa_debug_status *epa_debug;

  shapes_bracket shape_brackets[BRACKET_COUNT];

  bnd_config config;
  bnd_world_stats stats;
  bnd_allocator allocator;

  count_t id;
  count_t generation;
  count_t age;
};

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_shape shape;
  count_t index;
} shape_context;

typedef struct {
  bnd_body_shape a, b;
} collision_test_pair;

typedef struct {
  bnd_v3 position_a, position_b;
  bnd_quat rotation_a, rotation_b;

  bnd_v3 point;
  bnd_v3 normal;
  float depth;

  bool intersection;
} collision_test_case;

typedef struct {
  collision_test_pair *pairs;
  collision_test_case *cases;

  count_t num_pairs;
  count_t cases_per_pair;
} collision_test_suite;

typedef enum {
  DEBUG_EPA_NONE = 0,
  DEBUG_EPA_FACE_NEAREST = 1,
  DEBUG_EPA_FACE_REMOVED = 2,

  DEBUG_EPA_NORMAL_EDGE = 4,
  DEBUG_EPA_NORMAL_FACE = 8,
  DEBUG_EPA_NORMAL_NEAREST = 16,
} bnd_debug_epa_flags;

typedef void (*bnd_debug_draw_epa_face_fn)(bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_debug_epa_flags flags, void *user_data);
typedef void (*bnd_debug_draw_epa_normal_fn)(bnd_v3 origin, bnd_v3 unit_normal, bnd_debug_epa_flags flags, void *user_data);
typedef void (*bnd_debug_draw_epa_support_fn)(bnd_v3 point, void *user_data);

#define AlignTo(offset, alignment) ((offset) + (alignment) - 1) & ~((alignment) - 1)

typedef struct {
  bnd_debug_draw_epa_face_fn draw_face;
  bnd_debug_draw_epa_normal_fn draw_normal;
  bnd_debug_draw_epa_support_fn draw_support;
} bnd_debug_draw_epa_callbacks;


typedef support_point (*support_func)(const shape_context *, bnd_v3);

bnd_allocator         bnd_default_allocator(void);

bnd_error             resize_if_needed(bnd_allocator allocator, void **array, count_t element_size, uint64_t alignment, count_t count, count_t additional_count, count_t *capacity);

bnd_error             arena_init(bnd_allocator allocator, uint64_t capacity, bnd_arena *arena);
bnd_result_ptr        arena_alloc(bnd_arena *arena, uint64_t alignment, uint64_t size);
bnd_arena_stack_frame arena_new_stack_frame(bnd_arena *arena);
void                  arena_release_stack_frame(bnd_arena_stack_frame frame);
void                  arena_reset(bnd_arena *arena);

bnd_body_handle       make_body_handle(const bnd_world *world, bnd_body_type type, count_t index);
count_t               handle_to_inner_index(const bnd_world *world, bnd_body_handle handle);

float                 mix_restitution(const collision_detection_context *ctx);
float                 mix_friction(const collision_detection_context *ctx);

common_data          *as_common(bnd_world *world, bnd_body_type type);
const common_data    *as_common_const(const bnd_world *world, bnd_body_type type);

bnd_error             contacts_init(bnd_world *world);
void                  contacts_teardown(bnd_world *world);
bnd_error             contacts_ensure_capacity(bnd_world *world, count_t contacts_offset, count_t count);
void                  contacts_filter_largest_surface_area(contact *contacts, count_t contact_count, count_t *selected_indices);
void                  contacts_generate(bnd_world *world);
void                  resolve_constraints(bnd_world *world, float dt);

bnd_error             contacts_cache_init(bnd_world *world);
cache_entry          *contacts_cache_query(bnd_world *world, contact *contact, bnd_body_type type);
void                  contacts_cache_prune(bnd_world *world);
void                  contacts_cache_reset(bnd_world *world);

uint64_t              hash_table_create_key(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b, bnd_body_type type);
bool                  hash_table_has_key(const hash_table *table, uint64_t key);
count_t               hash_table_remove(hash_table *table, uint64_t key);
bool                  hash_table_update(hash_table *table, uint64_t key, count_t new_value);
bool                  hash_table_insert(hash_table *table, uint64_t key, count_t value);

void                  run_broad_phase(bnd_world *world);

void                  collision_detection_init(void);
count_t               collisions_detect(bnd_world *world, count_t contacts_offset, bnd_body_type type);
bnd_error             collision_detection_epa_context(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, collision_detection_context *ctx);

bnd_error             joints_init(bnd_world *world);
void                  joints_teardown(bnd_world *world);
void                  joints_reset(bnd_world *world);
count_t               joints_generate_contacts(bnd_world *world, count_t contacts_offset, bnd_body_type type);
void                  joints_remove_stale_if_needed(bnd_world *world, bnd_body_handle removed_body);

bnd_error             meshes_init(bnd_world *world);
void                  meshes_teardown(bnd_world *world);

bnd_collision_mask    layer_to_mask(bnd_collision_layer layer);
bnd_collision_mask    mask_for_count(uint8_t count);
bnd_error             materials_init(bnd_world *world);

bnd_error             shapes_init(bnd_world *world);
void                  shapes_teardown(bnd_world *world);
void                  shapes_reset(bnd_world *world);
void                  shapes_get_bracket_properties(const bnd_config *config, count_t bracket_index, count_t *blocks, count_t *shapes, count_t *capacity);
bool                  shapes_any_slot_available(const bnd_world *world, shape_dimension_bracket bracket);
bnd_error             shapes_expand_bracket(bnd_world *world, shape_dimension_bracket bracket);
bool                  shapes_put_into_empty_slot(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t *slot_number);
void                  shapes_clear_slot(bnd_world *world, shape_dimension_bracket bracket, count_t slot);
body_shapes           shapes_write(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t count);
bnd_body_shape       *shapes_get(const bnd_world *world, body_shapes shapes);

count_t               ephemeral_body_index(const common_data *data);

bnd_error             events_init(bnd_world *world);
void                  events_teardown(bnd_world *world);
void                  events_reset(bnd_world *world);
bool                  events_subscribed(const common_data *data, count_t index, bnd_event_type event_type);
bnd_error             events_push(bnd_world *world, common_data *data, count_t index, bnd_event event);

bnd_quat              integrate_rotation_midpoint(bnd_quat rotation, bnd_v3 angular_momentum, bnd_m3 base_inv_inertia, float dt);

bool                  gjk_check_intersection(const bnd_world *world, const collision_detection_context *ctx, simplex *simplex);

uint32_t              polytope_memory_size(uint16_t max_nodes);
count_t               epa_get_contact(bnd_world *world, const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact);
body_support          support(const collision_detection_context *ctx, bnd_v3 direction);

#if defined(BND_DEBUG)
void                  epa_debug_next_frame(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, epa_debug_status *status);
void                  epa_debug_capture(bnd_world *world);
bool                  epa_debug_draw(bnd_world *world, const epa_debug_status *debug_status, bnd_debug_draw_epa_callbacks callbacks, void *user_data);
#endif

float                 sqr_distance_to_triangle(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_v3 *closest);
float                 sqr_distance_to_line_segment(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 *closest);
bool                  aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b);

bnd_v3                body_center(const shape_context *ctx);
bnd_quat              body_rotation(const shape_context *ctx);

bnd_m3                quat_as_matrix(bnd_quat q);

bnd_v3                bnd_m3_rotate_inverse(bnd_v3 v, bnd_m3 m);
bnd_m3                bnd_m3_from_basis(bnd_v3 x, bnd_v3 y, bnd_v3 z);
bnd_m3                bnd_m3_skew_symmetric(bnd_v3 v);
bnd_m3                bnd_m3_initial_inertia(bnd_v3 inertia);
bnd_m3                bnd_m3_inertia(bnd_m3 initial_inertia, bnd_quat rotation);
bnd_m3                bnd_m3_displacement_inertia(bnd_m3 i0, bnd_v3 offset, float mass);

collision_test_suite *collision_tests_load(void);
void                  collision_tests_pair_spawn(bnd_world *world, const collision_test_pair *pair, bnd_body_handle *pair_handles);
void                  collision_tests_free(collision_test_suite *tests);
#endif
