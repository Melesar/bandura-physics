#ifndef BANDURA_H
#define BANDURA_H

#include <stdbool.h>
#include <stdint.h>

#if defined(_WIN32)
  #if defined(BND_BUILD_DLL)
    #define BNDAPI __declspec(dllexport)
  #elif defined(BND_USE_DLL)
    #define BNDAPI __declspec(dllimport)
  #else
    #define BNDAPI
  #endif
#else
  #if defined(BND_BUILD_DLL)
    #define BNDAPI __attribute__((visibility("default")))
  #else
    #define BNDAPI
  #endif
#endif

#if !defined(BND_CUSTOM_VEC3)
typedef struct {
  float x, y, z;
} bnd_v3;
#endif

#if !defined(BND_CUSTOM_QUAT)
typedef struct {
  float x, y, z, w;
} bnd_quat;
#endif

#if !defined(BND_CUSTOM_MAT3)
typedef struct {
  float m0[3]; // Row 0
  float m1[3]; // Row 1
  float m2[3]; // Row 2
} bnd_m3;
#endif

typedef enum {
  BND_OK,
  BND_ERROR_NO_SPACE_AVAILABLE,
  BND_ERROR_OUT_OF_MEMORY,
  BND_ERROR_INVALID_ALLOCATOR,
  BND_ERROR_INVALID_JOINT,
  BND_ERROR_INVALID_MESH,
  BND_ERROR_MESH_IS_CONCAVE,
  BND_ERROR_BODY_HANDLE_INVALID,
  BND_ERROR_INVALID_BODY_TYPE,
  BND_ERROR_INVALID_COLLISION_LAYER,
  BND_ERROR_INVALID_INPUT,

  // Debug mode errors
  BND_ERROR_INVALID_POLYTOPE,
  BND_ERROR_NOT_FOUND,
  BND_ERROR_EPA_NOT_APPLICABLE,
  BND_ERROR_EPA_NO_INTERSECTION,
} bnd_error_type;

typedef void* (*bnd_malloc_fn)(uint64_t alignment, uint64_t size);
typedef void* (*bnd_realloc_fn)(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size);
typedef void  (*bnd_free_fn)(void *ptr, uint64_t size);

typedef enum {
  BND_BODY_DYNAMIC,
  BND_BODY_STATIC,
} bnd_body_type;

typedef enum {
  BND_BOX,
  BND_SPHERE,
  BND_CAPSULE,
  BND_MESH,

  // Keep the plane at the end
  BND_PLANE,
  BND_SHAPES_COUNT
} bnd_shape_type;

typedef enum {
  BND_DEBUG_DRAW_NONE = 0,
  BND_DEBUG_DRAW_CONTACTS = 1,

  BND_DEBUG_DRAW_SHAPES_DYNAMIC = 2,
  BND_DEBUG_DRAW_SHAPES_STATIC = 4,
  BND_DEBUG_DRAW_SHAPES = BND_DEBUG_DRAW_SHAPES_DYNAMIC | BND_DEBUG_DRAW_SHAPES_STATIC,

  BND_DEBUG_DRAW_AABBS = 8,
  BND_DEBUG_DRAW_JOINTS = 16,

  BND_DEBUG_DRAW_ALL = ~0,
} bnd_debug_draw_flags;

typedef enum {
  BND_EVENT_COLLISION = 1,
  BND_EVENT_TRIGGER = 2,
} bnd_event_type;

typedef struct {
  bnd_malloc_fn malloc;
  bnd_realloc_fn realloc;
  bnd_free_fn free;
} bnd_allocator;

typedef struct {
  bnd_error_type type;
  char *message;
} bnd_error;

typedef struct {
  void *buffer;
  uint32_t elements_count;
  uint32_t element_size;
  uint32_t stride;
} bnd_mesh_buffer;

typedef struct {
  bnd_mesh_buffer vertex_buffer;
  bnd_mesh_buffer index_buffer;
} bnd_mesh_data;

typedef uint32_t   bnd_mesh_handle;
typedef uint32_t   bnd_material_handle;
typedef uint8_t    bnd_collision_layer;
typedef uint64_t   bnd_collision_mask;

typedef struct {
  bnd_v3 size;
} bnd_box;

typedef struct {
  bnd_v3 normal;
} bnd_plane;

typedef struct {
  float radius;
} bnd_sphere;

typedef struct {
  float radius;
  float height;
} bnd_capsule;

typedef union {
  bnd_box box;
  bnd_sphere sphere;
  bnd_capsule capsule;
  bnd_mesh_handle mesh;
  bnd_plane plane;
} bnd_shape;

typedef struct {
  bnd_shape_type type;
  bnd_shape value;

  bnd_v3 offset;
  bnd_quat rotation;
} bnd_body_shape;

typedef struct {
  bnd_v3 center;
  bnd_v3 half_extents;
} bnd_aabb;

typedef struct {
  bnd_v3 origin;
  bnd_v3 direction;
  float max_distance;
} bnd_ray;

typedef struct {
  bnd_body_type type;
  uint32_t world_id;
  uint32_t index;
  uint8_t generation;
} bnd_body_handle;

typedef struct {
  bnd_v3 point;
  bnd_v3 normal;
  float distance;
  bnd_body_handle body;
} bnd_raycast_hit;

typedef struct {
  uint32_t dynamics_capacity;
  uint32_t statics_capacity;
  uint32_t contacts_capacity;
  uint32_t joints_capacity;
  uint32_t meshes_capacity;
  uint32_t events_capacity;
  uint32_t materials_capacity;
} bnd_config_memory;

typedef struct {
  bnd_v3 gravity;
  float linear_drag;
  float angular_drag;
  float bounciness;
  float friction;
  float sleep_base_bias;
  float sleep_threshold;
  float min_bounce_velocity;
} bnd_config_simulation;

typedef struct {
  uint32_t max_age;
  uint32_t hash_table_capacity;
  uint32_t buffer_capacity;
  float feature_distance_threshold;
  float separation_threshold;
} bnd_config_contacts_cache;

typedef struct {
  uint32_t shapes_brackets_capacity[5];
  uint32_t max_gjk_iterations;
  float epa_tolerance;
  uint32_t resolution_attempts_factor;
  float penetration_epsilon;
  float velocity_epsilon;
  bnd_config_contacts_cache contacts_cache;
  uint16_t epa_max_nodes;
} bnd_config_advanced;

typedef struct {
  bnd_config_memory memory;
  bnd_config_simulation simulation;
  bnd_config_advanced advanced;
} bnd_config;

typedef struct {
  uint32_t body_count;
  uint32_t contacts_count;
  uint32_t incomplete_resolutions;
  uint32_t incomplete_collision_detections;
  uint32_t world_age;
} bnd_world_stats;

typedef struct {
  bnd_v3 point;
  bnd_v3 normal;
  float depth;
  bnd_body_handle body_a, body_b;
} bnd_contact;

typedef struct {
  bnd_body_handle bodies[2];
  bnd_v3 relative_contact_positions[2];
  float max_error;
} bnd_joint;

typedef void (*bnd_debug_draw_contact_fn)(bnd_v3 point, bnd_v3 normal, float depth, void *user_data);
typedef void (*bnd_debug_draw_shape_fn)(bnd_v3 position, bnd_quat rotation, bnd_body_handle body_handle, bnd_shape_type shape_type, bnd_shape shape, bool is_trigger, void *user_data);
typedef void (*bnd_debug_draw_aabb_fn)(bnd_v3 center, bnd_v3 size, bnd_body_handle body_handle, void *user_data);
typedef void (*bnd_debug_draw_joint_fn)(bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 point_a, bnd_v3 point_b, void *user_data);

typedef struct {
  bnd_debug_draw_contact_fn draw_contact;
  bnd_debug_draw_shape_fn draw_shape;
  bnd_debug_draw_aabb_fn draw_aabb;
  bnd_debug_draw_joint_fn draw_joint;
} bnd_debug_draw_callbacks;

typedef struct bnd_world_t bnd_world;

#define BND_RESULT_TYPE(suffix, type) \
  typedef struct { \
    bnd_error error; \
    type value; \
  } bnd_result_##suffix; \
  \
  bnd_result_##suffix bnd_result_##suffix##_error(bnd_error e);\


BND_RESULT_TYPE(world, bnd_world*)
BND_RESULT_TYPE(ptr, void*)
BND_RESULT_TYPE(v3, bnd_v3)
BND_RESULT_TYPE(quat, bnd_quat)
BND_RESULT_TYPE(aabb, bnd_aabb)
BND_RESULT_TYPE(u32, uint32_t)
BND_RESULT_TYPE(material, bnd_material_handle)
BND_RESULT_TYPE(layer, bnd_collision_layer)
BND_RESULT_TYPE(bool, bool)
BND_RESULT_TYPE(handle, bnd_body_handle)

#undef BND_RESULT_TYPE

typedef struct {
  bnd_body_handle handle;
  uint32_t generation;
} bnd_body_enumerator;

typedef bnd_body_enumerator bnd_body_enumerator_typed;

typedef struct {
  bnd_body_handle other;
} bnd_trigger;

typedef struct {
  bnd_event_type type;
  bnd_contact collision;
  bnd_trigger trigger;
} bnd_event;

typedef struct {
  uint32_t index;
  bnd_event e;
} bnd_event_enumerator;

#if defined(__cplusplus)
extern "C" {
#endif

BNDAPI bnd_config           bnd_default_config(void);
BNDAPI uint32_t             bnd_required_memory(const bnd_config *config);

BNDAPI bnd_world           *bnd_init(bnd_config config);
BNDAPI bnd_result_world     bnd_init_with_allocator(bnd_config config, bnd_allocator allocator);

BNDAPI bnd_error            bnd_add_plane(bnd_world *world, bnd_v3 point, bnd_v3 normal);
BNDAPI bnd_result_handle    bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size);
BNDAPI bnd_result_handle    bnd_add_box_static(bnd_world *world, bnd_v3 size);
BNDAPI bnd_result_handle    bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);
BNDAPI bnd_result_handle    bnd_add_sphere_static(bnd_world *world, float radius);
BNDAPI bnd_result_handle    bnd_add_capsule_static(bnd_world *world, float radius, float height);
BNDAPI bnd_result_handle    bnd_add_capsule_dynamic(bnd_world *world, float mass, float radius, float height);
BNDAPI bnd_result_handle    bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh);
BNDAPI bnd_result_handle    bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh);
BNDAPI bnd_result_handle    bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, uint32_t shapes_count);
BNDAPI bnd_result_handle    bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, uint32_t shapes_count);

BNDAPI bnd_result_handle    bnd_add_primitive_body(bnd_world *world, bnd_body_type type, bnd_shape_type shape_type, bnd_shape shape, float mass);
BNDAPI bnd_result_handle    bnd_add_compound_body(bnd_world *world, bnd_body_type type, bnd_body_shape *shapes, float *masses, uint32_t shapes_count);

BNDAPI bnd_material_handle  bnd_default_material(void);
BNDAPI bnd_result_material  bnd_create_material(bnd_world *world, float bounciness, float friction);
BNDAPI bnd_error            bnd_set_material_bounciness(bnd_world *world, bnd_material_handle material, float bounciness);
BNDAPI bnd_error            bnd_set_material_friction(bnd_world *world, bnd_material_handle material, float friction);
BNDAPI bnd_error            bnd_get_material_properties(bnd_world *world, bnd_material_handle material, float *bounciness, float *friction);

BNDAPI bnd_collision_mask   bnd_get_all_layers_mask(const bnd_world *world);
BNDAPI uint32_t             bnd_get_layers_count(const bnd_world *world);
BNDAPI bnd_error            bnd_set_layers_count(bnd_world *world, uint8_t new_count);
BNDAPI bnd_error            bnd_set_layers_collision(bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b, bool collide);
BNDAPI bool                 bnd_get_layers_collision(const bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b);

BNDAPI bnd_error            bnd_remove_body(bnd_world *world, bnd_body_handle handle);

BNDAPI bnd_result_u32       bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a, bnd_v3 contact_offset_b, float max_distance);
BNDAPI void                 bnd_remove_joint(bnd_world *world, uint32_t id);

BNDAPI bnd_error            bnd_apply_force(bnd_world *world, bnd_body_handle handle, bnd_v3 force);
BNDAPI bnd_error            bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, bnd_v3 force, bnd_v3 position);
BNDAPI bnd_error            bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse);
BNDAPI bnd_error            bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse, bnd_v3 position);

BNDAPI bnd_error            bnd_handle_valid(const bnd_world *world, bnd_body_handle handle);

BNDAPI uint32_t             bnd_body_count(const bnd_world *world, bnd_body_type type);
BNDAPI uint32_t             bnd_awake_count(const bnd_world *world);
BNDAPI uint32_t             bnd_collisions_count(const bnd_world *world);

BNDAPI bnd_config          *bnd_edit_config(bnd_world *world);
BNDAPI bnd_world_stats      bnd_stats(const bnd_world *world);
BNDAPI uint32_t             bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, uint32_t max_contacts);

BNDAPI bnd_result_v3        bnd_get_position(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_quat      bnd_get_rotation(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_u32       bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, bnd_body_shape *shapes, uint32_t max_shapes);
BNDAPI bnd_result_aabb      bnd_get_bounding_box(const bnd_world *world, bnd_body_handle);
BNDAPI bnd_result_v3        bnd_get_velocity(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_v3        bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_v3        bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_material  bnd_get_material(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_ptr       bnd_get_custom_data(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_result_layer     bnd_get_collision_layer(const bnd_world *world, bnd_body_handle handle);

BNDAPI bnd_error            bnd_set_position(bnd_world *world, bnd_body_handle handle, bnd_v3 position);
BNDAPI bnd_error            bnd_set_rotation(bnd_world *world, bnd_body_handle handle, bnd_quat rotation);
BNDAPI bnd_error            bnd_set_velocity(bnd_world *world, bnd_body_handle handle, bnd_v3 velocity);
BNDAPI bnd_error            bnd_set_angular_momentum(bnd_world *world, bnd_body_handle handle, bnd_v3 angular_momentum);
BNDAPI bnd_error            bnd_set_material(bnd_world *world, bnd_body_handle handle, bnd_material_handle material);
BNDAPI bnd_error            bnd_set_custom_data(bnd_world *world, bnd_body_handle handle, void *data);
BNDAPI bnd_error            bnd_set_collision_layer(bnd_world *world, bnd_body_handle handle, bnd_collision_layer layer);

BNDAPI bnd_error            bnd_set_trigger(bnd_world *world, bnd_body_handle handle, bool is_trigger);

BNDAPI bnd_error            bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);
BNDAPI bnd_error            bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);
BNDAPI bnd_error            bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body);
BNDAPI bnd_result_bool      bnd_event_any(bnd_world *world, bnd_body_handle body);
BNDAPI bnd_result_bool      bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator);
BNDAPI bool                 bnd_event_next(bnd_world *world, bnd_event_enumerator *enumerator);

BNDAPI bnd_error            bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass);

BNDAPI void                 bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator_typed *enumerator);
BNDAPI bool                 bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator);

BNDAPI void                 bnd_simulate(bnd_world *world, float dt);
BNDAPI bnd_error            bnd_awaken_body(bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_error            bnd_put_to_sleep(bnd_world *world, bnd_body_handle handle);
BNDAPI void                 bnd_reset_world(bnd_world *world);

BNDAPI bool                 bnd_raycast_closest(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *closest_hit);
BNDAPI uint32_t             bnd_raycast_multiple(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *hits, uint32_t max_hits);
BNDAPI uint32_t             bnd_overlap(const bnd_world *world, bnd_v3 origin, float radius, bnd_collision_mask mask, bnd_body_handle *overlaps, uint32_t max_overlaps);

BNDAPI void                 bnd_debug_draw(const bnd_world *world, bnd_debug_draw_flags flags, bnd_debug_draw_callbacks callbacks, void *user_data);

BNDAPI void                 bnd_teardown(bnd_world *world);

#if defined(__cplusplus)
}            //extern "C"
#endif

#endif
