#ifndef BANDURA_H
#define BANDURA_H

#include <stdbool.h>
#include <stdint.h>

#if defined(_WIN32)
  #define BNDAPI __declspec(dllexport)
#else
  #define BNDAPI __attribute((visibility("default")))
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

typedef uint32_t count_t;

typedef enum {
  BND_OK,
  BND_ERROR_NO_SPACE_AVAILABLE,
  BND_ERROR_OUT_OF_MEMORY,
  BND_ERROR_INVALID_ALLOCATOR,
  BND_ERROR_MESH_INVALID,
  BND_ERROR_MESH_IS_CONCAVE,
  BND_ERROR_BODY_HANDLE_INVALID,

  // Debug mode errors
  BND_ERROR_INVALID_POLYTOPE,
} bnd_error_type;

typedef void (*bnd_error_callback)(bnd_error_type error_type, char *error_message, void *error_data);

typedef void* (*bnd_malloc_fn)(uint64_t alignment, uint64_t size);
typedef void* (*bnd_realloc_fn)(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size);
typedef void  (*bnd_free_fn)(void *ptr, uint64_t size);

typedef enum {
  BND_DYNAMIC,
  BND_STATIC,
} bnd_body_type;

typedef enum {
  BND_BOX,
  BND_SPHERE,
  BND_CYLINDER,
  BND_MESH,

  // Keep the plane at the end
  BND_PLANE,
  BND_SHAPES_COUNT
} bnd_shape_type;

typedef enum {
  BND_EVENT_COLLISION = 1,
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
  count_t elements_count;
  count_t element_size;
  count_t stride;
} bnd_mesh_buffer;

typedef struct {
  bnd_mesh_buffer vertex_buffer;
  bnd_mesh_buffer index_buffer;
} bnd_mesh_data;

typedef uint32_t bnd_mesh_handle;

typedef struct {
  bnd_shape_type type;

  union {
    struct {
      bnd_v3 size;
    } box;

    struct {
      bnd_v3 normal;
    } plane;

    struct {
      float radius;
    } sphere;

    struct {
      float radius;
      float height;
    } cylinder;

    bnd_mesh_handle mesh;
  };

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
  uint8_t generation;
  count_t index;
} bnd_body_handle;

typedef struct {
  bnd_v3 *position;
  bnd_quat *rotation;
  bnd_v3 *velocity;
  bnd_v3 *angular_momentum;

  bnd_body_handle handle;
} bnd_body;

typedef struct {
  bnd_v3 point;
  bnd_v3 normal;
  float distance;
  bnd_body_handle body;
} bnd_raycast_hit;

typedef struct {
  struct {
    count_t dynamics_capacity;
    count_t statics_capacity;
    count_t contacts_capacity;
    count_t joints_capacity;
    count_t epa_max_nodes;
    count_t meshes_capacity;
    count_t events_capacity;
    count_t shapes_brackets_capacity[5];
  } memory;

  struct {
    bnd_v3 gravity;
    float linear_damping;
    float angular_damping;
    float restitution;
    float friction;
    float sleep_base_bias;
    float sleep_threshold;
  } simulation;

  struct {
    count_t max_gjk_iterations;
    float epa_tolerance;
  } collision_detection;

  struct {
    count_t resolution_attempts_factor;
    float penetration_epsilon;
    float velocity_epsilon;
    float restitution_damping_limit;
  } collision_resolution;
} bnd_config;

typedef struct {
  count_t body_count;
  count_t contacts_count;
  count_t incomplete_resolutions;
  count_t incomplete_collision_detections;
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

typedef struct bnd_world_t bnd_world;

typedef struct {
  bnd_body_handle handle;
  count_t generation;
} bnd_body_enumerator;

typedef bnd_body_enumerator bnd_body_enumerator_typed;

typedef struct {
  bnd_event_type type;

  union {
    struct {
      bnd_contact contact;
    } collision;
  };
} bnd_event;

typedef struct {
  count_t index;
  bnd_event e;
} bnd_event_enumerator;

BNDAPI bnd_config bnd_default_config();
BNDAPI count_t bnd_required_memory(const bnd_config *config);

BNDAPI bnd_world *bnd_init(bnd_config config);
BNDAPI bnd_world *bnd_init_with_allocator(bnd_config config, bnd_allocator allocator, bnd_error *error);

BNDAPI void bnd_register_error_callback(bnd_error_callback callback);

BNDAPI void bnd_add_plane(bnd_world *world, bnd_v3 point, bnd_v3 normal);
BNDAPI bnd_body bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size);
BNDAPI bnd_body bnd_add_box_static(bnd_world *world, bnd_v3 size);
BNDAPI bnd_body bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);
BNDAPI bnd_body bnd_add_sphere_static(bnd_world *world, float radius);
BNDAPI bnd_body bnd_add_cylinder_static(bnd_world *world, float radius, float height);
BNDAPI bnd_body bnd_add_cylinder_dynamic(bnd_world *world, float mass, float radius, float height);
BNDAPI bnd_body bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, count_t shapes_count);
BNDAPI bnd_body bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, count_t shapes_count);
BNDAPI bnd_body bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh);
BNDAPI bnd_body bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh);

BNDAPI void bnd_remove_body(bnd_world *world, bnd_body_handle handle);

BNDAPI count_t bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a,
                          bnd_v3 contact_offset_b, float max_distance);
BNDAPI void bnd_remove_joint(bnd_world *world, count_t id);
BNDAPI const bnd_joint *bnd_get_joints(const bnd_world *world, count_t *count);

BNDAPI void bnd_apply_force(bnd_world *world, bnd_body_handle handle, bnd_v3 force);
BNDAPI void bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, bnd_v3 force, bnd_v3 position);
BNDAPI void bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse);
BNDAPI void bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse, bnd_v3 position);

BNDAPI bool bnd_handle_valid(const bnd_world *world, bnd_body_handle handle);

BNDAPI count_t bnd_body_count(const bnd_world *world, bnd_body_type type);
BNDAPI count_t bnd_awake_count(const bnd_world *world);
BNDAPI count_t bnd_collisions_count(const bnd_world *world);

BNDAPI bnd_config *bnd_edit_config(bnd_world *world);
BNDAPI bnd_world_stats bnd_stats(const bnd_world *world);

BNDAPI bnd_v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_body_shape *bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, count_t *count);
BNDAPI bnd_aabb bnd_get_bounding_box(const bnd_world *world, bnd_body_handle);
BNDAPI bnd_v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_m3 bnd_get_inertia(const bnd_world *world, bnd_body_handle handle);
BNDAPI bnd_m3 bnd_get_base_inertia(const bnd_world *world, bnd_body_handle handle);
BNDAPI float bnd_get_motion_avg(const bnd_world *world, bnd_body_handle handle);
BNDAPI count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts);

BNDAPI void bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);
BNDAPI void bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);
BNDAPI void bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body);
BNDAPI bool bnd_event_any(bnd_world *world, bnd_body_handle body);
BNDAPI bool bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator);
BNDAPI bool bnd_event_next(bnd_world *world, bnd_event_enumerator *enumerator);

BNDAPI bnd_error bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass);

BNDAPI void bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator_typed *enumerator);
BNDAPI bool bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator);

BNDAPI void bnd_simulate(bnd_world *world, float dt);
BNDAPI void bnd_awaken_body(bnd_world *world, bnd_body_handle handle);
BNDAPI void bnd_put_to_sleep(bnd_world *world, bnd_body_handle handle);
BNDAPI void bnd_reset_world(bnd_world *world);

BNDAPI bool bnd_raycast_closest(const bnd_world *world, bnd_ray ray, bnd_raycast_hit *closest_hit);
BNDAPI count_t bnd_raycast_multiple(const bnd_world *world, bnd_ray ray, bnd_raycast_hit *hits, count_t max_hits);
BNDAPI count_t bnd_overlap(const bnd_world *world, bnd_v3 origin, float radius, bnd_body_handle *overlaps, count_t max_overlaps);

BNDAPI void bnd_teardown(bnd_world *world);

#endif
