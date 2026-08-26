#ifndef BANDURA_H
#define BANDURA_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file bandura.h
 * @brief Public C99 API for the Bandura 3D physics engine.
 *
 * A world owns all simulation state. Unless otherwise stated, handles and
 * pointers returned by this API are only valid until the corresponding body
 * or world is removed or destroyed. Functions that can fail return a
 * @c bnd_error directly or embed one in a @c bnd_result_* value.
 */

#define BND_VERSION_MAJOR 0
#define BND_VERSION_MINOR 1
#define BND_VERSION_PATCH 0

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

/** Three-dimensional single-precision vector. */
#if !defined(BND_CUSTOM_VEC3)
typedef struct {
  float x, y, z; /**< Vector components. */
} bnd_v3;
#endif

/** Quaternion represented as (x, y, z, w). */
#if !defined(BND_CUSTOM_QUAT)
typedef struct {
  float x, y, z, w; /**< Quaternion components. */
} bnd_quat;
#endif

/** Row-major 3x3 single-precision matrix. */
#if !defined(BND_CUSTOM_MAT3)
typedef struct {
  float m0[3]; /**< Row 0. */
  float m1[3]; /**< Row 1. */
  float m2[3]; /**< Row 2. */
} bnd_m3;
#endif

/** Status returned by operations that can fail. */
typedef enum {
  BND_OK,                                /**< No error. */
  BND_ERROR_NO_SPACE_AVAILABLE,          /**< Configured buffer capacity has exceeded and no reallocation callback is specified. @see bnd_init_with_allocator. */
  BND_ERROR_OUT_OF_MEMORY,               /**< Allocation callback returned null. */
  BND_ERROR_INVALID_ALLOCATOR,           /**< The allocator doesn't provide an allocation function */
  BND_ERROR_INVALID_JOINT,               /**< Joint binding two static bodies */
  BND_ERROR_INVALID_MESH,                /**< Malformed @ref bnd_mesh_data. @see bnd_import_mesh */
  BND_ERROR_MESH_IS_CONCAVE,             /**< The mesh is not convex. */
  BND_ERROR_BODY_HANDLE_INVALID,         /**< The provided body handle is stale, uninitialized or belongs to a different @ref bnd_world. */
  BND_ERROR_INVALID_BODY_TYPE,           /**< The operation is supported only on bodies with a certain @ref bnd_body_type. */
  BND_ERROR_INVALID_COLLISION_LAYER,     /**< Collision layer is out of bounds. @see bnd_set_layers_count */
  BND_ERROR_INVALID_INPUT,               /**< Function argument is invalid. See attached message. */

  // Debug mode errors
  BND_ERROR_INVALID_POLYTOPE,
  BND_ERROR_NOT_FOUND,
  BND_ERROR_EPA_NOT_APPLICABLE,
  BND_ERROR_EPA_NO_INTERSECTION,
} bnd_error_type;

/** Allocate @p size bytes with the requested alignment. */
typedef void* (*bnd_malloc_fn)(uint64_t alignment, uint64_t size);
/** Resize an allocation, preserving its existing contents. */
typedef void* (*bnd_realloc_fn)(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size);
/** Release an allocation previously returned by the allocator. */
typedef void  (*bnd_free_fn)(void *ptr, uint64_t size);

/** Whether a body is simulated or fixed in the environment. */
typedef enum {
  BND_BODY_DYNAMIC,
  BND_BODY_STATIC,
} bnd_body_type;

/** Shape primitive used by a body. */
typedef enum {
  BND_BOX,
  BND_SPHERE,
  BND_CAPSULE,
  BND_MESH,

  // Keep the plane at the end
  BND_PLANE,
  BND_SHAPES_COUNT
} bnd_shape_type;

/** Bit flags controlling @c bnd_debug_draw output. */
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

/** Event categories a body may receive. Values can be ORed together. */
typedef enum {
  BND_EVENT_COLLISION = 1,  /**< The body touches another body. */
  BND_EVENT_TRIGGER = 2,    /**< The body intersects a trigger. */
} bnd_event_type;

/** Set of allocation callbacks used by a world. */
typedef struct {
  bnd_malloc_fn malloc;   /**< Required allocation callback. Used only within @ref bnd_init_with_allocator function. */
  bnd_realloc_fn realloc; /**< Optional resize callback. Might be called within @ref bnd_simulate or within functions that add bodies to the world. */
  bnd_free_fn free;       /**< Optional deallocation callback. Will be called from @ref bnd_teardown */
} bnd_allocator;

/** Error code and optional human-readable diagnostic message. */
typedef struct {
  bnd_error_type type; /**< Machine-readable status. */
  char *message;       /**< Diagnostic string owned by the library. */
} bnd_error;

/** Strided vertex or index buffer used to import a mesh. */
typedef struct {
  void *buffer;             /**< Contiguous source data. */
  uint32_t elements_count;  /**< Number of elements in the buffer. */
  uint32_t element_size;    /**< Bytes copied from each element (1, 2, or 4 for indices). */
  uint32_t stride;          /**< Additional bytes between consecutive elements. */
} bnd_mesh_buffer;

/** Vertex and index buffers describing a mesh. */
typedef struct {
  bnd_mesh_buffer vertex_buffer; /**< Vertex positions, interpreted as float components. */
  bnd_mesh_buffer index_buffer;  /**< Triangle indices, interpreted as 8-, 16-, or 32-bit integers. */
} bnd_mesh_data;

/** Opaque index identifying an imported mesh. */
typedef uint32_t   bnd_mesh_handle;
/** Opaque index identifying a material in a world. */
typedef uint32_t   bnd_material_handle;
/** Index of a collision layer. */
typedef uint8_t    bnd_collision_layer;
/** Bit mask of collision layers. */
typedef uint64_t   bnd_collision_mask;

/** Box dimensions along the local x, y, and z axes. */
typedef struct {
  bnd_v3 size; /**< Full dimensions. */
} bnd_box;

/** Plane normal; the plane passes through the supplied body position. */
typedef struct {
  bnd_v3 normal;
} bnd_plane;

/** Sphere radius. */
typedef struct {
  float radius;
} bnd_sphere;

/** Capsule radius and length of its cylindrical section. */
typedef struct {
  float radius; /**< Radius of the cylindrical body and caps. */
  float height; /**< Length of the cylindrical section along local Y. */
} bnd_capsule;

/** Shape-specific data selected by @c bnd_shape_type. */
typedef union {
  bnd_box box;             /**< Box value. */
  bnd_sphere sphere;       /**< Sphere value. */
  bnd_capsule capsule;     /**< Capsule value. */
  bnd_mesh_handle mesh;    /**< Imported mesh handle. */
  bnd_plane plane;         /**< Plane value. */
} bnd_shape;

/** A shape and its transform relative to the body origin. */
typedef struct {
  bnd_shape_type type; /**< Type of the shape (box, sphere, etc). Chech this to access the appropriate field of @c value */
  bnd_shape value;     /**< Shape-specific dimensions or mesh handle. */

  bnd_v3 offset;       /**< Local-space translation from the body origin. */
  bnd_quat rotation;   /**< Local-space rotation. */
} bnd_body_shape;

/** Axis-aligned bounding box represented by center and half-extents. */
typedef struct {
  bnd_v3 center;       /**< Center of the box. */
  bnd_v3 half_extents; /**< Positive half-size along each world axis. */
} bnd_aabb;

/** Ray used by the world raycast queries. */
typedef struct {
  bnd_v3 origin;       /**< World-space ray origin. */
  bnd_v3 direction;    /**< Ray direction */
  float max_distance;  /**< Maximum distance to search. */
} bnd_ray;

/** Stable reference to a body within a world. */
typedef struct {
  bnd_body_type type; /**< Body storage category. */
  uint32_t world_id;  /**< World that created the handle. */
  uint32_t index;     /**< Stable body index. */
  uint8_t generation; /**< Generation used to reject stale handles. */
} bnd_body_handle;

/** Result of a raycast query. */
typedef struct {
  bnd_v3 point;          /**< World-space intersection point. */
  bnd_v3 normal;         /**< Surface normal at the intersection. */
  float distance;        /**< Distance from the ray origin. */
  bnd_body_handle body;  /**< Body hit by the ray. */
} bnd_raycast_hit;

/**
 * @brief Initial capacities for world-owned storage.
 *
 * @note The buffers may grow above these capacities during simulation or as a result of body addition.
 */
typedef struct {
  uint32_t dynamics_capacity;            /**< Initial dynamic-body capacity. */
  uint32_t statics_capacity;             /**< Initial static-body capacity. */
  uint32_t contacts_capacity;            /**< Initial contact capacity. */
  uint32_t joints_capacity;              /**< Initial joint capacity. */
  uint32_t meshes_capacity;              /**< Initial imported-mesh capacity. */
  uint32_t events_capacity;              /**< Initial event capacity. */
  uint32_t materials_capacity;           /**< Initial material capacity. */
  uint64_t internal_allocation_budget;   /**< How much memory the engine can allocate internally */
} bnd_config_memory;

/** Parameters controlling integration, damping, contacts, and sleeping. */
typedef struct {
  bnd_v3 gravity;              /**< Constant acceleration applied to dynamic bodies. */
  float linear_drag;           /**< Per-second linear damping factor. */
  float angular_drag;          /**< Per-second angular damping factor. */
  float bounciness;            /**< Default restitution. */
  float friction;              /**< Default friction coefficient. */
  float sleep_base_bias;       /**< Bias used when averaging motion for sleeping. */
  float sleep_threshold;       /**< Motion threshold below which a body sleeps. */
  float min_bounce_velocity;   /**< Minimum impact speed for restitution. */
} bnd_config_simulation;

/** Parameters controlling the persistent contact cache. */
typedef struct {
  uint32_t max_age;                   /**< Number of simulation frames a cached feature may remain unused. */
  uint32_t hash_table_capacity;       /**< Contact-cache hash table capacity. */
  uint32_t buffer_capacity;           /**< Number of cached contact features. */
  float feature_distance_threshold;   /**< Maximum witness-point movement for feature reuse. */
  float separation_threshold;         /**< Separation at which a cached contact is discarded. */
} bnd_config_contacts_cache;

/** Advanced collision-detection and solver parameters. */
typedef struct {
  uint32_t shapes_brackets_capacity[5]; /**< Capacities for compound-shape storage brackets. */
  uint32_t max_gjk_iterations;          /**< Maximum GJK iterations per collision test. */
  float epa_tolerance;                  /**< EPA convergence tolerance. */
  uint32_t resolution_attempts_factor;  /**< Solver iteration multiplier. */
  float penetration_epsilon;             /**< Positional-resolution tolerance. */
  float velocity_epsilon;                /**< Velocity-resolution tolerance. */
  bnd_config_contacts_cache contacts_cache; /**< Persistent contact-cache settings. */
  uint16_t epa_max_nodes;                /**< Maximum EPA polytope nodes. */
} bnd_config_advanced;

/** Complete world configuration. */
typedef struct {
  bnd_config_memory memory;          /**< Memory capacities. */
  bnd_config_simulation simulation;  /**< Simulation parameters. */
  bnd_config_advanced advanced;      /**< Collision and solver parameters. */
} bnd_config;

/** Counters collected during simulation. */
typedef struct {
  uint32_t body_count;                       /**< Total body count. */
  uint32_t contacts_count;                   /**< Contacts generated in the latest simulation. */
  uint32_t incomplete_resolutions;           /**< Solver passes that did not converge. */
  uint32_t incomplete_collision_detections;  /**< Collision tests that exceeded their budget. */
  uint32_t world_age;                        /**< Number of completed simulation steps. */
} bnd_world_stats;

/** Contact point generated by collision detection. The normal points from B toward A. */
typedef struct {
  bnd_v3 point;                      /**< World-space contact point. */
  bnd_v3 normal;                     /**< Normal from body B toward body A. */
  float depth;                       /**< Penetration depth. */
  bnd_body_handle body_a, body_b;    /**< Bodies participating in the contact. */
} bnd_contact;

/** Maximum-distance constraint between two body-local contact points. */
typedef struct {
  bnd_body_handle bodies[2];             /**< The two constrained bodies. */
  bnd_v3 relative_contact_positions[2];  /**< Body-local anchor points. */
  float max_error;                       /**< Maximum allowed distance error. */
} bnd_joint;

/**
 *  Draw one collision contact.
 *
 *  @param point World-space contact point.
 *  @param normal Contact normal pointing from body B toward body A.
 *  @param depth Penetration depth.
 *  @param user_data Value passed to @c bnd_debug_draw.
 */
typedef void (*bnd_debug_draw_contact_fn)(bnd_v3 point, bnd_v3 normal, float depth, void *user_data);

/**
 *  Draw one body shape.
 *
 *  @param position World-space position of the shape's center.
 *  @param rotation World-space rotation of the shape.
 *  @param body_handle Handle for the body to which the shape is attached.
 *  @param shape_type Type of the shape. @see bnd_body_type
 *  @param shape Shape dimensions.
 *  @param is_trigger Whether the body is a trigger.
 *  @param user_data Value passed to @c bnd_debug_draw.
 */
typedef void (*bnd_debug_draw_shape_fn)(bnd_v3 position, bnd_quat rotation, bnd_body_handle body_handle, bnd_shape_type shape_type, bnd_shape shape, bool is_trigger, void *user_data);

/**
 *  Draw one body axis-aligned bounding box.
 *
 *  @param center World-space center of the bounding box.
 *  @param size World-space size of the bounding box.
 *  @param body_handle Handle for the body represented by the bounding box.
 *  @param user_data Value passed to @c bnd_debug_draw.
 */
typedef void (*bnd_debug_draw_aabb_fn)(bnd_v3 center, bnd_v3 size, bnd_body_handle body_handle, void *user_data);

/**
 *  Draw one distance joint.
 *
 *  @param body_a Handle for the first body.
 *  @param body_b Handle for the second body.
 *  @param point_a World-space position of the joint anchor on the first body.
 *  @param point_b World-space position of the joint anchor on the second body.
 *  @param user_data Value passed to @c bnd_debug_draw.
 */
typedef void (*bnd_debug_draw_joint_fn)(bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 point_a, bnd_v3 point_b, void *user_data);

/** Optional callbacks consumed by @c bnd_debug_draw. */
typedef struct {
  bnd_debug_draw_contact_fn draw_contact;
  bnd_debug_draw_shape_fn draw_shape;
  bnd_debug_draw_aabb_fn draw_aabb;
  bnd_debug_draw_joint_fn draw_joint;
} bnd_debug_draw_callbacks;

/** Opaque simulation world. Create it with @c bnd_init or @c bnd_init_with_allocator. */
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

/** State used to enumerate bodies. */
typedef struct {
  bnd_body_handle handle; /**< Current body handle. */
  uint32_t generation;    /**< World generation captured at enumeration start. */
} bnd_body_enumerator;

typedef bnd_body_enumerator bnd_body_enumerator_typed;

/** Trigger event payload. */
typedef struct {
  bnd_body_handle other; /**< The other body involved in the trigger. */
} bnd_trigger;

/** Event payload returned by event enumeration. */
typedef struct {
  bnd_event_type type; /**< Event category. */
  bnd_contact collision; /**< Collision payload when @c type is BND_EVENT_COLLISION. */
  bnd_trigger trigger;   /**< Trigger payload when @c type is BND_EVENT_TRIGGER. */
} bnd_event;

/** State used to enumerate a body's events. */
typedef struct {
  uint32_t index; /**< Internal current event index. */
  bnd_event e;    /**< Event copied by the last successful @c bnd_event_next call. */
} bnd_event_enumerator;

#if defined(__cplusplus)
extern "C" {
#endif

BNDAPI bool bnd_check_version(int major, int minor, int patch);

/** Return a configuration populated with the engine's default values. */
BNDAPI bnd_config bnd_default_config(void);
/** Return the number of bytes required for a world with @p config. */
BNDAPI uint32_t bnd_required_memory(const bnd_config *config);

/** Create a world using the default allocator. */
BNDAPI bnd_world *bnd_init(bnd_config config);

/**
 * Create a world using caller-supplied allocation callbacks.
 *
 * @retval BND_ERROR_INVALID_ALLOCATOR `allocator.malloc` is NULL.
 * @retval BND_ERROR_OUT_OF_MEMORY The allocator cannot allocate the world or a buffer required during initialization.
 */
BNDAPI bnd_result_world bnd_init_with_allocator(bnd_config config, bnd_allocator allocator);

/**
 * Add an infinite static plane passing through @p point.
 *
 * @retval BND_ERROR_INVALID_INPUT `normal` is a zero vector.
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Static-body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A static-body buffer could not be grown.
 */
BNDAPI bnd_error bnd_add_plane(bnd_world *world, bnd_v3 point, bnd_v3 normal);

/**
 * Add a dynamic box with the given mass and dimensions.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size);

/**
 * Add a static box with the given dimensions.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_box_static(bnd_world *world, bnd_v3 size);

/**
 * Add a dynamic sphere with the given mass and radius.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);

/**
 * Add a static sphere with the given radius.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_sphere_static(bnd_world *world, float radius);

/**
 * Add a static capsule with the given radius and cylindrical height.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_capsule_static(bnd_world *world, float radius, float height);

/**
 * Add a dynamic capsule with the given mass, radius, and cylindrical height.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_capsule_dynamic(bnd_world *world, float mass, float radius, float height);

/**
 * Add a dynamic body using an imported convex mesh.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh);

/**
 * Add a static body using an imported convex mesh.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh);

/**
 * Add a static body composed of @p shapes.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, uint32_t shapes_count);

/**
 * Add a dynamic body composed of @p shapes and corresponding @p masses.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, uint32_t shapes_count);

/**
 * Add a primitive body using an explicit type and shape description.
 *
 * @retval BND_ERROR_INVALID_BODY_TYPE `type` is neither `BND_BODY_DYNAMIC` nor `BND_BODY_STATIC.
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_primitive_body(bnd_world *world, bnd_body_type type, bnd_shape_type shape_type, bnd_shape shape, float mass);

/**
 * Add a body composed of multiple shapes using an explicit body type.
 *
 * @retval BND_ERROR_INVALID_BODY_TYPE `type` is neither `BND_BODY_DYNAMIC` nor `BND_BODY_STATIC.
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Body capacity is exhausted and the allocator has no `realloc` callback.
 * @retval BND_ERROR_OUT_OF_MEMORY A body buffer could not be grown.
 */
BNDAPI bnd_result_handle bnd_add_compound_body(bnd_world *world, bnd_body_type type, bnd_body_shape *shapes, float *masses, uint32_t shapes_count);

/** Return the built-in default material handle. */
BNDAPI bnd_material_handle bnd_default_material(void);

/**
 * Create a material with the supplied bounciness and friction.
 *
 * @retval BND_ERROR_NO_SPACE_AVAILABLE Material capacity is exhausted and the allocator has no `realloc` callback, or its `realloc` callback failed.
 */
BNDAPI bnd_result_material bnd_create_material(bnd_world *world, float bounciness, float friction);

/**
 * Change a material's bounciness.
 *
 * @retval BND_ERROR_INVALID_INPUT `material` does not identify a material in the world.
 */
BNDAPI bnd_error bnd_set_material_bounciness(bnd_world *world, bnd_material_handle material, float bounciness);

/**
 * Change a material's friction.
 *
 * @retval BND_ERROR_INVALID_INPUT `material` does not identify a material in the world.
 */
BNDAPI bnd_error bnd_set_material_friction(bnd_world *world, bnd_material_handle material, float friction);

/**
 * Read a material's bounciness and friction.
 *
 * @retval BND_ERROR_INVALID_INPUT `material` does not identify a material in the world.
 */
BNDAPI bnd_error bnd_get_material_properties(bnd_world *world, bnd_material_handle material, float *bounciness, float *friction);

/** Return a mask containing all collision layers currently available. */
BNDAPI bnd_collision_mask bnd_get_all_layers_mask(const bnd_world *world);

/** Convert a variadic list of layer indices to a collision mask. */
BNDAPI bnd_collision_mask bnd_layers_to_mask(const bnd_world *world, uint32_t layers_count, ...);

/** Return the number of configured collision layers. */
BNDAPI uint32_t bnd_get_layers_count(const bnd_world *world);

/**
 * Change the number of collision layers.
 *
 * @retval BND_ERROR_INVALID_INPUT `new_count` is zero, greater than 64, or smaller than the current layer count.
 */
BNDAPI bnd_error bnd_set_layers_count(bnd_world *world, uint8_t new_count);

/**
 * Enable or disable collision between two layers.
 *
 * @retval BND_ERROR_INVALID_COLLISION_LAYER `layer_a` or `layer_b` is not currently available in the world.
 */
BNDAPI bnd_error bnd_set_layers_collision(bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b, bool collide);

/**
 * Return whether two collision layers collide. */
BNDAPI bool bnd_get_layers_collision(const bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b);

/**
 * Remove a body and invalidate its handle.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_error bnd_remove_body(bnd_world *world, bnd_body_handle handle);

/**
 * Add a maximum-distance joint and return its numeric identifier.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID Either body handle is invalid.
 * @retval BND_ERROR_INVALID_JOINT Both bodies are static.
 * @retval BND_ERROR_OUT_OF_MEMORY A joint buffer could not be grown.
 */
BNDAPI bnd_result_u32 bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a, bnd_v3 contact_offset_b, float max_distance);

/** Remove a joint by its identifier. */
BNDAPI void bnd_remove_joint(bnd_world *world, uint32_t id);

/**
 * Apply a force at the body's center of mass.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_apply_force(bnd_world *world, bnd_body_handle handle, bnd_v3 force);

/**
 * Apply a force at a world-space position, producing force and torque.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, bnd_v3 force, bnd_v3 position);

/**
 * Apply an impulse at the body's center of mass.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse);

/**
 * Apply an impulse at a world-space position, producing impulse and angular impulse.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse, bnd_v3 position);

/**
 * Validate a body handle and return an error describing why it is invalid.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID The handle belongs to another world, has the invalid-index sentinel, or refers to a removed body.
 */
BNDAPI bnd_error bnd_handle_valid(const bnd_world *world, bnd_body_handle handle);

/** Return the number of bodies of the specified type. */
BNDAPI uint32_t bnd_body_count(const bnd_world *world, bnd_body_type type);

/** Return the number of awake dynamic bodies. */
BNDAPI uint32_t bnd_awake_count(const bnd_world *world);

/** Return the number of collision contacts generated in the current frame. */
BNDAPI uint32_t bnd_collisions_count(const bnd_world *world);

/** Return a mutable pointer to the world's configuration. */
BNDAPI bnd_config *bnd_edit_config(bnd_world *world);

/** Return simulation statistics. */
BNDAPI bnd_world_stats bnd_stats(const bnd_world *world);

/** Copy up to @p max_contacts current contacts into @p contacts. */
BNDAPI uint32_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, uint32_t max_contacts);

/**
 * Get a body's world-space position.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a body's world-space rotation.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle);

/**
 * Copy up to @p max_shapes body shapes into @p shapes and return their count.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_u32 bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, bnd_body_shape *shapes, uint32_t max_shapes);

/**
 * Get a body's world-space axis-aligned bounding box.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_aabb bnd_get_bounding_box(const bnd_world *world, bnd_body_handle);

/**
 * Get a dynamic body's linear velocity.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID A dynamic `handle` is invalid for this world. Static bodies return the zero vector.
 */
BNDAPI bnd_result_v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a dynamic body's angular velocity.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID A dynamic `handle` is invalid for this world. Static bodies return the zero vector.
 */
BNDAPI bnd_result_v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a dynamic body's angular momentum.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID A dynamic `handle` is invalid for this world. Static bodies return the zero vector.
 */
BNDAPI bnd_result_v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a body's material handle.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_material bnd_get_material(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a body's caller-owned custom data pointer.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_ptr bnd_get_custom_data(const bnd_world *world, bnd_body_handle handle);

/**
 * Get a body's collision layer.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_result_layer bnd_get_collision_layer(const bnd_world *world, bnd_body_handle handle);

/**
 * Set a body's world-space position.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_error bnd_set_position(bnd_world *world, bnd_body_handle handle, bnd_v3 position);

/**
 * Set a body's world-space rotation.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_error bnd_set_rotation(bnd_world *world, bnd_body_handle handle, bnd_quat rotation);

/**
 * Set a dynamic body's linear velocity.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_set_velocity(bnd_world *world, bnd_body_handle handle, bnd_v3 velocity);

/**
 * Set a dynamic body's angular momentum.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static or invalid for this world.
 */
BNDAPI bnd_error bnd_set_angular_momentum(bnd_world *world, bnd_body_handle handle, bnd_v3 angular_momentum);

/**
 * Assign a material to a body.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 * @retval BND_ERROR_INVALID_INPUT `material` does not identify a material in the world.
 */
BNDAPI bnd_error bnd_set_material(bnd_world *world, bnd_body_handle handle, bnd_material_handle material);

/**
 * Store a caller-owned custom data pointer on a body.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 */
BNDAPI bnd_error bnd_set_custom_data(bnd_world *world, bnd_body_handle handle, void *data);

/**
 * Assign a collision layer to a body.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 * @retval BND_ERROR_INVALID_COLLISION_LAYER `layer` is not currently available in the world.
 */
BNDAPI bnd_error bnd_set_collision_layer(bnd_world *world, bnd_body_handle handle, bnd_collision_layer layer);

/**
 * Mark a body as a trigger, or restore ordinary collision behavior.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is invalid for this world.
 * @retval BND_ERROR_INVALID_BODY_TYPE A dynamic body is being made a trigger.
 */
BNDAPI bnd_error bnd_set_trigger(bnd_world *world, bnd_body_handle handle, bool is_trigger);

/**
 * Subscribe a body to a category of events.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `body` is invalid for this world.
 */
BNDAPI bnd_error bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);

/**
 * Remove a body subscription to a category of events.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `body` is invalid for this world.
 */
BNDAPI bnd_error bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type);

/**
 * Remove all event subscriptions from a body.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `body` is invalid for this world.
 */
BNDAPI bnd_error bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body);

/**
 * Return whether a body has any events in the current frame.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `body` is invalid for this world.
 */
BNDAPI bnd_result_bool bnd_event_any(bnd_world *world, bnd_body_handle body);

/**
 * Begin enumerating a body's events for the current frame.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `body` is invalid for this world.
 */
BNDAPI bnd_result_bool bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator);

/** Advance an event enumerator and copy the next event into it. */
BNDAPI bool bnd_event_next(bnd_world *world, bnd_event_enumerator *enumerator);

/**
 * Import a convex mesh and optionally return its center of mass.
 *
 * @note The mesh has to have non-null and non-empty vertex and index buffer; at least 4 vertices and 12 indicies; index buffer elements must be 1, 2, or 4 bytes long; vertex buffer elements must consist of 3 floats.
 *
 * @retval BND_ERROR_INVALID_MESH The mesh data violates the validation requirements described above.
 * @retval BND_ERROR_MESH_IS_CONCAVE The mesh is not convex.
 * @retval BND_ERROR_NO_SPACE_AVAILABLE A mesh buffer is full and cannot be grown because `allocator.realloc` is NULL, or a mesh buffer reallocation failed.
 * @retval BND_ERROR_OUT_OF_MEMORY Mesh metadata reallocation failed.
 */
BNDAPI bnd_error bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass);

/** Initialize enumeration of bodies of @p type. */
BNDAPI void bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator_typed *enumerator);

/** Advance a body enumerator and copy the next handle into it. */
BNDAPI bool bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator);

/** Advance the simulation by @p dt seconds. */
BNDAPI void bnd_simulate(bnd_world *world, float dt);

/**
 * Wake a dynamic body.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static, invalid for this world, or already awake.
 */
BNDAPI bnd_error bnd_awaken_body(bnd_world *world, bnd_body_handle handle);

/**
 * Put a dynamic body to sleep.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID `handle` is static, invalid for this world, or already asleep.
 */
BNDAPI bnd_error bnd_put_to_sleep(bnd_world *world, bnd_body_handle handle);

/** Remove bodies, contacts, and other transient state while retaining the world. */
BNDAPI void bnd_reset_world(bnd_world *world);

/** Find the closest hit along @p ray for bodies in @p mask. */
BNDAPI bool bnd_raycast_closest(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *closest_hit);

/** Find up to @p max_hits hits along @p ray for bodies in @p mask. */
BNDAPI uint32_t bnd_raycast_multiple(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *hits, uint32_t max_hits);

/** Find up to @p max_overlaps bodies intersecting a sphere query. */
BNDAPI uint32_t bnd_overlap(const bnd_world *world, bnd_v3 origin, float radius, bnd_collision_mask mask, bnd_body_handle *overlaps, uint32_t max_overlaps);

/** Invoke selected debug drawing callbacks for the current world state. */
BNDAPI void bnd_debug_draw(const bnd_world *world, bnd_debug_draw_flags flags, bnd_debug_draw_callbacks callbacks, void *user_data);

/** Destroy a world and release all memory owned by it. */
BNDAPI void bnd_teardown(bnd_world *world);

#if defined(__cplusplus)
}            //extern "C"
#endif

#endif
