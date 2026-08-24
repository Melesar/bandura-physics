#include <float.h>
#include <math.h>
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#if defined(BND_PROFILING)

#include <tracy/TracyC.h>
#endif

// ================
//   bandura.h
// ================


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


/** Three-dimensional single-precision vector. */
typedef struct {
  float x, y, z; /**< Vector components. */
} bnd_v3;

/** Quaternion represented as (x, y, z, w). */
typedef struct {
  float x, y, z, w; /**< Quaternion components. */
} bnd_quat;

/** Row-major 3x3 single-precision matrix. */
typedef struct {
  float m0[3]; /**< Row 0. */
  float m1[3]; /**< Row 1. */
  float m2[3]; /**< Row 2. */
} bnd_m3;

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
  uint32_t dynamics_capacity;  /**< Initial dynamic-body capacity. */
  uint32_t statics_capacity;   /**< Initial static-body capacity. */
  uint32_t contacts_capacity;  /**< Initial contact capacity. */
  uint32_t joints_capacity;    /**< Initial joint capacity. */
  uint32_t meshes_capacity;    /**< Initial imported-mesh capacity. */
  uint32_t events_capacity;    /**< Initial event capacity. */
  uint32_t materials_capacity; /**< Initial material capacity. */
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

#define BNDAPI

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



// ================
//   profiler.h
// ================

#ifndef BND_PROFILING

#define PROFILER_BLOCK_START(name) (void)(name)
#define PROFILER_BLOCK_END

#define PROFILER_FUNCTION_START
#define PROFILER_FUNCTION_END

#define PROFILER_FRAME_START
#define PROFILER_FRAME_END

#define PROFILER_REPORT_METRIC_INT(name, val)
#define PROFILER_REPORT_METRIC_FLOAT(name, val)

#else


#define PROFILER_BLOCK_START(name) TracyCZoneN(profiler_ctx, name, true)
#define PROFILER_BLOCK_END TracyCZoneEnd(profiler_ctx)

#define PROFILER_FRAME_START TracyCFrameMarkStart("bnd_simulate")
#define PROFILER_FRAME_END TracyCFrameMarkEnd("bnd_simulate")

#define PROFILER_FUNCTION_START PROFILER_BLOCK_START(__FUNCTION__)
#define PROFILER_FUNCTION_END PROFILER_BLOCK_END

#define PROFILER_REPORT_METRIC_INT(name, val) TracyCPlotI(name, val)
#define PROFILER_REPORT_METRIC_FLOAT(name, val) TracyCPlotF(name, val)

#endif


// ================
//   bnd-core.h
// ================


#define EPSILON 0.000001f
#define EPHEMERAL_BODIES_COUNT 4
#define DEFAULT_VERTEX_PER_MESH 512
#define DEFAULT_FACE_PER_MESH 256
#define MAX_CONTACTS_PER_PAIR 4
#define MAX_COLLISION_LAYERS 64

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

typedef struct {
  bnd_v3 witness_a, witness_b;
  bnd_v3 normal;
} contact_features;

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

typedef struct {
  bnd_collision_mask matrix[MAX_COLLISION_LAYERS];
  uint8_t layers_available;
} collision_matrix;

typedef enum {
  BODY_FLAG_NONE = 0,
  BODY_FLAG_TRIGGER = 1,
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

  bool dirty;
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
  epa_polytope epa_polytope;
  collision_matrix matrix;

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

typedef struct {
  bnd_debug_draw_epa_face_fn draw_face;
  bnd_debug_draw_epa_normal_fn draw_normal;
  bnd_debug_draw_epa_support_fn draw_support;
} bnd_debug_draw_epa_callbacks;

typedef support_point (*support_func)(const shape_context *, bnd_v3);

bnd_allocator         bnd_default_allocator(void);

bnd_body_handle       make_body_handle(const bnd_world *world, bnd_body_type type, count_t index);
count_t               handle_to_inner_index(const bnd_world *world, bnd_body_handle handle);

float                 mix_restitution(const collision_detection_context *ctx);
float                 mix_friction(const collision_detection_context *ctx);

common_data          *as_common(bnd_world *world, bnd_body_type type);
const common_data    *as_common_const(const bnd_world *world, bnd_body_type type);

bnd_error             contacts_init(bnd_world *world);
void                  contacts_teardown(bnd_world *world);
void                  contacts_reset(bnd_world *world);
bnd_error             contacts_ensure_capacity(bnd_world *world, count_t contacts_offset, count_t count);
void                  contacts_filter_largest_surface_area(contact *contacts, count_t contact_count, count_t *selected_indices);
void                  contacts_generate(bnd_world *world);
void                  contacts_resolve(bnd_world *world, float dt);

bnd_error             contacts_cache_init(bnd_world *world);
cache_entry          *contacts_cache_query(bnd_world *world, contact *contact, bnd_body_type type);
void                  contacts_cache_prune(bnd_world *world);
void                  contacts_cache_reset(bnd_world *world);

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
bnd_error             epa_init(bnd_world *world);
void                  epa_teardown(bnd_world *world);
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

// ================
//   bnd-math.h
// ================


static inline bnd_v3 bnd_v3_cross(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};
}

static inline float bnd_v3_dot(bnd_v3 x, bnd_v3 y) {
  return x.x * y.x + x.y * y.y + x.z * y.z;
}

static inline bnd_v3 bnd_v3_add(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.x + y.x, x.y + y.y, x.z + y.z};
}

static inline bnd_v3 bnd_v3_scale(bnd_v3 x, float y) {
  return (bnd_v3){x.x * y, x.y * y, x.z * y};
}

static inline float bnd_v3_len(bnd_v3 x) {
  return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z);
}

static inline bnd_v3 bnd_v3_normalize(bnd_v3 x) {
  float l = bnd_v3_len(x);
  if (l < 0.000001f) {
    return x;
  }

  float t = 1.0f / l;
  return (bnd_v3){ x.x * t, x.y * t, x.z * t };
}

static inline bnd_v3 bnd_v3_sub(bnd_v3 x, bnd_v3 y) {
  return (bnd_v3){x.x - y.x, x.y - y.y, x.z - y.z};
}

static inline float bnd_v3_lensqr(bnd_v3 x) {
  return x.x * x.x + x.y * x.y + x.z * x.z;
}

static inline float bnd_v3_distancesqr(bnd_v3 x, bnd_v3 y) {
  float dx = x.x - y.x;
  float dy = x.y - y.y;
  float dz = x.z - y.z;

  return dx * dx + dy * dy + dz * dz;
}

static inline bnd_v3 bnd_v3_zero(void) {
  return (bnd_v3){0, 0, 0};
}

static inline bnd_v3 bnd_v3_one(void) {
  return (bnd_v3){1, 1, 1};
}

static inline bnd_v3 bnd_v3_up(void) {
  return (bnd_v3){0, 1, 0};
}

static inline bnd_v3 bnd_v3_right(void) {
  return (bnd_v3){1, 0, 0};
}

static inline bnd_v3 bnd_v3_forward(void) {
  return (bnd_v3){0, 0, 1};
}

static inline bnd_v3 bnd_v3_rotate(bnd_v3 v, bnd_quat q) {
  bnd_v3 result;
  result.x =
    v.x * (q.x * q.x + q.w * q.w - q.y * q.y - q.z * q.z) +
    v.y * (2 * q.x * q.y - 2 * q.w * q.z) +
    v.z * (2 * q.x * q.z + 2 * q.w * q.y);

  result.y =
    v.x * (2 * q.w * q.z + 2 * q.x * q.y) +
    v.y * (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) +
    v.z * (-2 * q.w * q.x + 2 * q.y * q.z);

  result.z =
    v.x * (-2 * q.w * q.y + 2 * q.x * q.z) +
    v.y * (2 * q.w * q.x + 2 * q.y * q.z) +
    v.z * (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);

  return result;
}

static inline bnd_v3 bnd_v3_negate(bnd_v3 x) {
  return (bnd_v3){-x.x, -x.y, -x.z};
}

static inline bnd_v3 bnd_v3_barycentric(bnd_v3 p, bnd_v3 a, bnd_v3 b, bnd_v3 c) {
  bnd_v3 v0 = bnd_v3_sub(b, a);
  bnd_v3 v1 = bnd_v3_sub(c, a);
  bnd_v3 v2 = bnd_v3_sub(p, a);

  float d00 = bnd_v3_dot(v0, v0);
  float d01 = bnd_v3_dot(v0, v1);
  float d11 = bnd_v3_dot(v1, v1);
  float d20 = bnd_v3_dot(v2, v0);
  float d21 = bnd_v3_dot(v2, v1);

  float denom = d00 * d11 - d01 * d01;

  float y = (d11 * d20 - d01 * d21) / denom;
  float z = (d00 * d21 - d01 * d20) / denom;
  float x = 1.0f - z - y;

  return (bnd_v3){x, y, z};
}

static inline bnd_v3 bnd_v3_min(bnd_v3 a, bnd_v3 b) {
  return (bnd_v3){fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)};
}

static inline bnd_v3 bnd_v3_max(bnd_v3 a, bnd_v3 b) {
  return (bnd_v3){fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)};
}

static inline bnd_quat bnd_quat_add(bnd_quat x, bnd_quat y) {
  return (bnd_quat){x.x + y.x, x.y + y.y, x.z + y.z, x.w + y.w};
}

static inline bnd_quat bnd_quat_scale(bnd_quat x, float y) {
  return (bnd_quat){x.x * y, x.y * y, x.z * y, x.w * y};
}

static inline bnd_quat bnd_quat_mul(bnd_quat x, bnd_quat y) {
  float qax = x.x, qay = x.y, qaz = x.z, qaw = x.w;
  float qbx = y.x, qby = y.y, qbz = y.z, qbw = y.w;

  bnd_quat result;
  result.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
  result.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
  result.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
  result.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

  return result;
}

static inline bnd_quat bnd_quat_normalize(bnd_quat x) {
  float length = sqrtf(x.x * x.x + x.y * x.y + x.z * x.z + x.w * x.w);
  if (length == 0.0f) {
    length = 1.0f;
  }
  float ilength = 1.0f / length;

  return bnd_quat_scale(x, ilength);
}

static inline bnd_quat bnd_quat_invert(bnd_quat x) {
  bnd_quat result = x;
  float lengthSq = x.x * x.x + x.y * x.y + x.z * x.z + x.w * x.w;

  if (lengthSq != 0.0f) {
    float invLength = 1.0f / lengthSq;

    result.x *= -invLength;
    result.y *= -invLength;
    result.z *= -invLength;
    result.w *= invLength;
  }

  return result;
}

static inline bnd_quat bnd_quat_identity(void) {
  return (bnd_quat){0, 0, 0, 1};
}

bnd_m3 bnd_m3_identity(void);
bnd_m3 bnd_m3_transpose(bnd_m3 m);
bnd_m3 bnd_m3_inverse(bnd_m3 m);
bnd_m3 bnd_m3_add(bnd_m3 a, bnd_m3 b);
bnd_m3 bnd_m3_multiply(bnd_m3 a, bnd_m3 b);
bnd_m3 bnd_m3_scale(bnd_m3 m, float s);
bnd_m3 bnd_m3_negate(bnd_m3 m);
bnd_v3 bnd_m3_rotate(bnd_v3 v, bnd_m3 m);

#if defined(BND_TESTS)

// ================
//   testing.h
// ================


typedef void (*testing_func)();

struct test {
  char *label;
  testing_func function;
};

#define TESTS_BEGIN(name)                                                                                              \
  printf(name);                                                                                                        \
  printf(":\n\n");                                                                                                     \
  struct test tests[] = {

#define TEST(func) {#func, &func},

#define TESTS_END                                                                                                      \
  { NULL, NULL }                                                                                                       \
  }                                                                                                                    \
  ;                                                                                                                    \
  int test_index = 0;                                                                                                  \
  while (true) {                                                                                                       \
    struct test t = tests[test_index++];                                                                               \
    if (t.label == NULL && t.function == NULL)                                                                         \
      break;                                                                                                           \
    printf("  %s:  ", t.label);                                                                                        \
    t.function();                                                                                                      \
    printf("Success!\n");                                                                                              \
  };                                                                                                                   \
  printf("\n");

#endif

// ================
//   gjk.c
// ================


#define TOLERANCE FLT_EPSILON

inline static int sign(float x) {
  if (fabsf(x) < TOLERANCE) {
    return 0;
  }

  return x > 0 ? 1 : -1;
}

static inline bool is_zero(float x) {
  return fabsf(x) < TOLERANCE;
}

static void simplex_add_point(simplex *s, body_support p) {
  s->points[3] = s->points[2];
  s->points[2] = s->points[1];
  s->points[1] = s->points[0];
  s->points[0] = p;

  s->size += 1;
}

static bool simplex_update_2(simplex *s, bnd_v3 *direction) {
  bnd_v3 a = s->points[0].p;
  bnd_v3 b = s->points[1].p;
  bnd_v3 ab = bnd_v3_sub(b, a);
  bnd_v3 ao = bnd_v3_negate(a);

  bnd_v3 cr = bnd_v3_cross(ab, ao);
  float c = bnd_v3_dot(ab, ao);

  if (bnd_v3_lensqr(cr) < TOLERANCE && c > 0) {
    *direction = bnd_v3_zero();
    return false;
  }

  if (is_zero(c) || c > 0) {
    *direction = bnd_v3_cross(cr, ab);
  } else {
    s->size = 1;
    *direction = ao;
  }

  return false;
}

static bool simplex_update_3(simplex *s, bnd_v3 *direction) {
  body_support a = s->points[0];
  body_support b = s->points[1];
  body_support c = s->points[2];

  if (bnd_v3_distancesqr(a.p, b.p) < TOLERANCE || bnd_v3_distancesqr(a.p, c.p) < TOLERANCE) {
    *direction = bnd_v3_zero();
    return false;
  }

  bnd_v3 ab = bnd_v3_sub(b.p, a.p);
  bnd_v3 ac = bnd_v3_sub(c.p, a.p);
  bnd_v3 ao = bnd_v3_negate(a.p);

  bnd_v3 abc = bnd_v3_cross(ab, ac);

  float d1 = bnd_v3_dot(bnd_v3_cross(abc, ac), ao);
  if (is_zero(d1) || d1 > 0) {
    float d2 = bnd_v3_dot(ac, ao);
    if (is_zero(d2) || d2 > 0) {
      s->points[1] = c;
      s->size = 2;
      *direction = bnd_v3_cross(bnd_v3_cross(ac, ao), ac);
    } else {
    do_simplex3_edge_ab:;
      float d3 = bnd_v3_dot(ab, ao);
      if (is_zero(d3) || d3 > 0) {
        s->size = 2;
        *direction = bnd_v3_cross(bnd_v3_cross(ab, ao), ab);
      } else {
        s->size = 1;
        *direction = ao;
      }
    }
  } else {
    float d4 = bnd_v3_dot(bnd_v3_cross(ab, abc), ao);
    if (is_zero(d4) || d4 > 0) {
      goto do_simplex3_edge_ab;
    } else {
      float d5 = bnd_v3_dot(abc, ao);
      if (is_zero(d5) || d5 > 0) {
        *direction = abc;
      } else {
        body_support tmp = s->points[1];
        s->points[1] = s->points[2];
        s->points[2] = tmp;
        *direction = bnd_v3_negate(abc);
      }
    }
  }

  return false;
}

static bool simplex_update_4(simplex *s, bnd_v3 *direction) {
  body_support a = s->points[0];
  body_support b = s->points[1];
  body_support c = s->points[2];
  body_support d = s->points[3];

  bnd_v3 ab = bnd_v3_sub(b.p, a.p);
  bnd_v3 ac = bnd_v3_sub(c.p, a.p);
  bnd_v3 ad = bnd_v3_sub(d.p, a.p);
  bnd_v3 ao = bnd_v3_negate(a.p);

  bnd_v3 abc = bnd_v3_cross(ab, ac);
  bnd_v3 acd = bnd_v3_cross(ac, ad);
  bnd_v3 adb = bnd_v3_cross(ad, ab);

  int b_acd = sign(bnd_v3_dot(ab, acd));
  int c_adb = sign(bnd_v3_dot(ac, adb));
  int d_abc = sign(bnd_v3_dot(ad, abc));

  bool acd_o = sign(bnd_v3_dot(acd, ao)) == b_acd;
  bool adb_o = sign(bnd_v3_dot(adb, ao)) == c_adb;
  bool abc_o = sign(bnd_v3_dot(abc, ao)) == d_abc;

  if (acd_o && adb_o && abc_o) {
    return true;
  } else if (!acd_o) {
    s->points[1] = c;
    s->points[2] = d;
    s->size = 3;
  } else if (!adb_o) {
    s->points[2] = b;
    s->points[1] = d;
    s->size = 3;
  } else if (!abc_o) {
    s->size = 3;
  }

  return simplex_update_3(s, direction);
}

static bool simplex_update(simplex *s, bnd_v3 *direction) {
  switch (s->size) {
    case 4:
      return simplex_update_4(s, direction);

    case 3:
      return simplex_update_3(s, direction);

    case 2:
      return simplex_update_2(s, direction);
  }

  return false;
}

bool gjk_check_intersection(const bnd_world *world, const collision_detection_context *ctx, simplex *simplex) {
  PROFILER_FUNCTION_START

  bnd_v3 direction = (bnd_v3){ 1, 0, 0 };

  simplex->size = 0;

  body_support support_point = support(ctx, direction);
  simplex_add_point(simplex, support_point);
  direction = bnd_v3_normalize(bnd_v3_negate(support_point.p));

  count_t iterations = 0;
  for (iterations = 0; iterations < world->config.advanced.max_gjk_iterations; ++iterations) {
    support_point = support(ctx, direction);

    if (bnd_v3_dot(support_point.p, direction) < 0) {
      PROFILER_FUNCTION_END
      return false;
    }

    simplex_add_point(simplex, support_point);

    if (simplex_update(simplex, &direction)) {
      PROFILER_FUNCTION_END
      return true;
    }

    if (bnd_v3_lensqr(direction) < TOLERANCE) {
      PROFILER_FUNCTION_END
      return false;
    }

    direction = bnd_v3_normalize(direction);
  }

  PROFILER_FUNCTION_END

  return false;
}

// ================
//   world.c
// ================


#define INVALID_INDEX ((count_t)~0)
#define INVALID_HANDLE (bnd_body_handle) { 0, 0, INVALID_INDEX, 0 }

#define INVALID_BODY_TYPE ((bnd_result_handle) { .error = (bnd_error) { .type = BND_ERROR_INVALID_BODY_TYPE, .message = "Unknown body type" } })

#define ASSERT_BODY_DYNAMIC(handle) \
  if (handle.type != BND_BODY_DYNAMIC) { \
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID , "Operation is not valid for static bodies" }; \
  }

#define ASSERT_MATERIAL_VALID(material) \
  if (material >= world->materials.count) { \
    return (bnd_error) { BND_ERROR_INVALID_INPUT, "Provided material is invalid" }; \
  } \

#define REALLOCATE_IF_NEEDED(data, is_dynamic, allocator) \
  if ((data)->count + 1 > (data)->capacity) { \
    if (allocator.realloc == NULL) { \
      return (bnd_result_handle) { (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Bodies count exceeds capacity and Allocator.realloc is NULL. Increase capacity in bnd_config or provide a realloc function in the allocator" }, INVALID_HANDLE }; \
    } \
    \
    bnd_error e_realloc = realloc_data(data, allocator, is_dynamic); \
    if (e_realloc.type != BND_OK) { \
      return (bnd_result_handle) { e_realloc, INVALID_HANDLE }; \
    } \
  }

extern const count_t max_body_index;

static void swap_bodies(bnd_world *world, bnd_body_type type, count_t index_a, count_t index_b);
static void move_body(bnd_world *world, count_t src_index, count_t dst_index);

static bnd_v3 capsule_inertia(float radius, float height, float mass) {
  float r2 = radius * radius;
  float r3 = r2 * radius;
  float h2 = height * height;

  const float pi = 3.14159265358979323846f;
  float mcy = r2 * height * pi;
  float mhs = 2.0f / 3.0f * r3 * pi;
  float m = mcy + mhs + mhs;
  float scale = mass / m;

  float side = mcy * (h2 / 12.0f + r2 / 4.0f) + 2.0f * mhs * (2.0f * r2 / 5.0f + h2 / 2.0f + 3.0f * height * radius / 8.0f);
  float prime = mcy * r2 / 2.0f + 2 * mhs * 2 * r2 / 5.0f;
  return (bnd_v3) { scale * side, scale * prime, scale * side };
}

static bnd_v3 sphere_inertia(float radius, float mass) {
  float s = 2.0f * mass * radius * radius / 5.0f;
  return bnd_v3_scale(bnd_v3_one(), s);
}

static bnd_v3 box_inertia(bnd_v3 size, float mass) {
  float m = mass / 12.0f;
  float xx = size.x * size.x;
  float yy = size.y * size.y;
  float zz = size.z * size.z;

  bnd_v3 i = { yy + zz, xx + zz, xx + yy };
  return bnd_v3_scale(i, m);
}

static bnd_m3 mesh_inertia(const bnd_world *world, bnd_mesh_handle handle, float mass) {
  bnd_m3 base_inertia = world->meshes.inertias[handle];
  float scale = mass / world->meshes.volumes[handle];

  return bnd_m3_scale(base_inertia, scale);
}

static bnd_m3 inertia_matrix(const bnd_world *world, bnd_body_shape shape, float mass) {
  switch (shape.type) {
    case BND_BOX:
      return bnd_m3_initial_inertia(box_inertia(shape.value.box.size, mass));

    case BND_SPHERE:
      return bnd_m3_initial_inertia(sphere_inertia(shape.value.sphere.radius, mass));

    case BND_CAPSULE:
      return bnd_m3_initial_inertia(capsule_inertia(shape.value.capsule.radius, shape.value.capsule.height, mass));

    case BND_MESH:
      return mesh_inertia(world, shape.value.mesh, mass);

    default:
      return bnd_m3_initial_inertia(bnd_v3_one());
  }
}

static bnd_v3 rotated_box_half_extents(bnd_m3 rotation_matrix, bnd_v3 local_half_extends) {
  bnd_v3 half_extents;
  half_extents.x =
    fabsf(rotation_matrix.m0[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m0[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m0[2]) * local_half_extends.z;

  half_extents.y =
    fabsf(rotation_matrix.m1[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m1[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m1[2]) * local_half_extends.z;

  half_extents.z =
    fabsf(rotation_matrix.m2[0]) * local_half_extends.x +
    fabsf(rotation_matrix.m2[1]) * local_half_extends.y +
    fabsf(rotation_matrix.m2[2]) * local_half_extends.z;

  return half_extents;
}

static void calculate_aabb(bnd_world *world, common_data *data, count_t index) {
  bnd_v3 position = data->positions[index];
  bnd_quat rotation = data->rotations[index];
  body_shapes shapes_data = data->shapes[index];

  const bnd_body_shape *shapes = shapes_get(world, shapes_data);

  bnd_v3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
  bnd_v3 max = bnd_v3_negate(min);
  for (count_t i = 0; i < shapes_data.count; ++i) {
    bnd_body_shape shape = shapes[i];

    bnd_quat shape_rotation = bnd_quat_mul(rotation, shape.rotation);
    bnd_v3 shape_center = bnd_v3_add(position, bnd_v3_rotate(shape.offset, rotation));

    bnd_m3 rotation_matrix;
    bnd_v3 shape_min, shape_max;
    bnd_v3 half_extents;
    bnd_aabb local_aabb;
    switch (shape.type) {
      case BND_BOX:
        rotation_matrix = quat_as_matrix(shape_rotation);
        half_extents = rotated_box_half_extents(rotation_matrix, bnd_v3_scale(shape.value.box.size, 0.5f));
        break;

      case BND_SPHERE:
        half_extents = bnd_v3_scale(bnd_v3_one(), shape.value.sphere.radius);
        break;

      case BND_CAPSULE:
        rotation_matrix = quat_as_matrix(shape_rotation);
        local_aabb = (bnd_aabb) {
          .center = shape_center,
          .half_extents = { shape.value.capsule.radius, 0.5f * shape.value.capsule.height + shape.value.capsule.radius, shape.value.capsule.radius }
        };
        half_extents = rotated_box_half_extents(rotation_matrix, local_aabb.half_extents);
        break;

      case BND_MESH:
        rotation_matrix = quat_as_matrix(shape_rotation);
        local_aabb = world->meshes.aabbs[shape.value.mesh];
        half_extents = rotated_box_half_extents(rotation_matrix, local_aabb.half_extents);
        break;

      default:
        half_extents = (bnd_v3){FLT_MAX, FLT_MAX, FLT_MAX};
        break;
    }

    shape_min = bnd_v3_add(shape_center, bnd_v3_negate(half_extents));
    shape_max = bnd_v3_add(shape_center, half_extents);

    min = bnd_v3_min(min, shape_min);
    max = bnd_v3_max(max, shape_max);
  }

  data->aabbs[index] = (bnd_aabb) {
    .center = bnd_v3_scale(bnd_v3_add(min, max), 0.5f),
    .half_extents = bnd_v3_scale(bnd_v3_sub(max, min), 0.5f),
  };
}

static void clear_forces(bnd_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;

  const count_t size = sizeof(bnd_v3) * dynamics->count;
  memset(dynamics->forces, 0, size);
  memset(dynamics->torques, 0, size);
  memset(dynamics->impulses, 0, size);
  memset(dynamics->angular_impulses, 0, size);
  memset(dynamics->accelerations, 0, size);
}

static void awaken_body(bnd_world *world, count_t index) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count) {
    return;
  }

  dynamics->motion_avgs[index] = 2.0f * world->config.simulation.sleep_threshold;

  swap_bodies(world, BND_BODY_DYNAMIC, index, dynamics->awake_count);
  dynamics->awake_count += 1;
}

static void update_awake_statuses(bnd_world *world, float dt) {
  dynamic_bodies *dynamics = &world->dynamics;
  if (dynamics->count == 0) {
    return;
  }

  const float sleep_threshold = world->config.simulation.sleep_threshold;
  count_t awake_count = dynamics->awake_count;
  for (count_t i = 0; i < awake_count; ++i) {
    bnd_v3 angular_velocity = bnd_m3_rotate(dynamics->angular_momenta[i], dynamics->inv_intertias[i]);

    float current_motion = dynamics->motion_avgs[i];
    float new_motion = bnd_v3_lensqr(dynamics->velocities[i]) + bnd_v3_lensqr(angular_velocity);
    float bias = powf(world->config.simulation.sleep_base_bias, dt);

    float motion = current_motion * bias + new_motion * (1 - bias);
    motion = fminf(motion, 10 * sleep_threshold);

    dynamics->motion_avgs[i] = motion;
  }

  count_t left = 0;
  count_t right = dynamics->count - 1;
  while (left < awake_count && right >= awake_count) {
    while (dynamics->motion_avgs[left] > sleep_threshold) {
      left += 1;
    }

    while (dynamics->motion_avgs[right] <= sleep_threshold && right >= awake_count) {
      right -= 1;
    }

    if (left >= awake_count || right <= awake_count - 1) {
      break;
    }

    swap_bodies(world, BND_BODY_DYNAMIC, left, right);
  }

  for (count_t i = awake_count - 1; i >= left && i != (count_t)-1; --i) {
    if (dynamics->motion_avgs[i] >= sleep_threshold) {
      continue;
    }

    count_t target_index = awake_count - 1;
    if (i != target_index) {
      swap_bodies(world, BND_BODY_DYNAMIC, i, target_index);
    }

    dynamics->velocities[target_index] = dynamics->angular_momenta[target_index] = bnd_v3_zero();
    awake_count -= 1;
  }

  for (count_t i = awake_count; i <= right; ++i) {
    if (dynamics->motion_avgs[i] < sleep_threshold)
      continue;

    count_t target_index = awake_count;
    if (i != target_index)
      swap_bodies(world, BND_BODY_DYNAMIC, i, target_index);

    awake_count += 1;
  }

  dynamics->awake_count = awake_count;
}

static void bump_generations_upon_reset(common_data *data) {
  for (count_t i = data->first_outer_node; i != max_body_index; i = data->outer_lookup[i].next) {
    data->generations[i] += 1;
  }
}

static void calculate_compound_shape_dynamic(const bnd_world *world, bnd_body_shape *shapes, float *masses, count_t count, float *total_mass, bnd_m3 *inertia) {
  *total_mass = 0;
  for (count_t i = 0; i < count; ++i) {
    *total_mass += masses[i];
  }

  bnd_v3 center_of_mass = bnd_v3_zero();
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    center_of_mass = bnd_v3_add(center_of_mass, bnd_v3_scale(shape.offset, mass / *total_mass));
  }

  for (count_t i = 0; i < count; ++i) {
    shapes[i].offset = bnd_v3_sub(shapes[i].offset, center_of_mass);
  }

  *inertia = (bnd_m3){ 0 };
  for (count_t i = 0; i < count; ++i) {
    bnd_body_shape shape = shapes[i];
    float mass = masses[i];

    bnd_m3 body_inertia = inertia_matrix(world, shape, mass);
    body_inertia = bnd_m3_inertia(body_inertia, shape.rotation);
    body_inertia = bnd_m3_displacement_inertia(body_inertia, shape.offset, mass);

    *inertia = bnd_m3_add(*inertia, body_inertia);
  }
}

common_data *as_common(bnd_world *world, bnd_body_type type) {
  switch (type) {
    case BND_BODY_DYNAMIC:
      return (common_data *)&world->dynamics;

    case BND_BODY_STATIC:
      return (common_data *)&world->statics;

    default:
      return NULL;
  }
}

const common_data *as_common_const(const bnd_world *world, bnd_body_type type) {
  return as_common((bnd_world *)world, type);
}

bnd_body_handle make_body_handle(const bnd_world *world, bnd_body_type type, count_t index) {
  const common_data *data = as_common_const(world, type);
  count_t outer_index = data->inner_lookup[index];

  return (bnd_body_handle){
    .type = type,
    .world_id = world->id,
    .index = outer_index,
    .generation = data->generations[outer_index],
  };
}

count_t handle_to_inner_index(const bnd_world *world, bnd_body_handle handle) {
  return as_common_const(world, handle.type)->outer_lookup[handle.index].index;
}

void new_outer_lookup(common_data *data, outer_lookup_node *target_node, count_t index, count_t value) {
  if (data->first_outer_node == max_body_index) {
    data->first_outer_node = index;

    target_node->index = value;
    target_node->prev = max_body_index;
    target_node->next = max_body_index;
    return;
  }

  count_t current_index = data->first_outer_node;
  outer_lookup_node *node = &data->outer_lookup[current_index];
  while (node->next < index) {
    current_index = node->next;
    node = &data->outer_lookup[node->next];
  }

  if (node->next == max_body_index) {
    target_node->index = value;
    target_node->prev = current_index;
    target_node->next = max_body_index;

    node->next = index;
    return;
  }

  count_t next = node->next;
  data->outer_lookup[next].prev = index;
  node->next = index;

  target_node->index = value;
  target_node->prev = current_index;
  target_node->next = next;
}

static bnd_error realloc_data(common_data *data, bnd_allocator allocator, bool with_dynamics) {
  count_t old_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;
  while (data->count >= data->capacity) {
    data->capacity = data->capacity << 1;
  }

  count_t total_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;
  REALLOC_BUFFER4(data->positions, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->rotations, allocator, sizeof(bnd_quat), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->shapes, allocator, sizeof(body_shapes), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->aabbs, allocator, sizeof(bnd_aabb), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->materials, allocator, sizeof(bnd_material_handle), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->event_masks, allocator, sizeof(bnd_event_type), old_capacity, total_capacity);
  REALLOC_BUFFER8(data->custom_data, allocator, sizeof(void*), old_capacity, total_capacity);
  REALLOC_BUFFER1(data->flags, allocator, sizeof(uint8_t), old_capacity, total_capacity);
  REALLOC_BUFFER1(data->collision_layers, allocator, sizeof(bnd_collision_layer), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->event_links, allocator, sizeof(event_link), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->free_list, allocator, sizeof(count_t), old_capacity, total_capacity);
  REALLOC_BUFFER1(data->generations, allocator, sizeof(uint8_t), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->outer_lookup, allocator, sizeof(outer_lookup_node), old_capacity, total_capacity);
  REALLOC_BUFFER4(data->inner_lookup, allocator, sizeof(count_t), old_capacity, total_capacity);

  if (with_dynamics) {
    dynamic_bodies *dynamics = (dynamic_bodies *) data;
    REALLOC_BUFFER4(dynamics->forces, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->torques, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->impulses, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->angular_impulses, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->accelerations, allocator, sizeof(bnd_v3), old_capacity, total_capacity);

    REALLOC_BUFFER4(dynamics->inv_masses, allocator, sizeof(float), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->velocities, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->angular_momenta, allocator, sizeof(bnd_v3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->inv_inertia_tensors, allocator, sizeof(bnd_m3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->inv_intertias, allocator, sizeof(bnd_m3), old_capacity, total_capacity);
    REALLOC_BUFFER4(dynamics->motion_avgs, allocator, sizeof(float), old_capacity, total_capacity);
  }

  return OK;
}

static shape_dimension_bracket get_shapes_bracket(count_t shapes_count) {
  assert(shapes_count <= (1 << (BRACKET_COUNT - 1)));

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t bracket_capacity = 1 << i;
    if (shapes_count <= bracket_capacity) {
      return i;
    }
  }

  return BRACKET_COUNT;
}

static void init_body_common(bnd_world *world, common_data *data, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t index) {
  data->positions[index] = bnd_v3_zero();
  data->rotations[index] = bnd_quat_identity();
  data->shapes[index] = shapes_write(world, bracket, shapes, shapes_count);
  data->materials[index] = bnd_default_material();
  data->custom_data[index] = NULL;
  data->flags[index] = 0;
  data->collision_layers[index] = 0;
  data->event_masks[index] = 0;
  data->event_links[index] = (event_link) { 0 };

  calculate_aabb(world, data, index);
}

static void init_body_dynamic(bnd_world *world, float mass, bnd_m3 inertia_tensor, count_t index) {
  dynamic_bodies *data = &world->dynamics;

  data->inv_masses[index] = 1.0f / mass;
  data->velocities[index] = bnd_v3_zero();
  data->angular_momenta[index] = bnd_v3_zero();
  data->inv_inertia_tensors[index] = bnd_m3_inverse(inertia_tensor);
  data->inv_intertias[index] = data->inv_inertia_tensors[index];
  data->motion_avgs[index] = 2.0f * world->config.simulation.sleep_threshold;
  data->forces[index] = bnd_v3_zero();
  data->torques[index] = bnd_v3_zero();
  data->impulses[index] = bnd_v3_zero();
  data->angular_impulses[index] = bnd_v3_zero();
  data->accelerations[index] = bnd_v3_zero();
}

static count_t insert_new_dynamic_body(bnd_world *world) {
  dynamic_bodies *data = &world->dynamics;

  count_t prev_count = data->count;
  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : prev_count;

  count_t index;
  if (data->awake_count < prev_count) {
    move_body(world, data->awake_count, prev_count);

    index = data->awake_count;

    count_t prev_outer_index = data->inner_lookup[index];
    data->outer_lookup[prev_outer_index].index = prev_count;
    data->inner_lookup[prev_count] = prev_outer_index;

    new_outer_lookup((common_data *)data, &data->outer_lookup[outer_index], outer_index, index);
    data->inner_lookup[index] = outer_index;
  } else {
    index = prev_count;
    new_outer_lookup((common_data *)data, &data->outer_lookup[outer_index], outer_index, index);
    data->inner_lookup[index] = outer_index;
  }

  return index;
}

static bnd_result_handle add_primitive_body_static(bnd_world *world, bnd_body_shape shape) {
  common_data *data = as_common(world, BND_BODY_STATIC);
  REALLOCATE_IF_NEEDED(data, false, world->allocator)

  count_t index = data->count++;
  init_body_common(world, data, BRACKET_PRIMITIVE, &shape, 1, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_STATIC, index));
}

static bnd_result_handle add_primitive_body_dynamic(bnd_world *world, bnd_body_shape shape, float mass) {
  dynamic_bodies *data = &world->dynamics;
  REALLOCATE_IF_NEEDED((common_data *)data, true, world->allocator)

  count_t index = insert_new_dynamic_body(world);
  init_body_common(world, (common_data *)data, BRACKET_PRIMITIVE, &shape, 1, index);

  bnd_m3 inertia = inertia_matrix(world, shape, mass);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_DYNAMIC, index));
}

bnd_error bnd_add_plane(bnd_world *world, bnd_v3 point, bnd_v3 normal) {
  if (bnd_v3_lensqr(normal) < EPSILON * EPSILON) {
    return (bnd_error) { BND_ERROR_INVALID_INPUT, "Plane normal cannot be a zero-vector" };
  }

  normal = bnd_v3_normalize(normal);
  bnd_result_handle plane = add_primitive_body_static(world, (bnd_body_shape){ .type = BND_PLANE, .value = {.plane = { .normal = normal } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() });

  if (plane.error.type == BND_OK) {
    world->statics.positions[handle_to_inner_index(world, plane.value)] = point;
  }

  return plane.error;
}

bnd_result_handle bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_BOX, .value = {.box = { .size = size } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() }, mass);
}

bnd_result_handle bnd_add_box_static(bnd_world *world, bnd_v3 size) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_BOX, .value = {.box = { .size = size } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() });
}

bnd_result_handle bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_SPHERE, .value = {.sphere = { .radius = radius } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() }, mass);
}

bnd_result_handle bnd_add_sphere_static(bnd_world *world, float radius) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_SPHERE, .value = {.sphere = { .radius = radius } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() });
}

bnd_result_handle bnd_add_capsule_static(bnd_world *world, float radius, float height) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_CAPSULE, .value = {.capsule = { .radius = radius, .height = height } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() });
}

bnd_result_handle bnd_add_capsule_dynamic(bnd_world *world, float mass, float radius, float height) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_CAPSULE, .value = {.capsule = { .radius = radius, .height = height } }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() }, mass);
}

bnd_result_handle bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, count_t shapes_count) {
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);

  common_data *data = as_common(world, BND_BODY_STATIC);
  REALLOCATE_IF_NEEDED(data, false, world->allocator)

  count_t index = data->count++;
  init_body_common(world, (common_data *)&world->statics, bracket, shapes, shapes_count, index);

  count_t outer_index = data->free_count > 0 ? data->free_list[--data->free_count] : index;
  new_outer_lookup(data, &data->outer_lookup[outer_index], outer_index, index);
  data->inner_lookup[index] = outer_index;

  world->generation += 1;
  world->statics.dirty = true;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_STATIC, index));
}

bnd_result_handle bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, count_t shapes_count) {
  dynamic_bodies *data = &world->dynamics;
  REALLOCATE_IF_NEEDED((common_data *) data, true, world->allocator)

  count_t index = insert_new_dynamic_body(world);
  shape_dimension_bracket bracket = get_shapes_bracket(shapes_count);
  init_body_common(world, (common_data *)data, bracket, shapes, shapes_count, index);

  float mass;
  bnd_m3 inertia;
  bnd_body_shape *body_shapes = shapes_get(world, data->shapes[index]);
  calculate_compound_shape_dynamic(world, body_shapes, masses, shapes_count, &mass, &inertia);
  init_body_dynamic(world, mass, inertia, index);

  data->awake_count += 1;
  data->count += 1;

  world->generation += 1;

  return BND_RESULT_OK(handle, make_body_handle(world, BND_BODY_DYNAMIC, index));
}

bnd_result_handle bnd_add_mesh_dynamic(bnd_world *world, float mass, bnd_mesh_handle mesh) {
  return add_primitive_body_dynamic(world, (bnd_body_shape){ .type = BND_MESH, .value = {.mesh = mesh }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() }, mass);
}

bnd_result_handle bnd_add_mesh_static(bnd_world *world, bnd_mesh_handle mesh) {
  return add_primitive_body_static(world, (bnd_body_shape){ .type = BND_MESH, .value = {.mesh = mesh }, .offset = bnd_v3_zero(), .rotation = bnd_quat_identity() });
}

bnd_result_handle bnd_add_primitive_body(bnd_world *world, bnd_body_type type, bnd_shape_type shape_type, bnd_shape shape, float mass) {
  bnd_body_shape full_shape = (bnd_body_shape) { shape_type, shape, bnd_v3_zero(), bnd_quat_identity() };
  if (type == BND_BODY_DYNAMIC) {
    return add_primitive_body_dynamic(world, full_shape, mass);
  } else if (type == BND_BODY_STATIC) {
    return add_primitive_body_static(world, full_shape);
  }

  return INVALID_BODY_TYPE;
}

bnd_result_handle bnd_add_compound_body(bnd_world *world, bnd_body_type type, bnd_body_shape *shapes, float *masses, uint32_t shapes_count) {
  if (type == BND_BODY_DYNAMIC) {
    return bnd_add_compound_body_dynamic(world, shapes, masses, shapes_count);
  } else if (type == BND_BODY_STATIC) {
    return bnd_add_compound_body_static(world, shapes, shapes_count);
  }

  return INVALID_BODY_TYPE;
}

bnd_error bnd_remove_body(bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  data->generations[handle.index] += 1;
  data->free_list[data->free_count++] = handle.index; // We keep the outer index in the free list

  count_t index = handle_to_inner_index(world, handle);
  body_shapes shapes = data->shapes[index];
  shapes_clear_slot(world, shapes.bracket, shapes.offset);

  if (handle.type == BND_BODY_DYNAMIC) {
    count_t body_count = data->count;
    count_t awake_count = world->dynamics.awake_count;

    if (index < awake_count) {
      world->dynamics.awake_count -= 1;
      swap_bodies(world, handle.type, index, awake_count - 1);

      if (awake_count < body_count) {
        swap_bodies(world, handle.type, awake_count - 1, body_count - 1);
      }
    } else {
      swap_bodies(world, handle.type, index, body_count - 1);
    }
  } else {
    swap_bodies(world, handle.type, index, data->count - 1);
  }

  data->count -= 1;

  joints_remove_stale_if_needed(world, handle);

  outer_lookup_node *outer_node = &data->outer_lookup[handle.index];
  outer_node->index = max_body_index;

  if (data->first_outer_node == handle.index) {
    data->first_outer_node = outer_node->next;
  }

  if (outer_node->prev != max_body_index) {
    data->outer_lookup[outer_node->prev].next = outer_node->next;
  }

  if (outer_node->next != max_body_index) {
    data->outer_lookup[outer_node->next].prev = outer_node->prev;
  }

  world->generation += 1;

  return OK;
}

bnd_error bnd_apply_force(bnd_world *world, bnd_body_handle handle, bnd_v3 force) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.forces[index];
  world->dynamics.forces[index] = bnd_v3_add(prev_force, force);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, bnd_v3 force, bnd_v3 position) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.forces[index];
  bnd_v3 prev_torque = world->dynamics.torques[index];

  bnd_v3 r = bnd_v3_sub(position, world->dynamics.positions[index]);
  bnd_v3 torque = bnd_v3_cross(r, force);

  world->dynamics.forces[index] = bnd_v3_add(prev_force, force);
  world->dynamics.torques[index] = bnd_v3_add(prev_torque, torque);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_impulse = world->dynamics.impulses[index];
  world->dynamics.impulses[index] = bnd_v3_add(prev_impulse, impulse);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, bnd_v3 impulse, bnd_v3 position) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 prev_force = world->dynamics.impulses[index];
  bnd_v3 prev_angular_impulse = world->dynamics.angular_impulses[index];

  bnd_v3 r = bnd_v3_sub(position, world->dynamics.positions[index]);
  bnd_v3 angular_impulse = bnd_v3_cross(r, impulse);

  world->dynamics.impulses[index] = bnd_v3_add(prev_force, impulse);
  world->dynamics.angular_impulses[index] = bnd_v3_add(prev_angular_impulse, angular_impulse);

  awaken_body(world, index);

  return OK;
}

bnd_error bnd_handle_valid(const bnd_world *world, bnd_body_handle handle) {
  if (handle.world_id != world->id) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle belongs to a different world" };
  }

  const common_data *data = as_common_const(world, handle.type);
  if (handle.index == INVALID_INDEX) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle doesn't belong to an actual body" };
  }

  if (handle.generation != data->generations[handle.index]) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle refers to a body that has been removed" };
  }

  return OK;
}

bnd_config *bnd_edit_config(bnd_world *world) {
  return &world->config;
}

bnd_world_stats bnd_stats(const bnd_world *world) {
  return world->stats;
}

count_t bnd_body_count(const bnd_world *world, bnd_body_type type) {
  return as_common_const(world, type)->count;
}

count_t bnd_awake_count(const bnd_world *world) {
  return world->dynamics.awake_count;
}

count_t bnd_collisions_count(const bnd_world *world) {
  return world->contacts.count;
}

bnd_result_v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, data->positions[index]);
}

bnd_result_quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(quat, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(quat, data->rotations[index]);
}

bnd_result_u32 bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, bnd_body_shape *shapes, uint32_t max_shapes) {
  PROPAGATE_RESULT(u32, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);

  count_t index = handle_to_inner_index(world, handle);
  body_shapes body_shapes = data->shapes[index];
  bnd_body_shape *inner_shapes = shapes_get(world, body_shapes);

  count_t count = max_shapes >= body_shapes.count ? body_shapes.count : max_shapes;
  memcpy(shapes, inner_shapes, count * sizeof(bnd_body_shape));

  return BND_RESULT_OK(u32, count);
}

bnd_result_aabb bnd_get_bounding_box(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(aabb, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(aabb, data->aabbs[index]);
}

bnd_result_v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, world->dynamics.velocities[index]);
}

bnd_result_v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  const dynamic_bodies *dynamics = &world->dynamics;
  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  bnd_v3 momentum = dynamics->angular_momenta[index];
  bnd_quat rotation = dynamics->rotations[index];
  bnd_m3 inv_inertia = dynamics->inv_inertia_tensors[index];

  return BND_RESULT_OK(v3, bnd_m3_rotate(momentum, bnd_m3_inertia(inv_inertia, rotation)));
}

bnd_result_v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle) {
  if (handle.type != BND_BODY_DYNAMIC) {
    return BND_RESULT_OK(v3, bnd_v3_zero());
  }

  PROPAGATE_RESULT(v3, bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  return BND_RESULT_OK(v3, world->dynamics.angular_momenta[index]);
}

bnd_result_ptr bnd_get_custom_data(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(ptr, bnd_handle_valid(world, handle))

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  return BND_RESULT_OK(ptr, data->custom_data[index]);
}

bnd_error bnd_set_position(bnd_world *world, bnd_body_handle handle, bnd_v3 position) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  data->positions[index] = position;

  return OK;
}

bnd_error bnd_set_rotation(bnd_world *world, bnd_body_handle handle, bnd_quat rotation) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  data->rotations[index] = rotation;

  return OK;
}

bnd_error bnd_set_velocity(bnd_world *world, bnd_body_handle handle, bnd_v3 velocity) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  world->dynamics.velocities[index] = velocity;

  return OK;
}

bnd_error bnd_set_angular_momentum(bnd_world *world, bnd_body_handle handle, bnd_v3 angular_momentum) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  world->dynamics.angular_momenta[index] = angular_momentum;

  return OK;
}

bnd_error bnd_set_custom_data(bnd_world *world, bnd_body_handle handle, void *data) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle));

  common_data *cdata = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  cdata->custom_data[index] = data;

  return OK;
}

bnd_error bnd_set_collision_layer(bnd_world *world, bnd_body_handle handle, bnd_collision_layer layer) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle));

  if (layer >= world->matrix.layers_available) {
    return (bnd_error) { BND_ERROR_INVALID_COLLISION_LAYER, "Provided layer doesn't exist" };
  }

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  data->collision_layers[index] = layer;

  return OK;
}

count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts) {
  count_t count = world->contacts.count < max_contacts ? world->contacts.count : max_contacts;
  for (count_t i = 0; i < count; ++i) {
    contact full_contact = world->contacts.values[i];
    bnd_body_type type = i < world->contacts.dynamic_count ? BND_BODY_DYNAMIC : BND_BODY_STATIC;

    contacts[i] = (bnd_contact){
      .point = full_contact.point,
      .normal = full_contact.normal,
      .depth = full_contact.depth,
      .body_a = make_body_handle(world, BND_BODY_DYNAMIC, full_contact.index_a),
      .body_b = make_body_handle(world, type, full_contact.index_b)
    };
  }

  return count;
}


void bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator *enumerator) {
  enumerator->handle = (bnd_body_handle){ .type = type, .index = max_body_index & 0x7FFFFF };
  enumerator->generation = world->generation;
}

bool bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator) {
  if (enumerator->generation != world->generation) {
    return false;
  }

  const common_data *data = as_common_const(world, enumerator->handle.type);
  if (enumerator->handle.index == max_body_index) {
    if (data->count == 0) {
      return false;
    }

    enumerator->handle.index = data->first_outer_node;
    enumerator->handle.generation = data->generations[enumerator->handle.index];
    return true;
  }

  outer_lookup_node node = data->outer_lookup[enumerator->handle.index];
  if (node.next == max_body_index) {
    return false;
  }

  enumerator->handle.index = node.next;
  enumerator->handle.generation = data->generations[enumerator->handle.index];
  return true;
}

static void update_aabbs(bnd_world *world) {
  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    calculate_aabb(world, (common_data *) dynamics, i);
  }

  if (!world->statics.dirty) {
    return;
  }

  common_data *statics = as_common(world, BND_BODY_STATIC);
  for (count_t i = 0; i < statics->count; ++i) {
    calculate_aabb(world, statics, i);
  }

  world->statics.dirty = false;
}

static void integrate_bodies(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  bnd_v3 gravity_acc = world->config.simulation.gravity;
  float linear_damping = powf(world->config.simulation.linear_drag, dt);
  float angular_damping = powf(world->config.simulation.angular_drag, dt);

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    float inv_mass = dynamics->inv_masses[i];

    bnd_v3 acceleration = bnd_v3_scale(dynamics->forces[i], inv_mass);
    acceleration = bnd_v3_add(acceleration, gravity_acc);

    bnd_v3 impulse = bnd_v3_scale(dynamics->impulses[i], inv_mass);

    bnd_v3 velocity = dynamics->velocities[i];
    velocity = bnd_v3_add(velocity, bnd_v3_scale(acceleration, dt));
    velocity = bnd_v3_add(velocity, impulse);
    velocity = bnd_v3_scale(velocity, linear_damping);

    bnd_quat rotation = dynamics->rotations[i];
    bnd_m3 base_inv_inertia = dynamics->inv_inertia_tensors[i];

    bnd_v3 momentum_delta = bnd_v3_scale(dynamics->torques[i], dt);
    momentum_delta = bnd_v3_add(momentum_delta, dynamics->angular_impulses[i]);

    bnd_v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = bnd_v3_add(angular_momentum, momentum_delta);
    angular_momentum = bnd_v3_scale(angular_momentum, angular_damping);

    rotation = integrate_rotation_midpoint(rotation, angular_momentum, base_inv_inertia, dt);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
    dynamics->inv_intertias[i] = bnd_m3_inertia(base_inv_inertia, rotation);
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = bnd_v3_add(dynamics->positions[i], bnd_v3_scale(velocity, dt));
  }

  PROFILER_FUNCTION_END

}

void bnd_simulate(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  world->stats.body_count = world->dynamics.count + world->statics.count;
  world->stats.world_age = world->age;

  // TODO before changing the order of integration and collision detection,
  // revisit aabb generation.
  integrate_bodies(world, dt);
  update_aabbs(world);
  contacts_reset(world);
  events_reset(world);
#if defined(BND_DEBUG)
  epa_debug_capture(world);
#endif
  contacts_generate(world);
  contacts_resolve(world, dt);
  update_awake_statuses(world, dt);
  clear_forces(world);
  contacts_cache_prune(world);

  world->age += 1;

  PROFILER_FUNCTION_END

  PROFILER_REPORT_METRIC_INT("Dynamic bodies", world->dynamics.count);
  PROFILER_REPORT_METRIC_INT("Static bodies", world->statics.count);
  PROFILER_REPORT_METRIC_INT("Total bodies", world->dynamics.count + world->statics.count);
}

bnd_error bnd_put_to_sleep(bnd_world *world, bnd_body_handle handle) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  dynamic_bodies *dynamics = &world->dynamics;
  count_t index = handle_to_inner_index(world, handle);
  if (index >= dynamics->awake_count) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "The body is already asleep" };
  }

  count_t target_index = dynamics->awake_count > 0 ? dynamics->awake_count - 1 : 0;
  if (index != target_index) {
    swap_bodies(world, BND_BODY_DYNAMIC, index, target_index);
  }

  dynamics->awake_count -= 1;
  dynamics->motion_avgs[target_index] = 0;
  dynamics->velocities[target_index] = bnd_v3_zero();
  dynamics->angular_momenta[target_index] = bnd_v3_zero();

  return OK;
}

bnd_error bnd_awaken_body(bnd_world *world, bnd_body_handle handle) {
  ASSERT_BODY_DYNAMIC(handle)

  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  count_t index = handle_to_inner_index(world, handle);
  dynamic_bodies *dynamics = &world->dynamics;
  if (index < dynamics->awake_count || index >= dynamics->count) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "The body is already awake" };
  }

  count_t target_index = dynamics->awake_count;
  if (index != target_index) {
    swap_bodies(world, BND_BODY_DYNAMIC, index, target_index);
  }

  dynamics->motion_avgs[target_index] = 2.0f * world->config.simulation.sleep_threshold;
  dynamics->awake_count += 1;

  return OK;
}

void bnd_reset_world(bnd_world *world) {
  world->age = 0;

  bump_generations_upon_reset((common_data *) &world->dynamics);
  bump_generations_upon_reset((common_data *) &world->statics);

  world->dynamics.count = 0;
  world->dynamics.free_count = 0;
  world->dynamics.awake_count = 0;
  world->dynamics.first_outer_node = max_body_index;

  world->statics.count = 0;
  world->statics.free_count = 0;
  world->statics.first_outer_node = max_body_index;

  world->stats.incomplete_resolutions = 0;
  world->stats.incomplete_collision_detections = 0;

  contacts_reset(world);
  shapes_reset(world);
  joints_reset(world);
  contacts_cache_reset(world);
}

static void swap_bodies(bnd_world *world, bnd_body_type type, count_t index_a, count_t index_b) {
  common_data *data = as_common(world, type);

#define SWAP_COMMON(t, arr)                                                                                            \
  t tmp_##arr = data->arr[index_a];                                                                                    \
  data->arr[index_a] = data->arr[index_b];                                                                             \
  data->arr[index_b] = tmp_##arr;

#define SWAP_DYNAMIC(t, arr)                                                                                           \
  t tmp_##arr = world->dynamics.arr[index_a];                                                                          \
  world->dynamics.arr[index_a] = world->dynamics.arr[index_b];                                                         \
  world->dynamics.arr[index_b] = tmp_##arr;

  SWAP_COMMON(bnd_v3, positions)
  SWAP_COMMON(bnd_quat, rotations)
  SWAP_COMMON(body_shapes, shapes)
  SWAP_COMMON(bnd_aabb, aabbs)
  SWAP_COMMON(bnd_material_handle, materials)
  SWAP_COMMON(bnd_collision_layer, collision_layers)
  SWAP_COMMON(bnd_event_type, event_masks)
  SWAP_COMMON(uint8_t, flags)
  SWAP_COMMON(event_link, event_links)
  SWAP_COMMON(count_t, inner_lookup)
  SWAP_COMMON(void*, custom_data)

  if (type == BND_BODY_DYNAMIC) {
    SWAP_DYNAMIC(float, inv_masses)
    SWAP_DYNAMIC(bnd_v3, velocities)
    SWAP_DYNAMIC(bnd_v3, angular_momenta)
    SWAP_DYNAMIC(bnd_m3, inv_inertia_tensors)
    SWAP_DYNAMIC(bnd_m3, inv_intertias)
    SWAP_DYNAMIC(float, motion_avgs)

    SWAP_DYNAMIC(bnd_v3, forces)
    SWAP_DYNAMIC(bnd_v3, torques)
    SWAP_DYNAMIC(bnd_v3, impulses)
    SWAP_DYNAMIC(bnd_v3, angular_impulses)
    SWAP_DYNAMIC(bnd_v3, accelerations)
  }

  data->outer_lookup[data->inner_lookup[index_b]].index = index_b;
  data->outer_lookup[data->inner_lookup[index_a]].index = index_a;

  world->generation += 1;

#undef SWAP_COMMON
#undef SWAP_DYNAMIC
}

static void move_body(bnd_world *world, count_t src_index, count_t dst_index) {
  dynamic_bodies *data = &world->dynamics;

  data->positions[dst_index] = data->positions[src_index];
  data->rotations[dst_index] = data->rotations[src_index];
  data->shapes[dst_index] = data->shapes[src_index];
  data->aabbs[dst_index] = data->aabbs[src_index];
  data->materials[dst_index] = data->materials[src_index];
  data->custom_data[dst_index] = data->custom_data[src_index];
  data->collision_layers[dst_index] = data->collision_layers[src_index];
  data->flags[dst_index] = data->flags[src_index];
  data->event_masks[dst_index] = data->event_masks[src_index];
  data->event_links[dst_index] = data->event_links[src_index];
  data->inv_masses[dst_index] = data->inv_masses[src_index];
  data->velocities[dst_index] = data->velocities[src_index];
  data->angular_momenta[dst_index] = data->angular_momenta[src_index];
  data->inv_inertia_tensors[dst_index] = data->inv_inertia_tensors[src_index];
  data->inv_intertias[dst_index] = data->inv_intertias[src_index];
  data->motion_avgs[dst_index] = data->motion_avgs[src_index];

  data->forces[dst_index] = data->forces[src_index];
  data->torques[dst_index] = data->torques[src_index];
  data->impulses[dst_index] = data->impulses[src_index];
  data->angular_impulses[dst_index] = data->angular_impulses[src_index];
  data->accelerations[dst_index] = data->accelerations[src_index];
}

bnd_collision_mask bnd_get_all_layers_mask(const bnd_world *world) {
  return mask_for_count(world->matrix.layers_available);
}

uint32_t bnd_get_layers_count(const bnd_world *world) {
  return world->matrix.layers_available;
}

bnd_error bnd_set_layers_count(bnd_world *world, uint8_t new_count) {
  if (new_count == 0) {
    return (bnd_error) { BND_ERROR_INVALID_INPUT, "There should be at least one collision layer" };
  }

  if (new_count > MAX_COLLISION_LAYERS) {
    return (bnd_error) { BND_ERROR_INVALID_INPUT, "Maximum available collision layers is 64" };
  }

  collision_matrix *matrix = &world->matrix;

  uint8_t old_count = matrix->layers_available;
  if (new_count == old_count) {
    return OK;
  }

  if (new_count < old_count) {
    return (bnd_error) { BND_ERROR_INVALID_INPUT, "Reducing the collision layers count is unsafe. There might be bodies on layers which would become out-of-bounds" };
  }

  matrix->layers_available = new_count;  

  bnd_collision_mask old_mask = mask_for_count(old_count);
  bnd_collision_mask new_mask = mask_for_count(new_count);
  bnd_collision_mask diff_mask = new_mask & ~old_mask;

  // Make old layers collide with new ones
  for (count_t i = 0; i < old_count; ++i) {
    matrix->matrix[i] |= diff_mask;
  }

  // Make new layers collide with everything
  for (count_t i = old_count; i < new_count; ++i) {
    matrix->matrix[i] = new_mask;
  }

  return OK;
}

bnd_error bnd_set_layers_collision(bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b, bool collide) {
  collision_matrix *matrix = &world->matrix;
  if (layer_a >= matrix->layers_available || layer_b >= matrix->layers_available) {
    return (bnd_error) { BND_ERROR_INVALID_COLLISION_LAYER, "Provided invalid collision layer. Use bnd_set_collision_layers_count to make more room for new layers (up to 64)" };
  }

  bnd_collision_mask a_to_b_mask = layer_to_mask(layer_b);
  bnd_collision_mask b_to_a_mask = layer_to_mask(layer_a);

  bnd_collision_mask mask_a = matrix->matrix[layer_a];
  bnd_collision_mask mask_b = matrix->matrix[layer_b];

  if (collide) {
    mask_a |= a_to_b_mask;
    mask_b |= b_to_a_mask;
  } else {
    mask_a &= ~a_to_b_mask;
    mask_b &= ~b_to_a_mask;
  }
  matrix->matrix[layer_a] = mask_a;
  matrix->matrix[layer_b] = mask_b;

  return OK;
}

bool bnd_get_layers_collision(const bnd_world *world, bnd_collision_layer layer_a, bnd_collision_layer layer_b) {
  const collision_matrix *matrix = &world->matrix;
  if (layer_a >= matrix->layers_available || layer_b >= matrix->layers_available) {
    return false;
  }

  bnd_collision_mask mask = matrix->matrix[layer_a];
  return mask & layer_to_mask(layer_b);
}

bnd_collision_mask mask_for_count(uint8_t count) {
  return count == 64 ? UINT64_MAX : ((UINT64_C(1) << count) - 1);
}

bnd_collision_mask layer_to_mask(bnd_collision_layer layer) {
  return UINT64_C(1) << layer;
}


bnd_collision_mask bnd_layers_to_mask(const bnd_world *world, uint32_t layers_count, ...) {
  va_list list;
  va_start(list, layers_count);

  bnd_collision_mask mask = 0;
  for (uint32_t i = 0; i < layers_count; ++i) {
    bnd_collision_layer layer = (bnd_collision_layer) (va_arg(list, int) & 0xFF);
    if (layer >= world->matrix.layers_available) {
      continue;
    }

    mask |= (UINT64_C(1) << layer);
  }

  va_end(list);

  return mask;
}

bnd_material_handle  bnd_default_material(void) {
  return 0;
}

bnd_result_material bnd_create_material(bnd_world *world, float bounciness, float friction) {
  count_t count = world->materials.count;
  count_t capacity = world->materials.capacity;
  body_material *values = world->materials.values;

  if (count >= capacity) {
    if (world->allocator.realloc == NULL) {
      return (bnd_result_material) { { .type = BND_ERROR_NO_SPACE_AVAILABLE, "Failed to realloc materials buffer. Re-alloc function not provided" }, 0 };
    }

    count_t new_capacity = capacity << 1;
    world->materials.values = world->allocator.realloc(values, 4, sizeof(body_material) * capacity, sizeof(body_material) * new_capacity);
    if (world->materials.values == NULL) {
      return (bnd_result_material) { { .type = BND_ERROR_NO_SPACE_AVAILABLE, "Failed to realloc materials buffer. Re-alloc function returned null" }, 0 };
    }
    world->materials.capacity = new_capacity;
  }

  bnd_material_handle handle = world->materials.count++;
  body_material *material = &world->materials.values[handle];

  material->friction = fminf(fmaxf(0, friction), 1);
  material->restitution = fminf(fmaxf(0, bounciness), 1);

  return BND_RESULT_OK(material, handle);
}

bnd_result_material bnd_get_material(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(material, bnd_handle_valid(world, handle));

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  return BND_RESULT_OK(material, data->materials[index]);
}

bnd_result_layer bnd_get_collision_layer(const bnd_world *world, bnd_body_handle handle) {
  PROPAGATE_RESULT(layer, bnd_handle_valid(world, handle));

  const common_data *data = as_common_const(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  return BND_RESULT_OK(layer, data->collision_layers[index]);
}

bnd_error bnd_set_material(bnd_world *world, bnd_body_handle handle, bnd_material_handle material) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle));
  ASSERT_MATERIAL_VALID(material)

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  data->materials[index] = material;

  return OK;
}

bnd_error bnd_set_material_bounciness(bnd_world *world, bnd_material_handle material, float bounciness) {
  ASSERT_MATERIAL_VALID(material)

  body_material *m = &world->materials.values[material];
  m->restitution = bounciness;

  return OK;
}

bnd_error bnd_set_material_friction(bnd_world *world, bnd_material_handle material, float friction) {
  ASSERT_MATERIAL_VALID(material)

  body_material *m = &world->materials.values[material];
  m->friction = friction;

  return OK;
}

bnd_error bnd_get_material_properties(bnd_world *world, bnd_material_handle material, float *bounciness, float *friction) {
  ASSERT_MATERIAL_VALID(material)

  body_material *m = &world->materials.values[material];
  *bounciness = m->restitution;
  *friction = m->friction;

  return OK;
}

bnd_error materials_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;
  bnd_config config = world->config;

  world->materials.capacity = config.memory.materials_capacity >= 1 ? config.memory.materials_capacity : 1;
  world->materials.count = 1;
  ALLOC_BUFFER4(world->materials.values, config.memory.materials_capacity * sizeof(body_material));

  world->materials.values[bnd_default_material()] = (body_material){ config.simulation.bounciness, config.simulation.friction };

  return OK;
}

float mix_restitution(const collision_detection_context *ctx) {
  float a = ctx->world->materials.values[ctx->data_a->materials[ctx->body_a]].restitution;
  float b = ctx->world->materials.values[ctx->data_b->materials[ctx->body_b]].restitution;

  return fmaxf(a, b);
}

float mix_friction(const collision_detection_context *ctx) {
  float a = ctx->world->materials.values[ctx->data_a->materials[ctx->body_a]].friction;
  float b = ctx->world->materials.values[ctx->data_b->materials[ctx->body_b]].friction;

  return sqrtf(a * b);
}

bnd_error bnd_set_trigger(bnd_world *world, bnd_body_handle handle, bool is_trigger) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  if (is_trigger && handle.type == BND_BODY_DYNAMIC) {
    return (bnd_error) { BND_ERROR_INVALID_BODY_TYPE, "Dynamic bodies cannot become triggers" };
  }

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  if (is_trigger) {
    data->flags[index] |= BODY_FLAG_TRIGGER;
  } else {
    data->flags[index] &= ~BODY_FLAG_TRIGGER;
  }

  return OK;
}

// ================
//   mesh.c
// ================


static inline bnd_error mesh_validation_error(char *message) {
  return (bnd_error) { BND_ERROR_INVALID_MESH, message };
}

static inline bnd_error realloc_error(void) {
  return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Allocator.realloc failed to re-allocate mesh buffer" };
}

static bnd_error resize_buffer(bnd_allocator allocator, void **buffer, count_t alignment, count_t count, count_t *capacity, count_t element_size) {
  count_t old_capacity = *capacity;
  if (count <= old_capacity) {
    return OK;
  }

  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Mesh buffer is full and Allocator.realloc is NULL" };
  }

  count_t new_capacity = old_capacity;
  while (count > new_capacity) {
    new_capacity *= 2;
  }

  void *new_buffer = allocator.realloc(*buffer, alignment, old_capacity * element_size, new_capacity * element_size);
  if (new_buffer == NULL) {
    return realloc_error();
  }

  *buffer = new_buffer;
  *capacity = new_capacity;

  return OK;
}

static bnd_error ensure_meshes_capacity(bnd_allocator allocator, mesh_storage *meshes) {
  if (meshes->mesh_count + 1 < meshes->mesh_capacity) {
    return OK;
  }

  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Mesh buffer is full and Allocator.realloc is NULL" };
  }

  count_t old_capacity = meshes->mesh_capacity;
  while (meshes->mesh_count + 1 > meshes->mesh_capacity) {
    meshes->mesh_capacity *= 2;
  }

  REALLOC_BUFFER4(meshes->meshes, allocator, sizeof(bnd_mesh), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->inertias, allocator, sizeof(bnd_m3), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->volumes, allocator, sizeof(float), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->aabbs, allocator, sizeof(bnd_aabb), old_capacity, meshes->mesh_capacity);

  return OK;
}

static bnd_v3 read_vertex(const bnd_mesh_buffer *buffer, count_t i) {
  bnd_v3 vertex = bnd_v3_zero();
  uint8_t *verticies = buffer->buffer;
  uint32_t step = buffer->element_size + buffer->stride;

  memcpy(&vertex, &verticies[i * step], buffer->element_size);

  return vertex;
}

static uint32_t read_index(const bnd_mesh_buffer *buffer, count_t i) {
  uint8_t *indices = buffer->buffer;
  uint8_t *src = &indices[i * (buffer->element_size + buffer->stride)];

  switch (buffer->element_size) {
    case 1:
      return src[0];

    case 2: {
      uint16_t index;
      memcpy(&index, src, sizeof(index));
      return index;
    }

    case 4: {
      uint32_t index;
      memcpy(&index, src, sizeof(index));
      return index;
    }

    default:
      return 0;
  }
}

static bnd_error import_verticies(bnd_allocator allocator, const bnd_mesh_buffer *buffer, mesh_storage *meshes, bnd_v3 com) {
  count_t new_count = buffer->elements_count + meshes->vertex_count;
  bnd_error err = resize_buffer(allocator, (void **)&meshes->verticies, 4, new_count, &meshes->vertex_capacity, sizeof(bnd_v3));
  if (err.type != BND_OK) {
    return err;
  }

  bnd_v3 *dest = &meshes->verticies[meshes->vertex_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    bnd_v3 v = read_vertex(buffer, i);
    dest[i] = bnd_v3_sub(v, com);
  }

  meshes->vertex_count = new_count;

  return OK;
}

static bnd_error import_indicies(bnd_allocator allocator, const bnd_mesh_buffer *buffer, mesh_storage *meshes) {
  count_t new_count = meshes->index_count + buffer->elements_count;
  bnd_error err = resize_buffer(allocator, (void **)&meshes->indicies, 4, new_count, &meshes->index_capacity, sizeof(uint32_t));
  if (err.type != BND_OK) {
    return err;
  }

  uint32_t *dest = &meshes->indicies[meshes->index_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    uint32_t index = read_index(buffer, i);
    dest[i] = index + meshes->index_count;
  }

  meshes->index_count = new_count;
  return OK;
}

static float tetr_inertia_moment(bnd_m3 m, count_t i) {
  return m.m0[i] * m.m0[i] + m.m1[i] * m.m2[i] + m.m1[i] * m.m1[i] + m.m0[i] * m.m2[i] + m.m2[i] * m.m2[i] + m.m0[i] * m.m1[i];
}

static float tetr_inertia_product(bnd_m3 m, count_t i, count_t j) {
  return 2.0f * m.m0[i] * m.m0[j] + m.m1[i] * m.m2[j] + m.m2[i] * m.m1[j] +
    2.0f * m.m1[i] * m.m1[j] + m.m0[i] * m.m2[j] + m.m2[i] * m.m0[j] +
    2.0f * m.m2[i] * m.m2[j] + m.m0[i] * m.m1[j] + m.m1[i] * m.m0[j];
}

static bool is_mesh_convex(const bnd_mesh_data *data) {
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&data->index_buffer, i + 0);
    count_t i1 = read_index(&data->index_buffer, i + 1);
    count_t i2 = read_index(&data->index_buffer, i + 2);

    bnd_v3 v0 = read_vertex(&data->vertex_buffer, i0);
    bnd_v3 v1 = read_vertex(&data->vertex_buffer, i1);
    bnd_v3 v2 = read_vertex(&data->vertex_buffer, i2);

    bnd_v3 n = bnd_v3_cross(bnd_v3_sub(v2, v0), bnd_v3_sub(v1, v0));
    float d = -bnd_v3_dot(n, v2);

    bool has_sign = false;
    float s = 0;
    for (count_t j = 0; j < data->vertex_buffer.elements_count; ++j) {
      if (j == i0 || j == i1 || j == i2) {
        continue;
      }

      bnd_v3 v = read_vertex(&data->vertex_buffer, j);
      float sv = bnd_v3_dot(n, v) + d;
      if (fabsf(sv) < EPSILON) {
        continue;
      }

      if (!has_sign) {
        s = sv;
        has_sign = true;
      } else if ((sv < 0 && s > 0) || (sv > 0 && s < 0)) {
        return false;
      }
    }
  }

  return true;
}

static bnd_error validate_mesh(const bnd_mesh_data *data) {
  bnd_mesh_buffer index_buffer = data->index_buffer;
  if (index_buffer.buffer == NULL) {
    return mesh_validation_error("Mesh index buffer is NULL");
  }

  if (index_buffer.element_size != 1 && index_buffer.element_size != 2 && index_buffer.element_size != 4) {
    return mesh_validation_error("Unsupported index buffer element size. Supported sizes are 1, 2 or 4 bytes");
  }

  if (index_buffer.elements_count < 12) {
    return mesh_validation_error("Mesh index buffer must contain at least 12 elements");
  }

  if (index_buffer.elements_count % 3 != 0) {
    return mesh_validation_error("Mesh contains indicies count non-divisible by 3");
  }

  bnd_mesh_buffer vertex_buffer = data->vertex_buffer;
  if (vertex_buffer.buffer == NULL) {
    return mesh_validation_error("Mesh vertex buffer is NULL");
  }

  if (vertex_buffer.elements_count < 4) {
    return mesh_validation_error("Mesh vertex buffer must contain at least 4 elements");
  }

  if (vertex_buffer.element_size != 3 * sizeof(float)) {
    return mesh_validation_error("Vertex buffer is required to have elements composed of 3 floats");
  }

  count_t vertex_count = vertex_buffer.elements_count;
  for (count_t i = 0; i + 2 < index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&index_buffer, i + 0);
    count_t i1 = read_index(&index_buffer, i + 1);
    count_t i2 = read_index(&index_buffer, i + 2);

    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count) {
      return mesh_validation_error("One of mesh's indices is out of bounds");
    }

    // TODO properly check for degenerate faces and perhaps fix them.
  }

  return OK;
}

static void calculate_mass_properties(const bnd_mesh_data *data, bnd_m3 *inertia, bnd_v3 *com, float *volume) {
  /**
   * This function is a rewrite of SkComputeInertia3x3 from
   *
   * https://github.com/blackedout01/simkn
   */

  float ia = 0.0f, ib = 0.0f, ic = 0.0f, iap = 0.0f, ibp = 0.0f, icp = 0.0f;

  *volume = 0;
  *com = bnd_v3_zero();
  *inertia = (bnd_m3){ 0 };
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    bnd_v3 v0 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 0));
    bnd_v3 v1 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 1));
    bnd_v3 v2 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 2));

    bnd_m3 m = { { v0.x, v0.y, v0.z }, { v1.x, v1.y, v1.z }, { v2.x, v2.y, v2.z } };

    float det = bnd_v3_dot(v0, bnd_v3_cross(v1, v2));
    float tetr_volume = det / 6.0f;

    bnd_v3 tetr_com = v0;
    tetr_com = bnd_v3_add(tetr_com, v1);
    tetr_com = bnd_v3_add(tetr_com, v2);
    tetr_com = bnd_v3_scale(tetr_com, 0.25f);

    float v100 = tetr_inertia_moment(m, 0);
    float v010 = tetr_inertia_moment(m, 1);
    float v001 = tetr_inertia_moment(m, 2);

    ia += det * (v010 + v001);
    ib += det * (v100 + v001);
    ic += det * (v100 + v010);
    iap += det * tetr_inertia_product(m, 1, 2);
    ibp += det * tetr_inertia_product(m, 0, 1);
    icp += det * tetr_inertia_product(m, 0, 2);

    tetr_com = bnd_v3_scale(tetr_com, tetr_volume);
    *com = bnd_v3_add(*com, tetr_com);
    *volume += tetr_volume;
  }

  *com = bnd_v3_scale(*com, 1.0f / *volume);
  ia = ia / 60.0f - *volume * (com->y * com->y + com->z * com->z);
  ib = ib / 60.0f - *volume * (com->x * com->x + com->z * com->z);
  ic = ic / 60.0f - *volume * (com->x * com->x + com->y * com->y);
  iap = iap / 120.0f - *volume * (com->y * com->z);
  ibp = ibp / 120.0f - *volume * (com->x * com->y);
  icp = icp / 120.0f - *volume * (com->x * com->z);

  inertia->m0[0] = ia;
  inertia->m1[1] = ib;
  inertia->m2[2] = ic;
  inertia->m0[1] = inertia->m1[0] = -ibp;
  inertia->m0[2] = inertia->m2[0] = -icp;
  inertia->m1[2] = inertia->m2[1] = -iap;
}

static bnd_aabb mesh_calculate_aabb(const mesh_storage *meshes, submesh submesh) {
  count_t vertex_start = submesh.vertex_offset;
  count_t vertex_end = vertex_start + submesh.vertex_count;

  bnd_v3 min = (bnd_v3){FLT_MAX, FLT_MAX, FLT_MAX};
  bnd_v3 max = bnd_v3_negate(min);
  for (count_t i = vertex_start; i < vertex_end; ++i) {
    bnd_v3 v = meshes->verticies[i];

    min = bnd_v3_min(min, v);
    max = bnd_v3_max(max, v);
  }

  return (bnd_aabb) {
    .center = bnd_v3_scale(bnd_v3_add(min, max), 0.5),
    .half_extents = bnd_v3_scale(bnd_v3_sub(max, min), 0.5),
  };
}

bnd_error meshes_init(bnd_world *world) {
  mesh_storage *meshes = &world->meshes;
  count_t num_meshes = world->config.memory.meshes_capacity;
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(meshes->submeshes, num_meshes * sizeof(submesh));
  meshes->submesh_capacity = num_meshes;
  meshes->submesh_count = 0;

  ALLOC_BUFFER4(meshes->meshes, num_meshes * sizeof(bnd_mesh));
  meshes->mesh_capacity = num_meshes;
  meshes->mesh_count = 0;

  ALLOC_BUFFER4(meshes->verticies, num_meshes * DEFAULT_VERTEX_PER_MESH * sizeof(bnd_v3));
  meshes->vertex_capacity = num_meshes * DEFAULT_VERTEX_PER_MESH;
  meshes->vertex_count = 0;

  ALLOC_BUFFER4(meshes->indicies, num_meshes * DEFAULT_FACE_PER_MESH * 3 * sizeof(uint32_t));
  meshes->index_capacity = num_meshes * DEFAULT_FACE_PER_MESH * 3;
  meshes->index_count = 0;

  ALLOC_BUFFER4(meshes->inertias, num_meshes * sizeof(bnd_m3));
  ALLOC_BUFFER4(meshes->volumes, num_meshes * sizeof(float));
  ALLOC_BUFFER4(meshes->aabbs, num_meshes * sizeof(bnd_aabb));

  return OK;
}

void meshes_teardown(bnd_world *world) {
  mesh_storage meshes = world->meshes;

  world->allocator.free(meshes.submeshes, meshes.submesh_capacity * sizeof(submesh));
  world->allocator.free(meshes.meshes, meshes.mesh_capacity * sizeof(bnd_mesh));
  world->allocator.free(meshes.verticies, meshes.vertex_capacity * sizeof(bnd_v3));
  world->allocator.free(meshes.indicies, meshes.index_capacity * sizeof(uint32_t));
  world->allocator.free(meshes.inertias, meshes.mesh_capacity * sizeof(bnd_m3));
  world->allocator.free(meshes.volumes, meshes.mesh_capacity * sizeof(float));
  world->allocator.free(meshes.aabbs, meshes.mesh_capacity * sizeof(bnd_aabb));
}

bnd_error bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass) {
  bnd_error e = validate_mesh(data);
  if (e.type != BND_OK) {
    return e;
  }

  if (!is_mesh_convex(data)) {
    e.type = BND_ERROR_MESH_IS_CONCAVE;
    e.message = "Concave meshes are not properly supported at the moment";

    return e;
  }

  bnd_m3 inertia;
  float volume;
  calculate_mass_properties(data, &inertia, center_of_mass, &volume);

  mesh_storage *meshes = &world->meshes;

  submesh sm;
  sm.vertex_offset = meshes->vertex_count;
  sm.index_offset = meshes->index_count;
  sm.vertex_count = data->vertex_buffer.elements_count;
  sm.index_count = data->index_buffer.elements_count;

  bnd_allocator allocator = world->allocator;
  e = import_verticies(allocator, &data->vertex_buffer, meshes, *center_of_mass);
  if (e.type != BND_OK) {
    return e;
  }

  e = import_indicies(allocator, &data->index_buffer, meshes);
  if (e.type != BND_OK) {
    return e;
  }

  e = resize_buffer(allocator, (void **)&meshes->submeshes, 4, meshes->submesh_count + 1, &meshes->submesh_capacity, sizeof(submesh));
  if (e.type != BND_OK) {
    return e;
  }

  e = ensure_meshes_capacity(allocator, meshes);
  if (e.type != BND_OK) {
    return e;
  }

  count_t submesh_offset = meshes->submesh_count++;
  meshes->submeshes[submesh_offset] = sm;

  *handle = meshes->mesh_count++;
  meshes->meshes[*handle] = (bnd_mesh){ submesh_offset, 1 };
  meshes->inertias[*handle] = inertia;
  meshes->volumes[*handle] = volume;
  meshes->aabbs[*handle] = mesh_calculate_aabb(meshes, sm);

  return OK;
}

// ================
//   debug.c
// ================


static void draw_contacts(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_contact == NULL) {
    return;
  }

  for (count_t i = 0; i < world->contacts.count; i++) {
    const contact *contact = &world->contacts.values[i];
    callbacks.draw_contact(contact->point, contact->normal, contact->depth, user_data);
  }
}

static void draw_shapes(const bnd_world *world, bnd_body_type type, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_shape == NULL) {
    return;
  }

  const common_data *data = as_common_const(world, type);
  for (count_t i = 0; i < data->count; i++) {
    body_shapes body_shapes = data->shapes[i];
    bool is_trigger = data->flags[i] & BODY_FLAG_TRIGGER;
    bnd_body_shape *shapes = shapes_get(world, body_shapes);

    for (count_t j = 0; j < body_shapes.count; ++j) {
      bnd_body_shape shape = shapes[j];

      shape_context ctx = {
        .world = world,
        .data = data,
        .shape = shape,
        .index = i
      };

      bnd_v3 center = body_center(&ctx);
      bnd_quat rotation = body_rotation(&ctx);
      callbacks.draw_shape(center, rotation, make_body_handle(world, type, i), shape.type, shape.value, is_trigger, user_data);
    }
  }
}

void draw_aabbs(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_aabb == NULL) {
    return;
  }

  for (bnd_body_type type = BND_BODY_DYNAMIC; type <= BND_BODY_STATIC; type++) {
    const common_data *data = as_common_const(world, type);
    for (count_t i = 0; i < data->count; i++) {
      bnd_aabb aabb = data->aabbs[i];
      callbacks.draw_aabb(aabb.center, aabb.half_extents, make_body_handle(world, type, i), user_data);
    }
  }
}

void draw_joints(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_joint == NULL) {
    return;
  }

  const joints *joints = &world->joints;
  for (count_t i = 0; i < joints->count; ++i) {
    const bnd_joint *j = &joints->values[i];

    bnd_v3 points[2];
    for (count_t k = 0; k < 2; ++k) {
      count_t body_index = handle_to_inner_index(world, j->bodies[k]);
      const common_data *data = as_common_const(world, j->bodies[k].type);

      points[k] = bnd_v3_rotate(j->relative_contact_positions[k], data->rotations[body_index]);
      points[k] = bnd_v3_add(points[k], data->positions[body_index]);
    }
   
    callbacks.draw_joint(j->bodies[0], j->bodies[1], points[0], points[1], user_data);
  }
}

void bnd_debug_draw(const bnd_world *world, bnd_debug_draw_flags flags, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (flags & BND_DEBUG_DRAW_CONTACTS) {
    draw_contacts(world, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_SHAPES_DYNAMIC) {
    draw_shapes(world, BND_BODY_DYNAMIC, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_SHAPES_STATIC) {
    draw_shapes(world, BND_BODY_STATIC, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_AABBS) {
    draw_aabbs(world, callbacks, user_data);
  }
  if (flags & BND_DEBUG_DRAW_JOINTS) {
    draw_joints(world, callbacks, user_data);
  }
}

#if defined(BND_DEBUG)

void epa_debug_next_frame(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, epa_debug_status *status) {
  world->epa_debug = status;
  world->epa_debug->src_body_a = body_a;
  world->epa_debug->src_body_b = body_b;
  world->epa_debug->initialized = true;
}

void epa_debug_capture(bnd_world *world) {
  if (world->epa_debug == NULL) {
    return;
  }

  bnd_body_handle src[] = { world->epa_debug->src_body_a, world->epa_debug->src_body_b };
  bnd_body_handle *dst[] = { &world->epa_debug->dst_body_a, &world->epa_debug->dst_body_b };

  for (count_t i = 0; i < 2; ++i) {
    count_t index = handle_to_inner_index(world, src[i]);
    common_data *data = as_common(world, src[i].type);
    
    count_t ephemeral = ephemeral_body_index(data) + i + 2;
    data->positions[ephemeral] = data->positions[index];
    data->rotations[ephemeral] = data->rotations[index];
    data->shapes[ephemeral] = data->shapes[index];
    data->inner_lookup[ephemeral] = ephemeral;
    data->outer_lookup[ephemeral].index = ephemeral;
    data->generations[ephemeral] = 0;

    *dst[i] = make_body_handle(world, src[i].type, ephemeral);
  }

  bnd_error error = collision_detection_epa_context(world, *dst[0], *dst[1], &world->epa_debug->ctx);
  if (IS_ERROR(error)) {
    world->epa_debug->iterations_count_result = (bnd_result_u32) { error, 0 };
    world->epa_debug = NULL;
    return;
  }

  if (!gjk_check_intersection(world, &world->epa_debug->ctx, &world->epa_debug->s)) {
    world->epa_debug->iterations_count_result = (bnd_result_u32) { { BND_ERROR_EPA_NO_INTERSECTION, "Bodies do not intersect" }, 0 };
    world->epa_debug = NULL;
    return;
  }

  contact c;
  count_t iterations_count = epa_get_contact(
    world,
    &world->epa_debug->ctx,
    &world->epa_debug->s,
    world->config.advanced.epa_tolerance,
    &c);

  world->epa_debug->target_iteration = 0;
  world->epa_debug->iterations_count_result = (bnd_result_u32) { OK, iterations_count };
  world->epa_debug = NULL;
}

#endif

collision_test_suite *collision_tests_load(void) {
#ifdef COLLISION_TEST_SUITE_PATH
  char *path = COLLISION_TEST_SUITE_PATH;
  FILE *f = fopen(path, "r");
  if (!f) {
    printf("Failed to open file %s\n", path);
    return NULL;
  }

  count_t num_pairs;
  count_t cases_per_pair;
  collision_test_suite *suite = NULL;
  if (fscanf(f, "num_pairs: %u\n", &num_pairs) != 1) {
    fprintf(stderr, "Failed to read pairs count");
    goto fail;
  }

  if (fscanf(f, "cases_per_pair: %u\n", &cases_per_pair) != 1) {
    fprintf(stderr, "Failed to read number of cases per pair");
    goto fail;
  }

  suite = malloc(sizeof(collision_test_suite));
  if (suite == NULL) {
    goto fail;
  }

  suite->num_pairs = num_pairs;
  suite->cases_per_pair = cases_per_pair;
  suite->pairs = malloc(num_pairs * sizeof(collision_test_pair));
  suite->cases = malloc(num_pairs * cases_per_pair * sizeof(collision_test_case));

  if (suite->pairs == NULL || suite->cases == NULL) {
    goto fail;
  }

  count_t num;
  char buffer[64];
  for (count_t i = 0; i < num_pairs; ++i) {
    collision_test_pair *pair = &suite->pairs[i];

    fscanf(f, "---\n");
    for (count_t j = 0; j < 2; ++j) {
      bnd_body_shape *shape = j == 0 ? &pair->a : &pair->b;

      fscanf(f, "shape%u:\n", &num);
      fscanf(f, " type: %s\n", buffer);

      if (!strncmp(buffer, "sphere", 6)) {
        shape->type = BND_SPHERE;
        fscanf(f, " radius: %f\n", &shape->value.sphere.radius);
      } else if (!strncmp(buffer, "box", 3)) {
        shape->type = BND_BOX;

        bnd_v3 half_size;
        fscanf(f, " half_extents: (%f, %f, %f)\n", &half_size.x, &half_size.y, &half_size.z);

        shape->value.box.size = bnd_v3_scale(half_size, 2.0);
      } else if (!strncmp(buffer, "capsule", 7)) {
        shape->type = BND_CAPSULE;
        fscanf(f, " height: %f\n radius: %f\n", &shape->value.capsule.height, &shape->value.capsule.radius);
      }
    }

    fscanf(f, "cases:\n");

    for (count_t j = 0; j < cases_per_pair; ++j) {
      collision_test_case *test_case = &suite->cases[i * cases_per_pair + j];

      fscanf(f, " - case%d:\n", &num);
      fscanf(f, " positionA: (%f, %f, %f)\n", &test_case->position_a.x, &test_case->position_a.y, &test_case->position_a.z);
      fscanf(f, " positionB: (%f, %f, %f)\n", &test_case->position_b.x, &test_case->position_b.y, &test_case->position_b.z);
      fscanf(f, " orientationA: (%f, %f, %f, %f)\n", &test_case->rotation_a.x, &test_case->rotation_a.y, &test_case->rotation_a.z, &test_case->rotation_a.w);
      fscanf(f, " orientationB: (%f, %f, %f, %f)\n", &test_case->rotation_b.x, &test_case->rotation_b.y, &test_case->rotation_b.z, &test_case->rotation_b.w);
      fscanf(f, " intersection: %s\n", buffer);

      test_case->intersection = !strncmp(buffer, "true", 4);
      if (test_case->intersection) {
        fscanf(f, " point: (%f, %f, %f)\n", &test_case->point.x, &test_case->point.y, &test_case->point.z);
        fscanf(f, " normal:  (%f, %f, %f)\n", &test_case->normal.x, &test_case->normal.y, &test_case->normal.z);
        fscanf(f, " depth: %f\n", &test_case->depth);
      }
    }

    fscanf(f, "\n");
  }

  fclose(f);
  return suite;

  fail:
  collision_tests_free(suite);
  fclose(f);
  return NULL;
#else
  return NULL;
#endif
}

void collision_tests_pair_spawn(bnd_world *world, const collision_test_pair *pair, bnd_body_handle *pair_handles) {
  bnd_body_shape shapes[] = { pair->a, pair->b };

  for (count_t j = 0; j < 2; j++) {
    const bnd_body_shape *shape = &shapes[j];

    switch(shape->type)  {
      case BND_SPHERE:
        pair_handles[j] = bnd_add_sphere_dynamic(world, 5, shape->value.sphere.radius).value;
        break;

      case BND_BOX:
        pair_handles[j] = bnd_add_box_dynamic(world, 5, shape->value.box.size).value;
        break;

      case BND_CAPSULE:
        pair_handles[j] = bnd_add_capsule_dynamic(world, 5, shape->value.capsule.radius, shape->value.capsule.height).value;
        break;

      default:
        break;
    }
  }
}

void collision_tests_free(collision_test_suite *tests) {
  if (tests == NULL) {
    return;
  }

  free(tests->pairs);
  free(tests->cases);
  free(tests);
}

// ================
//   events.c
// ================

#define TRY_REALLOC(buffer, size, old_capacity, new_capacity) \
  world->events.buffer = world->allocator.realloc(world->events.buffer, 4, size * old_capacity, size * new_capacity); \
  if (world->events.buffer == NULL) { \
    return BND_RESULT_ERR(u32, BND_ERROR_OUT_OF_MEMORY, "Allocator.realloc failed to re-allocate the events memory buffer"); \
  }

static bnd_result_u32 new_event_index(bnd_world *world) {
  count_t new_count = world->events.count + 1;
  if (new_count >= world->events.capacity) {
    if (world->allocator.realloc == NULL) {
      return BND_RESULT_ERR(u32, BND_ERROR_NO_SPACE_AVAILABLE, "Events memory buffer is full and Allocator.realloc is NULL");
    }

    count_t old_capacity = world->events.capacity;
    while (new_count >= world->events.capacity)  {
      world->events.capacity *= 2;
    }

    TRY_REALLOC(events, sizeof(bnd_event), old_capacity, world->events.capacity)
    TRY_REALLOC(links, sizeof(count_t), old_capacity, world->events.capacity)
  }

  return BND_RESULT_OK(u32, world->events.count);
}

bnd_error bnd_event_subscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  common_data *data = as_common(world, body.type);
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] |= type;

  return OK;
}

bnd_error bnd_event_unsubscribe(bnd_world *world, bnd_body_handle body, bnd_event_type type) {
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  common_data *data = as_common(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] &= ~type;

  return OK;
}

bnd_error bnd_event_unsubscribe_all(bnd_world *world, bnd_body_handle body) {
  PROPAGATE_ERROR(bnd_handle_valid(world, body))

  common_data *data = as_common(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  data->event_masks[index] = 0;

  return OK;
}

bnd_result_bool bnd_event_any(bnd_world *world, bnd_body_handle body) {
  bnd_error e = bnd_handle_valid(world, body);
  if (e.type != BND_OK) {
    return BND_RESULT_ERR2(bool, e);
  }

  const common_data *data = as_common_const(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  return BND_RESULT_OK(bool, data->event_links[index].count != 0);
}

bnd_result_bool bnd_event_enumerate(bnd_world *world, bnd_body_handle body, bnd_event_enumerator *enumerator) {
  bnd_error e = bnd_handle_valid(world, body);
  if (e.type != BND_OK) {
    return BND_RESULT_ERR2(bool, e);
  }

  const common_data *data = as_common_const(world, body.type);
  count_t index = handle_to_inner_index(world, body);
  if (data->event_links[index].count == 0) {
    enumerator->index = 0xFFFFFFFF;
    return BND_RESULT_OK(bool, true);
  }

  enumerator->index = data->event_links[index].first;
  return BND_RESULT_OK(bool, true);
}

bool bnd_event_next(bnd_world *world, bnd_event_enumerator *enumerator) {
  if (enumerator->index == 0xFFFFFFFF) {
    return false;
  }

  enumerator->e = world->events.events[enumerator->index];
  enumerator->index = world->events.links[enumerator->index];

  return true;
}

bnd_error events_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(world->events.events, world->config.memory.events_capacity * sizeof(bnd_event));
  ALLOC_BUFFER4(world->events.links, world->config.memory.events_capacity * sizeof(count_t));

  world->events.capacity = world->config.memory.events_capacity;
  world->events.count = 0;

  return OK;
}

void events_teardown(bnd_world *world) {
  world->allocator.free(world->events.events, world->events.capacity * sizeof(bnd_event));
  world->allocator.free(world->events.links, world->events.capacity * sizeof(count_t));
}

void events_reset(bnd_world *world) {
  world->events.count = 0;
  memset(world->dynamics.event_links, 0, world->dynamics.count * sizeof(event_link));
  memset(world->statics.event_links, 0, world->statics.count * sizeof(event_link));
}

bool events_subscribed(const common_data *data, count_t index, bnd_event_type event_type) {
  return data->event_masks[index] & event_type;
}

bnd_error events_push(bnd_world *world, common_data *data, count_t index, bnd_event event) {
  event_link *link = &data->event_links[index];

  bnd_result_u32 event_index = new_event_index(world);
  if (event_index.error.type != BND_OK) {
    return event_index.error;
  }

  world->events.events[event_index.value] = event;
  if (link->count > 0) {
    world->events.links[link->last] = event_index.value;
  }
  world->events.links[event_index.value] = 0xFFFFFFFF;
  world->events.count += 1;

  if (link->count == 0) {
    link->first = event_index.value;
  }
  link->last = event_index.value;
  link->count += 1;

  return OK;
}

// ================
//   collision_detection.c
// ================



#define PROFILING_BLOCK_NAME "Contacts cache"

typedef count_t (*collision_detection_func)(bnd_world *world, const collision_detection_context *ctx);

typedef struct {
  collision_detection_func func;
  bool primary;
  bool use_cache;
} collision_detection_entry;

static collision_detection_entry collision_detection_table[BND_SHAPES_COUNT][BND_SHAPES_COUNT];

static void box_corners(bnd_v3 half_extents, bnd_v3 corners[8])  {
  corners[0] = (bnd_v3){ half_extents.x, half_extents.y, half_extents.z };
  corners[1] = (bnd_v3){ half_extents.x, half_extents.y, -half_extents.z };
  corners[2] = (bnd_v3){ half_extents.x, -half_extents.y, half_extents.z };
  corners[3] = (bnd_v3){ half_extents.x, -half_extents.y, -half_extents.z };
  corners[4] = (bnd_v3){ -half_extents.x, half_extents.y, half_extents.z };
  corners[5] = (bnd_v3){ -half_extents.x, half_extents.y, -half_extents.z };
  corners[6] = (bnd_v3){ -half_extents.x, -half_extents.y, half_extents.z };
  corners[7] = (bnd_v3){ -half_extents.x, -half_extents.y, -half_extents.z };
}

static collision_detection_context ctx_inverse(collision_detection_context ctx) {
  return (collision_detection_context){
    .world = ctx.world,
    .data_a = ctx.data_b,
    .data_b = ctx.data_a,
    .contacts_offset = ctx.contacts_offset,
    .body_a = ctx.body_b,
    .body_b = ctx.body_a,
    .shape_a = ctx.shape_b,
    .shape_b = ctx.shape_a,
  };
}

static contact *new_contact(const collision_detection_context *ctx, count_t offset) {
  contact *c = &ctx->world->contacts.values[ctx->contacts_offset + offset];
  c->index_a = ctx->body_a;
  c->index_b = ctx->body_b;
  c->friction = mix_friction(ctx);
  c->restitution = mix_restitution(ctx);
  c->from_cache = false;

  return c;
}

static bnd_v3 body_center_ex(bnd_v3 shape_offset, bnd_quat global_rotation, bnd_v3 body_position) {
  bnd_v3 center = shape_offset;
  center = bnd_v3_rotate(center, global_rotation);
  center = bnd_v3_add(center, body_position);

  return center;
}

static bnd_v3 body_a_center(const collision_detection_context *ctx) {
  return body_center_ex(ctx->shape_a.offset, ctx->data_a->rotations[ctx->body_a], ctx->data_a->positions[ctx->body_a]);
}

static bnd_v3 body_b_center(const collision_detection_context *ctx) {
  return body_center_ex(ctx->shape_b.offset, ctx->data_b->rotations[ctx->body_b], ctx->data_b->positions[ctx->body_b]);
}

bnd_v3 body_center(const shape_context *ctx) {
  return body_center_ex(ctx->shape.offset, ctx->data->rotations[ctx->index], ctx->data->positions[ctx->index]);
}

bnd_quat body_a_rotation(const collision_detection_context *ctx) {
  return bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
}

bnd_quat body_b_rotation(const collision_detection_context *ctx) {
  return bnd_quat_mul(ctx->data_b->rotations[ctx->body_b], ctx->shape_b.rotation);
}

bnd_quat body_rotation(const shape_context *ctx) {
  return bnd_quat_mul(ctx->data->rotations[ctx->index], ctx->shape.rotation);
}

bool aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b) {
  const bnd_aabb *a = &data_a->aabbs[index_a];
  const bnd_aabb *b = &data_b->aabbs[index_b];

  if (fabsf(a->center.x - b->center.x) > a->half_extents.x + b->half_extents.x) {
    return false;
  }

  if (fabsf(a->center.y - b->center.y) > a->half_extents.y + b->half_extents.y) {
    return false;
  }

  if (fabsf(a->center.z - b->center.z) > a->half_extents.z + b->half_extents.z) {
    return false;
  }

  return true;
}

static support_point sphere_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = bnd_v3_add(ctx->data->positions[ctx->index], ctx->shape.offset);
  float radius = ctx->shape.value.sphere.radius;

  return (support_point) { bnd_v3_add(center, bnd_v3_scale(direction, radius)), 0 };
}

static support_point box_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);
  bnd_quat inv_rotation = bnd_quat_invert(rotation);

  bnd_v3 local_direction = bnd_v3_normalize(bnd_v3_rotate(direction, inv_rotation));
  bnd_v3 v = (bnd_v3) {
    (local_direction.x > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.x * 0.5f,
    (local_direction.y > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.y * 0.5f,
    (local_direction.z > 0 ? 1.0f : -1.0f) * ctx->shape.value.box.size.z * 0.5f
  };

  v = bnd_v3_rotate(v, rotation);
  v = bnd_v3_add(center, v);

  uint16_t index = 0;
  if (local_direction.x > 0 && local_direction.z > 0) {
    index = 1;
  } else if (local_direction.x <= 0 && local_direction.z > 0) {
    index = 2;
  } else if (local_direction.x <= 0 && local_direction.z <= 0) {
    index = 3;
  }
  index += local_direction.y > 0 ? 4 : 0;

  return (support_point) { v, index };
}

static support_point capsule_support(const shape_context *ctx, bnd_v3 direction) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);
  bnd_quat inv_rotation = bnd_quat_invert(rotation);

  bnd_v3 local_direction = bnd_v3_normalize(bnd_v3_rotate(direction, inv_rotation));

  float radius = ctx->shape.value.capsule.radius;
  float height = ctx->shape.value.capsule.height;

  bnd_v3 cap = { 0, (local_direction.y >= 0 ? 1 : -1) * height * 0.5f, 0 };
  bnd_v3 p = bnd_v3_add(cap, bnd_v3_scale(local_direction, radius));

  p = bnd_v3_rotate(p, rotation);
  p = bnd_v3_add(p, center);

  return (support_point) { p, 0 };
}

static support_point mesh_support(const shape_context *ctx, bnd_v3 direction) {
  const mesh_storage *meshes = &ctx->world->meshes;
  const bnd_mesh_handle mesh_handle = ctx->shape.value.mesh;

  bnd_quat rotation = body_rotation(ctx);
  bnd_v3 position = body_center(ctx);
  bnd_v3 local_direction = bnd_v3_rotate(direction, bnd_quat_invert(rotation));

  bnd_mesh mesh = meshes->meshes[mesh_handle];
  count_t submesh_start = mesh.submesh_offset;
  count_t submesh_end = submesh_start + mesh.submesh_count;

  float max_dot = -FLT_MAX;
  count_t max_vertex = UINT32_MAX;
  for (count_t mesh_index = submesh_start; mesh_index < submesh_end; ++mesh_index) {
    submesh submesh = meshes->submeshes[mesh_index];
    count_t vertex_start = submesh.vertex_offset;
    count_t vertex_end = vertex_start + submesh.vertex_count;

    for (count_t vertex_index = vertex_start; vertex_index < vertex_end; ++vertex_index) {
      bnd_v3 vertex = meshes->verticies[vertex_index];
      float d = bnd_v3_dot(vertex, local_direction);

      if (d > max_dot) {
        max_dot = d;
        max_vertex = vertex_index;
      }
    }
  }

  bnd_v3 support = meshes->verticies[max_vertex];
  support = bnd_v3_rotate(support, rotation);
  support = bnd_v3_add(support, position);

  return (support_point) { support, (uint16_t)(max_vertex & 0xFFFF) };
}

support_func support_functions[] = { box_support, sphere_support, capsule_support, mesh_support };

body_support support(const collision_detection_context *ctx, bnd_v3 direction) {
  shape_context sa = { ctx->world, ctx->data_a, ctx->shape_a, ctx->body_a };
  shape_context sb = { ctx->world, ctx->data_b, ctx->shape_b, ctx->body_b };

  body_support result;
  result.p1 = support_functions[ctx->shape_a.type](&sa, direction);
  result.p2 = support_functions[ctx->shape_b.type](&sb, bnd_v3_negate(direction));
  result.p = bnd_v3_sub(result.p1.point, result.p2.point);

  return result;
}

static count_t sphere_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 center_a = body_a_center(ctx);
  bnd_v3 center_b = body_b_center(ctx);

  float radius_a = ctx->shape_a.value.sphere.radius;
  float radius_b = ctx->shape_b.value.sphere.radius;

  bnd_v3 offset = bnd_v3_sub(center_a, center_b);
  float distance = bnd_v3_len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return 0;
  }

  bnd_v3 normal = distance > EPSILON ? bnd_v3_scale(offset, 1 / distance) : bnd_v3_up();

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
  c->point = bnd_v3_add(center_b, bnd_v3_scale(normal, radius_b + penetration));
  c->normal = normal;
  c->depth = -penetration;

  return 1;
}

static count_t capsule_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  bnd_v3 capsule_center = body_a_center(ctx);
  bnd_quat capsule_rotation = body_a_rotation(ctx);
  bnd_quat capsule_inv_rotation = bnd_quat_invert(capsule_rotation);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_half_height = ctx->shape_a.value.capsule.height * 0.5f;

  bnd_v3 sphere_center = body_b_center(ctx);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, capsule_center), capsule_inv_rotation);
  float sphere_radius = ctx->shape_b.value.sphere.radius;

  if (fabsf(local_sphere_center.y) < capsule_half_height) {
    bnd_v3 horizontal_offset = { local_sphere_center.x, 0.0f, local_sphere_center.z };
    float horizontal_distance = bnd_v3_len(horizontal_offset);

    if (horizontal_distance < capsule_radius) {
      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = horizontal_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_right(), capsule_rotation);
      c->depth = capsule_radius - horizontal_distance + sphere_radius;

      return 1;
    } else if (horizontal_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(horizontal_offset, capsule_radius / horizontal_distance);
      closest.y = local_sphere_center.y;

      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      c->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation));
      c->depth = sphere_radius - horizontal_distance + capsule_radius;

      return 1;
    } else {
      return 0;
    }
  } else {
    bnd_v3 local_caps[] = {
      (bnd_v3) { 0, capsule_half_height, 0 },
      (bnd_v3) { 0, -capsule_half_height, 0 },
    };

    bnd_v3 cap = local_sphere_center.y > capsule_half_height ? local_caps[0] : local_caps[1];
    bnd_v3 cap_offset = (bnd_v3) { local_sphere_center.x, local_sphere_center.y - cap.y, local_sphere_center.z };

    float cap_distance = bnd_v3_len(cap_offset);
    if (cap_distance < capsule_radius) {
      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      c->normal = cap_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
      c->depth = capsule_radius - cap_distance + sphere_radius;

      return 1;
    } else if (cap_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(cap_offset, capsule_radius / cap_distance);
      closest = bnd_v3_add(cap, closest);

      contact *c = new_contact(ctx, 0);
      c->point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      c->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation));
      c->depth = sphere_radius - cap_distance + capsule_radius;

      return 1;
    } else {
      return 0;
    }
  }
}

static count_t box_sphere_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 half_extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5);
  bnd_v3 box_center = body_a_center(ctx);
  bnd_quat box_rotation = body_a_rotation(ctx);
  bnd_quat inv_box_rotation = bnd_quat_invert(box_rotation);

  bnd_v3 sphere_center = body_b_center(ctx);
  bnd_v3 local_sphere_center = bnd_v3_rotate(bnd_v3_sub(sphere_center, box_center), inv_box_rotation);
  float r = ctx->shape_b.value.sphere.radius;

  bnd_v3 closest = {
    fmaxf(-half_extents.x, fminf(local_sphere_center.x, half_extents.x)),
    fmaxf(-half_extents.y, fminf(local_sphere_center.y, half_extents.y)),
    fmaxf(-half_extents.z, fminf(local_sphere_center.z, half_extents.z))
  };

  float distancesqr = bnd_v3_distancesqr(closest, local_sphere_center);
  if (distancesqr > r * r) {
    return 0;
  }

  float *s = (float *)&half_extents;
  float *c = (float *)&closest;

  float depth = 0;
  float local_normal[3] = {0};

  if (fabsf(distancesqr) < EPSILON) {
    // Sphere is inside the box
    float min_dist = FLT_MAX;
    int min_axis = -1;
    for (int i = 0; i < 3; ++i) {
      float dist = s[i] - fabsf(c[i]);
      if (dist < min_dist) {
        min_dist = dist;
        min_axis = i;
      }
    }

    local_normal[min_axis] = c[min_axis] > 0 ? -1.0f : 1.0f;
    depth = min_dist + r;
  } else {
    bnd_v3 diff = bnd_v3_sub(closest, local_sphere_center);
    float dist = sqrtf(distancesqr);
    depth = r - dist;

    diff = bnd_v3_scale(diff, 1.0f / dist);
    memcpy(&local_normal, &diff, sizeof(bnd_v3));
  }

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *contact = new_contact(ctx, 0);
  memcpy(&contact->normal, local_normal, sizeof(bnd_v3));

  contact->point = bnd_v3_add(box_center, bnd_v3_rotate(closest, box_rotation));
  contact->normal = bnd_v3_rotate(contact->normal, box_rotation);
  contact->depth = depth;

  return 1;
}

static count_t box_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  bnd_quat shape_rotation = ctx->shape_a.rotation;

  bnd_v3 box_center = body_a_center(ctx);
  bnd_v3 extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5f);

  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];

  bnd_v3 corners[8];
  box_corners(extents, corners);

  const count_t max_contacts = 4;

  bnd_error e = contacts_ensure_capacity(world, ctx->contacts_offset, max_contacts);
  if (IS_ERROR(e)) {
    return 0;
  }

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    bnd_v3 corner = bnd_v3_add(box_center, bnd_v3_rotate(bnd_v3_rotate(corners[i], shape_rotation), box_rotation));
    float distance = bnd_v3_dot(bnd_v3_sub(corner, plane_point), plane_normal);
    if (distance > 0) {
      continue;
    }

    contact *c = new_contact(ctx, contact_count);
    c->normal = plane_normal;
    c->point = bnd_v3_add(corner, bnd_v3_scale(plane_normal, -0.5f * distance));
    c->depth = -distance;

    contact_count += 1;
  }

  return contact_count;
}

static count_t sphere_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 sphere_center = body_a_center(ctx);
  float sphere_radius = ctx->shape_a.value.sphere.radius;

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  float plane_sphere_distance = bnd_v3_dot(bnd_v3_sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius) {
    return 0;
  }

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *contact = new_contact(ctx, 0);
  contact->normal = plane_normal;
  contact->point = bnd_v3_add(sphere_center, bnd_v3_scale(plane_normal, -plane_sphere_distance));
  contact->depth = sphere_radius - plane_sphere_distance;

  return 1;
}

static count_t capsule_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 capsule_center = body_a_center(ctx);
  float capsule_radius = ctx->shape_a.value.capsule.radius;
  float capsule_height = ctx->shape_a.value.capsule.height;

  bnd_quat capsule_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_v3 capsule_axis = bnd_v3_rotate(bnd_v3_up(), capsule_rotation);
  bnd_v3 cap_top = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, capsule_height * 0.5f));
  bnd_v3 cap_bottom = bnd_v3_add(capsule_center, bnd_v3_scale(capsule_axis, -capsule_height * 0.5f));

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 points[] = { cap_top, cap_bottom };

  count_t contact_count = 0;
  for (int i = 0; i < 2; ++i) {
    bnd_v3 offset = bnd_v3_sub(points[i], plane_point);
    float d = bnd_v3_dot(offset, plane_normal);
    if (d > capsule_radius) {
      continue;
    }

    if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset + contact_count, 1))) {
      return contact_count;
    }

    contact *c = new_contact(ctx, contact_count);
    c->point = bnd_v3_add(points[i], bnd_v3_scale(plane_normal, -d));
    c->normal = plane_normal;
    c->depth = capsule_radius - d;

    contact_count += 1;
  }

  return contact_count;
}

static count_t mesh_plane_collision(bnd_world *world, const collision_detection_context *ctx) {
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 mesh_center = body_a_center(ctx);
  bnd_quat mesh_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_quat inv_mesh_rotation = bnd_quat_invert(mesh_rotation);

  bnd_v3 local_normal = bnd_v3_rotate(plane_normal, inv_mesh_rotation);
  bnd_v3 local_point = bnd_v3_rotate(bnd_v3_sub(plane_point, mesh_center), inv_mesh_rotation);

  const mesh_storage *meshes = &world->meshes;
  const bnd_mesh_handle mesh_handle = ctx->shape_a.value.mesh;

  bnd_mesh mesh = meshes->meshes[mesh_handle];
  count_t submesh_start = mesh.submesh_offset;
  count_t submesh_end = submesh_start + mesh.submesh_count;

  float min_dot = FLT_MAX;
  count_t collision_vertex = 0;
  for (count_t mesh_index = submesh_start; mesh_index < submesh_end; ++mesh_index) {
    count_t vertex_start = meshes->submeshes[mesh_index].vertex_offset;
    count_t vertex_end = vertex_start + meshes->submeshes[mesh_index].vertex_count;

    for (count_t vertex_index = vertex_start; vertex_index < vertex_end; ++vertex_index) {
      bnd_v3 vertex = meshes->verticies[vertex_index];
      bnd_v3 offset = bnd_v3_sub(vertex, local_point);
      float d = bnd_v3_dot(offset, local_normal);

      if (d < min_dot) {
        min_dot = d;
        collision_vertex = vertex_index;
      }
    }
  }

  if (min_dot > 0) {
    return 0;
  }

  bnd_v3 point = meshes->verticies[collision_vertex];
  point = bnd_v3_rotate(point, mesh_rotation);
  point = bnd_v3_add(point, mesh_center);
  point = bnd_v3_add(point, bnd_v3_scale(plane_normal, -min_dot)); // Project the deepest vertex back on the plane.

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
  c->point = point;
  c->normal = plane_normal;
  c->depth = -min_dot;

  return 1;
}

static count_t polytope_polytope_collision(bnd_world *world, const collision_detection_context *ctx) {
  simplex s;
  if (!gjk_check_intersection(world, ctx, &s)) {
    return 0;
  }

  if (IS_ERROR(contacts_ensure_capacity(world, ctx->contacts_offset, 1))) {
    return 0;
  }

  contact *c = new_contact(ctx, 0);
  epa_get_contact(world, ctx, &s, world->config.advanced.epa_tolerance, c);

  return 1;
}

bnd_error collision_detection_epa_context(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, collision_detection_context *ctx) {
  char *message = "EPA debugging requires two distinct single-shape bodies that use EPA collision detection";

  if (body_a.type > BND_BODY_STATIC || body_b.type > BND_BODY_STATIC) {
    return (bnd_error) { BND_ERROR_BODY_HANDLE_INVALID, "Handle has an invalid body type" };
  }

  PROPAGATE_ERROR(bnd_handle_valid(world, body_a))
  PROPAGATE_ERROR(bnd_handle_valid(world, body_b))

  if (body_a.type == body_b.type && body_a.index == body_b.index) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  const common_data *data_a = as_common_const(world, body_a.type);
  const common_data *data_b = as_common_const(world, body_b.type);
  count_t index_a = handle_to_inner_index(world, body_a);
  count_t index_b = handle_to_inner_index(world, body_b);

  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_DYNAMIC) {
    const common_data *tmp_data = data_a;
    data_a = data_b;
    data_b = tmp_data;

    count_t tmp_index = index_a;
    index_a = index_b;
    index_b = tmp_index;
  }

  if (data_a == data_b && index_a == index_b) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  body_shapes shapes_a = data_a->shapes[index_a];
  body_shapes shapes_b = data_b->shapes[index_b];
  if (shapes_a.count != 1 || shapes_b.count != 1) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  *ctx = (collision_detection_context) {
    .world = world,
    .data_a = data_a,
    .data_b = data_b,
    .body_a = index_a,
    .body_b = index_b,
    .shape_a = shapes_get(world, shapes_a)[0],
    .shape_b = shapes_get(world, shapes_b)[0],
  };

  collision_detection_entry entry = collision_detection_table[ctx->shape_a.type][ctx->shape_b.type];
  if (entry.func != polytope_polytope_collision) {
    return (bnd_error) { BND_ERROR_EPA_NOT_APPLICABLE, message };
  }

  if (!entry.primary) {
    *ctx = ctx_inverse(*ctx);
  }

  return OK;
}

count_t collisions_detect(bnd_world *world, count_t contacts_offset, bnd_body_type type) {
  const common_data *dynamics = as_common_const(world, BND_BODY_DYNAMIC);
  const common_data *data_b = as_common_const(world, type);

  collision_detection_context ctx = {
    .world = world,
    .data_a = dynamics,
    .data_b = data_b,
  };

  count_t count = 0;

  for (count_t i = 0; i < dynamics->count; ++i) {
    count_t until = type == BND_BODY_DYNAMIC ? i : data_b->count;
    for (count_t j = 0; j < until; ++j) {
      bnd_collision_mask validation_mask = layer_to_mask(dynamics->collision_layers[i]);
      bnd_collision_mask reference_mask = world->matrix.matrix[data_b->collision_layers[j]];
      if ((reference_mask & validation_mask) == 0) {
        continue;
      }

      if (!aabb_intersect(dynamics, data_b, i, j)) {
        continue;
      }

      ctx.body_a = i;
      ctx.body_b = j;

      body_shapes shapes_a = dynamics->shapes[i];
      body_shapes shapes_b = data_b->shapes[j];

      uint64_t cached_contacts_mask = 0;
      count_t pair_contacts_count = 0;
      count_t pair_offset = contacts_offset + count;

      for (count_t sa = 0; sa < shapes_a.count; ++sa) {
        bnd_body_shape shape_a = shapes_get(world, shapes_a)[sa];
        ctx.shape_a = shape_a;

        for (count_t sb = 0; sb < shapes_b.count; ++sb) {
          bnd_body_shape shape_b = shapes_get(world, shapes_b)[sb];
          count_t shape_offset = pair_offset + pair_contacts_count;

          ctx.shape_b = shape_b;
          ctx.contacts_offset = shape_offset;

          collision_detection_entry entry = collision_detection_table[shape_a.type][shape_b.type];
          if (entry.func == NULL) {
            continue;
          }

          collision_detection_context context = entry.primary ? ctx : ctx_inverse(ctx);
          count_t shape_contacts_count = entry.func(world, &context);

          if (!entry.primary) {
            for (count_t k = 0; k < shape_contacts_count; ++k) {
              contact *c = &world->contacts.values[shape_offset + k];
              c->index_a = ctx.body_a;
              c->index_b = ctx.body_b;
              c->normal = bnd_v3_negate(c->normal);

              if (entry.use_cache) {
                bnd_v3 tmp_witness = c->features.witness_a;
                c->features.witness_a = c->features.witness_b;
                c->features.witness_b = tmp_witness;
                c->features.normal = bnd_v3_negate(c->features.normal);
              }
            }
          }

          if (entry.use_cache) {
            uint64_t mask = (1 << shape_contacts_count) - 1;
            cached_contacts_mask |= mask << pair_contacts_count;
          }

          pair_contacts_count += shape_contacts_count;
        }
      }

      bool is_trigger = data_b->flags[j] & BODY_FLAG_TRIGGER;
      if (is_trigger && pair_contacts_count > 0) {
        // It's not ideal to process triggers here, since we've already done a lot of useless work.
        // This is going to change with the introduction of contact islands.

        if (events_subscribed((const common_data *)dynamics, i, BND_EVENT_TRIGGER)) {
          events_push(world, (common_data *)dynamics, i, (bnd_event) {
            .type = BND_EVENT_TRIGGER,
            .trigger = { .other = make_body_handle(world, type, j) }
          });
        }

        if (events_subscribed(data_b, j, BND_EVENT_TRIGGER)) {
          events_push(world, (common_data *)data_b, j, (bnd_event) {
            .type = BND_EVENT_TRIGGER,
            .trigger = { .other = make_body_handle(world, BND_BODY_DYNAMIC, i) }
          });
        }

        break;
      }

      count_t filtered_contact_indices[MAX_CONTACTS_PER_PAIR] = {0};
      if (cached_contacts_mask == 0) {
        if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
          contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);
          pair_contacts_count = MAX_CONTACTS_PER_PAIR;
        }

        count += pair_contacts_count;
        continue;
      }

      PROFILER_BLOCK_START(PROFILING_BLOCK_NAME);

      cache_entry *cached_entry = contacts_cache_query(world, world->contacts.values + pair_offset, type);
      if (cached_entry == NULL) {
        PROFILER_BLOCK_END;
        continue;
      }

      float distance_threshold = world->config.advanced.contacts_cache.feature_distance_threshold;
      float distance_threshold_sqr = distance_threshold * distance_threshold;
      float separation_threshold = world->config.advanced.contacts_cache.separation_threshold;

      bnd_v3 position_a = ctx.data_a->positions[ctx.body_a];
      bnd_v3 position_b = ctx.data_b->positions[ctx.body_b];
      bnd_quat rotation_a = ctx.data_a->rotations[ctx.body_a];
      bnd_quat rotation_b = ctx.data_b->rotations[ctx.body_b];

      uint8_t picked_features = 0;
      for (count_t k = 0; k < pair_contacts_count; ++k) {
        if ((cached_contacts_mask & (UINT64_C(1) << k)) == 0) {
          continue;
        }

        contact *c = &world->contacts.values[pair_offset + k];
        contact_features *features = &c->features;

        bnd_quat inv_rotation_a = bnd_quat_invert(rotation_a);
        bnd_quat inv_rotation_b = bnd_quat_invert(rotation_b);
        features->witness_a = bnd_v3_rotate(bnd_v3_sub(features->witness_a, position_a), inv_rotation_a);
        features->witness_b = bnd_v3_rotate(bnd_v3_sub(features->witness_b, position_b), inv_rotation_b);
        features->normal = bnd_v3_rotate(features->normal, inv_rotation_a);

        count_t matched_slot = cached_entry->feature_count;
        for (count_t h = 0; h < cached_entry->feature_count; ++h) {
          const contact_features *cached_features = &cached_entry->features[h];

          float distance_a_sqr = bnd_v3_distancesqr(cached_features->witness_a, features->witness_a);
          float distance_b_sqr = bnd_v3_distancesqr(cached_features->witness_b, features->witness_b);

          if (distance_a_sqr <= distance_threshold_sqr && distance_b_sqr <= distance_threshold_sqr) {
            matched_slot = h;
            break;
          }
        }

        count_t feature_count = cached_entry->feature_count;
        if (matched_slot < feature_count) {
          cached_entry->features[matched_slot] = *features;
          picked_features |= 1 << matched_slot;
        } else if (cached_entry->feature_count < MAX_CONTACTS_PER_PAIR) {
          cached_entry->features[feature_count] = *features;
          cached_entry->feature_count += 1;

          picked_features |= 1 << feature_count;
        }
      }

      count_t fresh_contacts_count = pair_contacts_count;
      count_t contacts_from_cache = 0;
      for (count_t h = 0; h < cached_entry->feature_count; ++h) {
        if (picked_features & (1 << h)) {
          continue;
        }

        const contact_features *cached_features = &cached_entry->features[h];

        bnd_v3 witness_a_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_a, rotation_a), position_a);
        bnd_v3 witness_b_world = bnd_v3_add(bnd_v3_rotate(cached_features->witness_b, rotation_b), position_b);
        bnd_v3 normal_world = bnd_v3_rotate(cached_features->normal, rotation_a);

        float separation = bnd_v3_dot(bnd_v3_sub(witness_a_world, witness_b_world), normal_world);
        if (separation > separation_threshold) {
          continue;
        }

        count_t contact_offset = pair_offset + fresh_contacts_count + contacts_from_cache;
        if (IS_ERROR(contacts_ensure_capacity(world, contact_offset, 1))) {
          break;
        }

        contact *c = &world->contacts.values[contact_offset];
        c->index_a = ctx.body_a;
        c->index_b = ctx.body_b;
        c->point = bnd_v3_scale(bnd_v3_add(witness_a_world, witness_b_world), 0.5f);
        c->normal = normal_world;
        c->depth = -separation;
        c->features = *cached_features;
        c->restitution = mix_restitution(&ctx);
        c->friction = mix_friction(&ctx);

        contacts_from_cache += 1;
      }

      pair_contacts_count += contacts_from_cache;

      if (pair_contacts_count > MAX_CONTACTS_PER_PAIR) {
        contacts_filter_largest_surface_area(world->contacts.values + pair_offset, pair_contacts_count, filtered_contact_indices);

        count_t feature_count = 0;
        for (count_t k = 0; k < MAX_CONTACTS_PER_PAIR; ++k) {
          contact *c = &world->contacts.values[pair_offset + k];
          count_t original_contact_index = filtered_contact_indices[k];

          bool fresh_cashable = original_contact_index < fresh_contacts_count && cached_contacts_mask & ((uint64_t)1 << original_contact_index);
          bool from_cache = original_contact_index >= fresh_contacts_count;
          if (fresh_cashable || from_cache) {
            cached_entry->features[feature_count++] = c->features;
          }
        }

        cached_entry->feature_count = feature_count;
        pair_contacts_count = MAX_CONTACTS_PER_PAIR;
      }

      count += pair_contacts_count;

      PROFILER_BLOCK_END;
    }
  }

  return count;
}

void collision_detection_init(void) {
  memset(collision_detection_table, 0, sizeof(collision_detection_table));

  collision_detection_table[BND_SPHERE][BND_SPHERE] = (collision_detection_entry) { sphere_sphere_collision, true, false };
  collision_detection_table[BND_BOX][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_MESH][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, true, true };

  collision_detection_table[BND_BOX][BND_SPHERE] = (collision_detection_entry) { box_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_BOX] = (collision_detection_entry) { box_sphere_collision, false, false };
  collision_detection_table[BND_BOX][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_CAPSULE][BND_SPHERE] = (collision_detection_entry) { capsule_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_CAPSULE] = (collision_detection_entry) { capsule_sphere_collision, false, false };

  collision_detection_table[BND_MESH][BND_BOX] = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_BOX][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_MESH][BND_SPHERE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, false };
  collision_detection_table[BND_MESH][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_MESH] = (collision_detection_entry) { polytope_polytope_collision, false, false };

  collision_detection_table[BND_BOX][BND_PLANE] = (collision_detection_entry) { box_plane_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_PLANE] = (collision_detection_entry) { sphere_plane_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_PLANE] = (collision_detection_entry) { capsule_plane_collision, true, false };
  collision_detection_table[BND_MESH][BND_PLANE] = (collision_detection_entry) { mesh_plane_collision, true, false };
}

// ================
//   queries.c
// ================


typedef bool (*raycast_func)(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit);

typedef struct {
  const bnd_world *world;
  const common_data *data;
  bnd_body_type type;
  bnd_collision_mask mask;
  count_t body_index;
  count_t shape_index;
} raycast_context;

static bnd_ray ray_transform(bnd_ray r, bnd_v3 witness, bnd_quat rotation) {
  bnd_quat inv_rotation = bnd_quat_invert(rotation);
  r.origin = bnd_v3_rotate(bnd_v3_sub(r.origin, witness), inv_rotation);
  r.direction = bnd_v3_rotate(r.direction, inv_rotation);

  return r;
}

static bool check_ray_cylinder(bnd_ray local_ray, float half_height, float radius, bnd_raycast_hit *local_hit) {
  const float epsilon = 1e-6f;

  // --- infinite cylinder (XZ plane) ---
  float a = local_ray.direction.x * local_ray.direction.x + local_ray.direction.z * local_ray.direction.z;
  float b = 2.0f * (local_ray.origin.x * local_ray.direction.x + local_ray.origin.z * local_ray.direction.z);
  float c = local_ray.origin.x * local_ray.origin.x + local_ray.origin.z * local_ray.origin.z - radius * radius;

  float t_body_enter = -FLT_MAX;
  float t_body_exit = FLT_MAX;
  bool body_hit = false;

  if (fabsf(a) > epsilon) {
    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
      return false;
    float sq = sqrtf(disc);
    float inv2a = 1.0f / (2.0f * a);
    t_body_enter = (-b - sq) * inv2a;
    t_body_exit = (-b + sq) * inv2a;
    body_hit = true;
  } else {
    // ray parallel to axis — must be inside the infinite cylinder
    if (c > 0.0f)
      return false;
  }

  // --- end caps (Y axis slab) ---
  float t_cap_enter, t_cap_exit;
  bnd_v3 normal_cap_enter, normal_cap_exit;

  if (fabsf(local_ray.direction.y) > epsilon) {
    float inv_dy = 1.0f / local_ray.direction.y;
    float t1 = (-half_height - local_ray.origin.y) * inv_dy;
    float t2 = (half_height - local_ray.origin.y) * inv_dy;
    if (t1 < t2) {
      t_cap_enter = t1;
      normal_cap_enter = (bnd_v3){0, -1, 0};
      t_cap_exit = t2;
      normal_cap_exit = (bnd_v3){0, 1, 0};
    } else {
      t_cap_enter = t2;
      normal_cap_enter = (bnd_v3){0, 1, 0};
      t_cap_exit = t1;
      normal_cap_exit = (bnd_v3){0, -1, 0};
    }
  } else {
    // ray parallel to caps — must be between them
    if (local_ray.origin.y < -half_height || local_ray.origin.y > half_height)
      return false;
    t_cap_enter = -FLT_MAX;
    normal_cap_enter = (bnd_v3){0, -1, 0};
    t_cap_exit = FLT_MAX;

    normal_cap_enter = (bnd_v3){0, -1, 0};
    t_cap_exit = FLT_MAX;
    normal_cap_exit = (bnd_v3){0, 1, 0};
  }

  // --- intersect intervals ---
  float t_enter = (body_hit && t_body_enter > t_cap_enter) ? t_body_enter : t_cap_enter;
  float t_exit = (body_hit && t_body_exit < t_cap_exit) ? t_body_exit : t_cap_exit;

  if (t_enter > t_exit)
    return false;

  float t = t_enter;
  if (t < 0.0f)
    t = t_exit;
  if (t < 0.0f || t > local_ray.max_distance)
    return false;

  // --- normal in local space ---
  bnd_v3 local_normal;
  if (t == t_body_enter || (t_enter < 0.0f && t == t_body_exit)) {
    bnd_v3 p = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
    bnd_v3 radial = (bnd_v3){p.x, 0, p.z};
    local_normal = bnd_v3_normalize(radial);
  } else {
    local_normal = (t == t_cap_enter) ? normal_cap_enter : normal_cap_exit;
  }

  local_hit->distance = t;
  local_hit->point = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
  local_hit->normal = local_normal;

  return true;
}

static bool check_ray_sphere(bnd_ray ray, bnd_v3 center, float radius, bnd_raycast_hit *hit) {
  bnd_v3 offset = bnd_v3_sub(center, ray.origin);
  float o = bnd_v3_lensqr(offset);
  float rr = radius * radius;

  float tc = bnd_v3_dot(offset, ray.direction);
  if (tc < 0.0f && o > rr)
    return false;

  float d2 = o - tc * tc;
  if (d2 > rr)
    return false;

  float delta = sqrtf(rr - d2);
  float t = (o > rr) ? tc - delta : tc + delta;

  if (t < 0.0f || t > ray.max_distance)
    return false;

  hit->distance = t;
  hit->point = bnd_v3_add(ray.origin, bnd_v3_scale(ray.direction, t));
  hit->normal = bnd_v3_normalize(bnd_v3_sub(hit->point, center));

  return true;
}

static bool raycast_sphere(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 position = body_center(ctx);

  return check_ray_sphere(r, position, ctx->shape.value.sphere.radius, hit);
}

static bool raycast_box(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 half = bnd_v3_scale(ctx->shape.value.box.size, 0.5f);
  bnd_v3 position = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  float tmin = -FLT_MAX;
  float tmax = FLT_MAX;
  bnd_v3 near_normal = bnd_v3_zero();
  bnd_v3 far_normal = bnd_v3_zero();

  const float epsilon = 1e-6f;

  for (count_t axis = 0; axis < 3; ++axis) {
    float o = ((float *)&local_ray.origin)[axis];
    float d = ((float *)&local_ray.direction)[axis];
    float h = ((float *)&half)[axis];

    if (fabsf(d) < epsilon) {
      if (o < -h || o > h) {
        return false;
      }
      continue;
    }

    float t1 = (-h - o) / d;
    float t2 = (h - o) / d;

    bnd_v3 n1 = bnd_v3_zero();
    bnd_v3 n2 = bnd_v3_zero();
    ((float *)&n1)[axis] = -1.0f;
    ((float *)&n2)[axis] = 1.0f;

    if (t1 > t2) {
      float temp = t1;
      t1 = t2;
      t2 = temp;

      bnd_v3 ntemp = n1;
      n1 = n2;
      n2 = ntemp;
    }

    if (t1 > tmin) {
      tmin = t1;
      near_normal = n1;
    }

    if (t2 < tmax) {
      tmax = t2;
      far_normal = n2;
    }

    if (tmin > tmax) {
      return false;
    }
  }

  float distance = tmin;
  bnd_v3 local_normal = near_normal;

  if (distance < 0.0f) {
    distance = tmax;
    local_normal = far_normal;
  }

  if (distance < 0.0f || distance > r.max_distance) {
    return false;
  }

  hit->distance = distance;
  hit->point = bnd_v3_add(r.origin, bnd_v3_scale(r.direction, distance));
  hit->normal = bnd_v3_rotate(local_normal, rotation);

  return true;
}

static bool raycast_capsule(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 center = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, center, rotation);

  float height = ctx->shape.value.capsule.height;
  float radius = ctx->shape.value.capsule.radius;

  bnd_v3 local_cap_top =    (bnd_v3) { 0,  0.5f * height, 0 };
  bnd_v3 local_cap_bottom = (bnd_v3) { 0, -0.5f * height, 0 };

  bnd_raycast_hit proxy_hit = { 0 };
  if (check_ray_sphere(local_ray, local_cap_top, radius, &proxy_hit) && proxy_hit.point.y > local_cap_top.y) {
    goto hit;
  }

  if (check_ray_sphere(local_ray, local_cap_bottom, radius, &proxy_hit) && proxy_hit.point.y < local_cap_bottom.y) {
    goto hit;
  }

  if (check_ray_cylinder(local_ray, 0.5f * height, radius, &proxy_hit)) {
    goto hit;
  }

  return false;

  hit:
  hit->point = bnd_v3_add(center, bnd_v3_rotate(proxy_hit.point, rotation));
  hit->normal = bnd_v3_rotate(proxy_hit.normal, rotation);
  hit->distance = proxy_hit.distance;
  return true;
}

static bool raycast_plane(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  float dod = bnd_v3_dot(bnd_v3_sub(ctx->data->positions[ctx->index], r.origin), ctx->shape.value.plane.normal);
  float dd = bnd_v3_dot(r.direction, ctx->shape.value.plane.normal);

  if (dd >= 0)
    return false;

  float distance = dod / dd;

  if (distance > r.max_distance)
    return false;

  hit->distance = distance;
  hit->point = bnd_v3_add(r.origin, bnd_v3_scale(r.direction, distance));
  hit->normal = ctx->shape.value.plane.normal;

  return true;
}

static bool raycast_mesh(bnd_ray r, const shape_context *ctx, bnd_raycast_hit *hit) {
  bnd_v3 position = body_center(ctx);
  bnd_quat rotation = body_rotation(ctx);

  bnd_ray local_ray = ray_transform(r, position, rotation);

  bool has_hit = false;
  float closest_distance = r.max_distance;
  bnd_v3 closest_point = {0}, normal = {0};

  const mesh_storage *meshes = &ctx->world->meshes;
  bnd_mesh m = meshes->meshes[ctx->shape.value.mesh];

  count_t submeshes_start = m.submesh_offset;
  count_t submeshes_end = submeshes_start + m.submesh_count;

  for (count_t i = submeshes_start; i < submeshes_end; ++i) {
    submesh sm = meshes->submeshes[i];

    count_t index_start = sm.index_offset;
    count_t index_end = index_start + sm.index_count;

    for (count_t j = index_start; j + 2 < index_end; j += 3) {
      bnd_v3 v0 = meshes->verticies[meshes->indicies[j + 0]];
      bnd_v3 v1 = meshes->verticies[meshes->indicies[j + 1]];
      bnd_v3 v2 = meshes->verticies[meshes->indicies[j + 2]];

      bnd_v3 n = bnd_v3_cross(bnd_v3_sub(v1, v0), bnd_v3_sub(v2, v0));
      float d = bnd_v3_dot(n, local_ray.direction);
      if (d >= -EPSILON) {
        continue;
      }

      float t = (bnd_v3_dot(n, v0) - bnd_v3_dot(n, local_ray.origin)) / d;
      if (t < 0 || t > closest_distance) {
        continue;
      }

      bnd_v3 p = bnd_v3_add(local_ray.origin, bnd_v3_scale(local_ray.direction, t));
      bnd_v3 bary = bnd_v3_barycentric(p, v0, v1, v2);

      if (bary.x < -EPSILON || bary.y < -EPSILON || bary.z < -EPSILON) {
        continue;
      }

      has_hit = true;
      closest_distance = t;
      closest_point = p;
      normal = n;
    }
  }

  if (!has_hit) {
    return false;
  }

  hit->point = bnd_v3_add(position, bnd_v3_rotate(closest_point, rotation));
  hit->normal = bnd_v3_normalize(bnd_v3_rotate(normal, rotation));
  hit->distance = closest_distance;

  return true;
}

static raycast_func raycasts[] = {
  raycast_box,
  raycast_sphere,
  raycast_capsule,
  raycast_mesh,
  raycast_plane,
};

static raycast_context begin_raycast(const bnd_world *world, bnd_collision_mask mask, bnd_body_type type) {
  const common_data *data = as_common_const(world, type);

  return (raycast_context) {
    .world = world,
    .data = data,
    .type = type,
    .mask = mask,
    .body_index = 0,
    .shape_index = 0,
  };
}

static bool next_raycast(raycast_context *ctx, bnd_ray r, bnd_raycast_hit *hit) {
  if (ctx->body_index >= ctx->data->count) {
    return false;
  }

  if (ctx->data->flags[ctx->body_index] & BODY_FLAG_TRIGGER) {
    ctx->body_index += 1;
    ctx->shape_index = 0;
    return next_raycast(ctx, r, hit);
  }

  bnd_collision_mask validation_mask = layer_to_mask(ctx->data->collision_layers[ctx->body_index]);
  if ((validation_mask & ctx->mask) == 0) {
    ctx->body_index += 1;
    ctx->shape_index = 0;
    return next_raycast(ctx, r, hit);
  }

  body_shapes shapes_info = ctx->data->shapes[ctx->body_index];
  bnd_body_shape *shapes = shapes_get(ctx->world, shapes_info);
  if (ctx->shape_index >= shapes_info.count) {
    ctx->body_index += 1;
    ctx->shape_index = 0;
    return next_raycast(ctx, r, hit);
  }

  bnd_body_shape shape = shapes[ctx->shape_index++];
  shape_context shape_ctx = { ctx->world, ctx->data, shape, ctx->body_index };

  bool is_hit = raycasts[shape.type](r, &shape_ctx, hit);
  if (is_hit) {
    hit->body = make_body_handle(ctx->world, ctx->type, ctx->body_index);
    return true;
  }

  return next_raycast(ctx, r, hit);
}

bool bnd_raycast_closest(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *closest_hit) {
  closest_hit->distance = FLT_MAX;
  if (bnd_v3_lensqr(ray.direction) < EPSILON * EPSILON)
    return false;

  ray.direction = bnd_v3_normalize(ray.direction);

  bnd_raycast_hit hit;
  raycast_context ctxs[] = { begin_raycast(world, mask, BND_BODY_DYNAMIC), begin_raycast(world, mask, BND_BODY_STATIC) };

  for (count_t i = 0; i < 2; ++i) {
    while(next_raycast(&ctxs[i], ray, &hit)) {
      if (hit.distance < closest_hit->distance) {
        *closest_hit = hit;
      }
    }
  }

  return closest_hit->distance < FLT_MAX;
}

count_t bnd_raycast_multiple(const bnd_world *world, bnd_ray ray, bnd_collision_mask mask, bnd_raycast_hit *hits, count_t max_hits) {
  if (max_hits == 0) {
    return 0;
  }

  if (bnd_v3_lensqr(ray.direction) < EPSILON * EPSILON)
    return false;

  ray.direction = bnd_v3_normalize(ray.direction);


  count_t num_hits = 0;
  raycast_context ctxs[] = { begin_raycast(world, mask, BND_BODY_DYNAMIC), begin_raycast(world, mask, BND_BODY_STATIC) };

  for (count_t i = 0; i < 2; ++i) {
    while(next_raycast(&ctxs[i], ray, &hits[num_hits])) {
      num_hits += 1;
      if (num_hits >= max_hits) {
        return num_hits;
      }
    }
  }

  return num_hits;
}

static count_t overlap_typed(const bnd_world *world, bnd_v3 origin, float radius, bnd_collision_mask mask, bnd_body_handle *overlaps, count_t max_overlaps, bnd_body_type type) {
  const common_data *data = as_common_const(world, type);

  count_t ephemeral_index = ephemeral_body_index(data);
  data->positions[ephemeral_index] = origin;
  data->aabbs[ephemeral_index] = (bnd_aabb){ origin, (bnd_v3){radius, radius, radius} };

  bnd_body_shape ephemeral_shape = { BND_SPHERE, { .sphere = { radius } }, bnd_v3_zero(), bnd_quat_identity() };

  count_t overlap_count = 0;
  simplex s = { 0 };
  for (count_t i = 0; i < data->count; ++i) {
    if (data->flags[i] & BODY_FLAG_TRIGGER) {
      continue;
    }

    bnd_collision_mask validation_mask = layer_to_mask(data->collision_layers[i]);
    if ((validation_mask & mask) == 0) {
      continue;
    }

    if (!aabb_intersect(data, data, ephemeral_index, i)) {
      continue;
    }

    collision_detection_context ctx;
    ctx.world = world;
    ctx.data_a = data;
    ctx.data_b = data;
    ctx.body_a = ephemeral_index;
    ctx.body_b = i;
    ctx.shape_a = ephemeral_shape;

    body_shapes shape_info = data->shapes[i];
    bnd_body_shape *shapes = shapes_get(world, shape_info);
    for (count_t j = 0; j < shape_info.count; ++j) {
      ctx.shape_b = shapes[j];

      if (ctx.shape_b.type == BND_SPHERE) {
        bnd_v3 sphere_center = data->positions[i];
        float r = ctx.shape_b.value.sphere.radius + radius;

        if (bnd_v3_distancesqr(sphere_center, origin) > r * r) {
          continue;
        }
      } else if (ctx.shape_b.type == BND_PLANE) {
        bnd_v3 plane_point = data->positions[i];
        bnd_v3 plane_normal = ctx.shape_b.value.plane.normal;

        if (bnd_v3_dot(plane_normal, bnd_v3_sub(origin, plane_point)) > radius) {
          continue;
        }
      } else if (!gjk_check_intersection(world, &ctx, &s)) {
        continue;
      }

      overlaps[overlap_count++] = make_body_handle(world, type, i);
      if (overlap_count >= max_overlaps) {
        return overlap_count;
      }
    }
  }

  return overlap_count;
}

count_t bnd_overlap(const bnd_world *world, bnd_v3 origin, float radius, bnd_collision_mask mask, bnd_body_handle *overlaps, count_t max_overlaps) {
  if (max_overlaps == 0) {
    return 0;
  }

  count_t overlap_count = overlap_typed(world, origin, radius, mask, overlaps, max_overlaps, BND_BODY_DYNAMIC);
  if (overlap_count == max_overlaps) {
    return overlap_count;
  }

  max_overlaps -= overlap_count;
  overlap_count += overlap_typed(world, origin, radius, mask, overlaps + overlap_count, max_overlaps, BND_BODY_STATIC);

  return overlap_count;
}

// ================
//   math.c
// ================

bnd_m3 bnd_m3_identity(void) {
  return (bnd_m3){ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
}

bnd_v3 bnd_m3_rotate(bnd_v3 v, bnd_m3 m) {
  return (bnd_v3){ bnd_v3_dot(*(bnd_v3 *)m.m0, v), bnd_v3_dot(*(bnd_v3 *)m.m1, v), bnd_v3_dot(*(bnd_v3 *)m.m2, v) };
}

bnd_m3 bnd_m3_transpose(bnd_m3 m) {
  return (bnd_m3){ { m.m0[0], m.m1[0], m.m2[0] }, { m.m0[1], m.m1[1], m.m2[1] }, { m.m0[2], m.m1[2], m.m2[2] } };
}

bnd_m3 bnd_m3_multiply(bnd_m3 a, bnd_m3 b) {
  bnd_m3 result;

  result.m0[0] = a.m0[0] * b.m0[0] + a.m0[1] * b.m1[0] + a.m0[2] * b.m2[0];
  result.m0[1] = a.m0[0] * b.m0[1] + a.m0[1] * b.m1[1] + a.m0[2] * b.m2[1];
  result.m0[2] = a.m0[0] * b.m0[2] + a.m0[1] * b.m1[2] + a.m0[2] * b.m2[2];

  result.m1[0] = a.m1[0] * b.m0[0] + a.m1[1] * b.m1[0] + a.m1[2] * b.m2[0];
  result.m1[1] = a.m1[0] * b.m0[1] + a.m1[1] * b.m1[1] + a.m1[2] * b.m2[1];
  result.m1[2] = a.m1[0] * b.m0[2] + a.m1[1] * b.m1[2] + a.m1[2] * b.m2[2];

  result.m2[0] = a.m2[0] * b.m0[0] + a.m2[1] * b.m1[0] + a.m2[2] * b.m2[0];
  result.m2[1] = a.m2[0] * b.m0[1] + a.m2[1] * b.m1[1] + a.m2[2] * b.m2[1];
  result.m2[2] = a.m2[0] * b.m0[2] + a.m2[1] * b.m1[2] + a.m2[2] * b.m2[2];

  return result;
}

bnd_v3 bnd_m3_rotate_inverse(bnd_v3 v, bnd_m3 m) {
  return bnd_m3_rotate(v, bnd_m3_transpose(m));
}

bnd_m3 bnd_m3_from_basis(bnd_v3 x, bnd_v3 y, bnd_v3 z) {
  return (bnd_m3){ { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } };
}

bnd_m3 bnd_m3_negate(bnd_m3 m) {
  return (bnd_m3){ { -m.m0[0], -m.m0[1], -m.m0[2] }, { -m.m1[0], -m.m1[1], -m.m1[2] }, { -m.m2[0], -m.m2[1], -m.m2[2] } };
}

bnd_m3 bnd_m3_inverse(bnd_m3 m) {
  float t4 = m.m0[0] * m.m1[1];
  float t6 = m.m0[0] * m.m1[2];
  float t8 = m.m0[1] * m.m1[0];
  float t10 = m.m0[2] * m.m1[0];
  float t12 = m.m0[1] * m.m2[0];
  float t14 = m.m0[2] * m.m2[0];

  // Calculate the determinant
  float t16 = (t4 * m.m2[2] - t6 * m.m2[1] - t8 * m.m2[2] + t10 * m.m2[1] + t12 * m.m1[2] - t14 * m.m1[1]);

  // Make sure the determinant is non-zero.
  if (t16 == (float)0.0f) {
    return m;
  }

  float t17 = 1 / t16;

  bnd_m3 result;
  result.m0[0] = (m.m1[1] * m.m2[2] - m.m1[2] * m.m2[1]) * t17;
  result.m0[1] = -(m.m0[1] * m.m2[2] - m.m0[2] * m.m2[1]) * t17;
  result.m0[2] = (m.m0[1] * m.m1[2] - m.m0[2] * m.m1[1]) * t17;
  result.m1[0] = -(m.m1[0] * m.m2[2] - m.m1[2] * m.m2[0]) * t17;
  result.m1[1] = (m.m0[0] * m.m2[2] - t14) * t17;
  result.m1[2] = -(t6 - t10) * t17;
  result.m2[0] = (m.m1[0] * m.m2[1] - m.m1[1] * m.m2[0]) * t17;
  result.m2[1] = -(m.m0[0] * m.m2[1] - t12) * t17;
  result.m2[2] = (t4 - t8) * t17;

  return result;
}

bnd_m3 bnd_m3_skew_symmetric(bnd_v3 v) {
  return (bnd_m3){ { 0, -v.z, v.y }, { v.z, 0, -v.x }, { -v.y, v.x, 0 } };
}

bnd_m3 bnd_m3_add(bnd_m3 a, bnd_m3 b) {
  return (bnd_m3){
    { a.m0[0] + b.m0[0], a.m0[1] + b.m0[1], a.m0[2] + b.m0[2] },
    { a.m1[0] + b.m1[0], a.m1[1] + b.m1[1], a.m1[2] + b.m1[2] },
    { a.m2[0] + b.m2[0], a.m2[1] + b.m2[1], a.m2[2] + b.m2[2] }
  };
}

bnd_m3 bnd_m3_scale(bnd_m3 m, float s) {
  return (bnd_m3){
    { m.m0[0] * s, m.m0[1] * s, m.m0[2] * s },
    { m.m1[0] * s, m.m1[1] * s, m.m1[2] * s },
    { m.m2[0] * s, m.m2[1] * s, m.m2[2] * s },
  };
}

bnd_m3 matrix_sub(bnd_m3 a, bnd_m3 b) {
  return (bnd_m3){
    { a.m0[0] - b.m0[0], a.m0[1] - b.m0[1], a.m0[2] - b.m0[2] },
    { a.m1[0] - b.m1[0], a.m1[1] - b.m1[1], a.m1[2] - b.m1[2] },
    { a.m2[0] - b.m2[0], a.m2[1] - b.m2[1], a.m2[2] - b.m2[2] }
  };
}

bnd_m3 bnd_m3_initial_inertia(bnd_v3 inertia) {
  return (bnd_m3){ { inertia.x, 0, 0 }, { 0, inertia.y, 0 }, { 0, 0, inertia.z } };
}

bnd_m3 bnd_m3_inertia(bnd_m3 initial_inertia, bnd_quat q) {
  float a2 = q.x * q.x;
  float b2 = q.y * q.y;
  float c2 = q.z * q.z;
  float ac = q.x * q.z;
  float ab = q.x * q.y;
  float bc = q.y * q.z;
  float ad = q.w * q.x;
  float bd = q.w * q.y;
  float cd = q.w * q.z;

  bnd_m3 rotation;

  rotation.m0[0] = 1 - 2 * (b2 + c2);
  rotation.m0[1] = 2 * (ab - cd);
  rotation.m0[2] = 2 * (ac + bd);

  rotation.m1[0] = 2 * (ab + cd);
  rotation.m1[1] = 1 - 2 * (a2 + c2);
  rotation.m1[2] = 2 * (bc - ad);

  rotation.m2[0] = 2 * (ac - bd);
  rotation.m2[1] = 2 * (bc + ad);
  rotation.m2[2] = 1 - 2 * (a2 + b2);

  return bnd_m3_multiply(bnd_m3_multiply(rotation, initial_inertia), bnd_m3_transpose(rotation));
}

bnd_m3 bnd_m3_displacement_inertia(bnd_m3 i0, bnd_v3 offset, float mass) {
  float r = bnd_v3_dot(offset, offset);

  bnd_m3 a = { 0 };
  a.m0[0] = r;
  a.m1[1] = r;
  a.m2[2] = r;

  bnd_m3 b;
  b.m0[0] = offset.x * offset.x;
  b.m0[1] = offset.x * offset.y;
  b.m0[2] = offset.x * offset.z;

  b.m1[0] = b.m0[1];
  b.m1[1] = offset.y * offset.y;
  b.m1[2] = offset.y * offset.z;

  b.m2[0] = b.m0[2];
  b.m2[1] = b.m1[2];
  b.m2[2] = offset.z * offset.z;

  return bnd_m3_add(i0, bnd_m3_scale(matrix_sub(a, b), mass));
}

bnd_quat integrate_rotation_midpoint(bnd_quat rotation, bnd_v3 angular_momentum, bnd_m3 base_inv_inertia, float dt) {
  bnd_m3 inv_inertia = bnd_m3_inertia(base_inv_inertia, rotation);
  bnd_v3 omega = bnd_m3_rotate(angular_momentum, inv_inertia);

  const float qdt = 0.25f * dt;
  const float hdt = 0.5f * dt;

  float half_angle = bnd_v3_len(omega) * qdt;
  bnd_quat half_step;
  if (half_angle < 1e-6f) {
    half_step = (bnd_quat){ omega.x * qdt, omega.y * qdt, omega.z * qdt, 1.0f };
    half_step = bnd_quat_normalize(half_step);
  } else {
    float scale_factor = sinf(half_angle) / bnd_v3_len(omega);
    half_step = (bnd_quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(half_angle) };
  }

  bnd_quat mid_rotation = bnd_quat_normalize(bnd_quat_mul(half_step, rotation));

  inv_inertia = bnd_m3_inertia(base_inv_inertia, mid_rotation);
  omega = bnd_m3_rotate(angular_momentum, inv_inertia);

  float angle = bnd_v3_len(omega) * hdt;
  if (angle < 1e-6f) {
    bnd_quat step = (bnd_quat){ omega.x * hdt, omega.y * hdt, omega.z * hdt, 1.0f };
    step = bnd_quat_normalize(step);
    return bnd_quat_normalize(bnd_quat_mul(step, rotation));
  }

  float scale_factor = sinf(angle) / bnd_v3_len(omega);
  bnd_quat step = (bnd_quat){ omega.x * scale_factor, omega.y * scale_factor, omega.z * scale_factor, cosf(angle) };

  return bnd_quat_normalize(bnd_quat_mul(step, rotation));
}

float sqr_distance_to_line_segment(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 *closest) {
  bnd_v3 d = bnd_v3_sub(b, a);
  bnd_v3 ao = bnd_v3_sub(a, from);

  float t = -1.0f * bnd_v3_dot(ao, d);
  t /= bnd_v3_lensqr(d);

  if (t <= 0) {
    *closest = a;
    return bnd_v3_distancesqr(a, from);
  } else if (t >= 1) {
    *closest = b;
    return bnd_v3_distancesqr(b, from);
  } else {
    *closest = bnd_v3_add(a, bnd_v3_scale(d, t));

    return bnd_v3_distancesqr(*closest, from);
  }
}

float sqr_distance_to_triangle(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_v3 *closest) {
  bnd_v3 d1 = bnd_v3_sub(b, a);
  bnd_v3 d2 = bnd_v3_sub(c, a);
  bnd_v3 ao = bnd_v3_sub(a, from);

  float v = bnd_v3_dot(d1, d1);
  float w = bnd_v3_dot(d2, d2);
  float p = bnd_v3_dot(ao, d1);
  float q = bnd_v3_dot(ao, d2);
  float r = bnd_v3_dot(d1, d2);

  float s, t;
  float distance = 0;
  float d = w * v - r * r;
  if (fabsf(d) < FLT_EPSILON) {
    s = t = -1;
  } else {
    s = (q * r - w * p) / d;
    t = (-s * r - q) / w;
  }

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1 && t + s <= 1) {
    d1 = bnd_v3_scale(d1, s);
    d2 = bnd_v3_scale(d2, t);

    *closest = bnd_v3_add(a, bnd_v3_add(d1, d2));
    distance = bnd_v3_distancesqr(*closest, from);
  } else {
    distance = sqr_distance_to_line_segment(from, a, b, closest);

    bnd_v3 closest_2;
    float distance2 = sqr_distance_to_line_segment(from, a, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }

    distance2 = sqr_distance_to_line_segment(from, b, c, &closest_2);
    if (distance2 < distance) {
      distance = distance2;
      *closest = closest_2;
    }
  }

  return distance;
}


bnd_m3 quat_as_matrix(bnd_quat q) {
  bnd_m3 result;

  float a2 = q.x*q.x;
  float b2 = q.y*q.y;
  float c2 = q.z*q.z;
  float ac = q.x*q.z;
  float ab = q.x*q.y;
  float bc = q.y*q.z;
  float ad = q.w*q.x;
  float bd = q.w*q.y;
  float cd = q.w*q.z;

  result.m0[0] = 1 - 2*(b2 + c2);
  result.m1[0] = 2*(ab + cd);
  result.m2[0] = 2*(ac - bd);

  result.m0[1] = 2*(ab - cd);
  result.m1[1] = 1 - 2*(a2 + c2);
  result.m2[1] = 2*(bc + ad);

  result.m0[2] = 2*(ac + bd);
  result.m1[2] = 2*(bc - ad);
  result.m2[2] = 1 - 2*(a2 + b2);

  return result;
}

// ================
//   contacts_resolution.c
// ================

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

static void prepare_contacts(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

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

  PROFILER_FUNCTION_END
}

static void resolve_interpenetration_contact(bnd_world *world, count_t contact_index, bnd_v3 *deltas) {
  PROFILER_FUNCTION_START

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
  float inv_inertia = 1.0f / total_inertia;
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];
    float sign = k ? -1.0f : 1.0f;
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

    if (fabsf(angular_move) < 0.001f) {
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

  PROFILER_FUNCTION_END
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
  PROFILER_FUNCTION_START

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

  PROFILER_FUNCTION_END
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
  count_t best_contact = UINT32_MAX;

  for (count_t i = 0; i < world->contacts.count; ++i) {
    contact *contact = &world->contacts.values[i];

    if (contact->desired_delta_velocity > max_velocity) {
      max_velocity = contact->desired_delta_velocity;
      best_contact = i;
    }
  }

  if (best_contact == UINT32_MAX) {
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
    world->dynamics.motion_avgs[contact->index_a] = 2.0f * sleep_threshold;
  }

  if (!body_b_awake) {
    world->dynamics.motion_avgs[contact->index_b] = 2.0f * sleep_threshold;
  }
}

static void resolve_interpenetrations(bnd_world *world) {
  PROFILER_FUNCTION_START

  const count_t count = world->contacts.count;
  const count_t max_iterations = count * world->config.advanced.resolution_attempts_factor;

  if (count == 0) {
    PROFILER_FUNCTION_END
    return;
  }

  count_t iterations = 0;
  count_t max_penetration_index = UINT32_MAX;
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

  PROFILER_FUNCTION_END
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

          contact->local_velocity = bnd_v3_add(contact->local_velocity, bnd_v3_scale(delta_velocity, (k ? -1.0f : 1.0f)));

          update_desired_velocity_delta(world, i, dt);
        }
      }
    }
  }
}

static void resolve_velocities(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  const count_t count = world->contacts.count;
  const count_t max_iterations = count * world->config.advanced.resolution_attempts_factor;
  if (count == 0) {
    PROFILER_FUNCTION_END
    return;
  }

  count_t iterations = 0;
  count_t worst_contact_index = UINT32_MAX;
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

  PROFILER_FUNCTION_END
}

void contacts_resolve(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  prepare_contacts(world, dt);
  resolve_interpenetrations(world);
  resolve_velocities(world, dt);

  PROFILER_FUNCTION_END
}

// ================
//   contacts.c
// ================


#define HASH_TABLE_TOMBSTONE UINT32_MAX
#define HASH_TABLE_EMPTY 0

typedef enum {
  SLOT_EMPTY      = 1,
  SLOT_SAME_KEY   = 2,

  SLOT_ANY        = SLOT_EMPTY | SLOT_SAME_KEY,
} hash_table_slot_flags;

static bnd_event make_collision_event(const bnd_world *world, bnd_body_type type, const contact *c) {
  return (bnd_event) { .type = BND_EVENT_COLLISION, .collision =  (bnd_contact) {
    .point = c->point,
    .normal = c->normal,
    .depth = c->depth,
    .body_a = make_body_handle(world, BND_BODY_DYNAMIC, c->index_a),
    .body_b = make_body_handle(world, type, c->index_b),
  }};
}

static void emit_collision_events(bnd_world *world, const contact *contacts, count_t count, bnd_body_type type) {
  common_data *data_a = as_common(world, BND_BODY_DYNAMIC);
  common_data *data_b = as_common(world, type);

  for (count_t i = 0; i < count; ++i) {
    const contact *c = &contacts[i];

    if (events_subscribed(data_a, c->index_a, BND_EVENT_COLLISION)) {
      events_push(world, data_a, c->index_a, make_collision_event(world, type, c));
    }

    if (events_subscribed(data_b, c->index_b, BND_EVENT_COLLISION)) {
      events_push(world, data_b, c->index_b, make_collision_event(world, type, c));
    }
  }
}

static inline uint64_t cache_key_hash(uint64_t key) {
  key ^= key >> 30; key *= 0xbf58476d1ce4e5b9ULL;
  key ^= key >> 27; key *= 0x94d049bb133111ebULL;
  key ^= key >> 31;

  return key;
}

static bnd_result_u32 cache_table_find_slot(bnd_world *world, uint64_t key, hash_table_slot_flags flags) {
  contacts_cache *cache = &world->contacts_cache;

  uint64_t hash = cache_key_hash(key);
  count_t hash_table_index = hash & (cache->hash_table_capacity - 1);

  bool search_empty = flags & SLOT_EMPTY;
  bool search_same_key = flags & SLOT_SAME_KEY;
  int32_t first_tombstone = -1;

  count_t i = hash_table_index;
  do {
    count_t index = cache->hash_table[i];

    if (index == HASH_TABLE_EMPTY) {
      if (search_same_key && search_empty && first_tombstone >= 0) {
        return BND_RESULT_OK(u32, first_tombstone);
      }

      if (search_empty) {
        return BND_RESULT_OK(u32, i);
      }

      return BND_RESULT_ERR(u32, BND_ERROR_NOT_FOUND, "");
    }

    if (index == HASH_TABLE_TOMBSTONE) {
      if (search_empty && search_same_key) {
        if (first_tombstone < 0) {
          first_tombstone = i;
        }
      } else if (search_empty) {
        return BND_RESULT_OK(u32, i);
      }

      goto next;
    }

    cache_entry *entry = &cache->entries[index];
    if ((flags & SLOT_SAME_KEY) && entry->key == key) {
      return BND_RESULT_OK(u32, i);
    }

    next:
    i = (i + 1) & (cache->hash_table_capacity - 1);
  } while(i != hash_table_index);

  if (search_same_key && search_empty && first_tombstone >= 0) {
    return BND_RESULT_OK(u32, first_tombstone);
  }

  return BND_RESULT_ERR(u32, BND_ERROR_OUT_OF_MEMORY, "Hash table is full");
}

static bnd_error cache_table_realloc_if_needed(bnd_world *world) {
  contacts_cache *cache = &world->contacts_cache;

  if (cache->hash_table_capacity * 0.75f < cache->entry_count) {
    count_t new_capacity = cache->hash_table_capacity * 2;
    REALLOC_BUFFER4(cache->hash_table, world->allocator, sizeof(count_t), cache->hash_table_capacity, new_capacity)
    memset(cache->hash_table, 0, new_capacity * sizeof(count_t));

    cache->hash_table_capacity = new_capacity;

    for (count_t i = 1; i <= cache->entry_count; ++i) {
      uint64_t key = cache->entries[i].key;
      bnd_result_u32 index = cache_table_find_slot(world, key, SLOT_ANY);
      if (index.error.type != BND_OK) {
        return index.error;
      }

      cache->hash_table[index.value] = i;
    }
  }

  if (cache->entry_count + 1 >= cache->buffer_capacity) {
    count_t new_capacity = cache->buffer_capacity * 2;

    REALLOC_BUFFER8(cache->entries, world->allocator, sizeof(cache_entry), cache->buffer_capacity, new_capacity);
    memset(cache->entries + cache->buffer_capacity, 0, cache->buffer_capacity * sizeof(cache_entry));

    cache->buffer_capacity = new_capacity;
  }

  return OK;
}

static bnd_result_u32 cache_table_insert(bnd_world *world, uint64_t key) {
  contacts_cache *cache = &world->contacts_cache;

  PROPAGATE_RESULT(u32, cache_table_realloc_if_needed(world));

  bnd_result_u32 hash_table_slot = cache_table_find_slot(world, key, SLOT_ANY);
  if (hash_table_slot.error.type != BND_OK) {
    return hash_table_slot;
  }

  cache_entry *entry;
  count_t entry_index = cache->hash_table[hash_table_slot.value];
  if (entry_index == HASH_TABLE_EMPTY || entry_index == HASH_TABLE_TOMBSTONE) {
    entry_index = ++cache->entry_count; // Prefix-increment because we want to skip 0 index

    entry = &cache->entries[entry_index];
    entry->key = key;
    entry->access_time = world->age;
    entry->feature_count = 0;

    cache->hash_table[hash_table_slot.value] = entry_index;
  } else if (cache->entries[entry_index].key == key) {
    entry = &cache->entries[entry_index];
    entry->access_time = world->age;
  }

  return BND_RESULT_OK(u32, entry_index);
}

void contacts_cache_reset(bnd_world *world) {
  contacts_cache *cache = &world->contacts_cache;
  cache->entry_count = 0;
  memset(cache->hash_table, 0, cache->hash_table_capacity * sizeof(uint32_t));
}

void contacts_generate(bnd_world *world) {
  PROFILER_FUNCTION_START

  count_t dynamic_count = collisions_detect(world, 0, BND_BODY_DYNAMIC);
  emit_collision_events(world, world->contacts.values, dynamic_count, BND_BODY_DYNAMIC);

  dynamic_count += joints_generate_contacts(world, dynamic_count, BND_BODY_DYNAMIC);

  const count_t static_offset = dynamic_count;
  count_t static_count = collisions_detect(world, static_offset, BND_BODY_STATIC);
  emit_collision_events(world, world->contacts.values + static_offset, static_count, BND_BODY_STATIC);

  static_count += joints_generate_contacts(world, static_offset + static_count, BND_BODY_STATIC);

  world->contacts.count = dynamic_count + static_count;
  world->contacts.dynamic_count = dynamic_count;
  world->stats.contacts_count = world->contacts.count;

  PROFILER_FUNCTION_END
}

void contacts_reset(bnd_world *world) {
  world->contacts.count = 0;
  world->contacts.dynamic_count = 0;
}

bnd_error contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(contacts->values, world->config.memory.contacts_capacity * sizeof(contact));

  contacts->capacity = world->config.memory.contacts_capacity;
  contacts->count = 0;
  contacts->dynamic_count = 0;

  collision_detection_init();
  PROPAGATE_ERROR(contacts_cache_init(world));

  return OK;
}

void contacts_teardown(bnd_world *world) {
  world->allocator.free(world->contacts.values, world->contacts.capacity * sizeof(contact));

  contacts_cache *cache = &world->contacts_cache;
  world->allocator.free(cache->hash_table, cache->hash_table_capacity * sizeof(count_t));
  world->allocator.free(cache->entries, cache->buffer_capacity * sizeof(cache_entry));
}

bnd_error contacts_ensure_capacity(bnd_world *world, count_t contacts_offset, count_t additional_count) {
  count_t desired_count = contacts_offset + additional_count;
  if (desired_count <= world->contacts.capacity) {
    return OK;
  }

  if (world->allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Contacts buffer is full and Allocator.realloc is NULL" };
  }

  count_t old_capacity = world->contacts.capacity;
  while (desired_count > world->contacts.capacity) {
    world->contacts.capacity <<= 1;

    if (world->contacts.capacity >= desired_count) {
      REALLOC_BUFFER4(world->contacts.values, world->allocator, sizeof(contact), old_capacity, world->contacts.capacity);
      break;
    }
  }

  return OK;
}

bnd_error contacts_cache_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;
  contacts_cache *cache = &world->contacts_cache;

  cache->entry_count = 0;
  cache->buffer_capacity = world->config.advanced.contacts_cache.buffer_capacity;

  count_t desired_capacity = world->config.advanced.contacts_cache.hash_table_capacity;
  cache->hash_table_capacity = 1;
  while (cache->hash_table_capacity < desired_capacity) {
    cache->hash_table_capacity *= 2;
  }

  ALLOC_BUFFER4(cache->hash_table, cache->hash_table_capacity * sizeof(uint32_t));
  ALLOC_BUFFER8(cache->entries, cache->buffer_capacity * sizeof(cache_entry));

  memset(cache->hash_table, 0, cache->hash_table_capacity * sizeof(uint32_t));
  memset(cache->entries, 0, cache->buffer_capacity * sizeof(cache_entry));

  return OK;
}

cache_entry *contacts_cache_query(bnd_world *world, contact *contact, bnd_body_type type) {
  const common_data *data_a = as_common_const(world, BND_BODY_DYNAMIC);
  const common_data *data_b = as_common_const(world, type);

  uint64_t index_a = data_a->inner_lookup[contact->index_a];
  uint64_t index_b = data_b->inner_lookup[contact->index_b];
  uint64_t gen_a = data_a->generations[index_a];
  uint64_t gen_b = data_b->generations[index_b];

  // Features are stored in A and B's local spaces, so the cache key must retain their order.
  const uint64_t mask_23bit = 0x7FFFFF;

  uint64_t key = (uint64_t)type << 62;
  key |= gen_a << 53;
  key |= (index_a & mask_23bit) << 31;
  key |= gen_b << 23;
  key |= index_b & mask_23bit;

  bnd_result_u32 index = cache_table_insert(world, key);
  if (index.error.type != BND_OK) {
    return NULL;
  }

  return &world->contacts_cache.entries[index.value];
}

void contacts_cache_prune(bnd_world *world) {
  PROFILER_FUNCTION_START

  contacts_cache *cache = &world->contacts_cache;
  cache_entry *entries = cache->entries;
  int32_t entry_count = (int32_t) cache->entry_count;

  for (int32_t i = entry_count; i > 0; --i) {
    count_t age = world->age - entries[i].access_time;

    if (age < world->config.advanced.contacts_cache.max_age) {
      continue;
    }

    bnd_result_u32 current_slot_index = cache_table_find_slot(world, entries[i].key, SLOT_SAME_KEY);
    if (current_slot_index.error.type == BND_OK) {
      cache->hash_table[current_slot_index.value] = HASH_TABLE_TOMBSTONE;
    }

    bnd_result_u32 replacement_slot_index = cache_table_find_slot(world, entries[entry_count].key, SLOT_SAME_KEY);
    if (replacement_slot_index.error.type == BND_OK) {
      cache->hash_table[replacement_slot_index.value] = i;
    }

    entries[i] = entries[entry_count--];
  }

  world->contacts_cache.entry_count = entry_count;

  PROFILER_FUNCTION_END
}

static float cross_2d(bnd_v3 a, bnd_v3 b, bnd_v3 c) {
  bnd_v3 ab = { b.x - a.x, b.y - a.y, 0 };
  bnd_v3 ac = { c.x - a.x, c.y - a.y, 0 };
  return ab.x * ac.y - ab.y * ac.x;
}

static void sort_points(bnd_v3 *points) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 value = points[i];
    count_t j = i;
    while (j > 0) {
      bnd_v3 previous = points[j - 1];
      if (previous.x < value.x || (previous.x == value.x && previous.y <= value.y)) {
        break;
      }

      points[j] = previous;
      --j;
    }
    points[j] = value;
  }
}

static float contact_set_area(contact *contacts, const count_t *indices, bnd_v3 origin, bnd_v3 tangent_x, bnd_v3 tangent_y) {
  bnd_v3 points[MAX_CONTACTS_PER_PAIR];
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 offset = bnd_v3_sub(contacts[indices[i]].point, origin);
    points[i] = (bnd_v3){
      .x = bnd_v3_dot(offset, tangent_x),
      .y = bnd_v3_dot(offset, tangent_y),
    };
  }

  sort_points(points);

  bnd_v3 hull[MAX_CONTACTS_PER_PAIR * 2];
  count_t hull_count = 0;

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    while (hull_count >= 2 && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  count_t lower_count = hull_count;
  for (count_t i = MAX_CONTACTS_PER_PAIR - 1; i < MAX_CONTACTS_PER_PAIR; --i) {
    while (hull_count > lower_count && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  if (hull_count <= 3) {
    return 0;
  }

  --hull_count;

  float area = 0;
  for (count_t i = 0; i < hull_count; ++i) {
    bnd_v3 a = hull[i];
    bnd_v3 b = hull[(i + 1) % hull_count];
    area += a.x * b.y - a.y * b.x;
  }

  return fabsf(area) * 0.5f;
}

static float contact_set_depth(contact *contacts, const count_t *indices) {
  float depth = 0;
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    depth += contacts[indices[i]].depth;
  }

  return depth;
}

static bool better_contact_set(float area, float depth, float best_area, float best_depth) {
  if (area > best_area + EPSILON) {
    return true;
  }

  if (fabsf(area - best_area) <= EPSILON && depth > best_depth + EPSILON) {
    return true;
  }

  return false;
}

static void sort_indices(count_t *indices) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    count_t value = indices[i];
    count_t j = i;
    while (j > 0 && indices[j - 1] > value) {
      indices[j] = indices[j - 1];
      --j;
    }
    indices[j] = value;
  }
}

void contacts_filter_largest_surface_area(contact *contacts, count_t contact_count, count_t *selected_indices) {
  count_t deepest = 0;
  for (count_t i = 1; i < contact_count; ++i) {
    if (contacts[i].depth > contacts[deepest].depth) {
      deepest = i;
    }
  }

  bnd_v3 normal = contacts[deepest].normal;
  bnd_v3 tangent_seed = fabsf(normal.y) < 0.70710678f ? bnd_v3_up() : bnd_v3_right();
  bnd_v3 tangent_x = bnd_v3_cross(tangent_seed, normal);
  if (bnd_v3_lensqr(tangent_x) <= EPSILON * EPSILON) {
    tangent_x = bnd_v3_cross(bnd_v3_forward(), normal);
  }
  tangent_x = bnd_v3_normalize(tangent_x);
  bnd_v3 tangent_y = bnd_v3_normalize(bnd_v3_cross(normal, tangent_x));
  bnd_v3 origin = contacts[deepest].point;

  float best_area = -FLT_MAX;
  float best_depth = -FLT_MAX;

  for (count_t i = 0; i < contact_count; ++i) {
    if (i == deepest) {
      continue;
    }

    for (count_t j = i + 1; j < contact_count; ++j) {
      if (j == deepest) {
        continue;
      }

      for (count_t k = j + 1; k < contact_count; ++k) {
        if (k == deepest) {
          continue;
        }

        count_t indices[MAX_CONTACTS_PER_PAIR] = { deepest, i, j, k };
        float area = contact_set_area(contacts, indices, origin, tangent_x, tangent_y);
        float depth = contact_set_depth(contacts, indices);

        if (better_contact_set(area, depth, best_area, best_depth)) {
          memcpy(selected_indices, indices, sizeof(indices));
          best_area = area;
          best_depth = depth;
        }
      }
    }
  }

  // Since will move the elements within the same buffer, having the indices in ascending order will prevent data corruption.
  sort_indices(selected_indices);

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    contacts[i] = contacts[selected_indices[i]];
  }
}

// ================
//   joints.c
// ================


static bnd_error resize_if_needed(bnd_allocator allocator, joints *joints) {
  if (joints->count < joints->capacity) {
    return OK;
  }

  count_t old_capacity = joints->capacity;
  while (joints->count >= joints->capacity) {
    joints->capacity *= 2;
  }

  REALLOC_BUFFER4(joints->values, allocator , sizeof(bnd_joint), old_capacity, joints->capacity);
  REALLOC_BUFFER4(joints->ids, allocator , sizeof(count_t), old_capacity, joints->capacity);

  return OK;
}

bnd_result_u32 bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_v3 contact_offset_a, bnd_v3 contact_offset_b, float max_distance) {
  PROPAGATE_RESULT(u32, bnd_handle_valid(world, body_a));
  PROPAGATE_RESULT(u32, bnd_handle_valid(world, body_b));

  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_STATIC) {
    return BND_RESULT_ERR(u32, BND_ERROR_INVALID_JOINT, "Two static bodies cannot be bound together");
  }

  // Let body_a always be dynamic - same as with contacts.
  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_DYNAMIC) {
    bnd_body_handle tmp_body = body_b;
    body_b = body_a;
    body_a = tmp_body;

    bnd_v3 tmp_pos = contact_offset_b;
    contact_offset_b = contact_offset_a;
    contact_offset_a = tmp_pos;
  }

  joints *joints = &world->joints;

  PROPAGATE_RESULT(u32, resize_if_needed(world->allocator, joints));

  count_t last_index = joints->count++;
  bool is_dynamic = body_b.type == BND_BODY_DYNAMIC;
  count_t id = joints->next_id++;

  count_t index;
  if (is_dynamic) {
    if (joints->dynamic_count < last_index) {
      // Move first static joint to the end
      joints->values[last_index] = joints->values[joints->dynamic_count];
      joints->ids[last_index] = joints->ids[joints->dynamic_count];
    }

    index = joints->dynamic_count;
    joints->dynamic_count += 1;
  } else {
    index = last_index;
  }

  joints->values[index] = (bnd_joint){
    .bodies = {body_a, body_b},
    .relative_contact_positions = {contact_offset_a, contact_offset_b},
    .max_error = max_distance,
  };
  joints->ids[index] = id;

  return BND_RESULT_OK(u32, id);
}

void bnd_remove_joint(bnd_world *world, count_t id) {
  joints *joints = &world->joints;

  count_t count = joints->count;
  count_t dynamic_count = joints->dynamic_count;
  for (count_t i = 0; i < count; ++i) {
    if (joints->ids[i] != id) {
      continue;
    }

    if (i < dynamic_count) {
      joints->values[i] = joints->values[dynamic_count - 1];
      joints->ids[i] = joints->ids[dynamic_count - 1];

      joints->values[dynamic_count - 1] = joints->values[count - 1];
      joints->ids[dynamic_count - 1] = joints->ids[count - 1];

      joints->dynamic_count -= 1;
    } else {
      joints->values[i] = joints->values[count - 1];
      joints->ids[i] = joints->ids[count - 1];
    }

    joints->count -= 1;

    break;
  }
}

static bool joint_is_stale(const bnd_joint *joint, bnd_body_handle removed_body) {
  for (count_t i = 0; i < 2; ++i) {
    bnd_body_handle body = joint->bodies[i];
    if (body.type == removed_body.type && body.index == removed_body.index) {
      return true;
    }
  }

  return false;
}

void joints_remove_stale_if_needed(bnd_world *world, bnd_body_handle removed_body) {
  joints *joints = &world->joints;

  count_t old_dynamic_count = joints->dynamic_count;
  count_t write = 0;

  for (count_t read = 0; read < old_dynamic_count; ++read) {
    if (joint_is_stale(&joints->values[read], removed_body)) {
      continue;
    }

    if (write != read) {
      joints->values[write] = joints->values[read];
      joints->ids[write] = joints->ids[read];
    }

    write += 1;
  }

  count_t new_dynamic_count = write;

  for (count_t read = old_dynamic_count; read < joints->count; ++read) {
    if (joint_is_stale(&joints->values[read], removed_body)) {
      continue;
    }

    if (write != read) {
      joints->values[write] = joints->values[read];
      joints->ids[write] = joints->ids[read];
    }

    write += 1;
  }

  joints->dynamic_count = new_dynamic_count;
  joints->count = write;
}

count_t joints_generate_contacts(bnd_world *world, count_t contacts_offset, bnd_body_type type) {
  const joints *joints = &world->joints;

  const count_t start = type == BND_BODY_DYNAMIC ? 0 : joints->dynamic_count;
  const count_t end = type == BND_BODY_DYNAMIC ? joints->dynamic_count : joints->count;
  const count_t max_count = end - start;

  if (IS_ERROR(contacts_ensure_capacity(world, contacts_offset, max_count))) {
    return 0;
  }

  contact *contacts = world->contacts.values;

  count_t spawned_count = 0;
  for (count_t i = start; i < end; ++i) {
    bnd_joint j = joints->values[i];

    const common_data *data[2];
    data[0] = as_common(world, BND_BODY_DYNAMIC);
    data[1] = as_common(world, type);

    bnd_v3 world_points[2];
    count_t indices[2];
    for (count_t k = 0; k < 2; ++k) {
      count_t index = handle_to_inner_index(world, j.bodies[k]);
      world_points[k] = bnd_v3_rotate(j.relative_contact_positions[k], data[k]->rotations[index]);
      world_points[k] = bnd_v3_add(world_points[k], data[k]->positions[index]);
      indices[k] = index;
    }

    bnd_v3 offset = bnd_v3_sub(world_points[1], world_points[0]);
    float distance = bnd_v3_len(offset);
    if (distance <= j.max_error) {
      continue;
    }

    contact *contact = contacts + contacts_offset + spawned_count;
    contact->index_a = indices[0];
    contact->index_b = indices[1];
    contact->point = bnd_v3_scale(bnd_v3_add(world_points[0], world_points[1]), 0.5f);
    contact->normal = bnd_v3_scale(offset, 1.0f / distance);
    contact->depth = distance - j.max_error;
    contact->friction = 1.0f;
    contact->restitution = 0.0f;

    spawned_count += 1;
  }

  return spawned_count;
}

bnd_error joints_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(world->joints.values, world->config.memory.joints_capacity * sizeof(bnd_joint));
  ALLOC_BUFFER4(world->joints.ids, world->config.memory.joints_capacity * sizeof(count_t));

  world->joints.capacity = world->config.memory.joints_capacity;

  joints_reset(world);

  return OK;
}

void joints_reset(bnd_world *world) {
  world->joints.count = 0;
  world->joints.next_id = 0;
  world->joints.dynamic_count = 0;
}

void joints_teardown(bnd_world *world) {
  world->allocator.free(world->joints.values, world->joints.capacity * sizeof(bnd_joint));
  world->allocator.free(world->joints.ids, world->joints.capacity * sizeof(count_t));
}

// ================
//   shapes.c
// ================

#define SHAPE_BRACKET_BLOCK_CAPACITY 64

void shapes_get_bracket_properties(const bnd_config *config, count_t bracket_index, count_t *blocks, count_t *shapes, count_t *capacity) {
  count_t blocks_count = config->advanced.shapes_brackets_capacity[bracket_index] / SHAPE_BRACKET_BLOCK_CAPACITY +
                          ((config->advanced.shapes_brackets_capacity[bracket_index] & (SHAPE_BRACKET_BLOCK_CAPACITY - 1)) > 0);
  count_t bracket_capacity = blocks_count * SHAPE_BRACKET_BLOCK_CAPACITY;

  count_t bracket_dimension = 1 << bracket_index;
  count_t shapes_count = bracket_capacity * bracket_dimension;

  *blocks = blocks_count;
  *shapes = shapes_count;
  *capacity = bracket_capacity;
}

static count_t bracket_block_count(const bnd_world *world, shape_dimension_bracket bracket) {
  return world->shape_brackets[bracket].capacity / SHAPE_BRACKET_BLOCK_CAPACITY;
}

bnd_error shapes_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;

  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count, shapes_count, bracket_capacity;
    shapes_get_bracket_properties(&world->config, i, &blocks_count, &shapes_count, &bracket_capacity);

    shapes_bracket *bracket = &world->shape_brackets[i];
    ALLOC_BUFFER8(bracket->slots, blocks_count * sizeof(uint64_t));
    ALLOC_BUFFER4(bracket->shapes, shapes_count * sizeof(bnd_body_shape));

    memset(bracket->slots, 0, blocks_count * sizeof(uint64_t));

    bracket->capacity = bracket_capacity;
  }

  return OK;
}

void shapes_teardown(bnd_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    shapes_bracket *bracket = &world->shape_brackets[i];
    count_t block_count = bracket_block_count(world, i);
    world->allocator.free(bracket->slots, block_count * sizeof(uint64_t));
    world->allocator.free(bracket->shapes, (1 << i) * block_count * SHAPE_BRACKET_BLOCK_CAPACITY * sizeof(bnd_body_shape));
  }
}

void shapes_reset(bnd_world *world) {
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    shapes_bracket *bracket = &world->shape_brackets[i];
    count_t blocks_count = bracket_block_count(world, i);

    memset(bracket->slots, 0, blocks_count * sizeof(uint64_t));
  }
}

bool shapes_any_slot_available(const bnd_world *world, shape_dimension_bracket bracket) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] < (uint64_t)~0) {
      return true;
    }
  }

  return false;
}

bnd_error shapes_expand_bracket(bnd_world *world, shape_dimension_bracket bracket) {
  count_t bracket_capacity = 1 << bracket;

  shapes_bracket *current_bracket = &world->shape_brackets[bracket];
  count_t current_capacity = current_bracket->capacity;
  count_t current_block_count = bracket_block_count(world, bracket);

  count_t new_capacity = current_capacity + SHAPE_BRACKET_BLOCK_CAPACITY;
  count_t new_block_count = current_block_count + 1;
  count_t shapes_count = bracket_capacity * new_block_count * SHAPE_BRACKET_BLOCK_CAPACITY;

  REALLOC_BUFFER8(current_bracket->slots, world->allocator, sizeof(uint64_t), current_block_count, new_block_count);
  REALLOC_BUFFER4(current_bracket->shapes, world->allocator, sizeof(bnd_body_shape), bracket_capacity * current_block_count * SHAPE_BRACKET_BLOCK_CAPACITY, shapes_count);

  memset(current_bracket->slots + current_block_count, 0, sizeof(uint64_t));

  current_bracket->capacity = new_capacity;

  return OK;

}

bool shapes_put_into_empty_slot(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t shapes_count, count_t *slot_number) {
  count_t blocks_count = bracket_block_count(world, bracket);
  uint64_t *slots = world->shape_brackets[bracket].slots;
  bnd_body_shape *shapes_buffer = world->shape_brackets[bracket].shapes;

  for (count_t i = 0; i < blocks_count; ++i) {
    if (slots[i] == (uint64_t)~0)
      continue;

    for (count_t k = 0; k < SHAPE_BRACKET_BLOCK_CAPACITY; ++k) {
      uint64_t mask = (uint64_t)1 << k;
      if ((slots[i] & mask) != 0)
        continue;

      count_t bracket_capacity = 1 << bracket;
      count_t shape_offset = (i * SHAPE_BRACKET_BLOCK_CAPACITY + k) * bracket_capacity;

      bnd_body_shape *slot = shapes_buffer + shape_offset;
      memcpy(slot, shapes, shapes_count * sizeof(bnd_body_shape));

      slots[i] |= mask;
      *slot_number = shape_offset;

      return true;
    }
  }

  return false;
}

void shapes_clear_slot(bnd_world *world, shape_dimension_bracket bracket, count_t slot) {
  count_t block_count = bracket_block_count(world, bracket);
  count_t bracket_capacity = 1 << bracket;

  uint64_t *slots = world->shape_brackets[bracket].slots;
  count_t block_index = slot / (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity);
  count_t bit_index = slot % (SHAPE_BRACKET_BLOCK_CAPACITY * bracket_capacity) / bracket_capacity;
  if (block_index < block_count) {
    slots[block_index] &= ~((uint64_t)1 << bit_index);
  }
}

body_shapes shapes_write(bnd_world *world, shape_dimension_bracket bracket, bnd_body_shape *shapes, count_t count) {
  assert(count <= (1 << (BRACKET_COUNT - 1)));

  if (!shapes_any_slot_available(world, bracket)) {
    shapes_expand_bracket(world, bracket);
  }

  count_t shape_slot;
  shapes_put_into_empty_slot(world, bracket, shapes, count, &shape_slot);

  return (body_shapes){.bracket = bracket, .offset = shape_slot, .count = count};
}

bnd_body_shape *shapes_get(const bnd_world *world, body_shapes shapes) {
  return world->shape_brackets[shapes.bracket].shapes + shapes.offset;
}

#ifdef BND_TESTS


void test_shapes_write_primitive_bracket_uses_second_block(void) {
  bnd_world world = {0};
  world.allocator = bnd_default_allocator();
  world.config.advanced.shapes_brackets_capacity[BRACKET_PRIMITIVE] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 65; ++i) {
    bnd_body_shape shape = {0};
    shape.type = BND_SPHERE;
    shape.value.sphere.radius = (float)i + 0.5f;

    body_shapes written = shapes_write(&world, BRACKET_PRIMITIVE, &shape, 1);
    bnd_body_shape *stored = shapes_get(&world, written);

    assert(written.bracket == BRACKET_PRIMITIVE);
    assert(written.count == 1);
    assert(written.offset == i);
    assert(memcmp(stored, &shape, sizeof(bnd_body_shape)) == 0);
  }

  assert(world.shape_brackets[BRACKET_PRIMITIVE].capacity == 128);
  assert(world.shape_brackets[BRACKET_PRIMITIVE].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_PRIMITIVE].slots[1] == 1);
  assert(shapes_any_slot_available(&world, BRACKET_PRIMITIVE));

  shapes_teardown(&world);
}

void test_shapes_write_four_bracket_keeps_alignment_across_blocks(void) {
  bnd_world world = {0};
  world.allocator = bnd_default_allocator();
  world.config.advanced.shapes_brackets_capacity[BRACKET_FOUR] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 65; ++i) {
    bnd_body_shape shapes[4] = {0};
    count_t count = (i & 1) == 0 ? 3 : 4;

    for (count_t k = 0; k < count; ++k) {
      shapes[k].type = BND_CAPSULE;
      shapes[k].value.capsule.radius = (float)(i * 10 + k + 1);
      shapes[k].value.capsule.height = (float)(100 + i * 10 + k);
    }

    body_shapes written = shapes_write(&world, BRACKET_FOUR, shapes, count);
    bnd_body_shape *stored = shapes_get(&world, written);

    assert(written.bracket == BRACKET_FOUR);
    assert(written.count == count);
    assert(written.offset == i * 4);
    assert(memcmp(stored, shapes, count * sizeof(bnd_body_shape)) == 0);
  }

  assert(world.shape_brackets[BRACKET_FOUR].capacity == 128);
  assert(world.shape_brackets[BRACKET_FOUR].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_FOUR].slots[1] == 1);

  shapes_teardown(&world);
}

void test_shapes_expand_bracket_preserves_existing_data_after_two_blocks(void) {
  bnd_world world = {0};
  world.allocator = bnd_default_allocator();
  world.config.advanced.shapes_brackets_capacity[BRACKET_TWO] = 65;

  shapes_init(&world);

  for (count_t i = 0; i < 128; ++i) {
    bnd_body_shape shapes[2] = {0};
    shapes[0].type = BND_BOX;
    shapes[0].value.box.size.x = (float)(i + 1);
    shapes[1].type = BND_SPHERE;
    shapes[1].value.sphere.radius = (float)(i + 200);

    body_shapes written = shapes_write(&world, BRACKET_TWO, shapes, 2);
    assert(written.offset == i * 2);
  }

  assert(world.shape_brackets[BRACKET_TWO].capacity == 128);
  assert(world.shape_brackets[BRACKET_TWO].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[1] == (uint64_t)~0);
  assert(!shapes_any_slot_available(&world, BRACKET_TWO));

  bnd_body_shape extra_shapes[2] = {0};
  extra_shapes[0].type = BND_CAPSULE;
  extra_shapes[0].value.capsule.radius = 7.0f;
  extra_shapes[0].value.capsule.height = 9.0f;
  extra_shapes[1].type = BND_SPHERE;
  extra_shapes[1].value.sphere.radius = 11.0f;

  body_shapes extra = shapes_write(&world, BRACKET_TWO, extra_shapes, 2);
  bnd_body_shape *first = world.shape_brackets[BRACKET_TWO].shapes;
  bnd_body_shape *middle = world.shape_brackets[BRACKET_TWO].shapes + 64 * 2;
  bnd_body_shape *stored_extra = shapes_get(&world, extra);

  assert(world.shape_brackets[BRACKET_TWO].capacity == 192);
  assert(extra.offset == 128 * 2);
  assert(world.shape_brackets[BRACKET_TWO].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[1] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_TWO].slots[2] == 1);

  assert(first[0].type == BND_BOX);
  assert(first[0].value.box.size.x == 1.0f);
  assert(first[1].type == BND_SPHERE);
  assert(first[1].value.sphere.radius == 200.0f);

  assert(middle[0].type == BND_BOX);
  assert(middle[0].value.box.size.x == 65.0f);
  assert(middle[1].type == BND_SPHERE);
  assert(middle[1].value.sphere.radius == 264.0f);

  assert(memcmp(stored_extra, extra_shapes, sizeof(extra_shapes)) == 0);

  shapes_teardown(&world);
}

void test_shapes_clear_slot_reuses_second_block_slot_with_bracket_alignment(void) {
  bnd_world world = {0};
  world.allocator = bnd_default_allocator();
  world.config.advanced.shapes_brackets_capacity[BRACKET_EIGHT] = 65;

  shapes_init(&world);

  body_shapes entries[66] = {0};
  for (count_t i = 0; i < 66; ++i) {
    bnd_body_shape shapes[8] = {0};

    for (count_t k = 0; k < 6; ++k) {
      shapes[k].type = BND_SPHERE;
      shapes[k].value.sphere.radius = (float)(i * 10 + k + 1);
    }

    entries[i] = shapes_write(&world, BRACKET_EIGHT, shapes, 6);
    assert(entries[i].offset == i * 8);
    assert(entries[i].count == 6);
  }

  assert(entries[64].offset == 64 * 8);
  assert(entries[65].offset == 65 * 8);
  assert(world.shape_brackets[BRACKET_EIGHT].slots[0] == (uint64_t)~0);
  assert(world.shape_brackets[BRACKET_EIGHT].slots[1] == 3);

  shapes_clear_slot(&world, BRACKET_EIGHT, entries[65].offset);

  bnd_body_shape replacement_shapes[8] = {0};
  for (count_t i = 0; i < 5; ++i) {
    replacement_shapes[i].type = BND_BOX;
    replacement_shapes[i].value.box.size.x = (float)(300 + i);
  }

  body_shapes replacement = shapes_write(&world, BRACKET_EIGHT, replacement_shapes, 5);
  bnd_body_shape *stored_replacement = shapes_get(&world, replacement);
  bnd_body_shape *preserved_neighbor = world.shape_brackets[BRACKET_EIGHT].shapes + entries[64].offset;

  assert(replacement.offset == entries[65].offset);
  assert(replacement.count == 5);
  assert(memcmp(stored_replacement, replacement_shapes, 5 * sizeof(bnd_body_shape)) == 0);
  assert(preserved_neighbor[0].type == BND_SPHERE);
  assert(preserved_neighbor[0].value.sphere.radius == 641.0f);

  shapes_teardown(&world);
}

void shapes_tests() {
  TESTS_BEGIN("Body shapes")

  TEST(test_shapes_write_primitive_bracket_uses_second_block)
  TEST(test_shapes_write_four_bracket_keeps_alignment_across_blocks)
  TEST(test_shapes_expand_bracket_preserves_existing_data_after_two_blocks)
  TEST(test_shapes_clear_slot_reuses_second_block_slot_with_bracket_alignment)

  TESTS_END
}
#endif

// ================
//   core.c
// ================



#define MAX_MESSAGE_SIZE 512

#define INVOKE(invocation) \
  e = invocation; \
  if (e.type != BND_OK) { \
    bnd_teardown(world); \
    return e; \
  }

#define ALLOC(buffer, size) \
  buffer = allocator.malloc(4, size); \
  if (buffer == NULL) { \
    bnd_teardown(world); \
    return (bnd_error){ BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc  to allocate memory" }; \
  }

BND_RESULT_FUNC_DECL(world, bnd_world*)
BND_RESULT_FUNC_DECL(v3, bnd_v3)
BND_RESULT_FUNC_DECL(quat, bnd_quat)
BND_RESULT_FUNC_DECL(aabb, bnd_aabb)
BND_RESULT_FUNC_DECL(u32, uint32_t)
BND_RESULT_FUNC_DECL(bool, bool)
BND_RESULT_FUNC_DECL(handle, bnd_body_handle)
BND_RESULT_FUNC_DECL(material, bnd_material_handle)
BND_RESULT_FUNC_DECL(layer, bnd_collision_layer)
BND_RESULT_FUNC_DECL(ptr, void*)

const count_t max_body_index = (count_t)~0 >> 9;
count_t next_world_id;

static void *std_malloc(uint64_t alignment, uint64_t size) {
  // malloc aligns its memory at 16-bytes boundary, which is sufficient for all allocations inside the engine.
  (void) alignment;
  return malloc(size);
}

static void *std_realloc(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size) {
  (void) alignment;
  (void) old_size;
  return realloc(ptr, new_size);
}

static void std_free(void *ptr, uint64_t size) {
  (void) size;
  free(ptr);
}

bnd_allocator bnd_default_allocator(void) {
  return (bnd_allocator){
    .malloc = std_malloc,
    .realloc = std_realloc,
    .free = std_free,
  };
}

count_t bnd_required_memory(const bnd_config *config) {
  count_t size = sizeof(bnd_world);

  count_t common_size = sizeof(bnd_v3)
    + sizeof(bnd_quat)
    + sizeof(body_shapes)
    + sizeof(bnd_aabb)
    + sizeof(bnd_material_handle)
    + sizeof(void*)
    + sizeof(bnd_collision_layer)
    + sizeof(bnd_event_type)
    + sizeof(event_link)
    + sizeof(uint8_t) * 2
    + sizeof(count_t)
    + sizeof(outer_lookup_node)
    + sizeof(count_t);

  count_t dynamic_size = common_size
    + 4 * sizeof(bnd_v3)
    + sizeof(float)
    + 2 * sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(float);

  count_t contact_size = sizeof(contact);
  count_t joint_size = sizeof(bnd_joint) + sizeof(count_t);
  count_t mesh_size = sizeof(bnd_v3) * DEFAULT_VERTEX_PER_MESH
    + sizeof(uint32_t) * DEFAULT_FACE_PER_MESH * 3
    + sizeof(submesh)
    + sizeof(bnd_mesh)
    + sizeof(bnd_m3)
    + sizeof(float)
    + sizeof(bnd_aabb);
  count_t material_size = sizeof(body_material);

  count_t event_size = sizeof(bnd_event) + sizeof(count_t);
  count_t shapes_size = 0;
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count, shapes_count, bracket_capacity;
    shapes_get_bracket_properties(config, i, &blocks_count, &shapes_count, &bracket_capacity);

    shapes_size += shapes_count * sizeof(bnd_body_shape)
      + blocks_count * sizeof(uint64_t);
  }

  count_t polytope_size = polytope_memory_size(config->advanced.epa_max_nodes);

  count_t cache_hash_table_capacity = 1;
  while (cache_hash_table_capacity < config->advanced.contacts_cache.hash_table_capacity) {
    cache_hash_table_capacity *= 2;
  }

  count_t contacts_cache_size = cache_hash_table_capacity * sizeof(uint32_t)
    + config->advanced.contacts_cache.buffer_capacity * sizeof(cache_entry);

  size += (config->memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT) * dynamic_size
    + (config->memory.statics_capacity + EPHEMERAL_BODIES_COUNT) * common_size
    + config->memory.contacts_capacity * contact_size
    + config->memory.joints_capacity * joint_size
    + config->memory.meshes_capacity * mesh_size
    + config->memory.events_capacity * event_size
    + config->memory.materials_capacity * material_size
    + shapes_size
    + polytope_size
    + contacts_cache_size;

  // Alignment
  size += 9 * 7; // 8-bytes for world, shapes slots, EPA polytope, custom data and the cache entries buffer
  size += 46 * 3; // 4-bytes for the rest of the buffers

  return size;
}

static bnd_error init_commons(common_data *data, count_t capacity, bnd_allocator allocator) {
  data->capacity = capacity;
  data->count = 0;
  data->free_count = 0;
  data->first_outer_node = max_body_index;

  count_t total_capacity = capacity + EPHEMERAL_BODIES_COUNT;
  ALLOC_BUFFER4(data->positions, sizeof(bnd_v3) * total_capacity);
  ALLOC_BUFFER4(data->rotations, sizeof(bnd_quat) * total_capacity);
  ALLOC_BUFFER4(data->shapes, sizeof(body_shapes) * total_capacity);
  ALLOC_BUFFER4(data->aabbs, sizeof(bnd_aabb) * total_capacity);
  ALLOC_BUFFER4(data->materials, sizeof(bnd_material_handle) * total_capacity);
  ALLOC_BUFFER8(data->custom_data, sizeof(void*) * total_capacity);
  ALLOC_BUFFER1(data->flags, sizeof(uint8_t) * total_capacity);
  ALLOC_BUFFER1(data->collision_layers, sizeof(bnd_collision_layer) * total_capacity);
  ALLOC_BUFFER4(data->event_masks, sizeof(bnd_event_type) * total_capacity);
  ALLOC_BUFFER4(data->event_links, sizeof(event_link) * total_capacity);
  ALLOC_BUFFER4(data->free_list, sizeof(count_t) * total_capacity);
  ALLOC_BUFFER1(data->generations, sizeof(uint8_t) * total_capacity);
  ALLOC_BUFFER4(data->outer_lookup, sizeof(outer_lookup_node) * total_capacity);
  ALLOC_BUFFER4(data->inner_lookup, sizeof(count_t) * total_capacity);

  return OK;
}

static void teardown_commons(common_data *data, bnd_allocator allocator) {
  count_t total_capacity = data->capacity + EPHEMERAL_BODIES_COUNT;

  allocator.free(data->positions, total_capacity * sizeof(bnd_v3));
  allocator.free(data->rotations, total_capacity * sizeof(bnd_quat));
  allocator.free(data->shapes, total_capacity * sizeof(body_shapes));
  allocator.free(data->aabbs, total_capacity * sizeof(bnd_aabb));
  allocator.free(data->materials, total_capacity * sizeof(bnd_material_handle));
  allocator.free(data->flags, total_capacity * sizeof(uint8_t));
  allocator.free(data->custom_data, total_capacity * sizeof(void*));
  allocator.free(data->collision_layers, total_capacity * sizeof(bnd_collision_layer));
  allocator.free(data->event_masks, total_capacity * sizeof(bnd_event_type));
  allocator.free(data->event_links, total_capacity * sizeof(event_link));
  allocator.free(data->free_list, total_capacity * sizeof(count_t));
  allocator.free(data->generations, total_capacity * sizeof(uint8_t));
  allocator.free(data->outer_lookup, total_capacity * sizeof(outer_lookup_node));
  allocator.free(data->inner_lookup, total_capacity * sizeof(count_t));
}

bnd_config bnd_default_config(void) {
  return (bnd_config){
    .simulation = {
      .gravity = (bnd_v3){0, -9.81f, 0},
      .linear_drag = 0.95f,
      .angular_drag = 0.8f,
      .bounciness = 0.2f,
      .friction = 0.9f,
      .sleep_base_bias = 0.5f,
      .sleep_threshold = 0.3f,
      .min_bounce_velocity = 0.25f,
    },
    .memory = {
      .dynamics_capacity = 32,
      .statics_capacity = 8,
      .contacts_capacity = 64,
      .joints_capacity = 64,
      .meshes_capacity = 32,
      .events_capacity = 128,
      .materials_capacity = 8,
    },
    .advanced = {
      .max_gjk_iterations = 100,
      .epa_tolerance = 0.01f,
      .epa_max_nodes = 512,
      .resolution_attempts_factor = 15,
      .penetration_epsilon = 0.01f,
      .velocity_epsilon = 0.01f,
      .shapes_brackets_capacity = {64, 1, 1, 1, 1},
      .contacts_cache = {
        .max_age = 3,
        .hash_table_capacity = 256,
        .buffer_capacity = 64,
        .feature_distance_threshold = 0.02f,
        .separation_threshold = 0.05f,
      }
    },
  };
}

static bnd_error bnd_init_internal(bnd_world *world, bnd_config config, bnd_allocator allocator) {
  world->allocator = allocator;
  world->config = config;
  world->id = next_world_id++; // TODO: make this thread-safe.

  bnd_error e;
  INVOKE(init_commons((common_data *)&world->dynamics, config.memory.dynamics_capacity, allocator))
  INVOKE(init_commons((common_data *)&world->statics, config.memory.statics_capacity, allocator))

  const count_t vectors = sizeof(bnd_v3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t floats = sizeof(float) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t matrices = sizeof(bnd_m3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);

  world->statics.dirty = false;

  ALLOC(world->dynamics.forces, vectors);
  ALLOC(world->dynamics.torques, vectors);
  ALLOC(world->dynamics.impulses, vectors);
  ALLOC(world->dynamics.angular_impulses, vectors);
  ALLOC(world->dynamics.accelerations, vectors);

  ALLOC(world->dynamics.inv_masses, floats);
  ALLOC(world->dynamics.velocities, vectors);
  ALLOC(world->dynamics.angular_momenta, vectors);
  ALLOC(world->dynamics.inv_inertia_tensors, matrices);
  ALLOC(world->dynamics.inv_intertias, matrices);
  ALLOC(world->dynamics.motion_avgs, floats);

  world->matrix.matrix[0] = 1;
  for(count_t i = 1; i < MAX_COLLISION_LAYERS; ++i) {
    world->matrix.matrix[i] = 0;
  }

  world->matrix.layers_available = 1;

  world->dynamics.awake_count = 0;
  world->generation = 0;
  world->age = 0;

  INVOKE(contacts_init(world))
  INVOKE(joints_init(world))
  INVOKE(shapes_init(world))
  INVOKE(meshes_init(world))
  INVOKE(events_init(world))
  INVOKE(epa_init(world))
  INVOKE(materials_init(world))

  world->epa_debug = NULL;

  return OK;
}

bnd_world *bnd_init(bnd_config config) {
  bnd_allocator allocator = bnd_default_allocator();
  bnd_world *world = allocator.malloc(8, sizeof(bnd_world));

  // The error is ignored here intentionally. With default allocator it *should not* fail, so we'd rather
  // provide a cleaner API by betting on a happy path.
  bnd_init_internal(world, config, allocator);
  return world;
}

bnd_result_world bnd_init_with_allocator(bnd_config config, bnd_allocator allocator) {
  if (allocator.malloc == NULL) {
    return BND_RESULT_ERR(world, BND_ERROR_INVALID_ALLOCATOR, "Allocator must define a malloc function");
  }

  bnd_world *world = allocator.malloc(8, sizeof(bnd_world));
  if (world == NULL) {
    return BND_RESULT_ERR(world, OOM_ERROR.type, OOM_ERROR.message);
  }

  memset(world, 0, sizeof(bnd_world));

  bnd_error e = bnd_init_internal(world, config, allocator);
  return (bnd_result_world) { e, world };
}

void bnd_teardown(bnd_world *world) {
  if (world->allocator.free == NULL) {
    return;
  }

  teardown_commons((common_data *)&world->dynamics, world->allocator);
  teardown_commons((common_data *)&world->statics, world->allocator);

  count_t dynamics_total_capacity = world->dynamics.capacity + EPHEMERAL_BODIES_COUNT;

  world->allocator.free(world->dynamics.forces, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.torques, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.impulses, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_impulses, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.accelerations, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.inv_masses, dynamics_total_capacity * sizeof(float));
  world->allocator.free(world->dynamics.velocities, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.angular_momenta, dynamics_total_capacity * sizeof(bnd_v3));
  world->allocator.free(world->dynamics.inv_inertia_tensors, dynamics_total_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.inv_intertias, dynamics_total_capacity * sizeof(bnd_m3));
  world->allocator.free(world->dynamics.motion_avgs, dynamics_total_capacity * sizeof(float));

  shapes_teardown(world);
  joints_teardown(world);
  contacts_teardown(world);
  meshes_teardown(world);
  events_teardown(world);
  epa_teardown(world);

  world->allocator.free(world->materials.values, sizeof(body_material) * world->materials.capacity);
  world->allocator.free(world, sizeof(bnd_world));
}

count_t ephemeral_body_index(const common_data *data) {
  return data->capacity;
}


// ================
//   epa.c
// ================


#define NIL 0
#define EPA_MAX_ATTEMPTS 128
#define VISIBLE_NODES_STACK_SIZE 16

#define polytope_for_each_node(p, index, type)                                                                         \
  for (uint16_t index = p->last_nodes[type]; index != NIL; index = p->nodes[index].prev)

typedef enum {
  EPA_STATUS_OK,
  EPA_STATUS_NOT_RUN,
  EPA_STATUS_CONVERGED,
  EPA_STATUS_INVALID_POLYTOPE,
  EPA_STATUS_EXPANSION_FAILED,
  EPA_STATUS_ITERATION_LIMIT,
} epa_status;

const float visibility_epsilon = 0.25;

typedef void (*epa_iteration_observer)(const epa_polytope *polytope, body_support support_point, count_t iteration, void *user_data);

typedef struct {
  epa_status status;
  count_t iterations;
} epa_run_result;

static uint32_t polytope_flags_size(uint16_t max_nodes) {
  return max_nodes + 1 + (max_nodes % 2 == 0);
}

uint32_t polytope_memory_size(uint16_t max_nodes) {
  return (max_nodes + 1) * sizeof(epa_polytope_node) + polytope_flags_size(max_nodes) + max_nodes * sizeof(uint16_t);
}

static void polytope_clear(epa_polytope *polytope) {
  polytope->node_count = 0;
  polytope->free_count = 0;

  memset(polytope->last_nodes, 0, EPA_NODE_TYPE_COUNT * sizeof(uint16_t));
  memset(polytope->flags, 0, polytope_flags_size(polytope->max_nodes));

  polytope->nearest = NIL;
  polytope->nearest_distance = FLT_MAX;
}

static uint16_t polytope_free_index(epa_polytope *polytope) {
  if (polytope->free_count > 0) {
    return polytope->free_list[--polytope->free_count];
  }

  if (polytope->node_count < polytope->max_nodes) {
    return polytope->node_count++ + 1;
  }

  return NIL;
}

static void polytope_add_node(epa_polytope *polytope, epa_polytope_node *node, uint16_t index) {
  uint16_t last_node_index = polytope->last_nodes[node->type];

  node->prev = last_node_index;

  polytope->last_nodes[node->type] = index;
}

static void polytope_remove_node(epa_polytope *polytope, uint16_t index) {
  epa_polytope_node node = polytope->nodes[index];

  uint16_t node_index = polytope->last_nodes[node.type];
  epa_polytope_node *current_node = &polytope->nodes[node_index];

  if (node_index == index) {
    polytope->last_nodes[node.type] = current_node->prev;
  } else {
    while (current_node->prev != index) {
      node_index = current_node->prev;
      current_node = &polytope->nodes[node_index];
    }

    current_node->prev = node.prev;
  }

  polytope->free_list[polytope->free_count++] = index;
}

static void polytope_attach_edge(epa_polytope *polytope, uint16_t edge, uint16_t vertex) {
  epa_polytope_node *vertex_node = &polytope->nodes[vertex];
  if (vertex_node->value.vertex.first_attached_edge == NIL) {
    vertex_node->value.vertex.first_attached_edge = edge;
  } else {
    uint16_t attached_edge = vertex_node->value.vertex.first_attached_edge;
    epa_polytope_node *edge_node;
    int i;
    do {
      edge_node = &polytope->nodes[attached_edge];
      i = edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
      attached_edge = edge_node->value.edge.next_attached_edges[i];
    } while (attached_edge != NIL);

    edge_node->value.edge.next_attached_edges[i] = edge;
  }
}

static void polytope_attach_face(epa_polytope *polytope, uint16_t face, uint16_t edge) {
  epa_polytope_node *edge_node = &polytope->nodes[edge];
  for (count_t i = 0; i < 2; ++i) {
    if (edge_node->value.edge.attached_faces[i] == NIL) {
      edge_node->value.edge.attached_faces[i] = face;
      break;
    }
  }
}

static void polytope_detach_face(epa_polytope *polytope, uint16_t face, uint16_t edge) {
  epa_polytope_node *edge_node = &polytope->nodes[edge];

  uint16_t *attached_faces = edge_node->value.edge.attached_faces;
  for (count_t i = 0; i < 2; ++i) {
    if (attached_faces[i] == face) {
      attached_faces[i] = NIL;
    }
  }
}

static void polytope_detach_edge(epa_polytope *polytope, uint16_t edge, uint16_t vertex) {
  epa_polytope_node *vertex_node = &polytope->nodes[vertex];

  uint16_t next_edge = vertex_node->value.vertex.first_attached_edge;
  epa_polytope_node *next_edge_node = &polytope->nodes[next_edge];
  int i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
  if (next_edge == edge) {
    vertex_node->value.vertex.first_attached_edge = next_edge_node->value.edge.next_attached_edges[i];
    if (vertex_node->value.vertex.first_attached_edge == NIL) {
      polytope->flags[vertex] |= EPA_FLAG_FOR_REMOVAL;
    }
    return;
  }

  next_edge = next_edge_node->value.edge.next_attached_edges[i];

  while (next_edge != edge) {
    next_edge_node = &polytope->nodes[next_edge];
    i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
    next_edge = next_edge_node->value.edge.next_attached_edges[i];
  }

  epa_polytope_node *current_node = next_edge_node;
  int current_i = i;

  next_edge_node = &polytope->nodes[next_edge];
  i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;

  current_node->value.edge.next_attached_edges[current_i] = next_edge_node->value.edge.next_attached_edges[i];
}

static void polytope_get_edge_verticies(const epa_polytope *polytope, uint16_t edge, bnd_v3 *v1, bnd_v3 *v2) {
  epa_polytope_node node = polytope->nodes[edge];
  uint16_t vertex_1 = node.value.edge.verticies[0];
  uint16_t vertex_2 = node.value.edge.verticies[1];

  *v1 = polytope->nodes[vertex_1].value.vertex.v.p;
  *v2 = polytope->nodes[vertex_2].value.vertex.v.p;
}

static void polytope_get_face_verticies(const epa_polytope *polytope, uint16_t face, bnd_v3 *v1, bnd_v3 *v2, bnd_v3 *v3) {
  epa_polytope_node node = polytope->nodes[face];
  uint16_t e1 = node.value.face.edges[0];
  uint16_t e2 = node.value.face.edges[1];
  uint16_t *edge_verts_1 = polytope->nodes[e1].value.edge.verticies;
  uint16_t *edge_verts_2 = polytope->nodes[e2].value.edge.verticies;

  *v1 = polytope->nodes[edge_verts_1[0]].value.vertex.v.p;
  *v2 = polytope->nodes[edge_verts_1[1]].value.vertex.v.p;

  *v3 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
    ? polytope->nodes[edge_verts_2[1]].value.vertex.v.p
    : polytope->nodes[edge_verts_2[0]].value.vertex.v.p;
}

static uint16_t polytope_add_vertex(epa_polytope *polytope, body_support p) {
  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_VERTEX;
  node->value.vertex.v = p;
  node->value.vertex.first_attached_edge = NIL;

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_edge(epa_polytope *polytope, uint16_t v1, uint16_t v2) {
  if (v1 == NIL || v2 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_EDGE;
  node->value.edge.verticies[0] = v1;
  node->value.edge.verticies[1] = v2;

  bnd_v3 closest;
  node->distance = sqr_distance_to_line_segment(bnd_v3_zero(), polytope->nodes[v1].value.vertex.v.p, polytope->nodes[v2].value.vertex.v.p, &closest);
  node->normal = closest;

  memset(node->value.edge.attached_faces, 0, 2 * sizeof(uint16_t));
  memset(node->value.edge.next_attached_edges, 0, 2 * sizeof(uint16_t));

  polytope_attach_edge(polytope, index, v1);
  polytope_attach_edge(polytope, index, v2);

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_face(epa_polytope *polytope, uint16_t e1, uint16_t e2, uint16_t e3) {
  if (e1 == NIL || e2 == NIL || e3 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_FACE;
  node->value.face.edges[0] = e1;
  node->value.face.edges[1] = e2;
  node->value.face.edges[2] = e3;

  bnd_v3 v1, v2, v3, p;
  polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

  if (bnd_v3_distancesqr(v1, v2) < EPSILON || bnd_v3_distancesqr(v2, v3) < EPSILON || bnd_v3_distancesqr(v3, v1) < EPSILON) {
    polytope->free_list[polytope->free_count++] = index;
    return NIL;
  }

  bnd_v3 normal = bnd_v3_cross(bnd_v3_sub(v3, v1), bnd_v3_sub(v2, v1));
  if (bnd_v3_dot(normal, v1) < 0) {
    normal = bnd_v3_negate(normal);
  }

  node->distance = sqr_distance_to_triangle(bnd_v3_zero(), v1, v2, v3, &p);
  node->normal = normal;

  polytope_attach_face(polytope, index, e1);
  polytope_attach_face(polytope, index, e2);
  polytope_attach_face(polytope, index, e3);

  polytope_add_node(polytope, node, index);

  return index;
}

static void polytope_remove_face(epa_polytope *polytope, uint16_t face) {
  if (face == NIL) {
    return;
  }

  epa_polytope_node *face_node = &polytope->nodes[face];
  uint16_t e1 = face_node->value.face.edges[0];
  uint16_t e2 = face_node->value.face.edges[1];
  uint16_t e3 = face_node->value.face.edges[2];

  polytope_detach_face(polytope, face, e1);
  polytope_detach_face(polytope, face, e2);
  polytope_detach_face(polytope, face, e3);

  polytope_remove_node(polytope, face);
}

static void polytope_remove_edge(epa_polytope *polytope, uint16_t edge) {
  if (edge == NIL) {
    return;
  }

  epa_polytope_node *edge_node = &polytope->nodes[edge];
  if (edge_node->value.edge.attached_faces[0] != NIL || edge_node->value.edge.attached_faces[1] != NIL) {
    return;
  }

  uint16_t v1 = edge_node->value.edge.verticies[0];
  uint16_t v2 = edge_node->value.edge.verticies[1];

  polytope_detach_edge(polytope, edge, v1);
  polytope_detach_edge(polytope, edge, v2);

  polytope_remove_node(polytope, edge);
}

static void polytope_remove_vertex(epa_polytope *polytope, uint16_t vertex) {
  if (vertex == NIL) {
    return;
  }

  polytope_remove_node(polytope, vertex);
}

static void polytope_clear_flags(epa_polytope *polytope) {
  memset(polytope->flags, 0, polytope->node_count + 1);
}

static void polytope_update_nearest(epa_polytope *polytope) {
  polytope->nearest_distance = FLT_MAX;

  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    epa_polytope_node node = polytope->nodes[index];

    float distance = node.distance;
    if (distance < polytope->nearest_distance) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    epa_polytope_node node = polytope->nodes[index];
    epa_polytope_node current_nearest = polytope->nodes[polytope->nearest];

    float distance = node.distance;
    if (distance < polytope->nearest_distance ||
        (distance == polytope->nearest_distance && current_nearest.type == EPA_NODE_FACE)) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }
  }
}

static bool polytope_is_face_visible(const epa_polytope_node *face, bnd_v3 support_point) {
  return bnd_v3_dot(bnd_v3_normalize(face->normal), bnd_v3_normalize(support_point)) > visibility_epsilon;
}

static bool polytope_contains_vertex(const epa_polytope *polytope, bnd_v3 point) {
  polytope_for_each_node(polytope, index, EPA_NODE_VERTEX) {
    if (bnd_v3_distancesqr(polytope->nodes[index].value.vertex.v.p, point) < EPSILON) {
      return true;
    }
  }

  return false;
}

static bool polytope_from_simplex(epa_polytope *polytope, const simplex *s) {
  polytope_clear(polytope);

  uint16_t verts[4];
  uint16_t edges[6];

  for (uint32_t i = 0; i < 4; ++i) {
    verts[i] = polytope_add_vertex(polytope, s->points[i]);
  }

  edges[0] = polytope_add_edge(polytope, verts[0], verts[1]);
  edges[1] = polytope_add_edge(polytope, verts[1], verts[2]);
  edges[2] = polytope_add_edge(polytope, verts[2], verts[0]);
  edges[3] = polytope_add_edge(polytope, verts[1], verts[3]);
  edges[4] = polytope_add_edge(polytope, verts[3], verts[2]);
  edges[5] = polytope_add_edge(polytope, verts[0], verts[3]);

  if (polytope_add_face(polytope, edges[0], edges[1], edges[2]) == NIL ||
      polytope_add_face(polytope, edges[0], edges[3], edges[5]) == NIL ||
      polytope_add_face(polytope, edges[2], edges[5], edges[4]) == NIL ||
      polytope_add_face(polytope, edges[1], edges[4], edges[3]) == NIL) {
    return false;
  }

  polytope_update_nearest(polytope);
  polytope_clear_flags(polytope);

  return true;
}

static void epa_invalid_contact(body_support p, contact *contact) {
  contact->point = bnd_v3_scale(bnd_v3_add(p.p1.point, p.p2.point), 0.5f);
  contact->normal = bnd_v3_up();
  contact->depth = 0.1f;

  contact->features.witness_a = p.p1.point;
  contact->features.witness_b = p.p2.point;
  contact->features.normal = contact->normal;
}

static void epa_calculate_contact(const epa_polytope *polytope, contact *contact) {
  epa_polytope_node node = polytope->nodes[polytope->nearest];
  bnd_v3 p1, p2;
  if (node.type == EPA_NODE_FACE) {
    uint16_t e1 = node.value.face.edges[0];
    uint16_t e2 = node.value.face.edges[1];
    uint16_t *edge_verts_1 = polytope->nodes[e1].value.edge.verticies;
    uint16_t *edge_verts_2 = polytope->nodes[e2].value.edge.verticies;

    body_support v0 = polytope->nodes[edge_verts_1[0]].value.vertex.v;
    body_support v1 = polytope->nodes[edge_verts_1[1]].value.vertex.v;

    body_support v2 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
      ? polytope->nodes[edge_verts_2[1]].value.vertex.v
      : polytope->nodes[edge_verts_2[0]].value.vertex.v;

    bnd_v3 barycenter = bnd_v3_barycentric(bnd_v3_zero(), v0.p, v1.p, v2.p);
    p1 = bnd_v3_add(bnd_v3_scale(v0.p1.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v1.p1.point, barycenter.y), bnd_v3_scale(v2.p1.point, barycenter.z)));
    p2 = bnd_v3_add(bnd_v3_scale(v0.p2.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v1.p2.point, barycenter.y), bnd_v3_scale(v2.p2.point, barycenter.z)));
  } else if (node.type == EPA_NODE_EDGE) {
    body_support v0 = polytope->nodes[node.value.edge.verticies[0]].value.vertex.v;
    body_support v1 = polytope->nodes[node.value.edge.verticies[1]].value.vertex.v;

    bnd_v3 d = bnd_v3_sub(v1.p, v0.p);
    float t = -1.0f * bnd_v3_dot(v0.p, d) / bnd_v3_lensqr(d);
    p1 = bnd_v3_add(v0.p1.point, bnd_v3_scale(bnd_v3_sub(v1.p1.point, v0.p1.point), t));
    p2 = bnd_v3_add(v0.p2.point, bnd_v3_scale(bnd_v3_sub(v1.p2.point, v0.p2.point), t));
  } else {
    return;
  }

  contact->point = bnd_v3_scale(bnd_v3_add(p1, p2), 0.5f);
  contact->depth = sqrtf(node.distance);

  float length = bnd_v3_len(node.normal);
  if (length > EPSILON) {
    contact->normal = bnd_v3_scale(node.normal, -1.0f / length);
  } else {
    contact->normal = bnd_v3_up();
  }

  contact->features.witness_a = p1;
  contact->features.witness_b = p2;
  contact->features.normal = contact->normal;
}

static void mark_edge_for_removal(epa_polytope *polytope, uint16_t edge_index, body_support p, uint16_t *stack, uint16_t *stack_ptr) {
  epa_polytope_node *edge_node = &polytope->nodes[edge_index];

  count_t visible_count = 0;
  for (count_t j = 0; j < 2; ++j) {
    uint16_t adjasent_face_index = edge_node->value.edge.attached_faces[j];
    const epa_polytope_node *adjasent_face_node = &polytope->nodes[adjasent_face_index];

    if (polytope->flags[adjasent_face_index] & EPA_FLAG_FOR_REMOVAL) {
      visible_count += 1;
    } else if (polytope_is_face_visible(adjasent_face_node, p.p)) {
      visible_count += 1;
      polytope->flags[adjasent_face_index] |= EPA_FLAG_FOR_REMOVAL;
      stack[*stack_ptr] = adjasent_face_index;
      *stack_ptr += 1;
    }
  }

  if (visible_count == 2) {
    polytope->flags[edge_index] |= EPA_FLAG_FOR_REMOVAL;
  } else if (visible_count == 1) {
    polytope->flags[edge_index] |= EPA_FLAG_BORDER_EDGE;
  }
}

static void epa_update_visible_nodes(epa_polytope *polytope, body_support p) {
  uint16_t stack_ptr = 1;
  uint16_t stack[VISIBLE_NODES_STACK_SIZE] = { polytope->nearest };

  epa_polytope_node *node = &polytope->nodes[polytope->nearest];
  polytope->flags[polytope->nearest] |= EPA_FLAG_FOR_REMOVAL;

  while (stack_ptr > 0) {
    uint16_t node_index = stack[--stack_ptr];
    node = &polytope->nodes[node_index];

    if (node->type == EPA_NODE_FACE) {
      for (count_t i = 0; i < 3; ++i) {
        uint16_t edge_index = node->value.face.edges[i];
        mark_edge_for_removal(polytope, edge_index, p, stack, &stack_ptr);
      }
    } else if (node->type == EPA_NODE_EDGE) {
      mark_edge_for_removal(polytope, node_index, p, stack, &stack_ptr);
    }
  }
}

static bool epa_expand_polytope(epa_polytope *polytope, body_support p) {
  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_face(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_edge(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_VERTEX) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_vertex(polytope, index);
    }
  }

  uint16_t new_vertex = polytope_add_vertex(polytope, p);

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    epa_polytope_node *edge_node = &polytope->nodes[index];
    if ((polytope->flags[index] & EPA_FLAG_BORDER_EDGE) == 0) {
      continue;
    }

    uint16_t edge_index = index;
    uint16_t first_connected_vertex_index = edge_node->value.edge.verticies[0];
    uint16_t first_new_edge = polytope_add_edge(polytope, new_vertex, first_connected_vertex_index);
    uint16_t prev_edge = first_new_edge;

    uint16_t connected_vertex_index = edge_node->value.edge.verticies[1];
    while (connected_vertex_index != first_connected_vertex_index) {
      /**
       * Sometimes, in rare cases, this routine might add a duplicate edge - the one which is identical to some other
       * one. I saw this happen when spawning objects at the same position, so that they overlap. Need to investigate
       * this further, but for now in this case we just bail out and return some bogus contact data.
       */
      uint16_t new_edge = polytope_add_edge(polytope, new_vertex, connected_vertex_index);
      if (polytope_add_face(polytope, prev_edge, new_edge, edge_index) == NIL) {
        return false;
      }

      epa_polytope_node *connected_vertex_node = &polytope->nodes[connected_vertex_index];
      uint16_t attached_edge_index = connected_vertex_node->value.vertex.first_attached_edge;
      while (attached_edge_index != NIL) {
        epa_polytope_node attached_edge_node = polytope->nodes[attached_edge_index];
        count_t i = attached_edge_node.value.edge.verticies[0] == connected_vertex_index ? 0 : 1;

        if (attached_edge_index == new_edge || attached_edge_index == edge_index) {
          attached_edge_index = attached_edge_node.value.edge.next_attached_edges[i];
          continue;
        }

        if (polytope->flags[attached_edge_index] & EPA_FLAG_BORDER_EDGE) {
          edge_index = attached_edge_index;
          connected_vertex_index = attached_edge_node.value.edge.verticies[1 - i];
          break;
        }

        attached_edge_index = attached_edge_node.value.edge.next_attached_edges[i];
      }

      prev_edge = new_edge;
    }

    if (polytope_add_face(polytope, first_new_edge, prev_edge, edge_index) == NIL) {
      return false;
    }

    break;
  }

  polytope_update_nearest(polytope);
  polytope_clear_flags(polytope);

  return true;
}

bnd_error epa_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;
  epa_polytope *polytope = &world->epa_polytope;

  memset(polytope, 0, sizeof(epa_polytope));

  uint16_t max_nodes = world->config.advanced.epa_max_nodes;
  ALLOC_BUFFER8(polytope->nodes, (max_nodes + 1) * sizeof(epa_polytope_node));
  ALLOC_BUFFER1(polytope->flags, polytope_flags_size(max_nodes));
  ALLOC_BUFFER2(polytope->free_list, max_nodes * sizeof(uint16_t));

  polytope->max_nodes = max_nodes;
  polytope->nearest_distance = FLT_MAX;

  return OK;
}

void epa_teardown(bnd_world *world) {
  epa_polytope *polytope = &world->epa_polytope;
  world->allocator.free(polytope->nodes, (polytope->max_nodes + 1) * sizeof(epa_polytope_node));
  world->allocator.free(polytope->flags, polytope_flags_size(polytope->max_nodes));
  world->allocator.free(polytope->free_list, polytope->max_nodes * sizeof(uint16_t));
}

static epa_status epa_run(epa_polytope *polytope, const collision_detection_context *ctx, body_support *support_point, float tolerance) {
  epa_polytope_node closest_node = polytope->nodes[polytope->nearest];
  bnd_v3 direction = closest_node.normal;
  bnd_v3 normal_direction = bnd_v3_normalize(direction);

  *support_point = support(ctx, normal_direction);

  float support_distance = bnd_v3_dot(normal_direction, support_point->p);
  float face_distance = sqrtf(closest_node.distance);
  if (support_distance - face_distance < tolerance) {
    return EPA_STATUS_CONVERGED;
  }

  float distance;
  bnd_v3 a, b, c, closest;
  if (closest_node.type == EPA_NODE_FACE) {
    polytope_get_face_verticies(polytope, polytope->nearest, &a, &b, &c);
    distance = sqr_distance_to_triangle(support_point->p, a, b, c, &closest);
  } else if (closest_node.type == EPA_NODE_EDGE) {
    polytope_get_edge_verticies(polytope, polytope->nearest, &a, &b);
    distance = sqr_distance_to_line_segment(support_point->p, a, b, &closest);
  } else {
    return EPA_STATUS_INVALID_POLYTOPE;
  }

  if (distance < tolerance) {
    return EPA_STATUS_CONVERGED;
  }

  // TODO don't check this every time, just when producing a zero-length edge.
  if (polytope_contains_vertex(polytope, support_point->p)) {
    return EPA_STATUS_CONVERGED;
  }

  epa_update_visible_nodes(polytope, *support_point);

  if (!epa_expand_polytope(polytope, *support_point)) {
    return EPA_STATUS_EXPANSION_FAILED;
  }

  return EPA_STATUS_OK;
}


count_t epa_get_contact(bnd_world *world, const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact) {
  PROFILER_FUNCTION_START

  epa_polytope *polytope = &world->epa_polytope;
  body_support support_point = simplex->points[0];
  if (!polytope_from_simplex(polytope, simplex)) {
    epa_invalid_contact(support_point, contact);
    PROFILER_FUNCTION_END
    return 0;
  }

  count_t attempts = 1;
  for (; attempts <= EPA_MAX_ATTEMPTS; ++attempts) {
    epa_status result = epa_run(polytope, ctx, &support_point, tolerance);
    switch(result) {
      case EPA_STATUS_OK:
        continue;

      case EPA_STATUS_CONVERGED:
        epa_calculate_contact(polytope, contact);
        PROFILER_FUNCTION_END
        return attempts;

      default:
        epa_invalid_contact(support_point, contact);
        PROFILER_FUNCTION_END
        return attempts;
    }
  }

  PROFILER_FUNCTION_END
  return attempts;
}

#if defined(BND_DEBUG)

static void epa_debug_render_iteration(const epa_polytope *polytope, body_support support_point, bnd_debug_draw_epa_callbacks callbacks, void *user_data) {
  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    const epa_polytope_node *face = &polytope->nodes[index];
    const bnd_v3 normal = face->normal;

    bnd_v3 a, b, c;
    polytope_get_face_verticies(polytope, index, &a, &b, &c);

    bnd_v3 winding = bnd_v3_cross(bnd_v3_sub(b, a), bnd_v3_sub(c, a));
    if (bnd_v3_dot(winding, normal) < 0) {
      bnd_v3 tmp = b;
      b = c;
      c = tmp;
    }

    bnd_debug_epa_flags flags = DEBUG_EPA_NONE;
    if (index == polytope->nearest) {
      flags |= DEBUG_EPA_FACE_NEAREST;
    }
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      flags |= DEBUG_EPA_FACE_REMOVED;
    }

    if (callbacks.draw_face != NULL) {
      callbacks.draw_face(a, b, c, flags, user_data);
    }

    if (callbacks.draw_normal != NULL) {
      flags |= DEBUG_EPA_NORMAL_FACE;
      bnd_v3 center = bnd_v3_scale(bnd_v3_add(bnd_v3_add(a, b), c), 0.333);
      callbacks.draw_normal(center, bnd_v3_normalize(normal), flags, user_data);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    const epa_polytope_node *edge = &polytope->nodes[index];

    bnd_v3 a, b;
    polytope_get_edge_verticies(polytope, index, &a, &b);

    bnd_debug_epa_flags flags = DEBUG_EPA_NORMAL_EDGE;
    if (index == polytope->nearest) {
      flags |= DEBUG_EPA_NORMAL_NEAREST;
    }

    if (callbacks.draw_normal != NULL) {
      bnd_v3 center = bnd_v3_scale(bnd_v3_add(a, b), 0.5);
      callbacks.draw_normal(center, bnd_v3_normalize(edge->normal), flags, user_data);
    }
  }

  if (callbacks.draw_support != NULL) {
    callbacks.draw_support(support_point.p, user_data);
  }
}

bool epa_debug_draw(bnd_world *world, const epa_debug_status *debug_status, bnd_debug_draw_epa_callbacks callbacks, void *user_data) {
  epa_polytope *polytope = &world->epa_polytope;
  if (!polytope_from_simplex(polytope, &debug_status->s)) {
    return false;
  }

  body_support support_point = debug_status->s.points[0];
  if (debug_status->target_iteration == 0) {
    epa_debug_render_iteration(polytope, support_point, callbacks, user_data);
    return true;
  }

  epa_status status = EPA_STATUS_OK;
  for (int iteration = 1; iteration < EPA_MAX_ATTEMPTS; ++iteration) {
    if (iteration > debug_status->target_iteration) {
      return false;
    }

    status = epa_run(polytope, &debug_status->ctx, &support_point, world->config.advanced.epa_tolerance);
    if (status != EPA_STATUS_OK && status != EPA_STATUS_CONVERGED) {
      return false;
    }

    if (iteration == debug_status->target_iteration) {
      bnd_v3 direction = bnd_v3_normalize(polytope->nodes[polytope->nearest].normal);
      body_support next_support = support(&debug_status->ctx, direction);

      epa_debug_render_iteration(polytope, next_support, callbacks, user_data);
      return true;
    }
  }

  return false;
}

#endif
