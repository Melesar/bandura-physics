#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <CUnit/CUnit.h>

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
  BND_EVENT_COLLISION_ENTER  = 1,  /**< The body started touching another body. */
  BND_EVENT_COLLISION        = 2,  /**< The body stays in contact with another body. */
  BND_EVENT_COLLISION_FINISH = 4,  /**< The body stopped touching another body */

  BND_EVENT_TRIGGER_ENTER  = 8,    /**< The body started intersecting a trigger. */
  BND_EVENT_TRIGGER        = 16,   /**< The body intersects a trigger */
  BND_EVENT_TRIGGER_FINISH = 32,   /**< The body stopped intersecting a trigger */
} bnd_event_type;

typedef enum {
  BND_JOINT_TYPE_DISTANCE,
} bnd_joint_type;

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

typedef struct {
  bnd_joint_type type;
  uint32_t world_id;
  uint32_t id;
} bnd_joint_handle;

typedef struct {
  bnd_v3 local_anchor_a;
  bnd_v3 local_anchor_b;
  float min_distance;
  float max_distance;
} bnd_distance_joint_config;
 
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
  uint32_t dynamics_capacity;                  /**< Initial dynamic-body capacity. */
  uint32_t statics_capacity;                   /**< Initial static-body capacity. */
  uint32_t contacts_capacity;                  /**< Initial contact capacity. */
  uint32_t joints_capacity;                    /**< Initial joint capacity. */
  uint32_t meshes_capacity;                    /**< Initial imported-mesh capacity. */
  uint32_t events_capacity;                    /**< Initial event capacity. */
  uint32_t materials_capacity;                 /**< Initial material capacity. */
  uint32_t hash_table_capacity;                /**< Initial capacity of the broad-phase hash table */
  uint32_t shapes_brackets_capacity[5];        /**< Capacities for compound-shape storage brackets. */
  uint64_t internal_allocator_capacity_bytes;  /**< How much memory the engine can allocate internally */
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
} bnd_config_simulation;

typedef struct {
  uint32_t iterations_count;           /**< Higher number means higher precision and stability but more time spent by the solver. */
  float    baumgarde_coefficient;      /**< How hard the solver tryies to push away penetrating bodies. */
  float    linear_slop;                /**< Minimum penetration depth for the solver to act upon. */
  float    max_baumgarde_velocity;     /**< Maximum separation velocity to apply to penetrating bodies. */
  bool     warm_start;                 /**< Whether the solver should apply impulses from previous ticks (improves stability). */
} bnd_config_solver;

typedef struct {
  uint32_t max_gjk_iterations;              /**< Maximum GJK iterations per collision test. */
  float epa_tolerance;                      /**< EPA convergence tolerance. */
  uint16_t epa_max_nodes;                   /**< Maximum EPA polytope nodes. */
  float feature_distance_threshold;        /**< Maximum cached witness matching distance and tangential drift. */
  float separation_threshold;              /**< Maximum separation for retaining a cached contact point. */
} bnd_config_collision_detection;

/** Complete world configuration. */
typedef struct {
  bnd_config_memory memory;                              /**< Memory capacities. */
  bnd_config_simulation simulation;                      /**< Simulation parameters. */
  bnd_config_solver solver;                              /**< Constraint solver parameters. */
  bnd_config_collision_detection collision_detection;    /**< Collision detection parameters. */
} bnd_config;

/** Counters collected during simulation. */
typedef struct {
  uint64_t max_internal_buffer_size;         /**< Maximum size of the internal memory buffer */
  uint32_t body_count;                       /**< Total body count. */
  uint32_t contacts_count;                   /**< Contacts generated in the latest simulation. */
  uint32_t incomplete_collision_detections;  /**< Collision tests that exceeded their budget. */
  uint32_t world_age;                        /**< Number of completed simulation steps. */
  uint16_t used_epa_nodes;                   /**< Nodes used for collision detection */
} bnd_world_stats;

/** Contact point generated by collision detection. The normal points from B toward A. */
typedef struct {
  bnd_v3 point;                      /**< World-space contact point. */
  bnd_v3 normal;                     /**< Normal from body B toward body A. */
  float depth;                       /**< Penetration depth. */
  bnd_body_handle body_a, body_b;    /**< Bodies participating in the contact. */
} bnd_contact;

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
BND_RESULT_TYPE(joint, bnd_joint_handle)

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
BNDAPI uint64_t bnd_required_memory(const bnd_config *config);

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
 * Add a maximum-distance joint and return its handle.
 *
 * @retval BND_ERROR_BODY_HANDLE_INVALID Either body handle is invalid.
 * @retval BND_ERROR_INVALID_JOINT Both bodies are static.
 * @retval BND_ERROR_OUT_OF_MEMORY A joint buffer could not be grown.
 */
BNDAPI bnd_result_joint bnd_add_distance_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_distance_joint_config config);

/** Remove a joint by its handle. */
BNDAPI void bnd_remove_joint(bnd_world *world, bnd_joint_handle handle);

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

#define PROFILER_REPORT_METRIC_INT(name, val) (void)(name); (void) (val)
#define PROFILER_REPORT_METRIC_FLOAT(name, val) (void)(name); (void) (val)

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

#define HASH_TABLE_TOMBSTONE 0x7FFFFFFFFFFFFFFF
#define HASH_TABLE_EMPTY 0

#define OK (bnd_error){BND_OK, NULL}
#define OOM_ERROR (bnd_error){BND_ERROR_OUT_OF_MEMORY, "Allocator.malloc failed to allocate memory"}

#define MUST_BE_TRUE(expr) bool expr_result_rrr = (expr); (void)expr_result_rrr; assert(expr_result_rrr);

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

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

typedef uint32_t count_t;

typedef enum {
  CONTACT_NONE                = 0x0,
  CONTACT_TOUCHING            = 0x1,
  CONTACT_BEGAN_TOUCHING      = 0x2,
  CONTACT_FINISHED_TOUCHING   = 0x4,

  CONTACT_TRIGGER_A           = 0x8,
  CONTACT_TRIGGER_B           = 0x10,
  CONTACT_TRIGGER_BOTH        = CONTACT_TRIGGER_A | CONTACT_TRIGGER_B,
} broad_contact_status;

typedef struct {
  bnd_v3 point;
  float depth;

  // World-space for narrow phase contacts.
  // Local-sapce for cached contacts.
  bnd_v3 witness_a, witness_b;

  float normal_impulse;
  float tangential_impulse[2];
} contact_point;

typedef struct {
  count_t count;
  bnd_v3 normal;
  bnd_v3 local_normal;
  contact_point points[MAX_CONTACTS_PER_PAIR];
} contact_manifold;

typedef struct {
  uint64_t key;
  count_t body_a, body_b;
  count_t shape_a, shape_b;

  count_t next;
  count_t next_body;

  float friction, restitution;

  broad_contact_status status;
  contact_manifold manifold;
} broad_phase_contact;

typedef struct {
  bnd_joint_type type;
  bnd_body_handle bodies[2];           
  bnd_v3 anchors[2];
  float min_distance;
  float max_distance;                     
  float impulse;
} bnd_joint;

typedef struct {
  bnd_joint *values;
  count_t *ids;

  count_t capacity;
  count_t count;

  count_t next_id;
  count_t dynamic_count;
} joints;

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
  uint64_t *keys;
  count_t  *values;

  count_t capacity;
  count_t entry_count;
} hash_table;

typedef struct {
  broad_phase_contact *contacts;
  count_t capacity;

  count_t first, last, next;
  count_t free_list;
} broad_contacts_set;

typedef struct {
  uint64_t *keys;
  count_t  *indices;
  count_t hash_table_capacity;
  count_t hash_table_entry_count;

  broad_contacts_set dynamics;
  broad_contacts_set statics;
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
  BODY_FLAG_IMPULSE_APPLIED = 4,
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

typedef bnd_error (*broad_contact_iterator)(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data);

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
bnd_error             resize_force(bnd_allocator allocator, void **array, count_t element_size, uint64_t alignment, count_t capacity_factor, count_t *capacity);

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
void                  contacts_reset(bnd_world *world);
void                  contacts_teardown(bnd_world *world);
void                  contacts_remove_for_body(bnd_world *world, bnd_body_handle handle);
void                  contacts_filter_largest_surface_area(contact_point *contacts, count_t contact_count, bnd_v3 normal, count_t *selected_indices);
bnd_error             resolve_constraints(bnd_world *world, float dt);

bnd_error             for_each_broad_contact(bnd_world *world, broad_contact_iterator func, void *custom_data);

uint64_t              hash_table_create_key(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b, bnd_body_type type);
bool                  hash_table_find_slot_for_key(const contacts *contacts, uint64_t key, count_t *slot);
bool                  hash_table_find_empty_slot(const contacts *contacts, uint64_t key, count_t *slot);
bnd_error             hash_table_resize_if_needed(bnd_world *world, count_t additional_count);

bnd_error             run_broad_phase(bnd_world *world);
bnd_error             run_narrow_phase(bnd_world *world);

void                  collision_detection_init(void);
count_t               collisions_detect(bnd_world *world, count_t contacts_offset, bnd_body_type type);
bnd_error             collision_detection_epa_context(const bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, collision_detection_context *ctx);

bnd_error             joints_init(bnd_world *world);
void                  joints_teardown(bnd_world *world);
void                  joints_reset(bnd_world *world);
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
bool                  events_subscribed(const common_data *data, count_t index, int mask);
bnd_error             events_push(bnd_world *world, common_data *data, count_t index, bnd_event event);
bnd_error             events_emit_contacts(bnd_world *world);
bnd_error             events_emit_finish(bnd_world *world, broad_contacts_set *contact_set, count_t contact_slot, bnd_body_type contact_type, bool is_trigger);

bnd_quat              integrate_rotation_midpoint(bnd_quat rotation, bnd_v3 angular_momentum, bnd_m3 base_inv_inertia, float dt);

bool                  gjk_check_intersection(const bnd_world *world, const collision_detection_context *ctx, simplex *simplex);

uint32_t              polytope_memory_size(uint16_t max_nodes);
count_t               epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact_manifold *manifold);
body_support          support(const collision_detection_context *ctx, bnd_v3 direction);

#if defined(BND_DEBUG)
void                  epa_debug_next_frame(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, epa_debug_status *status);
void                  epa_debug_capture(bnd_world *world);
bool                  epa_debug_draw(bnd_world *world, const epa_debug_status *debug_status, bnd_debug_draw_epa_callbacks callbacks, void *user_data);
#endif

float                 sqr_distance_to_triangle(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 c, bnd_v3 *closest);
float                 sqr_distance_to_line_segment(bnd_v3 from, bnd_v3 a, bnd_v3 b, bnd_v3 *closest);
bool                  body_aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b);

bnd_v3                body_center(const shape_context *ctx);
bnd_quat              body_rotation(const shape_context *ctx);
bnd_v3                bounding_box_extents(const bnd_world *world, bnd_shape_type type, bnd_shape shape, bnd_quat rotation);

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

// static inline bnd_quat bnd_quat_add(bnd_quat x, bnd_quat y) {
//   return (bnd_quat){x.x + y.x, x.y + y.y, x.z + y.z, x.w + y.w};
// }

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

// ================
//   joints.c
// ================


static bnd_error joints_resize_if_needed(bnd_allocator allocator, joints *joints) {
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

bnd_result_joint bnd_add_distance_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, bnd_distance_joint_config config) {
  PROPAGATE_RESULT(joint, bnd_handle_valid(world, body_a));
  PROPAGATE_RESULT(joint, bnd_handle_valid(world, body_b));

  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_STATIC) {
    return BND_RESULT_ERR(joint, BND_ERROR_INVALID_JOINT, "Two static bodies cannot be bound together");
  }

  // Let body_a always be dynamic - same as with contacts.
  if (body_a.type == BND_BODY_STATIC && body_b.type == BND_BODY_DYNAMIC) {
    bnd_body_handle tmp_body = body_b;
    body_b = body_a;
    body_a = tmp_body;

    bnd_v3 tmp_pos = config.local_anchor_a;
    config.local_anchor_a = config.local_anchor_b;
    config.local_anchor_b = tmp_pos;
  }

  joints *joints = &world->joints;

  PROPAGATE_RESULT(joint, joints_resize_if_needed(world->allocator, joints));

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
    .type = BND_JOINT_TYPE_DISTANCE,
    .bodies = {body_a, body_b},
    .anchors = {config.local_anchor_a, config.local_anchor_b},
    .min_distance = MAX(config.min_distance, 0.0f),
    .max_distance = MAX(config.max_distance, 0.0f),
  };
  joints->ids[index] = id;

  bnd_joint_handle handle = { BND_JOINT_TYPE_DISTANCE, world->id, id };
  return BND_RESULT_OK(joint, handle);
}

void bnd_remove_joint(bnd_world *world, bnd_joint_handle handle) {
  if (world->id != handle.world_id) {
    return;
  }

  joints *joints = &world->joints;

  count_t count = joints->count;
  count_t dynamic_count = joints->dynamic_count;
  for (count_t i = 0; i < count; ++i) {
    if (joints->ids[i] != handle.id) {
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
//   solver.c
// ================


#define ALIGNMENT_CONTACT_CONSTRAINT 4
#define ALIGNMENT_JOINT_CONSTRAINT   4

typedef struct {
  bnd_v3 relative_position[2];
  float normal_mass, tangent_mass[4];
  float normal_impulse;
  float tangent_impulse[2];
  float separation;
} constraint_point;

typedef struct {
  bnd_body_type type;
  count_t contact_index;

  count_t points_count;
  count_t body_a, body_b;
  bnd_m3 basis;

  float friction, restitution;

  constraint_point points[MAX_CONTACTS_PER_PAIR];
} contact_constraint;

typedef struct {
  bnd_body_type type;
  count_t joint_index;

  count_t body_a, body_b;
  bnd_v3 axis;

  bnd_v3 Jav;
  bnd_v3 Jbv;

  bnd_v3 Jaw;
  bnd_v3 Jbw;

  bnd_v3 relative_positions[2];

  float effective_mass;
  float bias;

  float lambda;
  float min_lambda;
  float max_lambda;
} joint_constraint;

typedef struct {
  float inv_dt;
  dynamic_bodies *dynamics;

  count_t current_iteration;

  contact_constraint *contacts;
  joint_constraint   *joints;

  count_t contacts_count;
  count_t joints_count;
  
  bnd_config_solver config;
} solver_context;


typedef struct {
  count_t body_count;
  count_t body_ids[2];

  bnd_v3 velocities[2];
  bnd_v3 momenta[2];
  bnd_v3 angular_velocities[2];
  float inv_masses[2];

  bnd_v3 normal;
  bnd_v3 tangent_a;
  bnd_v3 tangent_b;
} constraint_state;

static bnd_m3 contact_space_transform(const broad_phase_contact *contact) {
  bnd_v3 y_axis = contact->manifold.normal;
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

static bnd_error constraints_from_contacts(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data) {
  (void)contacts;

  if (contact->manifold.count == 0 || contact->status & CONTACT_TRIGGER_BOTH) {
    return OK;
  }

  count_t *constraints_count = custom_data;
  bnd_result_ptr constraint_ptr = arena_alloc(&world->arena, ALIGNMENT_CONTACT_CONSTRAINT, sizeof(contact_constraint));
  PROPAGATE_ERROR(constraint_ptr.error)

  dynamic_bodies *dynamics = &world->dynamics;
  const common_data *data_b = as_common_const(world, type);

  count_t body_ids[] = {
    dynamics->outer_lookup[contact->body_a].index,
    data_b->outer_lookup[contact->body_b].index,
  };
  count_t body_count = type == BND_BODY_DYNAMIC ? 2 : 1;

  for (count_t k = 0; k < body_count; ++k) {
    dynamics->flags[body_ids[k]] |= BODY_FLAG_IMPULSE_APPLIED;
  }

  contact_constraint *constraint = constraint_ptr.value;
  constraint->type = type;
  constraint->contact_index = index;
  constraint->body_a = body_ids[0];
  constraint->body_b = body_ids[1];
  constraint->friction = contact->friction;
  constraint->restitution = contact->restitution;
  constraint->points_count = contact->manifold.count;
  constraint->basis = contact_space_transform(contact);

  bnd_m3 world_to_contact = bnd_m3_transpose(constraint->basis);

  bnd_v3 position[2];
  bnd_quat rotation[2];
  bnd_m3 inv_inertia_tensor[2];
  bnd_m3 inv_inertia[2];
  float inv_mass[2] = {0};
  for (count_t k = 0; k < body_count; ++k) {
    count_t body_index = body_ids[k];

    inv_mass[k] = dynamics->inv_masses[body_index];
    position[k] = dynamics->positions[body_index];
    rotation[k] = dynamics->rotations[body_index];
    inv_inertia_tensor[k] = dynamics->inv_inertia_tensors[body_index];
    inv_inertia[k] = bnd_m3_inertia(inv_inertia_tensor[k], rotation[k]);

    dynamics->inv_intertias[body_index] = inv_inertia[k];
  }

  for (count_t i = 0; i < contact->manifold.count; ++i) {
    contact_point *mp = &contact->manifold.points[i];
    constraint_point *cp = &constraint->points[i];

    cp->separation = -mp->depth;
    cp->normal_impulse = mp->normal_impulse;
    cp->tangent_impulse[0] = mp->tangential_impulse[0];
    cp->tangent_impulse[1] = mp->tangential_impulse[1];

    bnd_m3 effective_mass = {0};
    for (count_t k = 0; k < body_count; ++k) {
      cp->relative_position[k] = bnd_v3_sub(mp->point, position[k]);

      bnd_m3 r_cross = bnd_m3_skew_symmetric(cp->relative_position[k]);

      bnd_m3 body_effective_mass = bnd_m3_multiply(r_cross, inv_inertia[k]);
      body_effective_mass = bnd_m3_multiply(body_effective_mass, r_cross);
      body_effective_mass = bnd_m3_negate(body_effective_mass);

      effective_mass = bnd_m3_add(effective_mass, body_effective_mass);
    }

    effective_mass = bnd_m3_multiply(world_to_contact, effective_mass);
    effective_mass = bnd_m3_multiply(effective_mass, constraint->basis);
    effective_mass.m0[0] += inv_mass[0] + inv_mass[1];
    effective_mass.m1[1] += inv_mass[0] + inv_mass[1];
    effective_mass.m2[2] += inv_mass[0] + inv_mass[1];

    float normal_mass = effective_mass.m1[1];
    cp->normal_mass = normal_mass > EPSILON ? 1.0f / normal_mass : 0.0f;

    float tan_00 = effective_mass.m0[0];
    float tan_01 = effective_mass.m0[2]; 
    float tan_10 = effective_mass.m2[0];
    float tan_11 = effective_mass.m2[2];
    float tan_det = tan_00 * tan_11 - tan_01 * tan_10;
    if (tan_det > EPSILON) {
      tan_det = 1.0f / tan_det;

      cp->tangent_mass[0] = tan_11 * tan_det;
      cp->tangent_mass[1] = -tan_01 * tan_det;
      cp->tangent_mass[2] = -tan_10 * tan_det;
      cp->tangent_mass[3] = tan_00 * tan_det;
    } else {
      memset(cp->tangent_mass, 0, sizeof(cp->tangent_mass));
    }
  }

  *constraints_count += 1;

  return OK;
}

static bnd_result_u32 constraints_from_joints(bnd_world *world) {
  joints *joints = &world->joints;

  const bnd_config_solver config = world->config.solver;
  const dynamic_bodies *dynamics = &world->dynamics;
  const common_data *statics  = (common_data *)&world->statics;

  count_t count = 0;
  for (count_t i = 0; i < joints->count; ++i) {
    const bnd_joint *j = &joints->values[i];

    const bool is_dynamic = i < joints->dynamic_count;
    const common_data *data_a = (common_data *)dynamics;
    const common_data *data_b = is_dynamic ? (common_data *)dynamics : statics;

    const count_t index_a = data_a->outer_lookup[j->bodies[0].index].index;
    const count_t index_b = data_b->outer_lookup[j->bodies[1].index].index;

    bnd_v3 position_a = data_a->positions[index_a];
    bnd_v3 position_b = data_b->positions[index_b];

    bnd_quat rotation_a = data_a->rotations[index_a];
    bnd_quat rotation_b = data_b->rotations[index_b];

    bnd_v3 pa = bnd_v3_add(position_a, bnd_v3_rotate(j->anchors[0], rotation_a));
    bnd_v3 pb = bnd_v3_add(position_b, bnd_v3_rotate(j->anchors[1], rotation_b));

    bnd_v3 offset_a = bnd_v3_sub(pa, position_a);
    bnd_v3 offset_b = bnd_v3_sub(pb, position_b);

    switch (j->type) {
      case BND_JOINT_TYPE_DISTANCE: {
        bnd_v3 offset = bnd_v3_sub(pb, pa);
        float distance_sqr = bnd_v3_lensqr(offset);
        float threshold_min = j->min_distance * j->min_distance;
        float threshold_max = j->max_distance * j->max_distance;

        if (distance_sqr <= threshold_max && distance_sqr >= threshold_min) {
          continue;
        }

        count += 1;

        float distance = distance_sqr > EPSILON * EPSILON ? sqrtf(distance_sqr) : 0.0f;

        bnd_v3 normal = distance_sqr > EPSILON * EPSILON
          ? bnd_v3_scale(offset, 1.0f / distance)
          : bnd_v3_normalize(bnd_v3_sub(position_b, position_b));

        bnd_v3 ra_n = bnd_v3_cross(offset_a, normal);
        bnd_v3 rb_n = bnd_v3_cross(offset_b, normal);

        bnd_result_ptr allocation = arena_alloc(&world->arena, ALIGNMENT_JOINT_CONSTRAINT, sizeof(joint_constraint));
        PROPAGATE_RESULT(u32, allocation.error);

        joint_constraint *constraint = allocation.value;
        constraint->type = is_dynamic ? BND_BODY_DYNAMIC : BND_BODY_STATIC;
        constraint->joint_index = i;
        constraint->body_a = index_a;
        constraint->body_b = index_b;
        constraint->axis = normal;

        constraint->relative_positions[0] = offset_a;
        constraint->relative_positions[1] = offset_b;

        constraint->Jav = normal;
        constraint->Jaw = ra_n;

        if (is_dynamic) {
          constraint->Jbv = bnd_v3_negate(normal);
          constraint->Jbw = bnd_v3_negate(rb_n);
        } else {
          constraint->Jbv = constraint->Jbw = bnd_v3_zero();
        }

        bnd_m3 inv_inertia_a = bnd_m3_inertia(dynamics->inv_inertia_tensors[index_a], rotation_a);

        float K = dynamics->inv_masses[index_a];
        K += bnd_v3_dot(ra_n, bnd_m3_rotate(ra_n, inv_inertia_a));

        if (is_dynamic) {
          bnd_m3 inv_inertia_b = bnd_m3_inertia(dynamics->inv_inertia_tensors[index_b], rotation_b);

          K += dynamics->inv_masses[index_b];
          K += bnd_v3_dot(rb_n, bnd_m3_rotate(rb_n, inv_inertia_b));
        }

        constraint->effective_mass = K > EPSILON ? 1.0f / K : 0.0f;
        constraint->lambda = j->impulse;

        float error;
        if (distance_sqr < threshold_min - config.linear_slop) {
          error = distance - j->min_distance;
          constraint->min_lambda = -INFINITY;
          constraint->max_lambda = 0.0f;
        } else if (distance_sqr > threshold_max + config.linear_slop) {
          error = j->max_distance - distance;
          constraint->min_lambda = 0.0f;
          constraint->max_lambda = INFINITY;
        } else {
          error = constraint->max_lambda = constraint->min_lambda = 0.0f;
        }

        constraint->bias = config.baumgarde_coefficient * error;
        constraint->bias = error > 0.0f
         ? MIN(constraint->bias, config.max_baumgarde_velocity)
         : MAX(constraint->bias, -config.max_baumgarde_velocity);

      } break;

      default:
        continue;
    }
  }

  return BND_RESULT_OK(u32, count);
}

static void apply_impulse(bnd_v3 impulse, const bnd_v3 *relative_positions, constraint_state *state) {
  float sign = 1.0;
  for (count_t k = 0; k < state->body_count; ++k) {
    state->velocities[k] = bnd_v3_add(state->velocities[k], bnd_v3_scale(impulse, state->inv_masses[k] * sign));
    state->momenta[k] = bnd_v3_add(state->momenta[k], bnd_v3_scale(bnd_v3_cross(relative_positions[k], impulse), sign));

    sign = -1;
  }
}

static constraint_state joint_constraint_state(const dynamic_bodies *dynamics, const joint_constraint *constraint) {
  constraint_state s = {
    .body_count = 1 + (constraint->type == BND_BODY_DYNAMIC),
    .body_ids = { constraint->body_a, constraint->body_b },

    .normal = constraint->axis,
  };
  
  for (count_t k = 0; k < 2; ++k) {
    count_t id = s.body_ids[k];
    s.velocities[k] = dynamics->velocities[id];
    s.momenta[k] = dynamics->angular_momenta[id];
    s.inv_masses[k] = dynamics->inv_masses[id];

    bnd_m3 inertia = bnd_m3_inertia(dynamics->inv_inertia_tensors[id], dynamics->rotations[id]);
    s.angular_velocities[k] = bnd_m3_rotate(s.momenta[k], inertia);
  }

  return s;
}

static constraint_state contact_constraint_state(const dynamic_bodies *dynamics, const contact_constraint *constraint) {
  constraint_state s = {
    .body_count = 1 + (constraint->type == BND_BODY_DYNAMIC),
    .body_ids = { constraint->body_a, constraint->body_b },

    .normal    = { constraint->basis.m0[1], constraint->basis.m1[1], constraint->basis.m2[1] },
    .tangent_a = { constraint->basis.m0[0], constraint->basis.m1[0], constraint->basis.m2[0] },
    .tangent_b = { constraint->basis.m0[2], constraint->basis.m1[2], constraint->basis.m2[2] },
  };

  for (count_t k = 0; k < 2; ++k) {
    s.velocities[k] = dynamics->velocities[s.body_ids[k]];
    s.momenta[k] = dynamics->angular_momenta[s.body_ids[k]];
    s.inv_masses[k] = dynamics->inv_masses[s.body_ids[k]];
  }

  return s;
}

static void write_back_constraint_state(dynamic_bodies *dynamics, const constraint_state *state) {
  for (count_t k = 0; k < state->body_count; ++k) {
    dynamics->velocities[state->body_ids[k]] = state->velocities[k];
    dynamics->angular_momenta[state->body_ids[k]] = state->momenta[k];
  }
}

static void write_back_impulses(bnd_world *world, solver_context *cx) {
  for (count_t i = 0; i < cx->contacts_count; ++i) {
    const contact_constraint * constraint = &cx->contacts[i];
    broad_contacts_set *contacts = constraint->type == BND_BODY_DYNAMIC ? &world->contacts.dynamics : &world->contacts.statics;

    contact_manifold *manifold = &contacts->contacts[constraint->contact_index].manifold;
    for (count_t j = 0; j < manifold->count; ++j) {
      manifold->points[j].normal_impulse = constraint->points[j].normal_impulse;
      manifold->points[j].tangential_impulse[0] = constraint->points[j].tangent_impulse[0];
      manifold->points[j].tangential_impulse[1] = constraint->points[j].tangent_impulse[1];
    }
  }

  for (count_t i = 0; i < cx->joints_count; ++i) {
    const joint_constraint *constraint = &cx->joints[i];
    bnd_joint *j = &world->joints.values[constraint->joint_index];

    j->impulse = constraint->lambda;
  }

}

static void warm_start_solver(solver_context *cx) {
  for (count_t i = 0; i < cx->contacts_count; ++i) {
    contact_constraint *constraint = &cx->contacts[i];
    constraint_state state = contact_constraint_state(cx->dynamics, constraint);

    for (count_t p = 0; p < constraint->points_count; ++p) {
      constraint_point *point = &constraint->points[p];
      bnd_v3 tangent_impulse = bnd_v3_add(bnd_v3_scale(state.tangent_a, point->tangent_impulse[0]), bnd_v3_scale(state.tangent_b, point->tangent_impulse[1]));
      bnd_v3 impulse = bnd_v3_add(bnd_v3_scale(state.normal, point->normal_impulse), tangent_impulse);

      apply_impulse(impulse, point->relative_position, &state);
    }

    write_back_constraint_state(cx->dynamics, &state);
  }

  for (count_t i = 0; i < cx->joints_count; ++i) {
    joint_constraint *constraint = &cx->joints[i];
    constraint_state state = joint_constraint_state(cx->dynamics, constraint);

    bnd_v3 impulse = bnd_v3_scale(state.normal, constraint->lambda);
    apply_impulse(impulse, constraint->relative_positions, &state);
    write_back_constraint_state(cx->dynamics, &state);
  }
}

static bnd_v3 contact_point_local_velocity(
  const dynamic_bodies *dynamics,
  const contact_constraint *constraint,
  const constraint_point *point,
  const constraint_state *state
) {
  bnd_v3 local_velocity[2] = {0};
  for (count_t k = 0; k < state->body_count; ++k) {
    bnd_v3 angular_velocity = bnd_m3_rotate(state->momenta[k], dynamics->inv_intertias[state->body_ids[k]]);
    bnd_v3 vel = bnd_v3_add(state->velocities[k], bnd_v3_cross(angular_velocity, point->relative_position[k]));
    local_velocity[k] = bnd_m3_rotate(vel, bnd_m3_transpose(constraint->basis));
  }

  return bnd_v3_sub(local_velocity[0], local_velocity[1]);
}

static void solve_contact(solver_context *cx, count_t index) {
  contact_constraint *constraint = &cx->contacts[index];
  constraint_state interim_state = contact_constraint_state(cx->dynamics, constraint);

  for (count_t p = 0; p < constraint->points_count; ++p) {
    constraint_point *point = &constraint->points[p];
    bnd_v3 local_velocity = contact_point_local_velocity(cx->dynamics, constraint, point, &interim_state);

    float bias = MAX(cx->config.baumgarde_coefficient * cx->inv_dt * MIN(0.0f, point->separation + cx->config.linear_slop), -cx->config.max_baumgarde_velocity);
    float vn = local_velocity.y;
    float normal_impulse = -point->normal_mass * (vn + bias) * (1 + constraint->restitution);
    float new_impulse = MAX(point->normal_impulse + normal_impulse, 0.0f);
    normal_impulse = new_impulse - point->normal_impulse;
    point->normal_impulse = new_impulse;

    bnd_v3 impulse = bnd_v3_scale(interim_state.normal, normal_impulse);
    apply_impulse(impulse, point->relative_position, &interim_state);
  }

  for (count_t p = 0; p < constraint->points_count; ++p) {
    constraint_point *point = &constraint->points[p];
    bnd_v3 local_velocity = contact_point_local_velocity(cx->dynamics, constraint, point, &interim_state);

    float delta_lambda[] = {
      -(local_velocity.x * point->tangent_mass[0] + local_velocity.z * point->tangent_mass[1]),
      -(local_velocity.x * point->tangent_mass[2] + local_velocity.z * point->tangent_mass[3]),
    };

    float candidate_lambda[] = {
      point->tangent_impulse[0] + delta_lambda[0],
      point->tangent_impulse[1] + delta_lambda[1]
    };

    float max_friction = constraint->friction * point->normal_impulse;
    float friction_impulse = sqrtf(candidate_lambda[0] * candidate_lambda[0] + candidate_lambda[1] * candidate_lambda[1]);
    if (friction_impulse > max_friction) {
      candidate_lambda[0] = candidate_lambda[0] / friction_impulse * max_friction;
      candidate_lambda[1] = candidate_lambda[1] / friction_impulse * max_friction;
    }

    float lambda[] = {
      candidate_lambda[0] - point->tangent_impulse[0],
      candidate_lambda[1] - point->tangent_impulse[1],
    };

    memcpy(point->tangent_impulse, candidate_lambda, sizeof(candidate_lambda));

    bnd_v3 impulse = bnd_v3_add(bnd_v3_scale(interim_state.tangent_a, lambda[0]), bnd_v3_scale(interim_state.tangent_b, lambda[1]));

    apply_impulse(impulse, point->relative_position, &interim_state);
  }

  write_back_constraint_state(cx->dynamics, &interim_state);
}

static void solve_joint(solver_context *cx, count_t index) {
  joint_constraint *constraint = &cx->joints[index];
  constraint_state state = joint_constraint_state(cx->dynamics, constraint);

  float bias = constraint->bias * cx->inv_dt;
  float JV = bnd_v3_dot(constraint->Jav, state.velocities[0])
    + bnd_v3_dot(constraint->Jbv, state.velocities[1])
    + bnd_v3_dot(constraint->Jaw, state.angular_velocities[0])
    + bnd_v3_dot(constraint->Jbw, state.angular_velocities[1]);

  float lambda_delta = -constraint->effective_mass * (JV + bias);
  float lambda_old = constraint->lambda;

  constraint->lambda = MAX(constraint->min_lambda, MIN(lambda_old + lambda_delta, constraint->max_lambda));

  float impulse_lambda = constraint->lambda - lambda_old;
  bnd_v3 impulse = bnd_v3_scale(constraint->axis, impulse_lambda);

  apply_impulse(impulse, constraint->relative_positions, &state);
  write_back_constraint_state(cx->dynamics, &state);
}

bnd_error resolve_constraints(bnd_world *world, float dt) {
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame(&world->arena);

  count_t contacts_count = 0;
  count_t joints_count = 0;

  uint64_t contacts_arena_offset = AlignTo(stack_frame.arena->offset, ALIGNMENT_CONTACT_CONSTRAINT);
  uint64_t joints_arena_offset = 0;

  {
    PROFILER_BLOCK_START("prepare_constraints");

    bnd_error e = for_each_broad_contact(world, constraints_from_contacts, &contacts_count);
    if (IS_ERROR(e)) {
      arena_release_stack_frame(stack_frame);
      PROFILER_BLOCK_END;
      return e;
    }

    joints_arena_offset = AlignTo(stack_frame.arena->offset, ALIGNMENT_JOINT_CONSTRAINT);

    bnd_result_u32 result_joints_count = constraints_from_joints(world);
    if (IS_ERROR(result_joints_count.error)) {
      arena_release_stack_frame(stack_frame);
      PROFILER_BLOCK_END;
      return result_joints_count.error;
    }
    joints_count = result_joints_count.value;

    PROFILER_BLOCK_END;
  }

  world->stats.contacts_count = contacts_count;

  if (dt <= 0.0f) {
    arena_release_stack_frame(stack_frame);
    return OK;
  }
  
  PROFILER_BLOCK_START("resolve_constraints");

  contact_constraint *contact_constraints = (contact_constraint *)(stack_frame.arena->buffer + contacts_arena_offset);
  joint_constraint *joint_constraints     = (joint_constraint *)(stack_frame.arena->buffer + joints_arena_offset);

  solver_context scx = {
    .inv_dt            = 1.0f / dt,
    .current_iteration = 0,
    .dynamics          = &world->dynamics,
    .config            = world->config.solver,
    .contacts          = contact_constraints,
    .joints            = joint_constraints,
    .contacts_count    = contacts_count,
    .joints_count      = joints_count
  };

  if (scx.config.warm_start) {
    warm_start_solver(&scx);
  }

  for (count_t i = 0; i < scx.config.iterations_count; ++i) {
    scx.current_iteration = i;

    for (count_t j = 0; j < scx.contacts_count; ++j) {
      solve_contact(&scx, j);
    }

    for (count_t j = 0; j < scx.joints_count; ++j) {
      solve_joint(&scx, j);
    }
  }

  if (scx.config.warm_start) {
    write_back_impulses(world, &scx);
  }

  PROFILER_BLOCK_END;

  arena_release_stack_frame(stack_frame);

  return OK;
}



// ================
//   world.c
// ================


#define INVALID_INDEX ((count_t)~0)
#define INVALID_HANDLE (bnd_body_handle) { 0, 0, INVALID_INDEX, 0 }

#define INVALID_BODY_TYPE ((bnd_result_handle) { .error = (bnd_error) { .type = BND_ERROR_INVALID_BODY_TYPE, .message = "Unknown body type" } })

#define BODY_MASK_WIDE(flags) (0x0101010101010101ULL * (flags))

#define TEMP_BODY_MASK (BODY_FLAG_DIRTY | BODY_FLAG_IMPULSE_APPLIED)
#define TEMP_BODY_MASK_WIDE BODY_MASK_WIDE(TEMP_BODY_MASK)
#define DIRTY_MASK_WIDE BODY_MASK_WIDE(BODY_FLAG_DIRTY)
#define CHUNK_SIZE sizeof(uint64_t)

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

bnd_v3 bounding_box_extents(const bnd_world *world, bnd_shape_type type, bnd_shape shape, bnd_quat rotation) {
  bnd_m3 rotation_matrix;
  bnd_v3 half_extents;
  switch (type) {
    case BND_BOX:
      rotation_matrix = quat_as_matrix(rotation);
      return rotated_box_half_extents(rotation_matrix, bnd_v3_scale(shape.box.size, 0.5f));

    case BND_SPHERE:
      return bnd_v3_scale(bnd_v3_one(), shape.sphere.radius);

    case BND_CAPSULE:
      rotation_matrix = quat_as_matrix(rotation);
      half_extents = (bnd_v3){ shape.capsule.radius, 0.5f * shape.capsule.height + shape.capsule.radius, shape.capsule.radius };
      return rotated_box_half_extents(rotation_matrix, half_extents);

    case BND_MESH:
      rotation_matrix = quat_as_matrix(rotation);
      half_extents = world->meshes.aabbs[shape.mesh].half_extents;
      return rotated_box_half_extents(rotation_matrix, half_extents);

    default:
      return (bnd_v3){FLT_MAX, FLT_MAX, FLT_MAX};
  }
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

    bnd_v3 shape_min, shape_max;
    bnd_v3 half_extents = bounding_box_extents(world, shape.type, shape.value, shape_rotation);

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

static void clear_flags(bnd_world *world) {
  common_data *bodies[] = { (common_data *)&world->dynamics, (common_data *)&world->statics };
  for (count_t k = 0; k < 2; ++k) {
    common_data *data = bodies[k];

    count_t i = 0;
    for (; i + CHUNK_SIZE <= data->count; i += CHUNK_SIZE) {
      uint64_t flags;
      memcpy(&flags, &data->flags[i], CHUNK_SIZE);

      flags &= ~TEMP_BODY_MASK_WIDE;

      memcpy(&data->flags[i], &flags, CHUNK_SIZE);
    }

    for (; i < data->count; ++i) {
      data->flags[i] &= ~TEMP_BODY_MASK;
    }
  }
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
  for (count_t i = 0; i < dynamics->count; ++i) {
    if (i >= awake_count && (dynamics->flags[i] & BODY_FLAG_IMPULSE_APPLIED) == 0) {
      continue;
    }
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
    while (left < awake_count && dynamics->motion_avgs[left] > sleep_threshold) {
      left += 1;
    }

    while (right >= awake_count && dynamics->motion_avgs[right] <= sleep_threshold) {
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

  joints_remove_stale_if_needed(world, handle);
  contacts_remove_for_body(world, handle);

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
  return world->stats.contacts_count;
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
  data->flags[index] |= BODY_FLAG_DIRTY;

  return OK;
}

bnd_error bnd_set_rotation(bnd_world *world, bnd_body_handle handle, bnd_quat rotation) {
  PROPAGATE_ERROR(bnd_handle_valid(world, handle))

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);
  data->rotations[index] = rotation;
  data->flags[index] |= BODY_FLAG_DIRTY;

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

  contacts_remove_for_body(world, handle);

  common_data *data = as_common(world, handle.type);
  count_t index = handle_to_inner_index(world, handle);

  data->collision_layers[index] = layer;

  return OK;
}

count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts) {
  if (contacts == NULL || max_contacts == 0) {
    return 0;
  }

  count_t count = 0;

  const broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };
  for (count_t k = 0; k < 2; ++k) {
    const broad_contacts_set *contact_set = sets[k];

    bnd_body_type contact_type = (bnd_body_type) k;
    const common_data *data_a = as_common_const(world, BND_BODY_DYNAMIC);
    const common_data *data_b = as_common_const(world, contact_type);

    count_t body_index = contact_set->first;
    while (body_index != UINT32_MAX) {
      const broad_phase_contact *body_contact = &contact_set->contacts[body_index];

      count_t shape_index = body_index;
      while (shape_index != UINT32_MAX) {
        const broad_phase_contact *shape_contact = &contact_set->contacts[shape_index];
        if (shape_contact->manifold.count == 0) {
          shape_index = shape_contact->next;
          continue;
        }

        for (count_t i = 0; i < shape_contact->manifold.count; ++i) {
          const contact_point *p = &shape_contact->manifold.points[i];

          contacts[count++] = (bnd_contact){
            .point = p->point,
            .normal = shape_contact->manifold.normal,
            .depth = p->depth,
            .body_a = (bnd_body_handle) { .type = BND_BODY_DYNAMIC, .world_id = world->id, .index = shape_contact->body_a, .generation = data_a->generations[shape_contact->body_a] },
            .body_b = (bnd_body_handle) { .type = contact_type,     .world_id = world->id, .index = shape_contact->body_b, .generation = data_b->generations[shape_contact->body_b] },
          };

          if (count >= max_contacts) {
            return count;
          }
        }

        shape_index = shape_contact->next;
      }

      body_index = body_contact->next_body;
    }
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
  count_t i = 0; 

  dynamic_bodies *dynamics = &world->dynamics;
  for (; i < dynamics->awake_count; ++i) {
    calculate_aabb(world, (common_data *) dynamics, i);
  }

  for (i = dynamics->awake_count; i + CHUNK_SIZE <= dynamics->count; i += CHUNK_SIZE) {
    uint64_t flags;
    memcpy(&flags, &dynamics->flags[i], CHUNK_SIZE);

    if ((flags & DIRTY_MASK_WIDE) == 0) {
      continue;
    }

    for (count_t j = 0; j < CHUNK_SIZE; ++j) {
      if (dynamics->flags[i + j] & BODY_FLAG_DIRTY) {
        calculate_aabb(world, (common_data *)dynamics, i + j);
      }
    }
  }

  for (; i < dynamics->count; ++i) {
    if (dynamics->flags[i] & BODY_FLAG_DIRTY) {
      calculate_aabb(world, (common_data *)dynamics, i);
    }
  }

  common_data *statics = (common_data *)&world->statics;
  for (i = 0; i + CHUNK_SIZE <= statics->count; i += CHUNK_SIZE) {
    uint64_t flags;
    memcpy(&flags, &statics->flags[i], CHUNK_SIZE);

    if ((flags & DIRTY_MASK_WIDE) == 0) {
      continue;
    }

    for (count_t j = 0; j < CHUNK_SIZE; ++j) {
      if (statics->flags[i + j] & BODY_FLAG_DIRTY) {
        calculate_aabb(world, statics, i + j);
      }
    }
  }

  for (; i < statics->count; ++i) {
    if (statics->flags[i] & BODY_FLAG_DIRTY) {
      calculate_aabb(world, statics, i);
    }
  }
}

static void integrate_velocities(bnd_world *world, float dt) {
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

    bnd_v3 momentum_delta = bnd_v3_scale(dynamics->torques[i], dt);
    momentum_delta = bnd_v3_add(momentum_delta, dynamics->angular_impulses[i]);

    bnd_v3 angular_momentum = dynamics->angular_momenta[i];
    angular_momentum = bnd_v3_add(angular_momentum, momentum_delta);
    angular_momentum = bnd_v3_scale(angular_momentum, angular_damping);

    dynamics->accelerations[i] = acceleration;
    dynamics->velocities[i] = velocity;
    dynamics->angular_momenta[i] = angular_momentum;
  }

  PROFILER_FUNCTION_END
}

static void integrate_positions(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  dynamic_bodies *dynamics = &world->dynamics;
  for (count_t i = 0; i < dynamics->awake_count; ++i) {
    bnd_v3 velocity = dynamics->velocities[i];
    bnd_v3 position = dynamics->positions[i];
    bnd_quat rotation = dynamics->rotations[i];

    bnd_m3 base_inv_inertia = dynamics->inv_inertia_tensors[i];
    bnd_v3 angular_momentum = dynamics->angular_momenta[i];

    rotation = integrate_rotation_midpoint(rotation, angular_momentum, base_inv_inertia, dt);
    position = bnd_v3_add(position, bnd_v3_scale(velocity, dt));

    dynamics->inv_intertias[i] = bnd_m3_inertia(base_inv_inertia, rotation);
    dynamics->rotations[i] = rotation;
    dynamics->positions[i] = position;
  }

  PROFILER_FUNCTION_END
}


void bnd_simulate(bnd_world *world, float dt) {
  PROFILER_FUNCTION_START

  world->age += 1;
  world->stats.body_count = world->dynamics.count + world->statics.count;
  world->stats.world_age = world->age;

  arena_reset(&world->arena);
  integrate_velocities(world, dt);
  update_aabbs(world);
  events_reset(world);

  // TODO Check for errors
  run_broad_phase(world);
  run_narrow_phase(world);
  events_emit_contacts(world);

#if defined(BND_DEBUG)
  epa_debug_capture(world);
#endif
  resolve_constraints(world, dt);

  update_awake_statuses(world, dt);
  integrate_positions(world, dt);
  clear_flags(world);
  clear_forces(world);

  if (world->arena.offset > world->stats.max_internal_buffer_size) {
    world->stats.max_internal_buffer_size = world->arena.offset;
  }

  PROFILER_FUNCTION_END

  PROFILER_REPORT_METRIC_INT("Dynamic bodies", world->dynamics.count);
  PROFILER_REPORT_METRIC_INT("Static bodies", world->statics.count);
  PROFILER_REPORT_METRIC_INT("Total bodies", world->dynamics.count + world->statics.count);
  PROFILER_REPORT_METRIC_INT("Contacts count", world->stats.contacts_count);
  PROFILER_REPORT_METRIC_INT("EPA nodes", world->stats.used_epa_nodes);
  PROFILER_REPORT_METRIC_INT("Max internal buffer size", world->arena.max_offset);
  PROFILER_REPORT_METRIC_INT("Internal buffer capacity", world->arena.capacity);
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

  world->stats.incomplete_collision_detections = 0;

  contacts_reset(world);
  shapes_reset(world);
  joints_reset(world);
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

  PROPAGATE_RESULT(material, resize_if_needed(world->allocator, (void **)&world->materials.values, sizeof(body_material), ALIGNMENT_BODY_MATERIAL, count, 1, &world->materials.capacity))

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

  contacts_remove_for_body(world, handle);

  return OK;
}

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
BND_RESULT_FUNC_DECL(joint, bnd_joint_handle)

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

bnd_error resize_force(bnd_allocator allocator, void **array, count_t element_size, uint64_t alignment, count_t capacity_factor, count_t *capacity) {
  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Allocator.realloc is NULL" };
  }
  
  count_t new_capacity = *capacity * capacity_factor;

  void *new_buffer = allocator.realloc(*array, alignment, *capacity * element_size, new_capacity * element_size);
  if (new_buffer == NULL) {
    return (bnd_error) { BND_ERROR_OUT_OF_MEMORY, "Failed to reallocate memory" };
  }

  *array = new_buffer;
  *capacity = new_capacity;

  return OK;
}

bnd_error resize_if_needed(bnd_allocator allocator, void **array, count_t element_size, uint64_t alignment, count_t count, count_t additional_count, count_t *capacity) {
  if (count + additional_count <= *capacity) {
    return OK;
  }

  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Allocator.realloc is NULL" };
  }

  count_t new_capacity = *capacity;
  while(new_capacity < count + additional_count) {
    new_capacity <<= 1;
  }

  void *new_buffer = allocator.realloc(*array, alignment, *capacity * element_size, new_capacity * element_size);
  if (new_buffer == NULL) {
    return (bnd_error) { BND_ERROR_OUT_OF_MEMORY, "Failed to reallocate memory" };
  }

  *array = new_buffer;
  *capacity = new_capacity;

  return OK;
}

uint64_t bnd_required_memory(const bnd_config *config) {
  uint64_t size = sizeof(bnd_world);

  uint64_t common_size = sizeof(bnd_v3)
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

  uint64_t dynamic_size = common_size
    + 4 * sizeof(bnd_v3)
    + sizeof(float)
    + 2 * sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(bnd_v3)
    + sizeof(bnd_m3)
    + sizeof(float);

  uint64_t joint_size = sizeof(bnd_joint) + sizeof(count_t);
  uint64_t mesh_size = sizeof(bnd_v3) * DEFAULT_VERTEX_PER_MESH
    + sizeof(uint32_t) * DEFAULT_FACE_PER_MESH * 3
    + sizeof(submesh)
    + sizeof(bnd_mesh)
    + sizeof(bnd_m3)
    + sizeof(float)
    + sizeof(bnd_aabb);
  uint64_t material_size = sizeof(body_material);

  uint64_t event_size = sizeof(bnd_event) + sizeof(count_t);
  uint64_t shapes_size = 0;
  for (count_t i = 0; i < BRACKET_COUNT; ++i) {
    count_t blocks_count, shapes_count, bracket_capacity;
    shapes_get_bracket_properties(config, i, &blocks_count, &shapes_count, &bracket_capacity);

    shapes_size += shapes_count * sizeof(bnd_body_shape)
      + blocks_count * sizeof(uint64_t);
  }

  uint64_t contacts_size =
    sizeof(uint64_t)            * config->memory.hash_table_capacity +
    sizeof(count_t)             * config->memory.hash_table_capacity +
    sizeof(broad_phase_contact) * config->memory.contacts_capacity;

  uint64_t arena_size = config->memory.internal_allocator_capacity_bytes;

  size += (config->memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT) * dynamic_size
    + (config->memory.statics_capacity + EPHEMERAL_BODIES_COUNT) * common_size
    + config->memory.joints_capacity * joint_size
    + config->memory.meshes_capacity * mesh_size
    + config->memory.events_capacity * event_size
    + config->memory.materials_capacity * material_size
    + contacts_size
    + shapes_size
    + arena_size;

  // Alignment
  size += 10 * 7; // 8-bytes for world, shapes slots, EPA polytope, custom data and the broad phase contacts
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
    },
    .memory = {
      .dynamics_capacity = 32,
      .statics_capacity = 8,
      .contacts_capacity = 64,
      .joints_capacity = 64,
      .meshes_capacity = 32,
      .events_capacity = 128,
      .materials_capacity = 8,
      .hash_table_capacity = 512,
      .shapes_brackets_capacity = {64, 1, 1, 1, 1},
      .internal_allocator_capacity_bytes = 8 << 10, // 8 Kb
    },
    .solver = {
      .iterations_count = 8,
      .baumgarde_coefficient = 0.2f,
      .linear_slop = 0.005f,
      .max_baumgarde_velocity = 4.0f,  
      .warm_start = true,
    },
    .collision_detection = {
      .max_gjk_iterations = 100,
      .epa_tolerance = 0.01f,
      .epa_max_nodes = 128,
      .feature_distance_threshold = 0.02f,
      .separation_threshold = 0.05f,
    
    }
  };
}

static void sanitize_config(bnd_config *config) {
  bnd_config_solver *solver = &config->solver;

  solver->linear_slop = MAX(solver->linear_slop, 0.0f);
  solver->baumgarde_coefficient = MAX(0.0f, MIN(solver->baumgarde_coefficient, 1.0f));
  solver->iterations_count = MAX(solver->iterations_count, 0);
  solver->max_baumgarde_velocity = MAX(solver->max_baumgarde_velocity, 0.0f);

  // TODO: other categories.
}

static bnd_error bnd_init_internal(bnd_world *world, bnd_config config, bnd_allocator allocator) {
  sanitize_config(&config);

  world->allocator = allocator;
  world->config = config;
  world->id = next_world_id++; // TODO: make this thread-safe.

  bnd_error e;
  INVOKE(init_commons((common_data *)&world->dynamics, config.memory.dynamics_capacity, allocator))
  INVOKE(init_commons((common_data *)&world->statics, config.memory.statics_capacity, allocator))

  const count_t vectors = sizeof(bnd_v3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t floats = sizeof(float) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);
  const count_t matrices = sizeof(bnd_m3) * (config.memory.dynamics_capacity + EPHEMERAL_BODIES_COUNT);

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

  INVOKE(arena_init(allocator, config.memory.internal_allocator_capacity_bytes, &world->arena))

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

  world->allocator.free(world->arena.buffer, world->arena.capacity);
  world->allocator.free(world->materials.values, sizeof(body_material) * world->materials.capacity);
  world->allocator.free(world, sizeof(bnd_world));
}

count_t ephemeral_body_index(const common_data *data) {
  return data->capacity;
}

bnd_error arena_init(bnd_allocator allocator, uint64_t capacity, bnd_arena *arena) {
  ALLOC_BUFFER1(arena->buffer, capacity);

  arena->capacity = capacity;
  arena->allocator = allocator;
  arena->offset = 0;
  arena->max_offset = 0;

  return OK;
}

bnd_result_ptr arena_alloc(bnd_arena *arena, uint64_t alignment, uint64_t size) {
  uint64_t new_offset = AlignTo(arena->offset, alignment);
  if (new_offset + size > arena->capacity) {
    if (arena->allocator.realloc == NULL) {
      return (bnd_result_ptr)  { (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Internal allocation buffer is full" }, NULL };
    }

    uint64_t new_capacity = arena->capacity;
    while(new_offset + size > new_capacity) {
      new_capacity <<= 1;
    }

    uint8_t *buffer = arena->allocator.realloc(arena->buffer, 1, arena->capacity, new_capacity);
    if (buffer == NULL) {
      return (bnd_result_ptr)  { (bnd_error) { BND_ERROR_OUT_OF_MEMORY, "Allocator.realloc returned null" }, NULL };
    }

    arena->buffer = buffer;
    arena->capacity = new_capacity;
  }

  uint8_t *value = arena->buffer + new_offset;
  arena->offset = new_offset + size;
  if (arena->offset > arena->max_offset) {
    arena->max_offset = arena->offset;
  }
  
  return BND_RESULT_OK(ptr, value);
}

static void arena_reset_to(bnd_arena *arena, uint64_t offset) {
  arena->offset = offset;
}

bnd_arena_stack_frame arena_new_stack_frame(bnd_arena *arena) {
  return (bnd_arena_stack_frame) {
    .offset = arena->offset,
    .arena = arena,
  };
}

void arena_release_stack_frame(bnd_arena_stack_frame frame) {
  arena_reset_to(frame.arena, frame.offset);
}

void arena_reset(bnd_arena *arena) {
  arena_reset_to(arena, 0);
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
//   shapes.c
// ================

#define SHAPE_BRACKET_BLOCK_CAPACITY 64

void shapes_get_bracket_properties(const bnd_config *config, count_t bracket_index, count_t *blocks, count_t *shapes, count_t *capacity) {
  const uint32_t *brackets_capacity_config = config->memory.shapes_brackets_capacity;

  count_t blocks_count = brackets_capacity_config[bracket_index] / SHAPE_BRACKET_BLOCK_CAPACITY +
                          ((brackets_capacity_config[bracket_index] & (SHAPE_BRACKET_BLOCK_CAPACITY - 1)) > 0);
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

static void epa_invalid_contact(body_support p, contact_manifold *manifold) {
  manifold->count = 1;
  manifold->normal = bnd_v3_up();

  manifold->points[0].point = bnd_v3_scale(bnd_v3_add(p.p1.point, p.p2.point), 0.5f);
  manifold->points[0].depth = 0.1f;

  manifold->points[0].witness_a = p.p1.point;
  manifold->points[0].witness_b = p.p2.point;
}

static void epa_calculate_contact(const epa_polytope *polytope, contact_manifold *manifold) {
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

  manifold->count = 1;
  manifold->points[0].point = bnd_v3_scale(bnd_v3_add(p1, p2), 0.5f);
  manifold->points[0].depth = sqrtf(node.distance);

  float length = bnd_v3_len(node.normal);
  if (length > EPSILON) {
    manifold->normal = bnd_v3_scale(node.normal, -1.0f / length);
  } else {
    manifold->normal = bnd_v3_up();
  }

  manifold->points[0].witness_a = p1;
  manifold->points[0].witness_b = p2;
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

static epa_polytope *init_polytope(const bnd_world *world, bnd_arena *arena) {
  uint16_t max_nodes = world->config.collision_detection.epa_max_nodes;
  uint64_t nodes_size = (max_nodes + 1) * sizeof(epa_polytope_node);
  uint64_t flags_size = polytope_flags_size(max_nodes);
  uint64_t free_list_size = max_nodes * sizeof(uint16_t);

  uint64_t total_polytope_size = sizeof(epa_polytope) + nodes_size + flags_size + free_list_size;
  bnd_result_ptr result = arena_alloc(arena, 8, total_polytope_size);
  if (IS_ERROR(result.error)) {
    return NULL;
  }

  uint8_t *memory = result.value;

  epa_polytope *polytope = (epa_polytope *)memory;
  memset(polytope, 0, sizeof(epa_polytope));

  memory += sizeof(epa_polytope);
  polytope->nodes = (epa_polytope_node *)memory;

  memory += nodes_size;
  polytope->flags = memory;

  memory += flags_size;
  polytope->free_list = (uint16_t *)memory;

  polytope->max_nodes = max_nodes;
  polytope->nearest_distance = FLT_MAX;

  return polytope;
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


count_t epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact_manifold *manifold) {
  PROFILER_FUNCTION_START

  count_t attempts = 0;
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame((bnd_arena *)&ctx->world->arena);

  epa_polytope *polytope = init_polytope(ctx->world, stack_frame.arena);
  if (polytope == NULL) {
    epa_invalid_contact(simplex->points[0], manifold);
    goto finish;
  }

  body_support support_point = simplex->points[0];
  if (!polytope_from_simplex(polytope, simplex)) {
    epa_invalid_contact(support_point, manifold);
    goto finish;
  }

  for (attempts = 1; attempts <= EPA_MAX_ATTEMPTS; ++attempts) {
    epa_status result = epa_run(polytope, ctx, &support_point, tolerance);
    switch(result) {
      case EPA_STATUS_OK:
        continue;

      case EPA_STATUS_CONVERGED:
        epa_calculate_contact(polytope, manifold);
        goto finish;

      default:
        epa_invalid_contact(support_point, manifold);
        goto finish;
    }
  }

  finish:
  {
    bnd_world_stats *stats = (bnd_world_stats *)&ctx->world->stats;
    stats->used_epa_nodes = polytope->node_count;
    arena_release_stack_frame(stack_frame);
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
  bnd_arena_stack_frame stack_frame = arena_new_stack_frame(&world->arena);

  epa_polytope *polytope = init_polytope(world, stack_frame.arena);
  if (polytope == NULL) {
    arena_release_stack_frame(stack_frame);
    return false;
  }

  if (!polytope_from_simplex(polytope, &debug_status->s)) {
    arena_release_stack_frame(stack_frame);
    return false;
  }

  body_support support_point = debug_status->s.points[0];
  if (debug_status->target_iteration == 0) {
    epa_debug_render_iteration(polytope, support_point, callbacks, user_data);
    arena_release_stack_frame(stack_frame);
    return true;
  }

  epa_status status = EPA_STATUS_OK;
  for (int iteration = 1; iteration < EPA_MAX_ATTEMPTS; ++iteration) {
    if (iteration > debug_status->target_iteration) {
      arena_release_stack_frame(stack_frame);
      return false;
    }

    status = epa_run(polytope, &debug_status->ctx, &support_point, world->config.collision_detection.epa_tolerance);
    if (status != EPA_STATUS_OK && status != EPA_STATUS_CONVERGED) {
      arena_release_stack_frame(stack_frame);
      return false;
    }

    if (iteration == debug_status->target_iteration) {
      bnd_v3 direction = bnd_v3_normalize(polytope->nodes[polytope->nearest].normal);
      body_support next_support = support(&debug_status->ctx, direction);

      epa_debug_render_iteration(polytope, next_support, callbacks, user_data);
      arena_release_stack_frame(stack_frame);
      return true;
    }
  }

  arena_release_stack_frame(stack_frame);
  return false;
}

#endif

// ================
//   contacts.c
// ================


typedef enum {
  SLOT_EMPTY      = 1,
  SLOT_SAME_KEY   = 2,

  SLOT_ANY        = SLOT_EMPTY | SLOT_SAME_KEY,
} hash_table_slot_flags;

static inline uint64_t cache_key_hash(uint64_t key) {
  key ^= key >> 30; key *= 0xbf58476d1ce4e5b9ULL;
  key ^= key >> 27; key *= 0x94d049bb133111ebULL;
  key ^= key >> 31;

  return key;
}


void contacts_reset(bnd_world *world) {
  contacts *contacts = &world->contacts;

  broad_contacts_set *sets[] = { &contacts->dynamics, &contacts->statics };
  for (count_t k = 0; k < 2; ++k) {
    sets[k]->first = sets[k]->last = sets[k]->free_list = UINT32_MAX;
    sets[k]->next = 0;
  }

  memset(contacts->keys, 0, sizeof(uint64_t) * contacts->hash_table_capacity);
  contacts->hash_table_entry_count = 0;
}

bnd_error contacts_init(bnd_world *world) {
  contacts *contacts = &world->contacts;
  bnd_allocator allocator = world->allocator;

  count_t hash_table_capacity = world->config.memory.hash_table_capacity;
  if (hash_table_capacity == 0 || hash_table_capacity & (hash_table_capacity - 1)) {
    // TODO make sure these conditions are false before proceeding. Watch out for reported memory value though.
  }

  ALLOC_BUFFER8(contacts->keys, sizeof(uint64_t) * hash_table_capacity);
  ALLOC_BUFFER4(contacts->indices, sizeof(count_t) * hash_table_capacity);

  memset(contacts->keys, 0, sizeof(uint64_t) * hash_table_capacity);
  contacts->hash_table_entry_count = 0;

  count_t dynamic_capacity, static_capacity;
  count_t contacts_capacity = world->config.memory.contacts_capacity;
  if (contacts_capacity & 1) {
    dynamic_capacity = (contacts_capacity >> 1) + 1;
    static_capacity = (contacts_capacity >> 1);
  } else {
    dynamic_capacity = contacts_capacity >> 1;
    static_capacity = dynamic_capacity;
  }

  ALLOC_BUFFER8(contacts->dynamics.contacts, sizeof(broad_phase_contact) * dynamic_capacity);
  ALLOC_BUFFER8(contacts->statics.contacts,  sizeof(broad_phase_contact) * static_capacity);

  contacts->hash_table_capacity = hash_table_capacity;
  contacts->dynamics.capacity = dynamic_capacity;
  contacts->statics.capacity = static_capacity;

  broad_contacts_set *sets[] = { &contacts->dynamics, &contacts->statics };
  for (count_t k = 0; k < 2; ++k) {
    sets[k]->first = sets[k]->last = sets[k]->free_list = UINT32_MAX;
    sets[k]->next = 0;
  }


  collision_detection_init();

  return OK;
}

void contacts_teardown(bnd_world *world) {
  contacts *contacts = &world->contacts;
  world->allocator.free(contacts->keys, contacts->hash_table_capacity * sizeof(uint64_t));
  world->allocator.free(contacts->indices, contacts->hash_table_capacity * sizeof(count_t));

  world->allocator.free(contacts->dynamics.contacts, contacts->dynamics.capacity * sizeof(broad_phase_contact));
  world->allocator.free(contacts->statics.contacts, contacts->statics.capacity * sizeof(broad_phase_contact));
}

uint64_t hash_table_create_key(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b, bnd_body_type type) {
  uint64_t outer_index_a = data_a->inner_lookup[index_a];
  uint64_t outer_index_b = data_b->inner_lookup[index_b];
  uint64_t gen_a = data_a->generations[outer_index_a];
  uint64_t gen_b = data_b->generations[outer_index_b];

  if (type == BND_BODY_DYNAMIC && outer_index_b > outer_index_a) {
    uint64_t tmp = outer_index_a;
    outer_index_a = outer_index_b;
    outer_index_b = tmp;

    tmp = gen_a;
    gen_a = gen_b;
    gen_b = tmp;
  }

  const uint64_t mask_23bit = 0x7FFFFF;

  uint64_t key = 0x8000000000000000; // 63-rd bit set. This indicates a valid (alive) key.
  key |= (uint64_t)type << 62;
  key |= gen_a << 54;
  key |= (outer_index_a & mask_23bit) << 31;
  key |= gen_b << 23;
  key |= outer_index_b & mask_23bit;
  
  return key;
}

bool hash_table_find_slot_for_key(const contacts *contacts, uint64_t key, count_t *slot) {
  uint64_t hash = cache_key_hash(key);
  count_t bucket_index = hash & (contacts->hash_table_capacity - 1);

  count_t i = bucket_index;
  do {
    uint64_t stored_key = contacts->keys[i];
    if (stored_key == key) {
      *slot = i;
      return true;
    }

    if (stored_key == HASH_TABLE_EMPTY) {
      return false;
    }

    i = (i + 1) & (contacts->hash_table_capacity - 1);
  } while (i != bucket_index);

  return false;
}

bool hash_table_find_empty_slot(const contacts *contacts, uint64_t key, count_t *slot) {
  uint64_t hash = cache_key_hash(key);
  count_t bucket_index = hash & (contacts->hash_table_capacity - 1);

  int32_t first_tombstone = -1;
  count_t i = bucket_index;
  do {
    uint64_t stored_key = contacts->keys[i];
    if (stored_key == key) {
      return false;
    }

    if (stored_key == HASH_TABLE_EMPTY) {
      if (first_tombstone >= 0) {
        *slot = (count_t) first_tombstone;
      } else {
        *slot = i;
      }

      return true;
    }

    if (stored_key == HASH_TABLE_TOMBSTONE && first_tombstone < 0) {
      first_tombstone = i;
    }

    i = (i + 1) & (contacts->hash_table_capacity - 1);
  } while (i != bucket_index);

  if (first_tombstone >= 0) {
    *slot = first_tombstone;
    return true;
  }

  return false;
}

bnd_error hash_table_resize_if_needed(bnd_world *world, count_t additional_count) {
  contacts *contacts = &world->contacts;

  const count_t initial_capacity = contacts->hash_table_capacity;
  const float threshold_factor = 0.75f;

  float threshold_capacity = initial_capacity * threshold_factor;
  float intended_count = (float)(contacts->hash_table_entry_count + additional_count);

  if (threshold_capacity > intended_count) {
    return OK;
  } 

  if (world->allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Not enough space for contacts and Allocator.realloc is null" };
  }

  count_t new_capacity = contacts->hash_table_capacity;
  while (new_capacity * threshold_factor < intended_count) {
    new_capacity <<= 1;
  }

  REALLOC_BUFFER8(contacts->keys, world->allocator, sizeof(uint64_t), initial_capacity, new_capacity);
  REALLOC_BUFFER4(contacts->indices, world->allocator, sizeof(count_t), initial_capacity, new_capacity);

  contacts->hash_table_capacity = new_capacity;

  memset(contacts->keys, 0, sizeof(uint64_t) * new_capacity);
  memset(contacts->indices, 0, sizeof(count_t) * new_capacity);

  count_t slot;
  broad_contacts_set *contact_sets[] = { &contacts->dynamics, &contacts->statics };

  for (count_t k = 0; k < 2; ++k) {
    broad_contacts_set *set = contact_sets[k];
    count_t index = set->first;

    while (index != UINT32_MAX) {
      const broad_phase_contact *c = &set->contacts[index];

      MUST_BE_TRUE(hash_table_find_empty_slot(contacts, c->key, &slot))

      contacts->keys[slot] = c->key;
      contacts->indices[slot] = index;

      index = c->next_body;
    }
  }

  return OK;
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

static float contact_set_area(contact_point *points, const count_t *indices, bnd_v3 origin, bnd_v3 tangent_x, bnd_v3 tangent_y) {
  bnd_v3 p[MAX_CONTACTS_PER_PAIR];
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 offset = bnd_v3_sub(points[indices[i]].point, origin);
    p[i] = (bnd_v3){
      .x = bnd_v3_dot(offset, tangent_x),
      .y = bnd_v3_dot(offset, tangent_y),
    };
  }

  sort_points(p);

  bnd_v3 hull[MAX_CONTACTS_PER_PAIR * 2];
  count_t hull_count = 0;

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    while (hull_count >= 2 && cross_2d(hull[hull_count - 2], hull[hull_count - 1], p[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = p[i];
  }

  count_t lower_count = hull_count;
  for (count_t i = MAX_CONTACTS_PER_PAIR - 1; i < MAX_CONTACTS_PER_PAIR; --i) {
    while (hull_count > lower_count && cross_2d(hull[hull_count - 2], hull[hull_count - 1], p[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = p[i];
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

static float contact_set_depth(contact_point *points, const count_t *indices) {
  float depth = 0;
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    depth += points[indices[i]].depth;
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

void contacts_filter_largest_surface_area(contact_point *points, count_t points_count, bnd_v3 normal, count_t *selected_indices) {
  count_t deepest = 0;
  for (count_t i = 1; i < points_count; ++i) {
    if (points[i].depth > points[deepest].depth) {
      deepest = i;
    }
  }

  bnd_v3 tangent_seed = fabsf(normal.y) < 0.70710678f ? bnd_v3_up() : bnd_v3_right();
  bnd_v3 tangent_x = bnd_v3_cross(tangent_seed, normal);
  if (bnd_v3_lensqr(tangent_x) <= EPSILON * EPSILON) {
    tangent_x = bnd_v3_cross(bnd_v3_forward(), normal);
  }
  tangent_x = bnd_v3_normalize(tangent_x);

  bnd_v3 tangent_y = bnd_v3_normalize(bnd_v3_cross(normal, tangent_x));
  bnd_v3 origin = points[deepest].point;

  float best_area = -FLT_MAX;
  float best_depth = -FLT_MAX;

  for (count_t i = 0; i < points_count; ++i) {
    if (i == deepest) {
      continue;
    }

    for (count_t j = i + 1; j < points_count; ++j) {
      if (j == deepest) {
        continue;
      }

      for (count_t k = j + 1; k < points_count; ++k) {
        if (k == deepest) {
          continue;
        }

        count_t indices[MAX_CONTACTS_PER_PAIR] = { deepest, i, j, k };
        float area = contact_set_area(points, indices, origin, tangent_x, tangent_y);
        float depth = contact_set_depth(points, indices);

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
    points[i] = points[selected_indices[i]];
  }
}

// ================
//   debug.c
// ================


typedef struct {
  bnd_debug_draw_contact_fn callback;
  void *user_data;
} draw_contact_data;

static bnd_error draw_contact(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data) {
  (void)world;
  (void)contacts;
  (void)type;
  (void)index;
  draw_contact_data *data = (draw_contact_data *) custom_data;

  for (count_t i = 0; i < contact->manifold.count; ++i) {
    const contact_point *p = &contact->manifold.points[i];

    data->callback(p->point, contact->manifold.normal, p->depth, data->user_data);
  }
 
  return OK;
}

static void draw_contacts(const bnd_world *world, bnd_debug_draw_callbacks callbacks, void *user_data) {
  if (callbacks.draw_contact == NULL) {
    return;
  }
  draw_contact_data data = { callbacks.draw_contact, user_data };

  for_each_broad_contact((bnd_world *)world, draw_contact, &data);
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

      points[k] = bnd_v3_rotate(j->anchors[k], data->rotations[body_index]);
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

  contact_manifold manifold = {0};
  count_t iterations_count = epa_get_contact(
    &world->epa_debug->ctx,
    &world->epa_debug->s,
    world->config.collision_detection.epa_tolerance,
    &manifold);

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
//   collision_detection.c
// ================



#define PROFILING_BLOCK_NAME "Contacts cache"

typedef bool (*collision_detection_func)(const collision_detection_context *ctx, contact_manifold *manifold);

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

static bool aabb_intersect(const bnd_aabb *a, const bnd_aabb *b) {
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

bool body_aabb_intersect(const common_data *data_a, const common_data *data_b, count_t index_a, count_t index_b) {
  const bnd_aabb *a = &data_a->aabbs[index_a];
  const bnd_aabb *b = &data_b->aabbs[index_b];

  return aabb_intersect(a, b);
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

static bool sphere_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 center_a = body_a_center(ctx);
  bnd_v3 center_b = body_b_center(ctx);

  float radius_a = ctx->shape_a.value.sphere.radius;
  float radius_b = ctx->shape_b.value.sphere.radius;

  bnd_v3 offset = bnd_v3_sub(center_a, center_b);
  float distance = bnd_v3_len(offset);
  float penetration = distance - radius_a - radius_b;
  if (penetration > 0) {
    return false;
  }

  manifold->count = 1;
  manifold->normal = distance > EPSILON ? bnd_v3_scale(offset, 1 / distance) : bnd_v3_up();

  manifold->points[0].point = bnd_v3_add(center_b, bnd_v3_scale(manifold->normal, radius_b + penetration));
  manifold->points[0].depth = -penetration;

  return true;
}

static bool capsule_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
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
      manifold->count = 1;
      manifold->normal = horizontal_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_right(), capsule_rotation);

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      manifold->points[0].depth = capsule_radius - horizontal_distance + sphere_radius;

      return true;
    } else if (horizontal_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(horizontal_offset, capsule_radius / horizontal_distance);
      closest.y = local_sphere_center.y;

      manifold->count = 1;
      manifold->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(horizontal_offset), capsule_rotation));

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      manifold->points[0].depth = sphere_radius - horizontal_distance + capsule_radius;

      return true;
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
      manifold->count = 1;
      manifold->normal = cap_distance > EPSILON
        ? bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation))
        : bnd_v3_rotate(bnd_v3_up(), capsule_rotation);

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(local_sphere_center, capsule_rotation));
      manifold->points[0].depth = capsule_radius - cap_distance + sphere_radius;

      return true;
    } else if (cap_distance < capsule_radius + sphere_radius) {
      bnd_v3 closest = bnd_v3_scale(cap_offset, capsule_radius / cap_distance);
      closest = bnd_v3_add(cap, closest);

      manifold->count = 1;
      manifold->normal = bnd_v3_normalize(bnd_v3_rotate(bnd_v3_negate(cap_offset), capsule_rotation));

      manifold->points[0].point = bnd_v3_add(capsule_center, bnd_v3_rotate(closest, capsule_rotation));
      manifold->points[0].depth = sphere_radius - cap_distance + capsule_radius;

      return true;
    }
  }

  return false;
}

static bool box_sphere_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
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
    return false;
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

  bnd_v3 normal;
  memcpy(&normal, local_normal, sizeof(normal));

  manifold->count = 1;
  manifold->normal = bnd_v3_rotate(normal, box_rotation);
  manifold->points[0].point = bnd_v3_add(box_center, bnd_v3_rotate(closest, box_rotation));
  manifold->points[0].depth = depth;

  return true;
}

static bool box_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_quat box_rotation = ctx->data_a->rotations[ctx->body_a];
  bnd_quat shape_rotation = ctx->shape_a.rotation;

  bnd_v3 box_center = body_a_center(ctx);
  bnd_v3 extents = bnd_v3_scale(ctx->shape_a.value.box.size, 0.5f);

  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];

  bnd_v3 corners[8];
  box_corners(extents, corners);

  const count_t max_contacts = 4;

  count_t contact_count = 0;
  for (count_t i = 0; i < 8 && contact_count < max_contacts; ++i) {
    bnd_v3 corner = bnd_v3_add(box_center, bnd_v3_rotate(bnd_v3_rotate(corners[i], shape_rotation), box_rotation));
    float distance = bnd_v3_dot(bnd_v3_sub(corner, plane_point), plane_normal);
    if (distance > 0) {
      continue;
    }

    manifold->points[contact_count].point = bnd_v3_add(corner, bnd_v3_scale(plane_normal, -0.5f * distance));
    manifold->points[contact_count].depth = -distance;
    manifold->points[contact_count].witness_a = corner;
    manifold->points[contact_count].witness_b = bnd_v3_zero();

    contact_count += 1;
  }

  manifold->normal = plane_normal;
  manifold->count = contact_count;

  return contact_count > 0;
}

static bool sphere_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 sphere_center = body_a_center(ctx);
  float sphere_radius = ctx->shape_a.value.sphere.radius;

  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  float plane_sphere_distance = bnd_v3_dot(bnd_v3_sub(sphere_center, plane_point), plane_normal);
  if (plane_sphere_distance > sphere_radius) {
    return false;
  }

  manifold->count = 1;
  manifold->normal = plane_normal;
  manifold->points[0].point = bnd_v3_add(sphere_center, bnd_v3_scale(plane_normal, -plane_sphere_distance));
  manifold->points[0].depth = sphere_radius - plane_sphere_distance;

  return true;
}

static bool capsule_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
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

    manifold->points[contact_count].point = bnd_v3_add(points[i], bnd_v3_scale(plane_normal, -d));
    manifold->points[contact_count].depth = capsule_radius - d;
    manifold->points[contact_count].witness_a = points[i];
    manifold->points[contact_count].witness_b = bnd_v3_zero();

    contact_count += 1;
  }

  manifold->normal = plane_normal;
  manifold->count = contact_count;

  return contact_count > 0;
}

static bool mesh_plane_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  bnd_v3 plane_point = ctx->data_b->positions[ctx->body_b];
  bnd_v3 plane_normal = ctx->shape_b.value.plane.normal;

  bnd_v3 mesh_center = body_a_center(ctx);
  bnd_quat mesh_rotation = bnd_quat_mul(ctx->data_a->rotations[ctx->body_a], ctx->shape_a.rotation);
  bnd_quat inv_mesh_rotation = bnd_quat_invert(mesh_rotation);

  bnd_v3 local_normal = bnd_v3_rotate(plane_normal, inv_mesh_rotation);
  bnd_v3 local_point = bnd_v3_rotate(bnd_v3_sub(plane_point, mesh_center), inv_mesh_rotation);

  const mesh_storage *meshes = &ctx->world->meshes;
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
    return false;
  }

  bnd_v3 point = meshes->verticies[collision_vertex];
  point = bnd_v3_rotate(point, mesh_rotation);
  point = bnd_v3_add(point, mesh_center);
  point = bnd_v3_add(point, bnd_v3_scale(plane_normal, -min_dot)); // Project the deepest vertex back on the plane.

  manifold->count = 1;
  manifold->normal = plane_normal;
  manifold->points[0].point = point;
  manifold->points[0].depth = -min_dot;
  manifold->points[0].witness_a = point;
  manifold->points[0].witness_b = bnd_v3_zero();

  return true;
}

static bool polytope_polytope_collision(const collision_detection_context *ctx, contact_manifold *manifold) {
  simplex s;
  if (!gjk_check_intersection(ctx->world, ctx, &s)) {
    return false;
  }

  epa_get_contact(ctx, &s, ctx->world->config.collision_detection.epa_tolerance, manifold);
  return true;
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

void collision_detection_init(void) {
  memset(collision_detection_table, 0, sizeof(collision_detection_table));

  collision_detection_table[BND_SPHERE][BND_SPHERE]   = (collision_detection_entry) { sphere_sphere_collision, true, false };
  collision_detection_table[BND_BOX][BND_BOX]         = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_CAPSULE] = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_MESH][BND_MESH]       = (collision_detection_entry) { polytope_polytope_collision, true, true };

  collision_detection_table[BND_BOX][BND_SPHERE]      = (collision_detection_entry) { box_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_BOX]      = (collision_detection_entry) { box_sphere_collision, false, false };
  collision_detection_table[BND_BOX][BND_CAPSULE]     = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_CAPSULE][BND_BOX]     = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_CAPSULE][BND_SPHERE]  = (collision_detection_entry) { capsule_sphere_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_CAPSULE]  = (collision_detection_entry) { capsule_sphere_collision, false, false };

  collision_detection_table[BND_MESH][BND_BOX]        = (collision_detection_entry) { polytope_polytope_collision, true, true };
  collision_detection_table[BND_BOX][BND_MESH]        = (collision_detection_entry) { polytope_polytope_collision, false, true };
  collision_detection_table[BND_MESH][BND_SPHERE]     = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_SPHERE][BND_MESH]     = (collision_detection_entry) { polytope_polytope_collision, false, false };
  collision_detection_table[BND_MESH][BND_CAPSULE]    = (collision_detection_entry) { polytope_polytope_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_MESH]    = (collision_detection_entry) { polytope_polytope_collision, false, false };

  collision_detection_table[BND_BOX][BND_PLANE]       = (collision_detection_entry) { box_plane_collision, true, true };
  collision_detection_table[BND_SPHERE][BND_PLANE]    = (collision_detection_entry) { sphere_plane_collision, true, false };
  collision_detection_table[BND_CAPSULE][BND_PLANE]   = (collision_detection_entry) { capsule_plane_collision, true, true };
  collision_detection_table[BND_MESH][BND_PLANE]      = (collision_detection_entry) { mesh_plane_collision, true, true };
}

bnd_error for_each_broad_contact(bnd_world *world, broad_contact_iterator func, void *custom_data) {
  broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };
  for (count_t k = 0; k < 2; ++k) {
    broad_contacts_set *contacts = sets[k];

    count_t body_contact_index = contacts->first;
    while (body_contact_index != UINT32_MAX) {
      broad_phase_contact *body_contact = &contacts->contacts[body_contact_index];

      count_t shape_contact_index = body_contact_index;
      while (shape_contact_index != UINT32_MAX) {
        broad_phase_contact *shape_contact = &contacts->contacts[shape_contact_index];

        PROPAGATE_ERROR(func(world, contacts, (bnd_body_type)k, shape_contact, shape_contact_index, custom_data));

        shape_contact_index = shape_contact->next;
      }

      body_contact_index = body_contact->next_body;
    }
  }

  return OK;
}

static void update_contact_status(broad_phase_contact *contact, bool is_intersection) {
  bool did_touch = contact->status & CONTACT_TOUCHING;

  if (is_intersection) {
    contact->status |= CONTACT_TOUCHING; 
    if (!did_touch) {
      contact->status |= CONTACT_BEGAN_TOUCHING;
    } else {
      contact->status &= ~CONTACT_BEGAN_TOUCHING;
    }
  } else if (did_touch) {
    contact->status |= CONTACT_FINISHED_TOUCHING;
    contact->status &= ~(CONTACT_TOUCHING | CONTACT_BEGAN_TOUCHING);
  } else {
    contact->status &= ~CONTACT_FINISHED_TOUCHING;
  }
}


static void update_manifold(const collision_detection_context *ctx, contact_manifold *target, const contact_manifold *new_manifold, const contact_manifold *old_manifold) {
  *target = *new_manifold;

  if (new_manifold->count == 0) {
    return;
  }

  const float distance = ctx->world->config.collision_detection.feature_distance_threshold;
  const float distance_sqr = distance * distance;

  const float separation_threshold = ctx->world->config.collision_detection.separation_threshold;

  bnd_v3 position_a = body_a_center(ctx);
  bnd_v3 position_b = body_b_center(ctx);
  bnd_quat rotation_a = body_a_rotation(ctx);
  bnd_quat rotation_b = body_b_rotation(ctx);
  bnd_quat inverse_a = bnd_quat_invert(rotation_a);
  bnd_quat inverse_b = bnd_quat_invert(rotation_b);

  bnd_v3 old_world_normal = bnd_v3_rotate(old_manifold->local_normal, rotation_b);
  bnd_v3 new_world_normal = new_manifold->normal;
  target->local_normal = bnd_v3_rotate(new_world_normal, inverse_b);

  contact_point candidates[MAX_CONTACTS_PER_PAIR * 2];

  count_t count = new_manifold->count;
  for (count_t i = 0; i < count; ++i) {
    candidates[i] = new_manifold->points[i];

    candidates[i].witness_a = bnd_v3_rotate(bnd_v3_sub(candidates[i].witness_a, position_a), inverse_a);
    candidates[i].witness_b = bnd_v3_rotate(bnd_v3_sub(candidates[i].witness_b, position_b), inverse_b);
  }

  bool normals_align = bnd_v3_dot(old_world_normal, new_world_normal) >= 0.95f;
  count_t cached_count = normals_align ? old_manifold->count : 0;

  for (count_t i = 0; i < cached_count; ++i) {
    contact_point point = old_manifold->points[i];

    bnd_v3 witness_a = bnd_v3_add(bnd_v3_rotate(point.witness_a, rotation_a), position_a);
    bnd_v3 witness_b = bnd_v3_add(bnd_v3_rotate(point.witness_b, rotation_b), position_b);

    bnd_v3 offset = bnd_v3_sub(witness_a, witness_b);
    float separation = bnd_v3_dot(offset, new_world_normal);
    bnd_v3 drift = bnd_v3_sub(offset, bnd_v3_scale(new_world_normal, separation));
    if (separation > separation_threshold || bnd_v3_lensqr(drift) > distance_sqr) {
      continue;
    }

    count_t match = count;
    for (count_t j = 0; j < count; ++j) {
      if (bnd_v3_distancesqr(point.witness_a, candidates[j].witness_a) <= distance_sqr &&
          bnd_v3_distancesqr(point.witness_b, candidates[j].witness_b) <= distance_sqr) {
        match = j;
        break;
      }
    }

    if (match < count) {
      // Pass the stored impulses to the new point.
      candidates[match].normal_impulse = point.normal_impulse;
      memcpy(candidates[match].tangential_impulse, point.tangential_impulse, sizeof(point.tangential_impulse));
      continue;
    }

    point.point = bnd_v3_scale(bnd_v3_add(witness_a, witness_b), 0.5f);
    point.depth = -separation;
    candidates[count++] = point;
  }

  if (count > MAX_CONTACTS_PER_PAIR) {
    contact_point reduction[MAX_CONTACTS_PER_PAIR * 2] = {0};
    for (count_t i = 0; i < count; ++i) {
      reduction[i].point = candidates[i].point;
      reduction[i].depth = candidates[i].depth;
    }

    count_t selected[MAX_CONTACTS_PER_PAIR];
    contacts_filter_largest_surface_area(reduction, count, new_world_normal, selected);

    for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
      target->points[i] = candidates[selected[i]];
    }
    target->count = MAX_CONTACTS_PER_PAIR;
  } else {
    memcpy(target->points, candidates, count * sizeof(contact_point));
    target->count = count;
  }
}

static bnd_error detect_narrow_collisions(bnd_world *world, broad_contacts_set *contacts, bnd_body_type type, broad_phase_contact *contact, count_t index, void *custom_data) {
  (void)custom_data;
  (void)index;
  (void)contacts;

  common_data *data_a = (common_data *)&world->dynamics;
  common_data *data_b = type == BND_BODY_DYNAMIC ? (common_data *)&world->dynamics : (common_data *)&world->statics;

  count_t body_a = data_a->outer_lookup[contact->body_a].index;
  count_t body_b = data_b->outer_lookup[contact->body_b].index;

  bnd_body_shape *shapes_a = shapes_get(world, data_a->shapes[body_a]);
  bnd_body_shape *shapes_b = shapes_get(world, data_b->shapes[body_b]);
  collision_detection_context ctx = {
    world,
    data_a,
    data_b,
    0,
    body_a,
    body_b,
    shapes_a[contact->shape_a],
    shapes_b[contact->shape_b],
  };

  contact_manifold new_manifold = {0};
  collision_detection_entry entry = collision_detection_table[ctx.shape_a.type][ctx.shape_b.type];
  collision_detection_context context = entry.primary ? ctx : ctx_inverse(ctx);

  bool intersection = entry.func(&context, &new_manifold);

  if (!entry.primary) {
    new_manifold.normal = bnd_v3_negate(new_manifold.normal);
    for (count_t i = 0; i < new_manifold.count; ++i) {
      contact_point *point = &new_manifold.points[i];

      bnd_v3 witness = point->witness_a;
      point->witness_a = point->witness_b;
      point->witness_b = witness;
    }
  }

  contact_manifold current_manifold = contact->manifold;

  if (entry.use_cache) {
    update_manifold(&ctx, &contact->manifold, &new_manifold, &current_manifold);
  } else {
    contact->manifold = new_manifold;
  }

  update_contact_status(contact, intersection);

  return OK;
}

bnd_error run_narrow_phase(bnd_world *world) {
  PROFILER_FUNCTION_START
  bnd_error e = for_each_broad_contact(world, detect_narrow_collisions, NULL);
  PROFILER_FUNCTION_END

  return e;
}

static bool find_existing_shapes_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts, count_t shape_a, count_t shape_b, broad_phase_contact **contact, broad_phase_contact **prev_contact, count_t *contact_index, count_t *prev_contact_index) {
  count_t index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[index];

  *prev_contact = NULL;
  *prev_contact_index = UINT32_MAX;
  if (c->shape_a == shape_a && c->shape_b == shape_b) {
    // Shapes contact in question is a root body contact. Now find the previous body contact.
    *contact = c;
    *contact_index = index;

    count_t next = contacts->first;
    while (next != index) {
      *prev_contact_index = next;
      *prev_contact = &contacts->contacts[next];
      next = (*prev_contact)->next_body;
    }

    return true;
  }

  broad_phase_contact *prev = NULL;
  do {
    if (c->next == UINT32_MAX) {
      return false;
    }

    *prev_contact_index = index;
    *contact_index = c->next;
    prev = c;

    index = c->next;
    c = &contacts->contacts[index];
  } while(c->shape_a != shape_a || c->shape_b != shape_b);

  *contact = c;
  *prev_contact = prev;

  return true;
}

static void free_list_append(broad_contacts_set *contacts, broad_phase_contact *contact, count_t index) {
  count_t current_head = contacts->free_list;
  contacts->free_list = index;
  contact->next = current_head;
}

static bool free_list_pop(broad_contacts_set *contacts, count_t *index) {
  if (contacts->free_list == UINT32_MAX) {
    return false;
  }

  *index = contacts->free_list;
  contacts->free_list = contacts->contacts[contacts->free_list].next;

  return true;
}

static bnd_error new_contact_index(bnd_world *world, broad_contacts_set *contacts, count_t *index) {
  if (contacts->next == contacts->capacity && contacts->free_list == UINT32_MAX) {
    PROPAGATE_ERROR(resize_force(world->allocator, (void **)&contacts->contacts, sizeof(broad_phase_contact), ALIGNMENT_BROAD_CONTACT, 2, &contacts->capacity));
  }

  if (!free_list_pop(contacts, index)) {
    *index = contacts->next++;
  }

  return OK;
}

static bnd_error create_shapes_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts,  broad_phase_contact **new_contact) {
  count_t index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[index];

  while (c->next != UINT32_MAX) {
    index = c->next;
    c = &contacts->contacts[index];
  }

  PROPAGATE_ERROR(new_contact_index(world, contacts, &index));

  c->next = index;

  *new_contact = &contacts->contacts[index];

  return OK;
}

static bnd_error create_body_contact(bnd_world *world, uint64_t hash_key, broad_contacts_set *contacts, broad_phase_contact **new_contact, count_t *hash_slot) {
  count_t slot, contact_index;

  PROPAGATE_ERROR(new_contact_index(world, contacts, &contact_index))
  PROPAGATE_ERROR(hash_table_resize_if_needed(world, 1))

  MUST_BE_TRUE(hash_table_find_empty_slot(&world->contacts, hash_key, &slot))

  world->contacts.keys[slot] = hash_key;
  world->contacts.indices[slot] = contact_index;
  world->contacts.hash_table_entry_count += 1;

  if (contacts->last != UINT32_MAX) {
    broad_phase_contact *last_contact = &contacts->contacts[contacts->last];
    last_contact->next_body = contact_index;
  }

  contacts->last = contact_index;

  if (contacts->first == UINT32_MAX) {
    contacts->first = contact_index;
  }

  *new_contact = &contacts->contacts[contact_index];
  *hash_slot = slot;

  return OK;
}

static bnd_error remove_body_contact(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts, broad_phase_contact *contact, broad_phase_contact *prev_contact, count_t index, count_t prev_index, bnd_body_type contact_type, bool *body_contact_still_exists) {
  if (contact->next == UINT32_MAX) {
    // Body contact has no attached shape contacts. Remove it altogether.
    world->contacts.keys[hash_slot] = HASH_TABLE_TOMBSTONE;
    world->contacts.hash_table_entry_count -= 1;

    if (prev_contact) {
      prev_contact->next_body = contact->next_body;
    } else {
      contacts->first = contact->next_body;
    }

    if (contacts->last == index) {
      contacts->last = prev_index;
    }

    *body_contact_still_exists = false;

    if (contact->status & CONTACT_TOUCHING) {
      PROPAGATE_ERROR(events_emit_finish(world, contacts, hash_slot, contact_type, contact->status & CONTACT_TRIGGER_BOTH));
    }
  } else {
    // Make the next attached shape contact the new body contact.
    broad_phase_contact *next_contact = &contacts->contacts[contact->next];
    next_contact->next_body = contact->next_body;
    if (prev_contact) {
      prev_contact->next_body = contact->next;
    } else {
      contacts->first = contact->next;
    }

    if (contacts->last == index) {
      contacts->last = contact->next;
    }

    world->contacts.indices[hash_slot] = contact->next;
    *body_contact_still_exists = true;
  }

  return OK;
}

static bnd_error remove_all_shape_contacts(bnd_world *world, count_t hash_slot, broad_contacts_set *contacts, bnd_body_type contact_type) {
  world->contacts.keys[hash_slot] = HASH_TABLE_TOMBSTONE;
  world->contacts.hash_table_entry_count -= 1;

  count_t contact_index = world->contacts.indices[hash_slot];
  broad_phase_contact *c = &contacts->contacts[contact_index];

  if (contacts->first == contact_index) {
    contacts->first = c->next_body;

    if (contacts->last == contact_index) {
      // At this point both will be UINT32_MAX
      contacts->last = c->next_body;
    }
  } else {
    count_t prev_index = contacts->first;
    broad_phase_contact *prev_contact = &contacts->contacts[prev_index];
    count_t next_index = prev_contact->next_body;

    while(next_index != contact_index) {
      prev_index = next_index;
      prev_contact = &contacts->contacts[prev_index];
      next_index = prev_contact->next_body;
    }

    prev_contact->next_body = c->next_body;

    if (contacts->last == contact_index) {
      contacts->last = prev_index;
    }
  }

  broad_contact_status accumulated_status = CONTACT_NONE;
  do {
    count_t next_contact = c->next;
    free_list_append(contacts, c, contact_index);

    accumulated_status |= c->status;

    contact_index = next_contact;
    if (next_contact != UINT32_MAX) {
      c = &contacts->contacts[next_contact];
    }
  } while (contact_index != UINT32_MAX);

  if (accumulated_status & CONTACT_TOUCHING) {
    PROPAGATE_ERROR(events_emit_finish(world, contacts, hash_slot, contact_type, accumulated_status & CONTACT_TRIGGER_BOTH));
  }

  return OK;
}

static void init_contact(broad_phase_contact *contact, uint64_t key, const collision_detection_context *ctx, count_t shape_a, count_t shape_b, broad_contact_status status) {
  contact->key = key;
  contact->body_a = ctx->data_a->inner_lookup[ctx->body_a];
  contact->body_b = ctx->data_b->inner_lookup[ctx->body_b];
  contact->shape_a = shape_a;
  contact->shape_b = shape_b;

  memset(&contact->manifold, 0, sizeof(contact_manifold));

  // Per-shape materials maybe??
  contact->friction = mix_friction(ctx);
  contact->restitution = mix_restitution(ctx);

  contact->next = UINT32_MAX;
  contact->next_body = UINT32_MAX;
  contact->status = status;
}

static bnd_error run_broad_phase_typed(bnd_world *world, broad_contacts_set *contact_set, bnd_body_type type) {
  collision_detection_context ctx = { .world = world };

  contacts *contacts = &world->contacts;

  common_data *data_a = (common_data *) &world->dynamics;
  common_data *data_b = (common_data *) as_common(world, type);


  for (count_t i = 0; i < data_a->count; ++i) {
    count_t until = type == BND_BODY_DYNAMIC ? i : data_b->count;
    for (count_t j = 0; j < until; j++) {
      count_t body_a = i;
      count_t body_b = j;
      if (type == BND_BODY_DYNAMIC && data_a->inner_lookup[i] > data_b->inner_lookup[j]) {
        body_a = j;
        body_b = i;
      }

      uint8_t flags_a = data_a->flags[body_a];
      uint8_t flags_b = data_b->flags[body_b];

      uint8_t trigger_flags = flags_a & BODY_FLAG_TRIGGER;
      trigger_flags |= (flags_b & BODY_FLAG_TRIGGER) << 1;

      broad_contact_status contact_trigger_status = (trigger_flags << 3) & CONTACT_TRIGGER_BOTH;
      if (contact_trigger_status == CONTACT_TRIGGER_BOTH) {
        continue;
      }

      ctx.data_a = data_a;
      ctx.data_b = data_b;

      ctx.body_a = body_a;
      ctx.body_b = body_b;

      bnd_collision_mask validation_mask = layer_to_mask(data_a->collision_layers[body_a]);
      bnd_collision_mask reference_mask = world->matrix.matrix[data_b->collision_layers[body_b]];

      if ((reference_mask & validation_mask) == 0) {
        continue;
      }

      bool potential_overlap = body_aabb_intersect(data_a, data_b, body_a, body_b);

      count_t slot;
      uint64_t key = hash_table_create_key(data_a, data_b, body_a, body_b, type);
      bool body_contact_exists = hash_table_find_slot_for_key(contacts, key, &slot);

      if (potential_overlap) {
        body_shapes shapes_a = data_a->shapes[body_a];
        body_shapes shapes_b = data_b->shapes[body_b];

        bnd_body_shape *shapes_buffer_a = shapes_get(world, shapes_a);
        bnd_body_shape *shapes_buffer_b = shapes_get(world, shapes_b);

        for (count_t sa = 0; sa < shapes_a.count; ++sa) {
          ctx.shape_a = shapes_buffer_a[sa];
          for (count_t sb = 0; sb < shapes_b.count; ++sb) {
            ctx.shape_b = shapes_buffer_b[sb];
            
            bnd_aabb a = {
              .center = body_a_center(&ctx),
              .half_extents = bounding_box_extents(world, ctx.shape_a.type, ctx.shape_a.value, body_a_rotation(&ctx))
            };

            bnd_aabb b = {
              .center = body_b_center(&ctx),
              .half_extents = bounding_box_extents(world, ctx.shape_b.type, ctx.shape_b.value, body_b_rotation(&ctx)),
            };

            bool shapes_overlap = aabb_intersect(&a, &b);
 
            count_t contact_index, prev_contact_index;
            broad_phase_contact *shapes_contact, *prev_contact;
            if (shapes_overlap) {
              if (body_contact_exists) {
                if (!find_existing_shapes_contact(world, slot, contact_set, sa, sb, &shapes_contact, &prev_contact, &contact_index, &prev_contact_index)) {
                  // Shapes potentially overlap, there is a root contact for the bodies, but not for the shapes.
                  PROPAGATE_ERROR(create_shapes_contact(world, slot, contact_set, &shapes_contact));
                  init_contact(shapes_contact, key, &ctx, sa, sb, contact_trigger_status);
                } else {
                  // Shapes potentially overlap and there is already a contact - do nothing.
                }
              } else { 
                // Shapes potentially overlap but there is not even a body contact. 
                PROPAGATE_ERROR(create_body_contact(world, key, contact_set, &shapes_contact, &slot));
                init_contact(shapes_contact, key, &ctx, sa, sb, contact_trigger_status);
                body_contact_exists = true;
              }
            } else { 
              // Shapes cannot overlap at all.
              if (body_contact_exists) {
                if (find_existing_shapes_contact(world, slot, contact_set, sa, sb, &shapes_contact, &prev_contact, &contact_index, &prev_contact_index)) {
                  // Should remove the existing contact.
                  if (prev_contact == NULL || shapes_contact->key != prev_contact->key) {
                    // Prev contact is for a different body or doesn't exist. That means that current contact is a root body contact.
                    remove_body_contact(world, slot, contact_set, shapes_contact, prev_contact, contact_index, prev_contact_index, type, &body_contact_exists);
                  } else {
                    // Simply remove one of the shape contacts.
                    prev_contact->next = shapes_contact->next;
                  }

                  free_list_append(contact_set, shapes_contact, contact_index);
                } else {
                  // No overlap and no contact - all good.
                }
              }
            }
          }
        }
      } else if (!potential_overlap && body_contact_exists) {
        PROPAGATE_ERROR(remove_all_shape_contacts(world, slot, contact_set, type));
      }
    }
  }

  return OK;
}

void contacts_remove_for_body(bnd_world *world, bnd_body_handle handle) {
  broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };

  for (count_t k = 0; k < 2; ++k) {
    if (k == 0 && handle.type == BND_BODY_STATIC) {
      continue;
    }

    broad_contacts_set *contacts = sets[k];
    count_t index = contacts->first;
    while(index != UINT32_MAX) {
      broad_phase_contact *contact = &contacts->contacts[index];
      index = contact->next_body;

      bool static_match = handle.type == BND_BODY_STATIC && contact->body_b == handle.index;
      bool full_dynamic_match = k == 0 && handle.type == BND_BODY_DYNAMIC && (contact->body_a == handle.index || contact->body_b == handle.index);
      bool partial_dynamic_match = k == 1 && handle.type == BND_BODY_DYNAMIC && contact->body_a == handle.index;
      if (static_match || full_dynamic_match || partial_dynamic_match) {
        count_t slot;
        if (hash_table_find_slot_for_key(&world->contacts, contact->key, &slot)) {
          remove_all_shape_contacts(world, slot, contacts, handle.type);
        }
      }
    }
  }
}

bnd_error run_broad_phase(bnd_world *world) {
  PROFILER_FUNCTION_START
  
  bnd_error e = run_broad_phase_typed(world, &world->contacts.dynamics, BND_BODY_DYNAMIC);
  if (IS_ERROR(e)) {
    PROFILER_FUNCTION_END
    return e;
  }

  e = run_broad_phase_typed(world, &world->contacts.statics, BND_BODY_STATIC);
  if (IS_ERROR(e)) {
    PROFILER_FUNCTION_END
    return e;
  }


  PROFILER_FUNCTION_END
  return OK;
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

bool events_subscribed(const common_data *data, count_t index, int mask) {
  return data->event_masks[index] & mask;
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

static bnd_error push_collision_events(bnd_world *world, const broad_phase_contact *contact, common_data *data, count_t body, count_t other_body, bnd_body_type body_type, bnd_body_type other_body_type) {
  bnd_body_handle body_a = make_body_handle(world, body_type, body);
  bnd_body_handle body_b = make_body_handle(world, other_body_type, other_body);

  bnd_contact contact_data = {
    .body_a = body_a,
    .body_b = body_b,
  };

  for (count_t i = 0; i < contact->manifold.count; ++i) {
    const contact_point *point = &contact->manifold.points[i];

    contact_data.point  = point->point;
    contact_data.normal = contact->manifold.normal;
    contact_data.depth  = point->depth;

    if (contact->status & CONTACT_BEGAN_TOUCHING && events_subscribed(data, body, BND_EVENT_COLLISION_ENTER)) {
      PROPAGATE_ERROR(events_push(world, data, body, (bnd_event) { .type = BND_EVENT_COLLISION_ENTER, .collision = contact_data }));
    } else if (contact->status & CONTACT_TOUCHING && events_subscribed(data, body, BND_EVENT_COLLISION)) {
      PROPAGATE_ERROR(events_push(world, data, body, (bnd_event) { .type = BND_EVENT_COLLISION, .collision = contact_data }));
    }
  }

  return OK;
}

static bnd_error push_trigger_events(bnd_world *world, common_data *data, count_t body, count_t other_body, bnd_body_type other_body_type, broad_contact_status accumulated_status) {
  bnd_trigger trigger = { .other = make_body_handle(world, other_body_type, other_body) };

  if (accumulated_status & CONTACT_TOUCHING) {
    if (events_subscribed(data, body, BND_EVENT_TRIGGER)) {
      PROPAGATE_ERROR(events_push(world, data, body, (bnd_event) { .type = BND_EVENT_TRIGGER, .trigger = trigger }));
    }
  } else if (accumulated_status & CONTACT_BEGAN_TOUCHING) {
    if (events_subscribed(data, body, BND_EVENT_TRIGGER_ENTER)) {
      PROPAGATE_ERROR(events_push(world, data, body, (bnd_event) { .type = BND_EVENT_TRIGGER_ENTER, .trigger = trigger }));
    }
  } else if (accumulated_status & CONTACT_FINISHED_TOUCHING) {
    if (events_subscribed(data, body, BND_EVENT_TRIGGER_FINISH)) {
      PROPAGATE_ERROR(events_push(world, data, body, (bnd_event) { .type = BND_EVENT_TRIGGER_FINISH, .trigger = trigger }));
    }
  }

  return OK;
}

bnd_error events_emit_contacts(bnd_world *world) {
  enum {
    TRIGGER_SUBSCRIPTION_MASK   = BND_EVENT_TRIGGER   | BND_EVENT_TRIGGER_ENTER   | BND_EVENT_TRIGGER_FINISH,
    COLLISION_SUBSCRIPTION_MASK = BND_EVENT_COLLISION | BND_EVENT_COLLISION_ENTER | BND_EVENT_COLLISION_FINISH,
  };

  broad_contacts_set *sets[] = { &world->contacts.dynamics, &world->contacts.statics };
  for (count_t k = 0; k < 2; ++k) {
    broad_contacts_set *contacts = sets[k];

    count_t body_contact_index = contacts->first;
    while (body_contact_index != UINT32_MAX) {
      broad_phase_contact *body_contact = &contacts->contacts[body_contact_index];
      bnd_body_type contact_type = (bnd_body_type) k;

      common_data *data_a = (common_data *) &world->dynamics;
      common_data *data_b = as_common(world, contact_type);

      count_t body_a = data_a->outer_lookup[body_contact->body_a].index;
      count_t body_b = data_b->outer_lookup[body_contact->body_b].index;

      broad_contact_status accumulated_status = CONTACT_NONE;
      count_t shape_contact_index = body_contact_index;
      while (shape_contact_index != UINT32_MAX) {
        broad_phase_contact *shape_contact = &contacts->contacts[shape_contact_index];
        accumulated_status |= shape_contact->status;

        if ((body_contact->status & CONTACT_TRIGGER_BOTH) == 0) {
          if (events_subscribed(data_a, body_a, COLLISION_SUBSCRIPTION_MASK)) {
            PROPAGATE_ERROR(push_collision_events(world, shape_contact, data_a, body_a, body_b, BND_BODY_DYNAMIC, contact_type));
          }

          if (events_subscribed(data_b, body_b, COLLISION_SUBSCRIPTION_MASK)) {
            PROPAGATE_ERROR(push_collision_events(world, shape_contact, data_b, body_b, body_a, contact_type, BND_BODY_DYNAMIC));
          }
        }

        shape_contact_index = shape_contact->next;
      }

      if (body_contact->status & CONTACT_TRIGGER_BOTH) {
        // Trigger pair. Events only occur for a pair, not on per-shape basis.
        if (events_subscribed(data_a, body_a, TRIGGER_SUBSCRIPTION_MASK)) {
          PROPAGATE_ERROR(push_trigger_events(world, data_a, body_a, body_b, contact_type, accumulated_status));
        }

        if (events_subscribed(data_b, body_b, TRIGGER_SUBSCRIPTION_MASK)) {
          PROPAGATE_ERROR(push_trigger_events(world, data_b, body_b, body_a, BND_BODY_DYNAMIC, accumulated_status));
        }
      } else if ((accumulated_status & ~CONTACT_TRIGGER_BOTH) == CONTACT_FINISHED_TOUCHING) {
        // Collision pair that stopped touching. Can also occur only once per a pair of bodies.
        if (events_subscribed(data_a, body_a, BND_EVENT_COLLISION_FINISH)) {
          PROPAGATE_ERROR(events_push(world, data_a, body_a, (bnd_event) { .type = BND_EVENT_COLLISION_FINISH }))
        }

        if (events_subscribed(data_b, body_b, BND_EVENT_COLLISION_FINISH)) {
          PROPAGATE_ERROR(events_push(world, data_b, body_b, (bnd_event) { .type = BND_EVENT_COLLISION_FINISH }))
        }
      }

      body_contact_index = body_contact->next_body;
    }
  }

  return OK;
}

bnd_error events_emit_finish(bnd_world *world, broad_contacts_set *contact_set, count_t contact_slot, bnd_body_type contact_type, bool is_trigger) {
  bnd_event_type event_type = is_trigger
    ? BND_EVENT_TRIGGER_FINISH
    : BND_EVENT_COLLISION_FINISH;

  count_t contact_index = world->contacts.indices[contact_slot];
  const broad_phase_contact *contact = &contact_set->contacts[contact_index];

  common_data *data_a = (common_data *) &world->dynamics;
  common_data *data_b = as_common(world, contact_type);

  count_t body_a = data_a->outer_lookup[contact->body_a].index;
  count_t body_b = data_b->outer_lookup[contact->body_b].index;

  if (events_subscribed(data_a, body_a, event_type)) {
    PROPAGATE_ERROR(events_push(world, data_a, body_a, (bnd_event) { .type = event_type }));
  }

  if (events_subscribed(data_b, body_b, event_type)) {
    PROPAGATE_ERROR(events_push(world, data_b, body_b, (bnd_event) { .type = event_type }));
  }

  return OK;
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

    if (!body_aabb_intersect(data, data, ephemeral_index, i)) {
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
  for (iterations = 0; iterations < world->config.collision_detection.max_gjk_iterations; ++iterations) {
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
