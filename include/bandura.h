#ifndef BANDURA_H
#define BANDURA_H

#include <stdbool.h>
#include <stdint.h>

#define RAYMATH_DISABLE_CPP_OPERATORS
#include "raymath.h"

#define cross(x, y) Vector3CrossProduct(x, y)
#define dot(x, y) Vector3DotProduct(x, y)
#define add(x, y) Vector3Add(x, y)
#define scale(x, y) Vector3Scale(x, y)
#define normalize(x) Vector3Normalize(x)
#define sub(x, y) Vector3Subtract(x, y)
#define len(x) Vector3Length(x)
#define lensq(x) Vector3LengthSqr(x)
#define distance(x, y) Vector3Distance(x, y)
#define distancesqr(x, y) Vector3DistanceSqr(x, y)
#define vec3(x, y, z)                                                                                                  \
  (v3) { x, y, z }
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3){0, 1, 0})
#define right() ((Vector3){1, 0, 0})
#define forward() ((Vector3){0, 0, 1})
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)
#define invert(x) Vector3Invert(x)
#define barycentric(p, a, b, c) Vector3Barycenter(p, a, b, c)

#define qadd(x, y) QuaternionAdd(x, y)
#define qscale(x, y) QuaternionScale(x, y)
#define qmul(x, y) QuaternionMultiply(x, y)
#define qnormalize(x) QuaternionNormalize(x)
#define qinvert(x) QuaternionInvert(x)
#define as_matrix(x) QuaternionToMatrix(x)
#define qidentity() QuaternionIdentity()

#define mul(x, y) MatrixMultiply(x, y)
#define transpose(x) MatrixTranspose(x)
#define translate(v) MatrixTranslate(v.x, v.y, v.z)
#define inverse(x) MatrixInvert(x)
#define m4identity(x) MatrixIdentity(x)

#define vlerp(x, y, t) Vector3Lerp(x, y, t)
#define lerp(x, y, t) Lerp(x, y, t)
#define slerp(x, y, t) QuaternionSlerp(x, y, t)

typedef Vector3 v3;
typedef Vector4 v4;
typedef Quaternion quat;
typedef Matrix m4;

typedef struct {
  float m0[3]; // Row 0
  float m1[3]; // Row 1
  float m2[3]; // Row 2
} m3;

m3 matrix_identity();
m3 matrix_transpose(m3 m);
m3 matrix_inverse(m3 m);
m3 matrix_add(m3 a, m3 b);
m3 matrix_multiply(m3 a, m3 b);
m3 matrix_negate(m3 m);
v3 matrix_rotate(v3 v, m3 m);
v3 matrix_rotate_inverse(v3 v, m3 m);
m3 matrix_from_basis(v3 x, v3 y, v3 z);
m3 matrix_skew_symmetric(v3 v);
m3 matrix_initial_inertia(v3 inertia);
m3 matrix_inertia(m3 initial_inertia, quat rotation);
m3 matrix_displacement_inertia(m3 i0, v3 offset, float mass);

typedef uint32_t count_t;

typedef enum { BND_ERROR_INVALID_POLYTOPE } bnd_error;

typedef void (*bnd_error_callback)(bnd_error error_type, char *error_message, void *error_data);

typedef enum {
  BODY_DYNAMIC,
  BODY_STATIC,
} bnd_body_type;

typedef enum {
  SHAPE_BOX,
  SHAPE_SPHERE,
  SHAPE_CYLINDER,

  // Keep the plane at the end
  SHAPE_PLANE,
  SHAPES_COUNT
} bnd_shape_type;

typedef struct {
  bnd_shape_type type;

  union {
    struct {
      v3 size;
    } box;

    struct {
      v3 normal;
    } plane;

    struct {
      float radius;
    } sphere;

    struct {
      float radius;
      float height;
    } cylinder;
  };

  v3 offset;
  quat rotation;
} bnd_body_shape;

typedef struct {
  count_t type : 1;
  count_t generation : 8;
  count_t index : 23;
} bnd_body_handle;

typedef struct {
  v3 *position;
  quat *rotation;
  v3 *velocity;
  v3 *angular_momentum;

  bnd_body_handle handle;
} bnd_body;

typedef struct {
  v3 point;
  v3 normal;
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
    count_t shapes_brackets_capacity[5];
  } memory;

  struct {
    v3 gravity;
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
  v3 point;
  v3 normal;
  float depth;
  bnd_body_handle body_a, body_b;
} bnd_contact;

typedef struct {
  bnd_body_handle bodies[2];
  v3 relative_contact_positions[2];
  float max_error;
} bnd_joint;

typedef struct bnd_world_t bnd_world;

typedef struct {
  bnd_body_handle handle;
  count_t generation;
} bnd_body_enumerator;

typedef bnd_body_enumerator bnd_body_enumerator_typed;

bnd_config bnd_default_config();

bnd_world *bnd_init(const bnd_config *config);

void bnd_register_error_callback(bnd_error_callback callback);

void bnd_add_plane(bnd_world *world, v3 point, v3 normal);
bnd_body bnd_add_box_dynamic(bnd_world *world, float mass, v3 size);
bnd_body bnd_add_box_static(bnd_world *world, v3 size);
bnd_body bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);
bnd_body bnd_add_cylinder_static(bnd_world *world, float radius, float height);
bnd_body bnd_add_cylinder_dynamic(bnd_world *world, float mass, float radius, float height);
bnd_body bnd_add_compound_body_static(bnd_world *world, bnd_body_shape *shapes, count_t shapes_count);
bnd_body bnd_add_compound_body_dynamic(bnd_world *world, bnd_body_shape *shapes, float *masses, count_t shapes_count);

void bnd_remove_body(bnd_world *world, bnd_body_handle handle);

count_t bnd_add_joint(bnd_world *world, bnd_body_handle body_a, bnd_body_handle body_b, v3 contact_offset_a,
                          v3 contact_offset_b, float max_distance);
void bnd_remove_joint(bnd_world *world, count_t id);
const bnd_joint *bnd_get_joints(const bnd_world *world, count_t *count);

void bnd_apply_force(bnd_world *world, bnd_body_handle handle, v3 force);
void bnd_apply_force_at(bnd_world *world, bnd_body_handle handle, v3 force, v3 position);
void bnd_apply_impulse(bnd_world *world, bnd_body_handle handle, v3 impulse);
void bnd_apply_impulse_at(bnd_world *world, bnd_body_handle handle, v3 impulse, v3 position);

count_t bnd_body_count(const bnd_world *world, bnd_body_type type);
count_t bnd_awake_count(const bnd_world *world);
count_t bnd_collisions_count(const bnd_world *world);

bnd_config *bnd_edit_config(bnd_world *world);
bnd_world_stats bnd_stats(const bnd_world *world);

v3 bnd_get_position(const bnd_world *world, bnd_body_handle handle);
quat bnd_get_rotation(const bnd_world *world, bnd_body_handle handle);
bnd_body_shape *bnd_get_shapes(const bnd_world *world, bnd_body_handle handle, count_t *count);
v3 bnd_get_velocity(const bnd_world *world, bnd_body_handle handle);
v3 bnd_get_angular_velocity(const bnd_world *world, bnd_body_handle handle);
v3 bnd_get_angular_momentum(const bnd_world *world, bnd_body_handle handle);
m3 bnd_get_inertia(const bnd_world *world, bnd_body_handle handle);
m3 bnd_get_base_inertia(const bnd_world *world, bnd_body_handle handle);
float bnd_get_motion_avg(const bnd_world *world, bnd_body_handle handle);
count_t bnd_get_contacts(const bnd_world *world, bnd_contact *contacts, count_t max_contacts);

void bnd_enumerate_bodies_typed(const bnd_world *world, bnd_body_type type, bnd_body_enumerator_typed *enumerator);
bool bnd_body_next_typed(const bnd_world *world, bnd_body_enumerator_typed *enumerator);

void bnd_simulate(bnd_world *world, float dt);
void bnd_awaken_body(bnd_world *world, bnd_body_handle handle);
void bnd_reset_world(bnd_world *world);

count_t bnd_raycast(const bnd_world *world, v3 origin, v3 direction, float max_distance, count_t max_hits,
                        bnd_raycast_hit *hits);

void bnd_teardown(bnd_world *world);
#endif
