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
#define vec3(x, y, z) (v3) { x, y, z }
#define zero() Vector3Zero()
#define one() Vector3One()
#define up() ((Vector3){0, 1, 0})
#define right() ((Vector3){1, 0, 0})
#define forward() ((Vector3){0, 0, 1})
#define rotate(x, y) Vector3RotateByQuaternion(x, y)
#define negate(x) Vector3Negate(x)
#define transform(x, y) Vector3Transform(x, y)
#define invert(x) Vector3Invert(x)

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

#define SMOOTH_VALUE_CAPACITY 8

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

typedef enum {
  BODY_DYNAMIC,
  BODY_STATIC,
} body_type;

typedef enum {
  SHAPE_BOX,
  SHAPE_SPHERE,
  SHAPE_PLANE,
  SHAPE_CYLINDER,

  SHAPES_COUNT
} shape_type;

typedef struct {
  shape_type type;

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
} body_shape;

typedef struct {
  count_t type : 1;
  count_t generation: 8;
  count_t index : 23;
} body_handle;

typedef struct {
  v3 *position;
  quat *rotation;
  v3 *velocity;
  v3 *angular_momentum;

  body_handle handle;
} body;

typedef struct {
  v3 point;
  v3 normal;
  float distance;
  body_handle body;
} raycast_hit;

typedef struct {
  v3 gravity;

  count_t dynamics_capacity;
  count_t statics_capacity;
  count_t contacts_capacity;
  count_t joints_capacity;
  count_t shapes_brackets_capacity[5];

  float linear_damping;
  float angular_damping;
  float restitution;
  float friction;

  count_t resolution_attempts_factor;

  float penetration_epsilon;
  float velocity_epsilon;

  float sleep_base_bias;
  float sleep_threshold;

  float restitution_damping_limit;
} physics_config;

typedef struct {
  float buffer[SMOOTH_VALUE_CAPACITY];
  uint8_t count;
  uint8_t pointer;
} smooth_value;

typedef struct {
  count_t body_count;
  count_t contacts_count;
  count_t incomplete_resolutions;
} physics_world_stats;

typedef struct {
  v3 point;
  v3 normal;
  float depth;
  body_handle body_a, body_b;
} contact_t;

typedef struct {
  body_handle bodies[2];
  v3 relative_contact_positions[2];
  float max_error;
} joint;

typedef struct physics_world_t physics_world;

typedef struct {
  body_handle handle;
  count_t generation;
} body_enumerator;

typedef body_enumerator body_enumerator_typed;

physics_config physics_default_config();

physics_world *physics_init(const physics_config *config);

void physics_add_plane(physics_world *world, v3 point, v3 normal);
body physics_add_box_dynamic(physics_world *world, float mass, v3 size);
body physics_add_box_static(physics_world *world, v3 size);
body physics_add_sphere_dynamic(physics_world *world, float mass, float radius);
body physics_add_cylinder_static(physics_world *world, float radius, float height);
body physics_add_cylinder_dynamic(physics_world *world, float mass, float radius, float height);
body physics_add_compound_body_static(physics_world *world, body_shape *shapes, count_t shapes_count);
body physics_add_compound_body_dynamic(physics_world *world, body_shape *shapes, float *masses, count_t shapes_count);

void physics_remove_body(physics_world *world, body_handle handle);

count_t physics_add_joint(physics_world *world, body_handle body_a, body_handle body_b, v3 contact_offset_a,
                          v3 contact_offset_b, float max_distance);
void physics_remove_joint(physics_world *world, count_t id);
const joint *physics_get_joints(const physics_world *world, count_t *count);

void physics_apply_force(physics_world *world, body_handle handle, v3 force);
void physics_apply_force_at(physics_world *world, body_handle handle, v3 force, v3 position);
void physics_apply_impulse(physics_world *world, body_handle handle, v3 impulse);
void physics_apply_impulse_at(physics_world *world, body_handle handle, v3 impulse, v3 position);

count_t physics_body_count(const physics_world *world, body_type type);
count_t physics_awake_count(const physics_world *world);
count_t physics_collisions_count(const physics_world *world);

physics_config *physics_edit_config(physics_world *world);
physics_world_stats physics_get_stats(const physics_world *world);

v3 physics_get_position(const physics_world *world, body_handle handle);
quat physics_get_rotation(const physics_world *world, body_handle handle);
body_shape *physics_get_shapes(const physics_world *world, body_handle handle, count_t *count);
v3 physics_get_velocity(const physics_world *world, body_handle handle);
v3 physics_get_angular_velocity(const physics_world *world, body_handle handle);
v3 physics_get_angular_momentum(const physics_world *world, body_handle handle);
m3 physics_get_inertia(const physics_world *world, body_handle handle);
m3 physics_get_base_inertia(const physics_world *world, body_handle handle);
float physics_get_motion_avg(const physics_world *world, body_handle handle);
count_t physics_get_contacts(const physics_world *world, contact_t *contacts, count_t max_contacts);

void physics_enumerate_bodies_typed(const physics_world *world, body_type type, body_enumerator_typed *enumerator);
bool physics_body_next_typed(const physics_world *world, body_enumerator_typed *enumerator);

void physics_step(physics_world *world, float dt);
void physics_awaken_body(physics_world *world, body_handle handle);
void physics_reset(physics_world *world);

count_t physics_raycast(const physics_world *world, v3 origin, v3 direction, float max_distance, count_t max_hits,
                        raycast_hit *hits);

void physics_teardown(physics_world *world);

float smooth_value_read(smooth_value v);
void smooth_value_post(smooth_value *v, float x);

#endif
