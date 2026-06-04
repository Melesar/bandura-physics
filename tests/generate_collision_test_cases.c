#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "ccd/ccd.h"
#include "ccd/quat.h"

#define PI 3.14159265358979323846
#define ROLLS_COUNT 25

typedef struct {
  ccd_vec3_t half_size;
} box;

typedef struct {
  float radius;
} sphere;

typedef struct {
  float radius;
  float height;
} capsule;

typedef struct {
  ccd_vec3_t normal;
} plane;

typedef union {
  box box;
  sphere sphere;
  capsule capsule;
  plane plane;
} shape;

typedef enum {
  BOX,
  SPHERE,
  CAPSULE,
  PLANE,
} shape_type;

typedef struct {
  shape_type type;
  shape shape;
  ccd_vec3_t position;
  ccd_quat_t orientation;
} body;

void sphere_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  body *b = (body *)obj;
  ccd_vec3_t center = b->position;

  ccd_vec3_t ndir;
  ccdVec3Copy(&ndir, dir);
  ccdVec3Normalize(&ndir);

  ccdVec3Copy(vec, &ndir);
  ccdVec3Scale(vec, b->shape.sphere.radius);
  ccdVec3Add(vec, &center);
}

void box_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  body *b = (body *)obj;

  ccd_vec3_t ndir;
  ccdVec3Copy(&ndir, dir);
  ccdVec3Normalize(&ndir);

  ccd_quat_t qinv;
  ccdQuatInvert2(&qinv, &b->orientation);
  ccdQuatRotVec(&ndir, &qinv);

  ccdVec3Set(vec, ccdSign(ccdVec3X(&ndir)) * ccdVec3X(&b->shape.box.half_size),
                ccdSign(ccdVec3Y(&ndir)) * ccdVec3Y(&b->shape.box.half_size),
                ccdSign(ccdVec3Z(&ndir)) * ccdVec3Z(&b->shape.box.half_size));

  ccdQuatRotVec(vec, &b->orientation);
  ccdVec3Add(vec, &b->position);
}

ccd_vec3_t random_position(float radius) {
  // theta: polar angle [0, pi], phi: azimuthal angle [0, 2*pi]
  float theta = (float)rand() / RAND_MAX * PI;
  float phi   = (float)rand() / RAND_MAX * 2.0f * PI;

  float x = radius * sinf(theta) * cosf(phi);
  float y = radius * sinf(theta) * sinf(phi);
  float z = radius * cosf(theta);

  ccd_vec3_t result;
  ccdVec3Set(&result, x, y, z);
  return result;
}

ccd_quat_t random_orinetation() {
  // Box-Muller: generate 4 independent normal-distributed values,
  // then normalize to get a uniform point on the 4D unit sphere.
  float u[4];
  for (int i = 0; i < 4; i += 2) {
    float r = sqrtf(-2.0f * logf((float)rand() / RAND_MAX + 1e-10f));
    float theta = 2.0f * PI * (float)rand() / RAND_MAX;
    u[i]     = r * cosf(theta);
    u[i + 1] = r * sinf(theta);
  }

  float len = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2] + u[3]*u[3]);
  ccd_quat_t q;
  ccdQuatSet(&q, u[0]/len, u[1]/len, u[2]/len, u[3]/len);
  return q;
}

int generate_test_cases(body* a, body* b, int num_cases, float r_min, float r_max, ccd_support_fn support1, ccd_support_fn support2) {
  for (int i = 0; i < ROLLS_COUNT; ++i) {
    float r = r_min + (float)rand() / RAND_MAX * (r_max - r_min);

    b->position = random_position(r);

    a->orientation = random_orinetation();
    b->orientation = random_orinetation();

    ccd_t ccd;
    CCD_INIT(&ccd);
    ccd.support1 = support1;
    ccd.support2 = support2;

    int intersection = ccdGJKIntersect(a, b, &ccd);

    printf("  - case%d:\n", num_cases + i + 1);
    printf("    positionA: (%f, %f, %f)\n", a->position.v[0], a->position.v[1], a->position.v[2]);
    printf("    positionB: (%f, %f, %f)\n", b->position.v[0], b->position.v[1], b->position.v[2]);
    printf("    orientationA: (%f, %f, %f, %f)\n", a->orientation.q[0], a->orientation.q[1], a->orientation.q[2], a->orientation.q[3]);
    printf("    orientationB: (%f, %f, %f, %f)\n", b->orientation.q[0], b->orientation.q[1], b->orientation.q[2], b->orientation.q[3]);
    printf("    intersection: %s\n", intersection ? "true" : "false");
  }

  return ROLLS_COUNT;
}

float bounding_sphere_radius(const body *body) {
  ccd_vec3_t extents;
  switch(body->type) {
    case SPHERE:
      return body->shape.sphere.radius;

    case BOX:
      ccdVec3Copy(&extents, &body->shape.box.half_size);
      return fmaxf(ccdVec3X(&extents), fmaxf(ccdVec3Y(&extents), ccdVec3Z(&extents)));

    case CAPSULE:
    case PLANE:
      return 0;
  }
}

void init_sphere(body* body, float radius, ccd_support_fn *support) {
  body->type = SPHERE;
  body->shape.sphere.radius = radius;

  ccdVec3Set(&body->position, 0, 0, 0);
  ccdQuatSet(&body->orientation, 0, 0, 0, 1);

  *support = sphere_support;
}

void init_box(body *body, ccd_vec3_t half_extents, ccd_support_fn *support) {
  body->type = BOX;
  ccdVec3Copy(&body->shape.box.half_size, &half_extents);

  ccdVec3Set(&body->position, 0, 0, 0);
  ccdQuatSet(&body->orientation, 0, 0, 0, 1);

  *support = box_support;
}

void print_shape_header(const body *body) {
  switch (body->type) {
    case SPHERE:
      printf("  type: sphere\n");
      printf("  radius: %.2f\n", body->shape.sphere.radius);
      break;
    case BOX:
      printf("  type: box\n");
      printf("  half_extents: (%.2f, %.2f, %.2f)\n", ccdVec3X(&body->shape.box.half_size), ccdVec3Y(&body->shape.box.half_size), ccdVec3Z(&body->shape.box.half_size));
      break;
  }
}

void generate_cases_for_bodies(body *a, body *b, ccd_support_fn support1, ccd_support_fn support2) {
  float bbr_a = bounding_sphere_radius(a);
  float bbr_b = bounding_sphere_radius(b);

  printf("---\n");
  printf("shape1:\n");
  print_shape_header(a);
  printf("shape2:\n");
  print_shape_header(b);
  printf("cases:\n");

  int num_cases = 0;

  float no_collision_radius = bbr_a + bbr_b;
  num_cases += generate_test_cases(a, b, num_cases, no_collision_radius + 0.1f, no_collision_radius + 10.0f, support1, support2);

  float some_collisions_radius = bbr_a + 0.5 * bbr_b;
  num_cases += generate_test_cases(a, b, num_cases, some_collisions_radius, no_collision_radius, support1, support2);

  float all_collisions_radius = bbr_a;
  num_cases += generate_test_cases(a, b, num_cases, all_collisions_radius, some_collisions_radius, support1, support2);

  printf("\n");
}

int main() {
  srand(time(NULL));

  body a, b;
  ccd_support_fn support_1, support_2;
  init_sphere(&a, 1.6, &support_1);
  init_sphere(&b, 2.1, &support_2);

  generate_cases_for_bodies(&a, &b, support_1, support_2);

  init_box(&b, (ccd_vec3_t)CCD_VEC3_STATIC(1.2, 3.4, 0.7), &support_2);
  generate_cases_for_bodies(&a, &b, support_1, support_2);

  init_box(&a, (ccd_vec3_t)CCD_VEC3_STATIC(3.1, 2.9, 1.2), &support_1);
  generate_cases_for_bodies(&a, &b, support_1, support_2);
}
