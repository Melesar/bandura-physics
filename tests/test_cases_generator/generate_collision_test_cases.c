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
#define CASES_PER_PAIR (3 * ROLLS_COUNT)

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

typedef union {
  box box;
  sphere sphere;
  capsule capsule;
} shape;

typedef enum {
  BOX,
  SPHERE,
  CAPSULE,
} shape_type;

typedef struct {
  shape_type type;
  shape shape;
  ccd_vec3_t position;
  ccd_quat_t orientation;
} body;

typedef struct {
  body a;
  body b;
} collision_pair;

typedef struct {
  ccd_vec3_t position_a, position_b;
  ccd_quat_t orientation_a, orientation_b;
  ccd_vec3_t point, normal;
  float depth;
  bool intersection;
} collision_test_case;

typedef struct {
  collision_pair *pairs;
  collision_test_case *test_cases;
  body *a, *b;
  ccd_support_fn *support_1, *support_2;

  int num_pairs, num_test_cases;
} generator_state;

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

void capsule_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  body *b = (body *)obj;

  ccd_vec3_t ndir;
  ccdVec3Copy(&ndir, dir);
  ccdVec3Normalize(&ndir);

  ccd_quat_t qinv;
  ccdQuatInvert2(&qinv, &b->orientation);
  ccdQuatRotVec(&ndir, &qinv);

  ccd_vec3_t cap;
  ccdVec3Set(&cap, 0, ccdSign(ccdVec3Y(&ndir)), 0);
  ccdVec3Scale(&cap, 0.5 * b->shape.capsule.height);

  ccdVec3Copy(vec, &cap);
  ccdVec3Scale(&ndir, b->shape.capsule.radius);
  ccdVec3Add(vec, &ndir);

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

int generate_test_cases(generator_state *state, int num_cases, float r_a, float r_min, float r_max) {
  for (int i = 0; i < ROLLS_COUNT; ++i) {
    float r = r_min + (float)rand() / RAND_MAX * (r_max - r_min);

    state->a->position = random_position(r_a);
    state->b->position = random_position(r);
    ccdVec3Add(&state->b->position, &state->a->position);

    state->a->orientation = random_orinetation();
    state->b->orientation = random_orinetation();

    ccd_t ccd;
    CCD_INIT(&ccd);
    ccd.support1 = *state->support_1;
    ccd.support2 = *state->support_2;
    ccd.max_iterations = 100;

    float depth;
    ccd_vec3_t normal, point;
    int intersection = ccdGJKPenetration(state->a, state->b, &ccd, &depth, &normal, &point);

    collision_test_case *test_case = &state->test_cases[state->num_pairs * CASES_PER_PAIR + num_cases + i];
    test_case->position_a = state->a->position;
    test_case->position_b = state->b->position;
    test_case->orientation_a = state->a->orientation;
    test_case->orientation_b = state->b->orientation;
    test_case->intersection = intersection == 0;
    test_case->point = point;
    test_case->normal = normal;
    test_case->depth = depth;
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
      return 0.5 * body->shape.capsule.height + body->shape.capsule.radius;
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

void init_capsule(body *body, float height, float radius, ccd_support_fn *support) {
  body->type = CAPSULE;
  body->shape.capsule.height = height;
  body->shape.capsule.radius = radius;

  ccdVec3Set(&body->position, 0, 0, 0);
  ccdQuatSet(&body->orientation, 0, 0, 0, 1);

  *support = capsule_support;
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

    case CAPSULE:
      printf("  type: capsule\n");
      printf("  height: %.2f\n", body->shape.capsule.height);
      printf("  radius: %.2f\n", body->shape.capsule.radius);
      break;
  }
}

void generate_cases_for_bodies(generator_state *state) {
  float bbr_a = bounding_sphere_radius(state->a);
  float bbr_b = bounding_sphere_radius(state->b);

  collision_pair *pair = &state->pairs[state->num_pairs];
  pair->a = *state->a;
  pair->b = *state->b;

  int num_cases = 0;

  float no_collision_radius = bbr_a + bbr_b;
  num_cases += generate_test_cases(state, num_cases, bbr_a, no_collision_radius + 0.1f, no_collision_radius + 10.0f);

  float some_collisions_radius = bbr_a + 0.5 * bbr_b;
  num_cases += generate_test_cases(state, num_cases, bbr_a, some_collisions_radius, no_collision_radius);

  float all_collisions_radius = bbr_a;
  num_cases += generate_test_cases(state, num_cases, bbr_a, all_collisions_radius, some_collisions_radius);

  state->num_pairs += 1;
}

int main() {
  srand(time(NULL));

  const int max_pairs = 20;
  const int pairs_capacity = max_pairs * sizeof(collision_pair);
  const int test_cases_capacity = max_pairs * CASES_PER_PAIR * sizeof(collision_test_case);

  body a, b;
  ccd_support_fn support_1, support_2;

  uint8_t *memory = malloc(pairs_capacity + test_cases_capacity);

  generator_state state = { 0 };
  state.pairs = memory;
  state.test_cases = memory + pairs_capacity;
  state.a = &a;
  state.b = &b;
  state.support_1 = &support_1;
  state.support_2 = &support_2;

  init_sphere(&a, 1.6, &support_1);
  init_sphere(&b, 2.1, &support_2);

  generate_cases_for_bodies(&state);

  init_box(&b, (ccd_vec3_t)CCD_VEC3_STATIC(1.2, 3.4, 0.7), &support_2);
  generate_cases_for_bodies(&state);

  init_box(&a, (ccd_vec3_t)CCD_VEC3_STATIC(3.1, 2.9, 1.2), &support_1);
  generate_cases_for_bodies(&state);

  init_capsule(&b, 1.12, 1.8, &support_2);
  generate_cases_for_bodies(&state);

  init_capsule(&a, 2.05, 1.7, &support_1);
  generate_cases_for_bodies(&state);

  init_sphere(&a, 3.5, &support_1);
  generate_cases_for_bodies(&state);

  printf("num_pairs: %d\n", state.num_pairs);
  printf("cases_per_pair: %d\n", CASES_PER_PAIR);

  for (int i = 0; i < state.num_pairs; ++i) {
    collision_pair pair = state.pairs[i];
    printf("---\n");
    printf("shape1:\n");
    print_shape_header(&pair.a);
    printf("shape2:\n");
    print_shape_header(&pair.b);
    printf("cases:\n");

    int num_cases = 0;
    for (int j = 0; j < CASES_PER_PAIR; ++j) {
      collision_test_case test_case = state.test_cases[i * CASES_PER_PAIR + j];
      printf("  - case%d:\n", j + 1);
      printf("    positionA: (%f, %f, %f)\n", test_case.position_a.v[0], test_case.position_a.v[1], test_case.position_a.v[2]);
      printf("    positionB: (%f, %f, %f)\n", test_case.position_b.v[0], test_case.position_b.v[1], test_case.position_b.v[2]);
      printf("    orientationA: (%f, %f, %f, %f)\n", test_case.orientation_a.q[0], test_case.orientation_a.q[1], test_case.orientation_a.q[2], test_case.orientation_a.q[3]);
      printf("    orientationB: (%f, %f, %f, %f)\n", test_case.orientation_b.q[0], test_case.orientation_b.q[1], test_case.orientation_b.q[2], test_case.orientation_b.q[3]);
      printf("    intersection: %s\n", test_case.intersection ? "true" : "false");

      if (test_case.intersection) {
        printf("    point: (%f, %f, %f)\n", test_case.point.v[0], test_case.point.v[1], test_case.point.v[2]);
        printf("    normal: (%f, %f, %f)\n", test_case.normal.v[0], test_case.normal.v[1], test_case.normal.v[2]);
        printf("    depth: %f\n", test_case.depth);
      }
    }

    printf("\n");
  }
}
