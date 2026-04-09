#include "raylib.h"
#include "physics.h"
#include "raymath.h"
#include "stdlib.h"
#include "string.h"

#define SOLVER_TOLERANCE 0.01
#define MAX_GJK_ATTEMPTS 20
#define GJK_TOLERANCE 0.0001





Vector3 sphere_support(Vector3 center, float radius, Vector3 direction) {
  return Vector3Add(center, Vector3Scale(direction, radius));
}

Vector3 cylinder_support(Vector3 center, float radius, float height, Quaternion rotation, Vector3 direction) {
  Quaternion inv_rotation = QuaternionInvert(rotation);
  Vector3 local_dir = Vector3RotateByQuaternion(direction, inv_rotation);

  float half_height = 0.5f * height;

  float axial_component = local_dir.y;
  Vector3 radial_dir = (Vector3){ local_dir.x, 0.0f, local_dir.z };
  float radial_magnitude = sqrtf(radial_dir.x * radial_dir.x + radial_dir.z * radial_dir.z);

  Vector3 local_support;

  if (radial_magnitude > 1e-6f) {
    local_support.x = (radial_dir.x / radial_magnitude) * radius;
    local_support.z = (radial_dir.z / radial_magnitude) * radius;
  } else {
    local_support.x = 0.0f;
    local_support.z = 0.0f;
  }
  if (axial_component > 0.0f) {
    local_support.y = half_height;
  } else {
    local_support.y = -half_height;
  }

  Vector3 world_support = Vector3RotateByQuaternion(local_support, rotation);
  return Vector3Add(center, world_support);
}

Vector3 shape_support(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b, Vector3 direction) {
  Vector3 support_a, support_b;
  if (shape_a == SHAPE_CYLINDER && shape_b == SHAPE_SPHERE) {
    support_a = cylinder_support(rb_a->p, params_a.y, params_a.x, rb_a->r, direction);
    support_b = sphere_support(rb_b->p, params_b.x, Vector3Negate(direction));
  } else if (shape_a == SHAPE_SPHERE && shape_b == SHAPE_CYLINDER) {
    support_a = sphere_support(rb_a->p, params_a.x, direction);
    support_b = cylinder_support(rb_b->p, params_b.y, params_b.x, rb_b->r, Vector3Negate(direction));
  } else {
    support_a = support_b = Vector3Zero();
  }

  return Vector3Subtract(support_a, support_b);
}

bool gjk_update_simplex(Vector3 *points, int *count, Vector3 *direction) {
  Vector3 a, b, c, d;
  Vector3 ab, ac, ad, ao;
  Vector3 abc, acd, adb;

  switch(*count) {
    case 4:
      a = points[0];
      b = points[1];
      c = points[2];
      d = points[3];

      ab = Vector3Subtract(b, a);
      ac = Vector3Subtract(c, a);
      ad = Vector3Subtract(d, a);
      ao = Vector3Negate(a);

      abc = Vector3CrossProduct(ab, ac);
      acd = Vector3CrossProduct(ac, ad);
      adb = Vector3CrossProduct(ad, ab);

      if (Vector3DotProduct(abc, ao) > 0) {
        *count = 3;
        // Fallthrough to case 3
      } else if (Vector3DotProduct(acd, ao) > 0) {
        points[1] = c;
        points[2] = d;
        *count = 3;
        //Fallthrough to case 3
      } else if (Vector3DotProduct(adb, ao) > 0) {
        points[1] = d;
        points[2] = b;
        *count = 3;
        //Fallthrough to case 3
      } else {
        return true;
      }

    case 3:
      a = points[0];
      b = points[1];
      c = points[2];

      ab = Vector3Subtract(b, a);
      ac = Vector3Subtract(c, a);
      ao = Vector3Negate(a);

      Vector3 abc = Vector3CrossProduct(ab, ac);
      if (Vector3DotProduct(Vector3CrossProduct(abc, ac), ao) > 0) {
    		if (Vector3DotProduct(ac, ao) > 0) {
    			points[1] = c;
    			*count = 2;
    			*direction = Vector3Normalize(Vector3CrossProduct(Vector3CrossProduct(ac, ao), ac));

    			return false;
    		} else {
    		  *count = 2;
    		  // Fallthrough to case 2
    		}
    	} else {
    		if (Vector3DotProduct(Vector3CrossProduct(ab, abc), ao) > 0) {
    		  *count = 2;
    		  // Fallthrough to case 2
    		} else {
    			if (Vector3DotProduct(abc, ao) > 0) {
    				*direction = Vector3Normalize(abc);
    				return false;
    			} else {
    			  points[2] = b;
    			  points[1] = c;

    				*direction = Vector3Normalize(Vector3Negate(abc));
    				return false;
    			}
    		}
    	}

    case 2:
      ab = Vector3Subtract(points[1], points[0]);
      ao = Vector3Negate(points[0]);

      if (Vector3DotProduct(ab, ao) > 0) {
        *direction = Vector3Normalize(Vector3CrossProduct(Vector3CrossProduct(ab, ao), ab));
      } else {
        *count = 1;
        *direction = Vector3Normalize(ao);
      }
      return false;

    case 1:
      *direction = Vector3Normalize(Vector3Negate(points[0]));
      return false;
  }

  return false;
}

collision gjk_check_collision(shape_type shape_a, shape_type shape_b, const rigidbody *rb_a, const rigidbody *rb_b, Vector2 params_a, Vector2 params_b) {
  collision result = {0};
  Vector3 direction = (Vector3) { 1, 0, 0 };
  int num_points = 0;
  Vector3 points[4];
  int num_attempts = 0;

  while(++num_attempts < MAX_GJK_ATTEMPTS) {
    Vector3 support = shape_support(shape_a, shape_b, rb_a, rb_b, params_a, params_b, direction);

    if (Vector3DotProduct(support, direction) < GJK_TOLERANCE) {
      return result;
    }

    points[3] = points[2];
    points[2] = points[1];
    points[1] = points[0];
    points[0] = support;

    num_points += 1;

    if (gjk_update_simplex(points, &num_points, &direction)) {
      result.valid = true;
      return result; // TODO calculate additional info about collision
    }
  }

  TraceLog(LOG_WARNING, "Max GJK attempts reached withot the result. Collision detection failed");
  return result;
}

collision cylinder_sphere_check_collision(const rigidbody *cylinder_rb, const rigidbody *sphere_rb, float cylinder_height, float cylinder_radius, float sphere_radius) {
  return check_collision(SHAPE_SPHERE, SHAPE_CYLINDER, sphere_rb, cylinder_rb, (Vector2) { sphere_radius, 0 }, (Vector2) { cylinder_height, cylinder_radius });
}

constraints constraints_new(int num_bodies, int num_constraints, int num_dof, float stabilization, int gauss_seidel_iterations) {
  constraints c;
  c.beta = stabilization;
  c.num_bodies = num_bodies;
  c.num_constraints = num_constraints;
  c.num_dof = num_dof;
  c.gauss_seidel_iterations = gauss_seidel_iterations;

  c.errors = (float*) malloc(num_constraints * sizeof(float));
  c.j = (float*) malloc(num_constraints * num_bodies * num_dof * sizeof(float));
  c.inv_m = (float*) malloc(num_dof * num_bodies * sizeof(float));
  c.v = (float*) malloc(num_bodies * num_dof * sizeof(float));

  c.a = (float*) malloc(num_constraints * num_constraints * sizeof(float));
  c.b = (float*) malloc(num_constraints * sizeof(float));
  c.lambda = (float*) malloc(num_constraints * sizeof(float));

  c.dv = (float*) malloc(num_constraints * num_dof * sizeof(float));

  return c;
}

static void gauss_seidel_solve(float* a, float* b, float* solution, int num_dimensions, int max_iterations) {
  memset(solution, 0, num_dimensions * sizeof(float));

  float max_delta = 0.0;
  for (int iter = 0; iter < max_iterations; iter++) {
    max_delta = 0.0;
    for (int i = 0; i < num_dimensions; i++) {
      float sigma = 0.0;

      for (int j = 0; j < i; j++) {
        sigma += a[i * num_dimensions + j] * solution[j];
      }

      for (int j = i + 1; j < num_dimensions; j++) {
        sigma += a[i * num_dimensions + j] * solution[j];
      }

      float x_new = (b[i] - sigma) / a[i * num_dimensions + i];
      float delta = fabs(x_new - solution[i]);
      if (delta > max_delta) {
        max_delta = delta;
      }

      solution[i] = x_new;
    }

    if (max_delta < SOLVER_TOLERANCE) {
        return;
    }
  }
}

void constraints_solve(constraints *c, float dt) {
  int row_size = c->num_bodies * c->num_dof;
  int nc = c->num_constraints;

  for (int i = 0; i < nc; i++) {
    for (int j = 0; j < nc; ++j) {
      float aij = 0;
      for (int k = 0; k < row_size; ++k) {
        aij += c->j[i * row_size + k] * c->inv_m[k] * c->j[j * row_size + k];
      }
      c->a[i * nc + j] = aij;
    }
  }

  float inv_t = 1.0 / dt;
  for (int i = 0; i < nc; ++i) {
    float bi = 0;
    for (int j = 0; j < row_size; ++j) {
      bi += c->j[i * row_size + j] * c->v[j];
    }

    c->b[i] = -(bi + c->beta * c->errors[i] * inv_t);
  }

  gauss_seidel_solve(c->a, c->b, c->lambda, nc, c->gauss_seidel_iterations);

  for (int i = 0; i < row_size; ++i) {
    float jt_lambda = 0;
    for (int j = 0; j < nc; ++j) {
      jt_lambda += c->j[j * row_size + i] * c->lambda[j];
    }
    c->dv[i] = jt_lambda * c->inv_m[i];
  }
}

void constraints_free(constraints c) {
  free(c.errors);
  free(c.j);
  free(c.inv_m);
  free(c.a);
  free(c.b);
  free(c.lambda);
  free(c.v);
  free(c.dv);
}

oscillation_period oscillation_period_new() {
  return (oscillation_period) { .timestamp = GetTime() };
}

void oscillation_period_track(oscillation_period* period, const rigidbody* current, const rigidbody* prev) {
  float prev_velocity = prev->v.x;
  float current_velocity = current->v.x;

  if (prev_velocity * current_velocity < 0) {
    period->num_turns += 1;
  }

  float num_oscillations = 0.5f * period->num_turns;
  float time_passed = GetTime() - period->timestamp;

  if (num_oscillations > 0) {
    period->period = time_passed / num_oscillations;
  }
}
