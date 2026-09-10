#include "bnd-core.h"

typedef struct {
  bnd_body_type type;
  count_t contact_index;

  count_t body_a, body_b;
  bnd_v3 normal;

  float friction, restitution;
} contact_constraint;

void resolve_constraints(bnd_world *world, float dt) {
  (void) world;
  (void) dt;
}
