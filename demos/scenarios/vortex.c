#include "scenario-core.h"
#include "bnd-math.h"
#include "raymath.h"
#include <assert.h>
#include <math.h>
#include <string.h>

#define SEED 100
#define VORTEX_COUNT 6

#define MAX_CONTACTS 512
#define GROUNDED_BODIES_BUFFER_SIZE 10
#define SCATTER_RADIUS 50.0f

#define FORCE_FALLOFF_DISTANCE 3
#define FORCE_ROLLOFF_FACTOR 2
#define VORTEX_VISUAL_THICKNESS 0.15f
#define VORTEX_VISUAL_SEGMENTS 48

bnd_contact contacts_buffer[MAX_CONTACTS];
uint64_t grounded_bodies_lookup[GROUNDED_BODIES_BUFFER_SIZE];

typedef struct {
  bnd_v3 position;
  float kick_radius;
  float kick_cone_angle;
  float impulse;
  float pull_force;
} vortex;

imported_mesh cone_mesh;
vortex vorticies[VORTEX_COUNT];

static void on_error(bnd_error error, char *message, void *data) {
  TraceLog(LOG_ERROR, message);
}

static vortex vortex_create(bnd_v3 pos, float pull_force, float kick_radius) {
  return (vortex){
    .position = pos, .kick_radius = kick_radius, .kick_cone_angle = PI / 4.0f, .impulse = 70, .pull_force = pull_force
  };
}

static float force_falloff(vortex v, bnd_v3 body_position) {
  float dist = bnd_v3_distance(v.position, body_position);
  float ln = logf(1 + dist / FORCE_FALLOFF_DISTANCE);
  return v.pull_force * FORCE_FALLOFF_DISTANCE / (1 + FORCE_ROLLOFF_FACTOR * ln);
}

static float random_unit_float(void) { return GetRandomValue(0, 1000000) / 1000000.0f; }

static float random_range(float min_value, float max_value) { return Lerp(min_value, max_value, random_unit_float()); }

static bnd_v3 random_scatter_position(float min_height, float max_height) {
  float angle = random_range(0.0f, 2.0f * PI);
  float radius = sqrtf(random_unit_float()) * SCATTER_RADIUS;

  return (bnd_v3){
    cosf(angle) * radius,
    random_range(min_height, max_height),
    sinf(angle) * radius,
  };
}

static void scatter_dynamic_body(bnd_world *world, bnd_body b, float min_height, float max_height) {
  *b.position = random_scatter_position(min_height, max_height);
  *b.rotation = QuaternionFromEuler(random_range(-0.35f, 0.35f), random_range(0.0f, 2.0f * PI), random_range(-0.35f, 0.35f));
  *b.angular_momentum = (bnd_v3){
    random_range(-2.5f, 2.5f),
    random_range(-2.5f, 2.5f),
    random_range(-2.5f, 2.5f),
  };
}

static bnd_v3 random_upward_cone_direction(float cone_angle) {
  if (cone_angle <= 0.0f)
    return bnd_v3_up();

  float max_angle = fminf(cone_angle, PI);
  float min_cos = cosf(max_angle);
  float cos_theta = Lerp(1.0f, min_cos, random_unit_float());
  float sin_theta = sqrtf(fmaxf(0.0f, 1.0f - cos_theta * cos_theta));
  float phi = 2.0f * PI * random_unit_float();

  return (bnd_v3){
    sin_theta * cosf(phi),
    cos_theta,
    sin_theta * sinf(phi),
  };
}

static void collect_grounded_bodies(const bnd_world *world) {
  memset(grounded_bodies_lookup, 0, GROUNDED_BODIES_BUFFER_SIZE * sizeof(uint64_t));

  count_t contacts_count = bnd_get_contacts(world, contacts_buffer, MAX_CONTACTS);
  for (count_t i = 0; i < contacts_count; ++i) {
    bnd_contact contact = contacts_buffer[i];

    // Looking for collisions with the ground, which is static
    if (contact.body_b.type != BND_STATIC)
      continue;

    count_t n;
    bnd_body_shape *shapes = bnd_get_shapes(world, contact.body_b, &n);

    // The ground is a plane.
    if (shapes[0].type != BND_PLANE)
      continue;

    count_t grounded_body_index = contact.body_a.index;

    assert(grounded_body_index < GROUNDED_BODIES_BUFFER_SIZE * 64);

    count_t element_index = grounded_body_index / 64;
    uint64_t bit_mask = (uint64_t)1 << (grounded_body_index % 64);

    grounded_bodies_lookup[element_index] |= bit_mask;
  }
}

static bool is_grounded(bnd_body_handle handle) {
  count_t element_index = handle.index / 64;
  uint64_t bit_mask = (uint64_t)1 << (handle.index % 64);

  return (grounded_bodies_lookup[element_index] & bit_mask) != 0;
}

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Vortex";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };

  physics_config->simulation.friction = 0.3;

}

void scenario_initialize(bnd_world *world) {
  bnd_register_error_callback(on_error);
  SetRandomSeed(33);

  cone_mesh.success = import_raylib_mesh(world, GenMeshCone(1, 3, 16), &cone_mesh.mesh);
}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  if (cone_mesh.success) {
    bnd_body cone = bnd_add_mesh_dynamic(world, 5, cone_mesh.mesh);
    scatter_dynamic_body(world, cone, 2, 7);
  }

  for (int i = 0; i < 15; ++i) {
    float radius = random_range(0.3f, 0.75f);
    float mass = random_range(1.2f, 4.8f);
    bnd_body sphere = bnd_add_sphere_dynamic(world, mass, radius);
    scatter_dynamic_body(world, sphere, 1.0f, 5.0f);
  }

  for (int i = 0; i < 10; ++i) {
    bnd_v3 size = {
      random_range(0.5f, 1.6f),
      random_range(0.5f, 1.7f),
      random_range(0.5f, 1.6f),
    };
    float mass = random_range(1.8f, 5.8f);
    bnd_body box = bnd_add_box_dynamic(world, mass, size);
    scatter_dynamic_body(world, box, 1.0f, 5.0f);
  }

  {
    bnd_body_shape hammer_shapes[] = {
      (bnd_body_shape){ .type = BND_BOX,
        .box = { .size = (bnd_v3){ 0.28f, 1.7f, 0.28f } },
        .offset = (bnd_v3){ 0.0f, 0.0f, 0.0f },
        .rotation = bnd_qidentity() },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.32f, .height = 1.05f },
        .offset = (bnd_v3){ 0.0f, 1.0f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, PI * 0.5f) },
    };
    float hammer_masses[] = { 2.0f, 3.2f };

    for (int i = 0; i < 10; ++i) {
      bnd_body hammer = bnd_add_compound_body_dynamic(world, hammer_shapes, hammer_masses, 2);
      scatter_dynamic_body(world, hammer, 2.0f, 6.0f);
    }
  }

  {
    bnd_body_shape dumbbell_shapes[] = {
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.16f, .height = 2.1f },
        .offset = (bnd_v3){ 0.0f, 0.0f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, PI * 0.5f) },
      (bnd_body_shape){ .type = BND_SPHERE,
        .sphere = { .radius = 0.37f },
        .offset = (bnd_v3){ 1.05f, 0.0f, 0.0f },
        .rotation = bnd_qidentity() },
      (bnd_body_shape){ .type = BND_SPHERE,
        .sphere = { .radius = 0.37f },
        .offset = (bnd_v3){ -1.05f, 0.0f, 0.0f },
        .rotation = bnd_qidentity() },
    };
    float dumbbell_masses[] = { 1.6f, 2.0f, 2.0f };

    bnd_body dumbbell = bnd_add_compound_body_dynamic(world, dumbbell_shapes, dumbbell_masses, 3);
    scatter_dynamic_body(world, dumbbell, 2.0f, 6.0f);
  }

  {
    bnd_body_shape stickman_shapes[] = {
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.18f, .height = 1.1f },
        .offset = (bnd_v3){ 0.0f, 0.90f, 0.0f },
        .rotation = bnd_qidentity() },
      (bnd_body_shape){ .type = BND_SPHERE,
        .sphere = { .radius = 0.28f },
        .offset = (bnd_v3){ 0.0f, 1.75f, 0.0f },
        .rotation = bnd_qidentity() },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.11f, .height = 1.2f },
        .offset = (bnd_v3){ 0.0f, 1.20f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, PI * 0.5f) },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.10f, .height = 1.0f },
        .offset = (bnd_v3){ -0.22f, 0.15f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, 0.18f) },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.10f, .height = 1.0f },
        .offset = (bnd_v3){ 0.22f, 0.15f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, -0.18f) },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.09f, .height = 0.9f },
        .offset = (bnd_v3){ -0.62f, 1.18f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, 1.05f) },
      (bnd_body_shape){ .type = BND_CYLINDER,
        .cylinder = { .radius = 0.09f, .height = 0.9f },
        .offset = (bnd_v3){ 0.62f, 1.18f, 0.0f },
        .rotation = QuaternionFromEuler(0.0f, 0.0f, -1.05f) },
    };
    float stickman_masses[] = { 2.1f, 1.2f, 1.0f, 0.8f, 0.8f, 0.25f, 0.25f };

    for (int i = 0; i < 5; i++) {
      bnd_body stickman = bnd_add_compound_body_dynamic(world, stickman_shapes, stickman_masses, 7);
      scatter_dynamic_body(world, stickman, 2.0f, 6.0f);
    }
  }

  bnd_body cylinder = bnd_add_cylinder_static(world, 2, 5);
  *cylinder.position = (bnd_v3){ 0, 2.5, 0 };

  vorticies[0] = vortex_create((bnd_v3){ 0, 0, -8 }, 20, 1);
  vorticies[1] = vortex_create((bnd_v3){ 8, 0, 5 }, 10, 1.5);
  vorticies[2] = vortex_create((bnd_v3){ -8, 0, 5 }, 35, 1);
  vorticies[3] = vortex_create((bnd_v3){ 10, 0, 10 }, 20, 1);
  vorticies[4] = vortex_create((bnd_v3){ 10, 0, -18 }, 10, 1.5);
  vorticies[5] = vortex_create((bnd_v3){ -10, 0, 18 }, 35, 1);
}

void scenario_simulate(bnd_world *world, float dt) {
  collect_grounded_bodies(world);

  bnd_body_enumerator_typed enumerator;
  bnd_enumerate_bodies_typed(world, BND_DYNAMIC, &enumerator);

  while (bnd_body_next_typed(world, &enumerator)) {
    if (!is_grounded(enumerator.handle))
      continue;

    bnd_v3 position = bnd_get_position(world, enumerator.handle);

    for (count_t i = 0; i < VORTEX_COUNT; ++i) {
      vortex v = vorticies[i];
      float pull_force = force_falloff(v, position);
      bnd_v3 offset = bnd_v3_sub(v.position, position);
      float distance = bnd_v3_len(offset);
      if (distance <= v.kick_radius) {
        bnd_v3 kick_direction = random_upward_cone_direction(v.kick_cone_angle);
        bnd_apply_impulse(world, enumerator.handle, bnd_v3_scale(kick_direction, v.impulse));
      } else {
        bnd_v3 direction = bnd_v3_normalize(bnd_v3_sub(v.position, position));
        bnd_apply_force(world, enumerator.handle, bnd_v3_scale(direction, pull_force));
      }
    }
  }

  bnd_simulate(world, dt);
}

void scenario_draw_scene(bnd_world *world) {
  Color vortex_fill_color = (Color){ 0x7d, 0xe3, 0xff, 0x90 };
  Color vortex_ring_color = (Color){ 0x7d, 0xe3, 0xff, 0xff };

  for (count_t i = 0; i < VORTEX_COUNT; ++i) {
    vortex v = vorticies[i];
    bnd_v3 center = v.position;
    center.y = 0.04f;

    DrawCircle3D(center, v.kick_radius, bnd_v3_right(), 90.0f, vortex_fill_color);
    DrawCylinderWires((Vector3){ center.x, center.y + VORTEX_VISUAL_THICKNESS * 0.5f, center.z }, v.kick_radius,
        v.kick_radius, VORTEX_VISUAL_THICKNESS, VORTEX_VISUAL_SEGMENTS, vortex_ring_color);
  }
}

void scenario_build_ui(bnd_world *world) {}

void scenario_handle_input(bnd_world *world, Camera *camera) {}

void scenario_teardown() {}
