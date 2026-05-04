#include "raylib.h"
#include "scenario-core.h"

Mesh rl_meshes[16];

bool has_hit;
bnd_raycast_hit raycast_hit;

static void on_error(bnd_error error, char *message, void *data) {
  TraceLog(LOG_ERROR, message);
}

void scenario_initialize(program_config *config, bnd_config *physics_config) {
  config->window_title = "Vortex";
  config->camera_position = (v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (v3){ 0, 0, 0 };

  bnd_register_error_callback(on_error);
}

void scenario_setup_scene(bnd_world *world) {
  rl_meshes[0] = GenMeshCone(1, 2, 16);
  rl_meshes[1] = GenMeshCylinder(1, 3, 16);
  rl_meshes[2] = GenMeshTorus(0.5, 3, 16, 16);

  bnd_mesh_handle cone, cylinder, torus;
  bnd_body b;
  if (import_raylib_mesh(world, rl_meshes[0], &cone)) {
    b = bnd_add_mesh_dynamic(world, 5, cone);
    *b.position = vec3(1.5, 7, 0);
  }

  if (import_raylib_mesh(world, rl_meshes[1], &cylinder)) {
    b = bnd_add_mesh_dynamic(world, 5, cylinder);
    *b.position = vec3(-1, 7, 0);
  }

  if (import_raylib_mesh(world, rl_meshes[2], &torus)) {
    b = bnd_add_mesh_static(world, torus);
    *b.position = vec3(-5, 7, 0);
  }
}

void scenario_handle_input(bnd_world *world, Camera *camera) {
  Ray r = GetScreenToWorldRay(GetMousePosition(), *camera);

  bnd_raycast_hit hit;
  if (bnd_raycast(world, r.position, r.direction, 100, 1, &hit)) {
    has_hit = true;
    raycast_hit = hit;
  } else {
    has_hit = false;
  }
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {
  if (!has_hit) {
    return;
  }

  DrawSphere(raycast_hit.point, 0.1, RED);
}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {
  for (count_t i = 0; i < 3; ++i) {
    UnloadMesh(rl_meshes[i]);
  }
}
