#include "raylib.h"
#include "raymath.h"
#include "scenario-core.h"

Mesh m;

void scenario_initialize(program_config *config, bnd_config *physics_config) {
  config->window_title = "Vortex";
  config->camera_position = (v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (v3){ 0, 0, 0 };
}

void scenario_setup_scene(bnd_world *world) {
  m = GenMeshCone(1, 2, 16);
  bnd_mesh_data data = raylib_mesh_to_bnd(m);
  bnd_mesh_handle handle = bnd_import_mesh(world, &data);
  bnd_body b = bnd_add_mesh_dynamic(world, 5, handle);
  *b.position = vec3(1.5, 7, 0);
  *b.rotation = QuaternionFromEuler(0, 0, PI / 4);

  register_mesh_for_rendering(handle, m);

  bnd_body box = bnd_add_box_static(world, vec3(3, 0.5, 2));
  *box.position = vec3(1.5, 0.25, 1);

  bnd_body sphere = bnd_add_sphere_dynamic(world, 5, 1);
  *sphere.position = vec3(1.5, 1.5, 1);
}

void scenario_handle_input(bnd_world *world, Camera *camera) {}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {}
