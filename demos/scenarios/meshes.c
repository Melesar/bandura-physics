#include "scenario-core.h"
#include "bnd-math.h"

Mesh rl_meshes[16];
imported_mesh imported_meshes[4];

static void on_error(bnd_error_type error, char *message, void *data) {
  TraceLog(LOG_ERROR, message);
}

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Vortex";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };

  bnd_register_error_callback(on_error);
}

void scenario_initialize(bnd_world *world) {
  rl_meshes[0] = GenMeshCone(1, 2, 16);
  rl_meshes[1] = GenMeshCylinder(1, 3, 16);
  rl_meshes[2] = GenMeshTorus(0.5, 3, 16, 16);
  rl_meshes[3] = GenMeshSphere(1, 16, 16);

  imported_meshes[0].success = import_raylib_mesh(world, rl_meshes[0], &imported_meshes[0].mesh);
  imported_meshes[1].success = import_raylib_mesh(world, rl_meshes[1], &imported_meshes[1].mesh);
  imported_meshes[2].success = import_raylib_mesh(world, rl_meshes[2], &imported_meshes[2].mesh);
  imported_meshes[3].success = import_raylib_mesh(world, rl_meshes[3], &imported_meshes[3].mesh);
}

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  bnd_body_handle b;
  if (imported_meshes[0].success) {
    b = bnd_add_mesh_dynamic(world, 5, imported_meshes[0].mesh);
    bnd_set_position(world, b, (bnd_v3){1.5, 7, 0});
  }

  if (imported_meshes[1].success) {
    b = bnd_add_mesh_dynamic(world, 5, imported_meshes[1].mesh);
    bnd_set_position(world, b, (bnd_v3){-1, 7, 0});
  }

  if (imported_meshes[2].success) {
    b = bnd_add_mesh_dynamic(world, 5, imported_meshes[2].mesh);
    bnd_set_position(world, b, (bnd_v3){3, 7, 0});
  }

  if (imported_meshes[3].success) {
    b = bnd_add_mesh_static(world, imported_meshes[3].mesh);
    bnd_set_position(world, b, (bnd_v3){-5, 7, 0});
  }
}

void scenario_handle_input(bnd_world *world, Camera *camera) {}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {}

void scenario_teardown() {
  for (uint32_t i = 0; i < 4; ++i) {
    UnloadMesh(rl_meshes[i]);
  }
}
