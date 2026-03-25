#include "core.h"
#include "raymath.h"

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = (v3) { 0, 5, -10 };
  config->camera_target = (v3) { 0, 5, 10 };
}

void scenario_setup_scene(physics_world *world) {
 body head = physics_add_sphere_dynamic(world, 0.3, 0.4);
 *head.position = (v3) { 0, 5, 0 };

 body neck = physics_add_cylinder_dynamic(world, 0.1, 0.1, 0.1);
 *neck.position = (v3) { 0, 4.55, 0 };

 body chest = physics_add_cylinder_dynamic(world, 0.5, 0.3, 1);
 *chest.position = (v3) { 0, 4, 0 };

 body belly = physics_add_cylinder_dynamic(world, 0.5, 0.25, 1);
 *belly.position = (v3) { 0, 2.95, 0 };

 body left_thigh = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
 *left_thigh.position = (v3) { 0.23, 1.8, -0.2 };
 *left_thigh.rotation = QuaternionFromEuler(PI / 6, 0, 0);

 body left_ancle = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
 *left_ancle.position = (v3) { 0.23, 0.6, -0.2 };
 *left_ancle.rotation = QuaternionFromEuler(-PI / 6, 0, 0);

 body right_thigh = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
 *right_thigh.position = (v3) { -0.23, 1.8, 0 };

 body right_ancle = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
 *right_ancle.position = (v3) { -0.23, 0.6, 0 };

 body left_shoulder = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
 *left_shoulder.position = (v3) { 0.4, 3.9, -0.4 };
 *left_shoulder.rotation = QuaternionFromEuler(PI / 5, 0, 0);

 body left_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
 *left_arm.position = (v3) { 0.43, 3.37, -1.45 };
 *left_arm.rotation = QuaternionFromEuler(PI / 2, 0, 0);

 body right_shoulder = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
 *right_shoulder.position = (v3) { -0.43, 3.8, 0 };

 body right_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
 *right_arm.position = (v3) { -0.43, 2.63, -0.3 };
 *right_arm.rotation = QuaternionFromEuler(PI / 6, 0, 0);
}

void scenario_handle_input(physics_world *world, Camera *camera) {


}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
