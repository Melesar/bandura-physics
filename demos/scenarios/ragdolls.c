#include "core.h"
#include "raymath.h"

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = (v3) { 0, 5, -10 };
  config->camera_target = (v3) { 0, 5, 10 };
}

void scenario_setup_scene(physics_world *world) {


}

void scenario_handle_input(physics_world *world, Camera *camera) {


}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
