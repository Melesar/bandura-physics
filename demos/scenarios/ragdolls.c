#include "scenario-core.h"
#include "bnd-math.h"
#include "raygui.h"
#include "raylib.h"
#include <stdlib.h>

ragdoll hanging_doll;
ragdoll normal_doll;

bool widget_collapsed;

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = (bnd_v3){0, 5, -10};
  config->camera_target = (bnd_v3){0, 2, 10};
}

void scenario_initialize(bnd_world *world) { }

void scenario_setup_scene(bnd_world *world) {
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  normal_doll = ragdoll_create(world, bnd_v3_scale(bnd_v3_up(), 3));

  bnd_body_shape ramp_shapes[] = {
      (bnd_body_shape){
        .type = BND_BOX, .value.box = {.size = (bnd_v3){1, 10, 1}}, .offset = (bnd_v3){0, 5, 0}, .rotation = bnd_qidentity()},
      (bnd_body_shape){
        .type = BND_BOX, .value.box = {.size = (bnd_v3){3, 1, 1}}, .offset = (bnd_v3){1, 10, 0}, .rotation = bnd_qidentity()},
  };

  bnd_body_handle ramp = bnd_add_compound_body_static(world, ramp_shapes, 2).value;
  bnd_set_position(world, ramp, (bnd_v3) { 5, 0, 5 });

  hanging_doll = ragdoll_create(world, (bnd_v3){8, 5, 5});
  bnd_add_joint(world, ramp, hanging_doll[RIGHT_LOWER_ARM], (bnd_v3){3, 10, 0}, (bnd_v3){0, -0.6, 0}, 0.05);
}

void scenario_handle_input(bnd_world *world, Camera *camera) {
  if (IsKeyPressed(KEY_J)) {
    bnd_apply_impulse(world, normal_doll[PELVIS], bnd_v3_scale(bnd_v3_up(), 12));
  }
}

void scenario_simulate(bnd_world *world, float dt) { bnd_simulate(world, dt); }

void scenario_draw_scene(bnd_world *world) {}

void scenario_build_ui(bnd_world *world) {
  if (ui_begin_area("Ragdolls", &widget_collapsed)) {
    for (bone b = HEAD; b < BONE_COUNT; ++b) {
      bnd_body_handle body = hanging_doll[b];
      float velocity = bnd_v3_len(bnd_get_velocity(world, body).value);
      float angular_velocity = bnd_v3_len(bnd_get_angular_velocity(world, body).value);
      float angular_momentum = bnd_v3_len(bnd_get_angular_momentum(world, body).value);

      char *bone_name;
      switch (b) {
        case HEAD:
          bone_name = "Head";
          break;
        case TORSO:
          bone_name = "Torso";
          break;
        case PELVIS:
          bone_name = "Pelvis";
          break;
        case LEFT_UPPER_LEG:
          bone_name = "Left upper leg";
          break;
        case LEFT_LOWER_LEG:
          bone_name = "Left lower leg";
          break;
        case RIGHT_UPPER_LEG:
          bone_name = "Right upper leg";
          break;
        case RIGHT_LOWER_LEG:
          bone_name = "Right lower leg";
          break;
        case LEFT_UPPER_ARM:
          bone_name = "Left upper arm";
          break;
        case LEFT_LOWER_ARM:
          bone_name = "Left lower arm";
          break;
        case RIGHT_UPPER_ARM:
          bone_name = "Right upper arm";
          break;
        case RIGHT_LOWER_ARM:
          bone_name = "Right lower arm";
          break;
        default:
          break;
      }

      CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
        ui_label(bone_name);

        CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .padding = {.left = 20}}}) {
          ui_label_float("- v :", velocity);
          ui_label_float("- av:", angular_velocity);
          ui_label_float("- am:", angular_momentum);
        }
      }
    }
  }

  ui_end_area();
}

void scenario_teardown() {
  free(normal_doll);
  free(hanging_doll);
}
