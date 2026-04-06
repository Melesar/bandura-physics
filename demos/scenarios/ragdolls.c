#include "core.h"
#include "raygui.h"
#include "raylib.h"
#include <stdlib.h>

ragdoll hanging_doll;
ragdoll normal_doll;

bool widget_collapsed;

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = vec3(0, 5, -10);
  config->camera_target = vec3(0, 2, 10);
}

void scenario_setup_scene(physics_world *world) {
  normal_doll = ragdoll_create(world, scale(up(), 3));

  body_shape ramp_shapes[] = {
    (body_shape) { .type = SHAPE_BOX, .box = { .size = vec3(1, 10, 1) }, .offset = vec3(0, 5, 0), .rotation = qidentity() },
    (body_shape) { .type = SHAPE_BOX, .box = { .size = vec3(3, 1, 1) }, .offset = vec3(1, 10, 0), .rotation = qidentity() },
  };

  body ramp = physics_add_compound_body_static(world, ramp_shapes, 2);
  *ramp.position = vec3(5, 0, 5);

  hanging_doll = ragdoll_create(world, vec3(8, 5, 5));
  physics_add_joint(world, ramp.handle, hanging_doll[RIGHT_LOWER_ARM], vec3(3, 10, 0), vec3(0, -0.6, 0), 0.05);
}

void scenario_handle_input(physics_world *world, Camera *camera) {
  if (IsKeyPressed(KEY_J)) {
    physics_apply_impulse(world, normal_doll[PELVIS], scale(up(), 12));
  }
}

void scenario_simulate(physics_world *world, float dt) {
}

void scenario_draw_scene(physics_world *world) {
}

void scenario_build_ui(physics_world *world) {
  if (ui_begin_area("Ragdolls", &widget_collapsed)) {
    for(bone b = HEAD; b < BONE_COUNT; ++b) {
      body_handle body = hanging_doll[b];
      float velocity = len(physics_get_velocity(world, body));
      float angular_velocity = len(physics_get_angular_velocity(world, body));
      float angular_momentum = len(physics_get_angular_momentum(world, body));

      char *bone_name;
      switch(b) {
        case HEAD: bone_name = "Head"; break;
        case TORSO: bone_name = "Torso"; break;
        case PELVIS: bone_name = "Pelvis"; break;
        case LEFT_UPPER_LEG: bone_name = "Left upper leg"; break;
        case LEFT_LOWER_LEG: bone_name = "Left lower leg"; break;
        case RIGHT_UPPER_LEG: bone_name = "Right upper leg"; break;
        case RIGHT_LOWER_LEG: bone_name = "Right lower leg"; break;
        case LEFT_UPPER_ARM: bone_name = "Left upper arm"; break;
        case LEFT_LOWER_ARM: bone_name = "Left lower arm"; break;
        case RIGHT_UPPER_ARM: bone_name = "Right upper arm"; break;
        case RIGHT_LOWER_ARM: bone_name = "Right lower arm"; break;
        default: break;
      }

      CLAY_AUTO_ID({
        .layout = { .layoutDirection = CLAY_TOP_TO_BOTTOM }
      }) {
        ui_label(bone_name);

        CLAY_AUTO_ID({
          .layout = { .layoutDirection = CLAY_TOP_TO_BOTTOM, .padding = { .left = 20 } }
        }){
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
