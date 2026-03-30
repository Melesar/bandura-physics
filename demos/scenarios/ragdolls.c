#include "core.h"
#include "raymath.h"
#include <stdlib.h>

typedef enum {
  HEAD,
  TORSO,
  PELVIS,
  LEFT_UPPER_LEG,
  LEFT_LOWER_LEG,
  RIGHT_UPPER_LEG,
  RIGHT_LOWER_LEG,
  LEFT_UPPER_ARM,
  LEFT_LOWER_ARM,
  RIGHT_UPPER_ARM,
  RIGHT_LOWER_ARM,

  BONE_COUNT
} bone;

typedef body* ragdoll;

ragdoll normal_doll;

ragdoll ragdoll_create(physics_world *world, v3 position) {
  ragdoll doll = malloc(BONE_COUNT * sizeof(body));

  doll[HEAD] = physics_add_sphere_dynamic(world, 0.3, 0.4);
  *doll[HEAD].position = Vector3Add(position, vec3(0, 5, 0));

  doll[TORSO] = physics_add_cylinder_dynamic(world, 0.5, 0.3, 1.0);
  *doll[TORSO].position = Vector3Add(position, vec3(0, 4, 0));

  doll[PELVIS] = physics_add_cylinder_dynamic(world, 0.5, 0.25, 1.0);
  *doll[PELVIS].position = Vector3Add(position, vec3(0, 3, 0));

  doll[LEFT_UPPER_LEG] = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *doll[LEFT_UPPER_LEG].position = Vector3Add(position, vec3(0.23, 1.8, -0.2));
  *doll[LEFT_UPPER_LEG].rotation = QuaternionFromEuler(PI / 6, 0, 0);

  doll[LEFT_LOWER_LEG] = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *doll[LEFT_LOWER_LEG].position = Vector3Add(position, vec3(0.23, 0.6, -0.2));
  *doll[LEFT_LOWER_LEG].rotation = QuaternionFromEuler(-PI / 6, 0, 0);

  doll[RIGHT_UPPER_LEG] = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *doll[RIGHT_UPPER_LEG].position = Vector3Add(position, vec3(-0.23, 1.8, 0));

  doll[RIGHT_LOWER_LEG] = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *doll[RIGHT_LOWER_LEG].position = Vector3Add(position, vec3(-0.23, 0.6, 0));

  doll[LEFT_UPPER_ARM] = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *doll[LEFT_UPPER_ARM].position = Vector3Add(position, vec3(0.4, 3.9, -0.4));
  *doll[LEFT_UPPER_ARM].rotation = QuaternionFromEuler(PI / 5, 0, 0);

  doll[LEFT_LOWER_ARM] = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *doll[LEFT_LOWER_ARM].position = Vector3Add(position, vec3(0.43, 3.37, -1.45));
  *doll[LEFT_LOWER_ARM].rotation = QuaternionFromEuler(PI / 2, 0, 0);

  doll[RIGHT_UPPER_ARM] = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *doll[RIGHT_UPPER_ARM].position = Vector3Add(position, vec3(-0.43, 3.8, 0));

  doll[RIGHT_LOWER_ARM] = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *doll[RIGHT_LOWER_ARM].position = Vector3Add(position, vec3(-0.43, 2.63, -0.3));
  *doll[RIGHT_LOWER_ARM].rotation = QuaternionFromEuler(PI / 6, 0, 0);

  const float joint_margin = 0.1;

  physics_add_joint(world, doll[HEAD].handle, doll[TORSO].handle, vec3(0, -0.4, 0), vec3(0, 0.5, 0), joint_margin);
  physics_add_joint(world, doll[TORSO].handle, doll[PELVIS].handle, vec3(0, -0.5, 0), vec3(0, 0.5, 0), joint_margin);

  physics_add_joint(world, doll[TORSO].handle, doll[LEFT_UPPER_ARM].handle, vec3(0.3, 0.45, 0), vec3(-0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, doll[LEFT_UPPER_ARM].handle, doll[LEFT_LOWER_ARM].handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, doll[TORSO].handle, doll[RIGHT_UPPER_ARM].handle, vec3(-0.3, 0.45, 0), vec3(0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, doll[RIGHT_UPPER_ARM].handle, doll[RIGHT_LOWER_ARM].handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, doll[PELVIS].handle, doll[LEFT_UPPER_LEG].handle, vec3(0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, doll[LEFT_UPPER_LEG].handle, doll[LEFT_LOWER_LEG].handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, doll[PELVIS].handle, doll[RIGHT_UPPER_LEG].handle, vec3(-0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, doll[RIGHT_UPPER_LEG].handle, doll[RIGHT_LOWER_LEG].handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  return doll;
}

body_handle ragdoll_get_bone(ragdoll doll, bone bone) {
  return doll[bone].handle;
}

void scenario_initialize(program_config* config, physics_config *physics_config) {
  config->window_title = "Ragdolls";
  config->camera_position = vec3(0, 5, -10);
  config->camera_target = vec3(0, 2, 10);
}

void scenario_setup_scene(physics_world *world) {
  if (normal_doll) {
    free(normal_doll);
  }

  normal_doll = ragdoll_create(world, scale(up(), 3));
}

void scenario_handle_input(physics_world *world, Camera *camera) {


}

void scenario_simulate(physics_world *world, float dt) {

}

void scenario_draw_scene(physics_world *world) {

}

void scenario_draw_ui(struct nk_context* ctx) {

}
