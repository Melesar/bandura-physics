#include "core.h"
#include "bandura.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>

Mesh arrow_base;
Mesh arrow_head;
Material mat;

static void set_arrow_color(Color c) { mat.maps[MATERIAL_MAP_DIFFUSE].color = c; }

void init_debugging() {
  arrow_base = GenMeshCylinder(0.1, 1, 8);
  arrow_head = GenMeshCone(0.2, 0.5, 8);
  mat = LoadMaterialDefault();
}

void draw_arrow(Vector3 start, Vector3 direction, Color color) {
  const float scale = 0.2;

  Vector3 end = Vector3Add(start, direction);
  float distance = Vector3Length(direction);
  Vector3 n = Vector3Scale(direction, 1.0 / distance);

  set_arrow_color(color);

  Matrix base_translation = MatrixTranslate(start.x, start.y, start.z);
  Matrix base_rotation = QuaternionToMatrix(QuaternionFromVector3ToVector3((Vector3){0, 1, 0}, n));
  Matrix base_scale = MatrixScale(scale, distance, scale);
  Matrix base_transform = MatrixMultiply(MatrixMultiply(base_scale, base_rotation), base_translation);

  Matrix head_translation = MatrixTranslate(end.x, end.y, end.z);
  Matrix head_rotation = base_rotation;
  Matrix head_scale = MatrixScale(scale, scale, scale);
  Matrix head_transform = MatrixMultiply(MatrixMultiply(head_scale, head_rotation), head_translation);

  DrawMesh(arrow_base, mat, base_transform);
  DrawMesh(arrow_head, mat, head_transform);
}

void draw_model_with_wireframe(Model model, Vector3 position, float scale, Color color) {
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
  DrawModel(model, position, scale, WHITE);

  rlEnableWireMode();
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_WIREFRAME;
  DrawModel(model, position, 1.01 * scale, COLOR_WIREFRAME);
  rlDisableWireMode();
}

static v3 joint_attachment_point(const physics_world *world, joint j, count_t index) {
  v3 position = physics_get_position(world, j.bodies[index]);
  quat rotation = physics_get_rotation(world, j.bodies[index]);

  v3 point = j.relative_contact_positions[index];
  point = rotate(point, rotation);
  point = add(point, position);

  return point;
}

void physics_draw_collisions(const physics_world *world) {
#define max_count 50

  contact_t contacts[max_count];
  count_t count = physics_get_contacts(world, contacts, max_count);

  for (count_t i = 0; i < count; ++i) {
    contact_t contact = contacts[i];

    draw_arrow(contact.point, scale(contact.normal, 0.2), RED);
  }

  const joint *joints = physics_get_joints(world, &count);
  for (count_t i = 0; i < count; ++i) {
    v3 p1 = joint_attachment_point(world, joints[i], 0);
    v3 p2 = joint_attachment_point(world, joints[i], 1);

    DrawSphere(p1, 0.05, GREEN);
    DrawSphere(p2, 0.05, BLUE);
  }

#undef max_count
}

ragdoll ragdoll_create(physics_world *world, v3 position) {
  body head = physics_add_sphere_dynamic(world, 3, 0.4);
  *head.position = Vector3Add(position, vec3(0, 5, 0));

  body torso = physics_add_cylinder_dynamic(world, 25, 0.3, 1.0);
  *torso.position = Vector3Add(position, vec3(0, 4, 0));

  body pelvis = physics_add_cylinder_dynamic(world, 20, 0.25, 1.0);
  *pelvis.position = Vector3Add(position, vec3(0, 3, 0));

  body left_upper_leg = physics_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *left_upper_leg.position = Vector3Add(position, vec3(0.23, 1.8, -0.2));
  *left_upper_leg.rotation = QuaternionFromEuler(PI / 6, 0, 0);

  body left_lower_leg = physics_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *left_lower_leg.position = Vector3Add(position, vec3(0.23, 0.6, -0.2));
  *left_lower_leg.rotation = QuaternionFromEuler(-PI / 6, 0, 0);

  body right_upper_leg = physics_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_upper_leg.position = Vector3Add(position, vec3(-0.23, 1.8, 0));

  body right_lower_leg = physics_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_lower_leg.position = Vector3Add(position, vec3(-0.23, 0.6, 0));

  body left_upper_arm = physics_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_upper_arm.position = Vector3Add(position, vec3(0.4, 3.9, -0.4));
  *left_upper_arm.rotation = QuaternionFromEuler(PI / 5, 0, 0);

  body left_lower_arm = physics_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_lower_arm.position = Vector3Add(position, vec3(0.43, 3.37, -1.45));
  *left_lower_arm.rotation = QuaternionFromEuler(PI / 2, 0, 0);

  body right_upper_arm = physics_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_upper_arm.position = Vector3Add(position, vec3(-0.43, 3.8, 0));

  body right_lower_arm = physics_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_lower_arm.position = Vector3Add(position, vec3(-0.43, 2.63, -0.3));
  *right_lower_arm.rotation = QuaternionFromEuler(PI / 6, 0, 0);

  const float joint_margin = 0.1;

  physics_add_joint(world, head.handle, torso.handle, vec3(0, -0.4, 0), vec3(0, 0.5, 0), joint_margin);
  physics_add_joint(world, torso.handle, pelvis.handle, vec3(0, -0.5, 0), vec3(0, 0.5, 0), joint_margin);

  physics_add_joint(world, torso.handle, left_upper_arm.handle, vec3(0.3, 0.45, 0), vec3(-0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, left_upper_arm.handle, left_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0),
                    joint_margin);

  physics_add_joint(world, torso.handle, right_upper_arm.handle, vec3(-0.3, 0.45, 0), vec3(0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, right_upper_arm.handle, right_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0),
                    joint_margin);

  physics_add_joint(world, pelvis.handle, left_upper_leg.handle, vec3(0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, left_upper_leg.handle, left_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0),
                    joint_margin);

  physics_add_joint(world, pelvis.handle, right_upper_leg.handle, vec3(-0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, right_upper_leg.handle, right_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0),
                    joint_margin);

  ragdoll doll = malloc(BONE_COUNT * sizeof(body_handle));
  doll[HEAD] = head.handle;
  doll[TORSO] = torso.handle;
  doll[PELVIS] = pelvis.handle;
  doll[LEFT_UPPER_ARM] = left_upper_arm.handle;
  doll[LEFT_LOWER_ARM] = left_lower_arm.handle;
  doll[RIGHT_UPPER_ARM] = right_upper_arm.handle;
  doll[RIGHT_LOWER_ARM] = right_lower_arm.handle;
  doll[LEFT_UPPER_LEG] = left_upper_leg.handle;
  doll[LEFT_LOWER_LEG] = left_lower_leg.handle;
  doll[RIGHT_UPPER_LEG] = right_upper_leg.handle;
  doll[RIGHT_LOWER_LEG] = right_lower_leg.handle;

  return doll;
}
