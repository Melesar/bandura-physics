#include "scenario-core.h"
#include "bnd-math.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>
#include <string.h>

Mesh arrow_base;
Mesh arrow_head;
Material mat;

static void set_arrow_color(Color c) { mat.maps[MATERIAL_MAP_DIFFUSE].color = c; }

void init_debugging() {
  arrow_base = GenMeshCylinder(0.1, 1, 8);
  arrow_head = GenMeshCone(0.2, 0.5, 8);
  mat = LoadMaterialDefault();
}

void draw_arrow(bnd_v3 start, bnd_v3 direction, Color color) {
  const float scale_factor = 0.2;

  bnd_v3 end = Vector3Add(start, direction);
  float distance = Vector3Length(direction);
  if (distance < EPSILON) {
    return;
  }
  bnd_v3 n = Vector3Scale(direction, 1.0 / distance);

  set_arrow_color(color);

  Matrix base_translation = MatrixTranslate(start.x, start.y, start.z);
  Matrix base_rotation = QuaternionToMatrix(QuaternionFromVector3ToVector3((Vector3){ 0, 1, 0 }, n));
  Matrix base_scale = MatrixScale(scale_factor, distance, scale_factor);
  Matrix base_transform = MatrixMultiply(MatrixMultiply(base_scale, base_rotation), base_translation);

  Matrix head_translation = MatrixTranslate(end.x, end.y, end.z);
  Matrix head_rotation = base_rotation;
  Matrix head_scale = MatrixScale(scale_factor, scale_factor, scale_factor);
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

static void draw_bounding_boxes_typed(const bnd_world *world, bnd_body_type type, Color color) {
  bnd_body_enumerator_typed enumerator;
  bnd_enumerate_bodies_typed(world, type, &enumerator);

  while (bnd_body_next_typed(world, &enumerator)) {
    bnd_aabb bounding_box = bnd_get_bounding_box(world, enumerator.handle).value;

    DrawCubeWires(
      bounding_box.center,
      2 * bounding_box.half_extents.x,
      2 * bounding_box.half_extents.y,
      2 * bounding_box.half_extents.z,
      color
    );
  }
}

void draw_bounding_boxes(const bnd_world *world) {
  draw_bounding_boxes_typed(world, BND_BODY_DYNAMIC, ORANGE);
  draw_bounding_boxes_typed(world, BND_BODY_STATIC, GREEN);
}

static bnd_v3 joint_attachment_point(const bnd_world *world, bnd_joint j, uint32_t index) {
  bnd_v3 position = bnd_get_position(world, j.bodies[index]).value;
  bnd_quat rotation = bnd_get_rotation(world, j.bodies[index]).value;

  bnd_v3 point = j.relative_contact_positions[index];
  point = Vector3RotateByQuaternion(point, rotation);
  point = Vector3Add(point, position);

  return point;
}

void physics_draw_collisions(const bnd_world *world) {
#define max_count 50

  bnd_contact contacts[max_count];
  uint32_t count = bnd_get_contacts(world, contacts, max_count);

  for (uint32_t i = 0; i < count; ++i) {
    bnd_contact contact = contacts[i];

    draw_arrow(contact.point, Vector3Scale(contact.normal, 0.2), RED);
  }

  const bnd_joint *joints = bnd_get_joints(world, &count);
  for (uint32_t i = 0; i < count; ++i) {
    bnd_v3 p1 = joint_attachment_point(world, joints[i], 0);
    bnd_v3 p2 = joint_attachment_point(world, joints[i], 1);

    DrawSphere(p1, 0.05, GREEN);
    DrawSphere(p2, 0.05, BLUE);
  }

#undef max_count
}

ragdoll ragdoll_create(bnd_world *world, bnd_v3 position) {
  bnd_body_handle head = bnd_add_sphere_dynamic(world, 3, 0.4).value;
  bnd_set_position(world, head, Vector3Add(position, (Vector3){ 0, 5, 0}));

  bnd_body_handle torso = bnd_add_cylinder_dynamic(world, 25, 0.3, 1.0).value;
  bnd_set_position(world, torso, Vector3Add(position, (Vector3){ 0, 4, 0}));

  bnd_body_handle pelvis = bnd_add_cylinder_dynamic(world, 20, 0.25, 1.0).value;
  bnd_set_position(world, pelvis, Vector3Add(position, (Vector3){ 0, 3, 0}));

  bnd_body_handle left_upper_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2).value;
  bnd_set_position(world, left_upper_leg, Vector3Add(position, (Vector3){ 0.23, 1.8, -0.2}));
  bnd_set_rotation(world, left_upper_leg, QuaternionFromEuler(PI / 6, 0, 0));

  bnd_body_handle left_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2).value;
  bnd_set_position(world, left_lower_leg, Vector3Add(position,(Vector3) {0.23, 0.6, -0.2}));
  bnd_set_rotation(world, left_lower_leg, QuaternionFromEuler(-PI / 6, 0, 0));

  bnd_body_handle right_upper_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2).value;
  bnd_set_position(world, right_upper_leg, bnd_v3_add(position, (Vector3){-0.23, 1.8, 0}));

  bnd_body_handle right_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2).value;
  bnd_set_position(world, right_lower_leg, bnd_v3_add(position, (Vector3){-0.23, 0.6, 0}));

  bnd_body_handle left_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2).value;
  bnd_set_position(world, left_upper_arm, bnd_v3_add(position, (Vector3){0.4, 3.9, -0.4}));
  bnd_set_rotation(world, left_upper_arm, QuaternionFromEuler(PI / 5, 0, 0));

  bnd_body_handle left_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2).value;
  bnd_set_position(world, left_lower_arm, bnd_v3_add(position, (Vector3){0.43, 3.37, -1.45}));
  bnd_set_rotation(world, left_lower_arm, QuaternionFromEuler(PI / 2, 0, 0));

  bnd_body_handle right_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2).value;
  bnd_set_position(world, right_upper_arm, bnd_v3_add(position, (Vector3){-0.43, 3.8, 0}));

  bnd_body_handle right_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2).value;
  bnd_set_position(world, right_lower_arm, bnd_v3_add(position, (Vector3){-0.43, 2.63, -0.3}));
  bnd_set_rotation(world, right_lower_arm, QuaternionFromEuler(PI / 6, 0, 0));

  const float joint_margin = 0.1;

  bnd_add_joint(world, head, torso, (Vector3){0, -0.4, 0}, (Vector3){0, 0.5, 0}, joint_margin);
  bnd_add_joint(world, torso, pelvis, (Vector3){0, -0.5, 0}, (Vector3){0, 0.5, 0}, joint_margin);

  bnd_add_joint(world, torso, left_upper_arm, (Vector3){0.3, 0.45, 0}, (Vector3){-0.1, 0.6, 0}, joint_margin);
  bnd_add_joint(world, left_upper_arm, left_lower_arm, (Vector3){0, -0.6, 0}, (Vector3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, torso, right_upper_arm, (Vector3){-0.3, 0.45, 0}, (Vector3){0.1, 0.6, 0}, joint_margin);
  bnd_add_joint(world, right_upper_arm, right_lower_arm, (Vector3){0, -0.6, 0}, (Vector3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, pelvis, left_upper_leg, (Vector3){0.23, -0.5, 0}, (Vector3){0, 0.6, 0}, joint_margin);
  bnd_add_joint(world, left_upper_leg, left_lower_leg, (Vector3){0, -0.6, 0}, (Vector3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, pelvis, right_upper_leg, (Vector3){-0.23, -0.5, 0}, (Vector3){0, 0.6, 0}, joint_margin);
  bnd_add_joint(world, right_upper_leg, right_lower_leg, (Vector3){0, -0.6, 0}, (Vector3){0, 0.6, 0}, joint_margin);

  ragdoll doll = malloc(BONE_COUNT * sizeof(bnd_body_handle));
  doll[HEAD] = head;
  doll[TORSO] = torso;
  doll[PELVIS] = pelvis;
  doll[LEFT_UPPER_ARM] = left_upper_arm;
  doll[LEFT_LOWER_ARM] = left_lower_arm;
  doll[RIGHT_UPPER_ARM] = right_upper_arm;
  doll[RIGHT_LOWER_ARM] = right_lower_arm;
  doll[LEFT_UPPER_LEG] = left_upper_leg;
  doll[LEFT_LOWER_LEG] = left_lower_leg;
  doll[RIGHT_UPPER_LEG] = right_upper_leg;
  doll[RIGHT_LOWER_LEG] = right_lower_leg;

  return doll;
}

static bnd_mesh_data raylib_mesh_to_bnd(Mesh m) {
  bnd_mesh_data data = {
    .vertex_buffer = { .buffer = m.vertices,
      .element_size = 3 * sizeof(float),
      .elements_count = m.vertexCount,
      .stride = 0 },
  };

  if (m.indices != NULL) {
    data.index_buffer.buffer = m.indices;
    data.index_buffer.element_size = sizeof(unsigned short);
    data.index_buffer.elements_count = 3 * m.triangleCount;
    data.index_buffer.stride = 0;
  } else {
    uint32_t *buffer = malloc(3 * m.triangleCount * sizeof(uint32_t));
    data.index_buffer.buffer = buffer;
    data.index_buffer.element_size = sizeof(uint32_t);
    data.index_buffer.elements_count = 3 * m.triangleCount;
    data.index_buffer.stride = 0;

    for (int i = 0; i < m.triangleCount; ++i) {
      buffer[3 * i + 0] = 3 * i + 0;
      buffer[3 * i + 1] = 3 * i + 1;
      buffer[3 * i + 2] = 3 * i + 2;
    }
  }

  return data;
}

bool import_raylib_mesh(bnd_world *world, Mesh mesh, bnd_mesh_handle *handle) {
  bnd_v3 com;
  bnd_mesh_data data = raylib_mesh_to_bnd(mesh);
  bnd_error e = bnd_import_mesh(world, &data, handle, &com);
  if (e.type != BND_OK) {
    TraceLog(LOG_ERROR, "Failed to import mesh: %s", e.message);
    return false;
  }

  for (int i = 0; i < mesh.vertexCount; ++i) {
    mesh.vertices[3 * i] = mesh.vertices[3 * i] - com.x;
    mesh.vertices[3 * i + 1] = mesh.vertices[3 * i + 1] - com.y;
    mesh.vertices[3 * i + 2] = mesh.vertices[3 * i + 2] - com.z;
  }

  UpdateMeshBuffer(mesh, RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);

  register_mesh_for_rendering(*handle, mesh);

  return true;
}
