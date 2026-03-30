#include "core.h"
#include "bandura.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <float.h>
#include <limits.h>
#include <stdlib.h>

Mesh arrow_base;
Mesh arrow_head;
Material mat;

static void begin_debug_row(struct nk_context* ctx) {
  nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 2);
  nk_layout_row_push(ctx, 0.1f);
  nk_label(ctx, " ", NK_TEXT_ALIGN_LEFT);
  nk_layout_row_push(ctx, 0.9f);
}

static void set_arrow_color(Color c) {
  mat.maps[MATERIAL_MAP_DIFFUSE].color = c;
}

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
  Matrix base_rotation = QuaternionToMatrix(QuaternionFromVector3ToVector3((Vector3) { 0, 1, 0 }, n));
  Matrix base_scale = MatrixScale(scale, distance, scale);
  Matrix base_transform = MatrixMultiply(MatrixMultiply(base_scale, base_rotation), base_translation);

  Matrix head_translation = MatrixTranslate(end.x, end.y, end.z);
  Matrix head_rotation = base_rotation;
  Matrix head_scale = MatrixScale(scale, scale, scale);
  Matrix head_transform = MatrixMultiply(MatrixMultiply(head_scale, head_rotation), head_translation);

  DrawMesh(arrow_base, mat, base_transform);
  DrawMesh(arrow_head, mat, head_transform);
}

void draw_stat_float(struct nk_context* ctx, char* title, float value) {
  begin_debug_row(ctx);
  nk_value_float(ctx, title, value);
  nk_layout_row_end(ctx);
}

void draw_stat_int(struct nk_context* ctx, char* title, int value) {
  begin_debug_row(ctx);
  nk_value_int(ctx, title, value);
  nk_layout_row_end(ctx);
}

void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value) {
  begin_debug_row(ctx);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s: (%.3f, %.3f, %.3f)", title, value.x, value.y, value.z);
  nk_layout_row_end(ctx);
}

void draw_stat_matrix(struct nk_context* ctx, char* title, Matrix value) {
  begin_debug_row(ctx);
  nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "%s:", title);
  nk_layout_row_end(ctx);

  for (int i = 0; i < 4; ++i) {
    float *m = ((float*)(&value) + i * 4);
    nk_layout_row_begin(ctx, NK_DYNAMIC, 15, 1);
    nk_layout_row_push(ctx, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "\t%.3f\t%.3f\t%.3f\t%.3f", m[0], m[1], m[2], m[3]);
    nk_layout_row_end(ctx);
  }
}

bool begin_widget_window(
  struct nk_context* ctx,
  const char* window_name,
  const char* title,
  float x,
  float y,
  float width,
  float row_height,
  int row_count
) {
  const nk_flags window_flags = NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_MINIMIZABLE |
    NK_WINDOW_NO_SCROLLBAR | NK_WINDOW_TITLE;

  float header_height = ctx->style.font->height + ctx->style.window.header.padding.y * 2.0f;
  float padding_y = ctx->style.window.padding.y;
  float spacing_y = ctx->style.window.spacing.y;
  float content_height = (row_height * row_count) + (spacing_y * (row_count - 1));
  float window_height = header_height + (padding_y * 2.0f) + content_height + 25.0;

  if (nk_begin_titled(ctx, window_name, title, nk_rect(x, y, width, window_height), window_flags)) {
    if (!nk_window_is_collapsed(ctx, window_name)) {
      nk_window_set_size(ctx, window_name, nk_vec2(width, window_height));
      return true;
    }
  }

  return false;
}

void draw_edit_float(struct nk_context* ctx, char* title, float* value) {
  draw_property_float(ctx, title, value, -FLT_MAX, FLT_MAX, 0.1f, 0.01f);
}

void draw_edit_int(struct nk_context* ctx, char* title, int* value) {
  draw_property_int(ctx, title, value, INT_MIN, INT_MAX, 1, 1.0f);
}

bool draw_button(struct nk_context* ctx, char* title) {
  bool pressed = false;

  begin_debug_row(ctx);
  pressed = nk_button_label(ctx, title) != 0;
  nk_layout_row_end(ctx);

  return pressed;
}

bool draw_selectable(struct nk_context* ctx, char* title, bool* selected) {
  nk_bool value = *selected ? nk_true : nk_false;
  nk_bool changed;

  begin_debug_row(ctx);
  changed = nk_selectable_label(ctx, title, NK_TEXT_ALIGN_LEFT, &value);
  nk_layout_row_end(ctx);

  *selected = value != 0;
  return changed != 0;
}

void draw_property_float(struct nk_context* ctx, char* title, float* value, float min, float max, float step_arrow, float step_drag) {
  begin_debug_row(ctx);
  nk_property_float(ctx, title, min, value, max, step_arrow, step_drag);
  nk_layout_row_end(ctx);
}

void draw_property_int(struct nk_context* ctx, char* title, int* value, int min, int max, int step, float step_drag) {
  begin_debug_row(ctx);
  nk_property_int(ctx, title, min, value, max, step, step_drag);
  nk_layout_row_end(ctx);
}

void draw_model_with_wireframe(Model model, Vector3 position, float scale, Color color) {
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
  DrawModel(model, position, scale, WHITE);

  rlEnableWireMode();
  model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_WIREFRAME;
  DrawModel(model, position, 1.01 * scale, COLOR_WIREFRAME);
  rlDisableWireMode();
}

void physics_draw_stats(const physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_world_stats";
  const float row_height = 18.0f;
  const float window_width = 240.0f;
  const int row_count = 4;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics world stats", 20.0f, 200.0f, window_width, row_height, row_count);

  if (draw_content) {
    count_t dynamic_count = physics_body_count(world, BODY_DYNAMIC);
    count_t static_count = physics_body_count(world, BODY_STATIC);
    count_t collisions_total = physics_collisions_count(world);
    count_t awake_total = physics_awake_count(world);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_label(ctx, "Body count:", NK_TEXT_ALIGN_LEFT);

    nk_layout_row_begin(ctx, NK_DYNAMIC, row_height, 2);
    nk_layout_row_push(ctx, 0.5f);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Dynamic %u", dynamic_count);
    nk_layout_row_push(ctx, 0.5f);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Static %u", static_count);
    nk_layout_row_end(ctx);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Collisions count: %u", collisions_total);

    nk_layout_row_dynamic(ctx, row_height, 1);
    nk_labelf(ctx, NK_TEXT_ALIGN_LEFT, "Awake bodies: %u", awake_total);
  }

  nk_end(ctx);
}

void physics_draw_config_widget(physics_world *world, struct nk_context* ctx) {
  static const char* window_name = "physics_config_widget";
  const float row_height = 15.0f;
  const float window_width = 260.0f;
  const int row_count = 6;

  bool draw_content = begin_widget_window(ctx, window_name, "Physics config", 20.0f, 360.0f, window_width, row_height, row_count);

  physics_config *config = physics_edit_config(world);
  if (draw_content) {
    int max_penetration_iterations = (int) config->max_penentration_iterations;
    int max_velocity_iterations = (int) config->max_velocity_iterations;

    draw_edit_float(ctx, "Linear damping", &config->linear_damping);
    draw_edit_float(ctx, "Angular damping", &config->angular_damping);
    draw_edit_float(ctx, "Restitution", &config->restitution);
    draw_edit_float(ctx, "Friction", &config->friction);
    draw_edit_int(ctx, "Penetration iterations", &max_penetration_iterations);
    draw_edit_int(ctx, "Velocity iterations", &max_velocity_iterations);
    draw_edit_float(ctx, "Penetration epsilon", &config->penetration_epsilon);
    draw_edit_float(ctx, "Velocity epsilon", &config->velocity_epsilon);
    draw_edit_float(ctx, "Restitution damp limit", &config->restitution_damping_limit);

    if (max_penetration_iterations < 0)
      max_penetration_iterations = 0;
    if (max_velocity_iterations < 0)
      max_velocity_iterations = 0;

    config->max_penentration_iterations = (count_t) max_penetration_iterations;
    config->max_velocity_iterations = (count_t) max_velocity_iterations;
  }

  nk_end(ctx);
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
  body head = physics_add_sphere_dynamic(world, 0.3, 0.4);
  *head.position = Vector3Add(position, vec3(0, 5, 0));

  body torso = physics_add_cylinder_dynamic(world, 0.5, 0.3, 1.0);
  *torso.position = Vector3Add(position, vec3(0, 4, 0));

  body pelvis = physics_add_cylinder_dynamic(world, 0.5, 0.25, 1.0);
  *pelvis.position = Vector3Add(position, vec3(0, 3, 0));

  body left_upper_leg = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *left_upper_leg.position = Vector3Add(position, vec3(0.23, 1.8, -0.2));
  *left_upper_leg.rotation = QuaternionFromEuler(PI / 6, 0, 0);

  body left_lower_leg = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *left_lower_leg.position = Vector3Add(position, vec3(0.23, 0.6, -0.2));
  *left_lower_leg.rotation = QuaternionFromEuler(-PI / 6, 0, 0);

  body right_upper_leg = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *right_upper_leg.position = Vector3Add(position, vec3(-0.23, 1.8, 0));

  body right_lower_leg = physics_add_cylinder_dynamic(world, 0.4, 0.2, 1.2);
  *right_lower_leg.position = Vector3Add(position, vec3(-0.23, 0.6, 0));

  body left_upper_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *left_upper_arm.position = Vector3Add(position, vec3(0.4, 3.9, -0.4));
  *left_upper_arm.rotation = QuaternionFromEuler(PI / 5, 0, 0);

  body left_lower_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *left_lower_arm.position = Vector3Add(position, vec3(0.43, 3.37, -1.45));
  *left_lower_arm.rotation = QuaternionFromEuler(PI / 2, 0, 0);

  body right_upper_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *right_upper_arm.position = Vector3Add(position, vec3(-0.43, 3.8, 0));

  body right_lower_arm = physics_add_cylinder_dynamic(world, 0.2, 0.1, 1.2);
  *right_lower_arm.position = Vector3Add(position, vec3(-0.43, 2.63, -0.3));
  *right_lower_arm.rotation = QuaternionFromEuler(PI / 6, 0, 0);

  const float joint_margin = 0.1;

  physics_add_joint(world, head.handle, torso.handle, vec3(0, -0.4, 0), vec3(0, 0.5, 0), joint_margin);
  physics_add_joint(world, torso.handle, pelvis.handle, vec3(0, -0.5, 0), vec3(0, 0.5, 0), joint_margin);

  physics_add_joint(world, torso.handle, left_upper_arm.handle, vec3(0.3, 0.45, 0), vec3(-0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, left_upper_arm.handle, left_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, torso.handle, right_upper_arm.handle, vec3(-0.3, 0.45, 0), vec3(0.1, 0.6, 0), joint_margin);
  physics_add_joint(world, right_upper_arm.handle, right_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, pelvis.handle, left_upper_leg.handle, vec3(0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, left_upper_leg.handle, left_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  physics_add_joint(world, pelvis.handle, right_upper_leg.handle, vec3(-0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  physics_add_joint(world, right_upper_leg.handle, right_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

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
