#include "core.h"
#include "bandura.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <float.h>
#include <limits.h>

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

void physics_draw_collisions(const physics_world *world) {
  #define max_count 50

  contact_t contacts[max_count];
  count_t count = physics_get_contacts(world, contacts, max_count);

  for (count_t i = 0; i < count; ++i) {
    contact_t contact = contacts[i];

    draw_arrow(contact.point, scale(contact.normal, 0.2), RED);
  }

  #undef count
}
