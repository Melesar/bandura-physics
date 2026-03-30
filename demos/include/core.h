#ifndef CORE_H
#define CORE_H

#define NK_INCLUDE_STANDARD_VARARGS
#include "raylib-nuklear.h"
#include "bandura.h"

#define COLOR_GREEN_ACTIVE   (Color){0x00, 0xff, 0x88, 0xFF}
#define COLOR_RED_HIGHLIGHT  (Color){0xff, 0x33, 0x66, 0xFF}
#define COLOR_BLUE_STATIC    (Color){0x33, 0x66, 0xff, 0xFF}
#define COLOR_YELLOW_INFO    (Color){0xff, 0xcc, 0x00, 0xFF}

#define COLOR_BACKGROUND     (Color){0x12, 0x12, 0x14, 0xFF}
#define COLOR_GROUND         (Color){0x08, 0x08, 0x08, 0xFF}
#define COLOR_GRID_MAIN      (Color){0x44, 0x44, 0x44, 0xFF}
#define COLOR_GRID_SUB       (Color){0x22, 0x22, 0x22, 0xFF}

#define COLOR_WIREFRAME      (Color){0x12, 0x12, 0x12, 25}

typedef struct {
  char* window_title;
  Vector3 camera_position;
  Vector3 camera_target;
} program_config;

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

typedef body_handle* ragdoll;

bool begin_widget_window(
  struct nk_context* ctx,
  const char* window_name,
  const char* title,
  float x,
  float y,
  float width,
  float row_height,
  int row_count
);

int register_gizmo(Vector3 *pos, Quaternion *rot);
void unregister_gizmo(int id);

void draw_arrow(Vector3 start, Vector3 direction, Color color);
void draw_stat_float(struct nk_context* ctx, char* title, float value);
void draw_stat_int(struct nk_context* ctx, char* title, int value);
void draw_stat_float3(struct nk_context* ctx, char* title, Vector3 value);
void draw_stat_matrix(struct nk_context* ctx, char* title, Matrix value);

void draw_edit_float(struct nk_context* ctx, char* title, float* value);
void draw_edit_int(struct nk_context* ctx, char* title, int* value);
bool draw_button(struct nk_context* ctx, char* title);
bool draw_selectable(struct nk_context* ctx, char* title, bool* selected);

void draw_property_float(struct nk_context* ctx, char* title, float* value, float min, float max, float step_arrow, float step_drag);
void draw_property_int(struct nk_context* ctx, char* title, int* value, int min, int max, int step, float step_drag);

void draw_model_with_wireframe(Model model, Vector3 position, float scale, Color color);

void physics_draw_stats(const physics_world *world, struct nk_context* ctx);
void physics_draw_config_widget(physics_world *world, struct nk_context* ctx);
void physics_draw_collisions(const physics_world *world);

ragdoll ragdoll_create(physics_world *world, v3 position);
#endif
