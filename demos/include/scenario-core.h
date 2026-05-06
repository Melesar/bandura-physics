#ifndef CORE_H
#define CORE_H

#include "raylib.h"
#include "bandura.h"
#include "clay.h"

#define COLOR_GREEN_ACTIVE                                                                                             \
  (Color) { 0x00, 0xff, 0x88, 0xFF }
#define COLOR_RED_HIGHLIGHT                                                                                            \
  (Color) { 0xff, 0x33, 0x66, 0xFF }
#define COLOR_BLUE_STATIC                                                                                              \
  (Color) { 0x33, 0x66, 0xff, 0xFF }
#define COLOR_YELLOW_INFO                                                                                              \
  (Color) { 0xff, 0xcc, 0x00, 0xFF }

#define COLOR_BACKGROUND                                                                                               \
  (Color) { 0x12, 0x12, 0x14, 0xFF }
#define COLOR_GROUND                                                                                                   \
  (Color) { 0x08, 0x08, 0x08, 0xFF }
#define COLOR_GRID_MAIN                                                                                                \
  (Color) { 0x44, 0x44, 0x44, 0xFF }
#define COLOR_GRID_SUB                                                                                                 \
  (Color) { 0x22, 0x22, 0x22, 0xFF }

#define COLOR_WIREFRAME                                                                                                \
  (Color) { 0x12, 0x12, 0x12, 25 }

typedef struct {
  char *window_title;
  bool draw_ground;
  v3 camera_position;
  v3 camera_target;
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

typedef bnd_body_handle *ragdoll;

int register_gizmo(Vector3 *pos, Quaternion *rot);
void unregister_gizmo(int id);

bool import_raylib_mesh(bnd_world *world, Mesh mesh, bnd_mesh_handle *handle);
void register_mesh_for_rendering(bnd_mesh_handle handle, Mesh mesh);

void draw_arrow(v3 start, v3 direction, Color color);
void draw_bounding_boxes(const bnd_world *world);

void draw_model_with_wireframe(Model model, Vector3 position, float scale, Color color);

void physics_draw_collisions(const bnd_world *world);

ragdoll ragdoll_create(bnd_world *world, v3 position);

void ui_initialize();
void ui_teardown();
void ui_begin();
void ui_end(float dt);

void ui_set_debug(bool is_debug);

bool ui_begin_area(const char *title, bool *collapsed);
void ui_end_area();

void ui_label(char *label);
void ui_label_bool(const char *lable, bool value);
void ui_label_float(char *label, float value);
void ui_label_int(const char *label, count_t value);
void ui_label_v3(const char *label, v3 value);
void ui_label_stat(const char *label, float value);
void ui_label_matrix(const char *label, m3 value);
void ui_label_string(char *label, char *value);

void ui_value_int(const char *label, int *value, int min_value, int max_value);
void ui_value_float(const char *label, float *value, float min_value, float max_value);

bool ui_button(const char *text);

void ui_checkbox(const char *label, bool *is_checked);
bool ui_dropdown(const char *label, char **values, count_t values_count, count_t *selected, bool *active);

Clay_Color ui_text_color(int control);

quat ray_quat(Quaternion q);
v3 ray_vec(Vector3 x);

Vector3 vec_ray(v3 v);
Quaternion quat_ray(quat q);


#endif
