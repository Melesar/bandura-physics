#include "scenario-core.h"
#include "raymath.h"

#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

const float gizmo_arrow_length = 2.5f;
const float gizmo_arrow_radius = 0.15f;
const float gizmo_rotation_ring_radius = 2.0f;
const float gizmo_selection_threshold = 0.3f;
const int initial_buffer_capacity = 10;

typedef enum {
  GIZMO_NONE,
  GIZMO_TRANSLATE_X,
  GIZMO_TRANSLATE_Y,
  GIZMO_TRANSLATE_Z,
  GIZMO_ROTATE_X,
  GIZMO_ROTATE_Y,
  GIZMO_ROTATE_Z
} gizmo_type;

typedef struct {
  bnd_world *world;
  bnd_body_handle handle;

  int id;

  gizmo_type selected_gizmo;
  gizmo_type hovered_gizmo;

  Vector3 start_pos;
  Quaternion start_rot;
  Vector3 drag_start_pos;
  float drag_start_angle;
  float drag_start_offset;

  bool is_dragging;
} gizmo;

gizmo *gizmos;

int count = 0;
int capacity = initial_buffer_capacity;
int current_id = 0;

static void draw_rotation_gizmo(Vector3 center, Vector3 axis, Color color) {
  const int segments = 64;
  Vector3 perpendicular;

  // Find a perpendicular vector to the axis
  if (fabsf(axis.y) < 0.9f) {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 0, 1, 0 }));
  } else {
    perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 1, 0, 0 }));
  }

  Vector3 prev_point = Vector3Add(center, Vector3Scale(perpendicular, gizmo_rotation_ring_radius));

  for (int i = 1; i <= segments; i++) {
    float angle = (float)i / segments * 2 * PI;

    // Rotate perpendicular around axis
    Quaternion rot = QuaternionFromAxisAngle(axis, angle);
    Matrix rot_mat = QuaternionToMatrix(rot);
    Vector3 rotated = Vector3Transform(perpendicular, rot_mat);
    Vector3 point = Vector3Add(center, Vector3Scale(rotated, gizmo_rotation_ring_radius));

    DrawLine3D(prev_point, point, color);
    prev_point = point;
  }
}

static float ray_to_line_distance(Ray ray, Vector3 line_start, Vector3 line_end, Vector3 *closest_point_on_ray) {
  Vector3 line_dir = Vector3Normalize(Vector3Subtract(line_end, line_start));
  Vector3 ray_dir = Vector3Normalize(ray.direction);

  Vector3 w0 = Vector3Subtract(ray.position, line_start);

  float a = Vector3DotProduct(ray_dir, ray_dir);
  float b = Vector3DotProduct(ray_dir, line_dir);
  float c = Vector3DotProduct(line_dir, line_dir);
  float d = Vector3DotProduct(ray_dir, w0);
  float e = Vector3DotProduct(line_dir, w0);

  float denom = a * c - b * b;
  float t_ray = (b * e - c * d) / denom;

  if (closest_point_on_ray != NULL) {
    *closest_point_on_ray = Vector3Add(ray.position, Vector3Scale(ray_dir, t_ray));
  }

  float t_line = (a * e - b * d) / denom;
  t_line = fmaxf(0.0f, fminf(1.0f, t_line));

  Vector3 point_on_ray = Vector3Add(ray.position, Vector3Scale(ray_dir, t_ray));
  Vector3 point_on_line =
      Vector3Add(line_start, Vector3Scale(line_dir, t_line * Vector3Length(Vector3Subtract(line_end, line_start))));

  return Vector3Distance(point_on_ray, point_on_line);
}

static float ray_to_circle_distance(Ray ray, Vector3 center, Vector3 axis, float radius, Vector3 *intersection_point) {
  // Project ray onto the plane of the circle
  float denom = Vector3DotProduct(ray.direction, axis);

  if (fabsf(denom) < 0.001f) {
    return 1000.0f; // Ray parallel to plane
  }

  float t = Vector3DotProduct(Vector3Subtract(center, ray.position), axis) / denom;

  if (t < 0) {
    return 1000.0f; // Plane behind ray
  }

  Vector3 plane_intersection = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
  Vector3 to_center = Vector3Subtract(plane_intersection, center);
  float dist_to_center = Vector3Length(to_center);

  if (intersection_point != NULL) {
    *intersection_point = plane_intersection;
  }

  return fabsf(dist_to_center - radius);
}

gizmo_type check_gizmo_hover(Ray mouse_ray, Vector3 cylinder_pos) {
  float min_dist = gizmo_selection_threshold;
  gizmo_type result = GIZMO_NONE;

  // Check translation gizmos
  Vector3 x_end = Vector3Add(cylinder_pos, (Vector3){ gizmo_arrow_length, 0, 0 });
  float dist_x = ray_to_line_distance(mouse_ray, cylinder_pos, x_end, NULL);
  if (dist_x < min_dist) {
    min_dist = dist_x;
    result = GIZMO_TRANSLATE_X;
  }

  Vector3 y_end = Vector3Add(cylinder_pos, (Vector3){ 0, gizmo_arrow_length, 0 });
  float dist_y = ray_to_line_distance(mouse_ray, cylinder_pos, y_end, NULL);
  if (dist_y < min_dist) {
    min_dist = dist_y;
    result = GIZMO_TRANSLATE_Y;
  }

  Vector3 z_end = Vector3Add(cylinder_pos, (Vector3){ 0, 0, gizmo_arrow_length });
  float dist_z = ray_to_line_distance(mouse_ray, cylinder_pos, z_end, NULL);
  if (dist_z < min_dist) {
    min_dist = dist_z;
    result = GIZMO_TRANSLATE_Z;
  }

  // Check rotation gizmos
  float dist_rx =
      ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){ 1, 0, 0 }, gizmo_rotation_ring_radius, NULL);
  if (dist_rx < min_dist) {
    min_dist = dist_rx;
    result = GIZMO_ROTATE_X;
  }

  float dist_ry =
      ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){ 0, 1, 0 }, gizmo_rotation_ring_radius, NULL);
  if (dist_ry < min_dist) {
    min_dist = dist_ry;
    result = GIZMO_ROTATE_Y;
  }

  float dist_rz =
      ray_to_circle_distance(mouse_ray, cylinder_pos, (Vector3){ 0, 0, 1 }, gizmo_rotation_ring_radius, NULL);
  if (dist_rz < min_dist) {
    min_dist = dist_rz;
    result = GIZMO_ROTATE_Z;
  }

  return result;
}

void init_gizmos() { gizmos = (gizmo *)malloc(capacity * sizeof(gizmo)); }

void manipulate_gizmos(Camera *camera) {
  for (int i = count - 1; i >= 0; i--) {
    gizmo *g = &gizmos[i];
    if (bnd_handle_valid(g->world, g->handle).type != BND_OK) {
      gizmos[i] = gizmos[--count];
    }
  }

  for (int i = 0; i < count; ++i) {
    gizmo *g = &gizmos[i];
    Vector3 pos = bnd_get_position(g->world, g->handle).value;
    Quaternion rot = bnd_get_rotation(g->world, g->handle).value;

    Ray mouse_ray = GetScreenToWorldRay(GetMousePosition(), *camera);

    if (!g->is_dragging) {
      g->hovered_gizmo = check_gizmo_hover(mouse_ray, pos);

      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && g->hovered_gizmo != GIZMO_NONE) {
        g->is_dragging = true;
        g->selected_gizmo = g->hovered_gizmo;
        g->drag_start_pos = mouse_ray.position;
        g->start_pos = pos;
        g->start_rot = rot;

        // For translation, calculate initial offset along axis
        if (g->selected_gizmo >= GIZMO_TRANSLATE_X && g->selected_gizmo <= GIZMO_TRANSLATE_Z) {
          Vector3 axis = { 0, 0, 0 };
          if (g->selected_gizmo == GIZMO_TRANSLATE_X)
            axis = (Vector3){ 1, 0, 0 };
          else if (g->selected_gizmo == GIZMO_TRANSLATE_Y)
            axis = (Vector3){ 0, 1, 0 };
          else if (g->selected_gizmo == GIZMO_TRANSLATE_Z)
            axis = (Vector3){ 0, 0, 1 };

          Vector3 line_start = pos;
          Vector3 line_end = Vector3Add(pos, Vector3Scale(axis, 100));

          Vector3 closest_point;
          ray_to_line_distance(mouse_ray, line_start, line_end, &closest_point);

          Vector3 offset = Vector3Subtract(closest_point, pos);
          g->drag_start_offset = Vector3DotProduct(offset, axis);
        }

        // For rotation, calculate initial angle
        if (g->selected_gizmo >= GIZMO_ROTATE_X) {
          Vector3 axis = { 0, 0, 0 };
          if (g->selected_gizmo == GIZMO_ROTATE_X)
            axis = (Vector3){ 1, 0, 0 };
          else if (g->selected_gizmo == GIZMO_ROTATE_Y)
            axis = (Vector3){ 0, 1, 0 };
          else if (g->selected_gizmo == GIZMO_ROTATE_Z)
            axis = (Vector3){ 0, 0, 1 };

          Vector3 intersection;
          ray_to_circle_distance(mouse_ray, pos, axis, gizmo_rotation_ring_radius, &intersection);
          Vector3 to_intersection = Vector3Subtract(intersection, pos);

          // Project to find perpendicular for angle calculation
          Vector3 perpendicular;
          if (fabsf(axis.y) < 0.9f) {
            perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 0, 1, 0 }));
          } else {
            perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 1, 0, 0 }));
          }
          Vector3 binormal = Vector3CrossProduct(axis, perpendicular);

          float x = Vector3DotProduct(to_intersection, perpendicular);
          float y = Vector3DotProduct(to_intersection, binormal);
          g->drag_start_angle = atan2f(y, x);
        }
      }
    }

    if (g->is_dragging) {
      if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        if (g->selected_gizmo == GIZMO_TRANSLATE_X || g->selected_gizmo == GIZMO_TRANSLATE_Y ||
            g->selected_gizmo == GIZMO_TRANSLATE_Z) {
          // Translation
          Vector3 axis = { 0, 0, 0 };
          if (g->selected_gizmo == GIZMO_TRANSLATE_X)
            axis = (Vector3){ 1, 0, 0 };
          else if (g->selected_gizmo == GIZMO_TRANSLATE_Y)
            axis = (Vector3){ 0, 1, 0 };
          else if (g->selected_gizmo == GIZMO_TRANSLATE_Z)
            axis = (Vector3){ 0, 0, 1 };

          // Project mouse movement onto axis
          Vector3 line_start = pos;
          Vector3 line_end = Vector3Add(g->start_pos, Vector3Scale(axis, 100));

          Vector3 closest_point;
          ray_to_line_distance(mouse_ray, line_start, line_end, &closest_point);

          Vector3 offset = Vector3Subtract(closest_point, g->start_pos);
          float current_offset = Vector3DotProduct(offset, axis);

          // Calculate delta from initial click position
          float delta = current_offset - g->drag_start_offset;

          pos = Vector3Add(g->start_pos, Vector3Scale(axis, delta));
          bnd_set_position(g->world, g->handle, pos);
        } else {
          // Rotation
          Vector3 axis = { 0, 0, 0 };
          if (g->selected_gizmo == GIZMO_ROTATE_X)
            axis = (Vector3){ 1, 0, 0 };
          else if (g->selected_gizmo == GIZMO_ROTATE_Y)
            axis = (Vector3){ 0, 1, 0 };
          else if (g->selected_gizmo == GIZMO_ROTATE_Z)
            axis = (Vector3){ 0, 0, 1 };

          Vector3 intersection;
          float dist = ray_to_circle_distance(mouse_ray, pos, axis, gizmo_rotation_ring_radius, &intersection);

          if (dist < 1.0f) {
            Vector3 to_intersection = Vector3Subtract(intersection, pos);

            Vector3 perpendicular;
            if (fabsf(axis.y) < 0.9f) {
              perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 0, 1, 0 }));
            } else {
              perpendicular = Vector3Normalize(Vector3CrossProduct(axis, (Vector3){ 1, 0, 0 }));
            }
            Vector3 binormal = Vector3CrossProduct(axis, perpendicular);

            float x = Vector3DotProduct(to_intersection, perpendicular);
            float y = Vector3DotProduct(to_intersection, binormal);
            float current_angle = atan2f(y, x);

            float angle_delta = current_angle - g->drag_start_angle;

            Quaternion rotation = QuaternionFromAxisAngle(axis, angle_delta);
            rot = QuaternionMultiply(rotation, g->start_rot);
            bnd_set_rotation(g->world, g->handle, rot);
          }
        }
      } else {
        g->is_dragging = false;
        g->selected_gizmo = GIZMO_NONE;
      }
    }
  }
}

void draw_gizmos() {
  for (int i = 0; i < count; ++i) {
    gizmo g = gizmos[i];
    Vector3 pos = bnd_get_position(g.world, g.handle).value;
    Quaternion rot = bnd_get_rotation(g.world, g.handle).value;

    Color x_color = (g.hovered_gizmo == GIZMO_TRANSLATE_X || g.selected_gizmo == GIZMO_TRANSLATE_X) ? ORANGE : RED;
    Color y_color = (g.hovered_gizmo == GIZMO_TRANSLATE_Y || g.selected_gizmo == GIZMO_TRANSLATE_Y) ? ORANGE : GREEN;
    Color z_color = (g.hovered_gizmo == GIZMO_TRANSLATE_Z || g.selected_gizmo == GIZMO_TRANSLATE_Z) ? ORANGE : BLUE;

    draw_arrow(pos, (bnd_v3){ gizmo_arrow_length, 0, 0 }, x_color);
    draw_arrow(pos, (bnd_v3){ 0, gizmo_arrow_length, 0 }, y_color);
    draw_arrow(pos, (bnd_v3){ 0, 0, gizmo_arrow_length }, z_color);

    // Draw rotation gizmos
    Color rx_color = (g.hovered_gizmo == GIZMO_ROTATE_X || g.selected_gizmo == GIZMO_ROTATE_X) ? ORANGE : RED;
    Color ry_color = (g.hovered_gizmo == GIZMO_ROTATE_Y || g.selected_gizmo == GIZMO_ROTATE_Y) ? ORANGE : GREEN;
    Color rz_color = (g.hovered_gizmo == GIZMO_ROTATE_Z || g.selected_gizmo == GIZMO_ROTATE_Z) ? ORANGE : BLUE;

    draw_rotation_gizmo(pos, (Vector3){ 1, 0, 0 }, rx_color);
    draw_rotation_gizmo(pos, (Vector3){ 0, 1, 0 }, ry_color);
    draw_rotation_gizmo(pos, (Vector3){ 0, 0, 1 }, rz_color);
  }
}

int register_gizmo(bnd_world *world, bnd_body_handle handle) {
  if (capacity <= count) {
    while (capacity <= count)
      capacity *= 2;

    gizmo *new_gizmos = (gizmo *)malloc(capacity * sizeof(gizmo));
    memcpy(new_gizmos, gizmos, count * sizeof(gizmo));
    free(gizmos);
    gizmos = new_gizmos;
  }

  gizmo g = { 0 };
  g.world = world;
  g.handle = handle;
  g.id = current_id++;

  gizmos[count++] = g;

  return g.id;
}

void unregister_gizmo(int id) {
  for (int i = 0; i < count; i++) {
    if (gizmos[i].id == id) {
      gizmos[i] = gizmos[--count];
      return;
    }
  }
}
