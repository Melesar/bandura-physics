#include "core.h"
#include "raylib.h"

typedef struct {
  v3 points[4];
  uint8_t size;
} simplex;

bool gjk_check_intersection_bodies(physics_world *world, body_handle body_1, body_handle body_2, simplex *simplex);

body_handle sphere_1;
body_handle sphere_2;

void scenario_initialize(program_config *config, physics_config *physics_config) {
  config->window_title = "EPA";
  config->draw_ground = false;
  config->camera_position = (v3){0, 5, -10};
  config->camera_target = (v3){0, 5, 10};
}

void scenario_setup_scene(physics_world *world) {
  body s1 = physics_add_sphere_dynamic(world, 2, 1);
  body s2 = physics_add_sphere_dynamic(world, 2, 1.5);

  *s1.position = vec3(1, 3, 0);
  *s2.position = vec3(0, 5, 0);

  sphere_1 = s1.handle;
  sphere_2 = s2.handle;

  register_gizmo(s1.position, s1.rotation);
  register_gizmo(s2.position, s2.rotation);
}

void scenario_handle_input(physics_world *world, Camera *camera) {

}

void scenario_simulate(physics_world *world, float dt) {
}

void scenario_draw_scene(physics_world *world) {
  count_t n;

  float r1 = physics_get_shapes(world, sphere_1, &n)[0].sphere.radius;
  float r2 = physics_get_shapes(world, sphere_2, &n)[0].sphere.radius;

  v3 p1 = physics_get_position(world, sphere_1);
  v3 p2 = physics_get_position(world, sphere_2);

  v3 center = sub(p1, p2);
  float radius = r1 + r2;

  // Draw Minkowski difference
  DrawSphereWires(center, radius, 32, 64, BLACK);

  simplex s;
  bool collision = gjk_check_intersection_bodies(world, sphere_1, sphere_2, &s);
  if (!collision) {
    return;
  }

  // Draw simplex
  const uint8_t alpha = 100;
  Color yellow = YELLOW;
  Color orange = ORANGE;
  Color blue = BLUE;
  Color green = GREEN;

  yellow.a = alpha;
  orange.a = alpha;
  blue.a = alpha;
  green.a = alpha;

  BeginBlendMode(BLEND_ALPHA);
  DrawTriangle3D(s.points[2], s.points[0], s.points[1], yellow);
  DrawTriangle3D(s.points[0], s.points[2], s.points[3], orange);
  DrawTriangle3D(s.points[0], s.points[3], s.points[1], blue);
  DrawTriangle3D(s.points[3], s.points[2], s.points[1], green);
  EndBlendMode();

  // Proceed with EPA
}

void scenario_build_ui(physics_world *world) {

}

void scenario_teardown() {

}
