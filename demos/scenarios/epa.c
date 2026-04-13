#include "core.h"
#include "raylib.h"
#include <cerrno>
#include <string.h>
#include <float.h>

#define NIL 0
#define POLYTOPE_MAX_NODES 1024

typedef struct {
  v3 points[4];
  uint8_t size;
} simplex;

typedef enum {
  NODE_VERTEX,
  NODE_EDGE,
  NODE_FACE,

  NODE_TYPE_COUNT,
} node_type;

typedef struct {
  node_type type;
  v3 nearest_point;
  float distance;
  uint16_t next;
  uint16_t prev;

  union {
    v3 vertex;

    struct {
      uint16_t v1;
      uint16_t v2;
    } edge;

    struct {
      uint16_t e1;
      uint16_t e2;
      uint16_t e3;
    } face;
  };

} polytope_node;

typedef struct {
  polytope_node nodes[POLYTOPE_MAX_NODES];
  uint16_t free_list[POLYTOPE_MAX_NODES];

  uint16_t last_nodes[NODE_TYPE_COUNT];

  uint16_t node_count;
  uint16_t free_count;

  uint16_t nearest;
  float nearest_distance;
} polytope;

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c);
float distance_to_line_segment(v3 from, v3 a, v3 b);
bool gjk_check_intersection_bodies(physics_world *world, body_handle body_1, body_handle body_2, simplex *simplex);

static void render_minkowski_difference(const physics_world *world);
static void render_simplex(const simplex *s);

body_handle sphere_1;
body_handle sphere_2;
polytope pt;

static void polytope_init(polytope *polytope) { polytope->nearest_distance = FLT_MAX; }

static void polytope_clear(polytope *polytope) {
  memset(polytope, 0, sizeof(polytope));
  polytope_init(polytope);
}

static uint16_t polytope_free_index(polytope *polytope) {
  return polytope->free_count > 0 ? polytope->free_list[--polytope->free_count] : polytope->node_count++;
}

static void polytope_add_node(polytope *polytope, polytope_node *node, uint16_t index) {
  node->prev = polytope->last_nodes[node->type];
  node->next = NIL;

  if (node->distance < polytope->nearest_distance) {
    polytope->nearest_distance = node->distance;
    polytope->nearest = index;
  }

  polytope->last_nodes[node->type] = index;
}

static uint16_t polytope_add_vertex(polytope *polytope, v3 v) {
  uint16_t index = polytope_free_index(polytope);

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_VERTEX;
  node->vertex = v;
  node->nearest_point = v;
  node->distance = lensq(v);

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_edge(polytope *polytope, uint16_t v1, uint16_t v2) {
  uint16_t index = polytope_free_index(polytope);

  v3 nearest_point;

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_EDGE;
  node->edge.v1 = v1;
  node->edge.v2 = v2;
  node->distance = distance_to_line_segment(zero(), polytope->nodes[v1].vertex, polytope->nodes[v2].vertex/*, &nearest_point*/);
  // node->nearest_point = nearest_point;

  polytope_add_node(polytope, node, index);

  return index;
}

static void polytope_from_simplex(polytope *polytope, const simplex *s) {
  polytope_clear(polytope);

  uint16_t verts[4];
  uint16_t edges[6];

  for (uint32_t i = 0; i < 4; ++i) {
    verts[i] = polytope_add_vertex(polytope, s->points[i]);
  }
}

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

  polytope_init(&pt);
}

void scenario_draw_scene(physics_world *world) {
  render_minkowski_difference(world);

  simplex s;
  bool collision = gjk_check_intersection_bodies(world, sphere_1, sphere_2, &s);
  if (!collision) {
    return;
  }

  render_simplex(&s);

  polytope_from_simplex(&pt, &s);

  // Proceed with EPA
}

static void render_minkowski_difference(const physics_world *world) {
  count_t n;

  float r1 = physics_get_shapes(world, sphere_1, &n)[0].sphere.radius;
  float r2 = physics_get_shapes(world, sphere_2, &n)[0].sphere.radius;

  v3 p1 = physics_get_position(world, sphere_1);
  v3 p2 = physics_get_position(world, sphere_2);

  v3 center = sub(p1, p2);
  float radius = r1 + r2;

  DrawSphereWires(center, radius, 32, 64, BLACK);
}

static void render_simplex(const simplex *s) {
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
  DrawTriangle3D(s->points[2], s->points[0], s->points[1], yellow);
  DrawTriangle3D(s->points[0], s->points[2], s->points[3], orange);
  DrawTriangle3D(s->points[0], s->points[3], s->points[1], blue);
  DrawTriangle3D(s->points[3], s->points[2], s->points[1], green);
  EndBlendMode();
}

void scenario_handle_input(physics_world *world, Camera *camera) {}

void scenario_simulate(physics_world *world, float dt) {}

void scenario_build_ui(physics_world *world) {}

void scenario_teardown() {}
