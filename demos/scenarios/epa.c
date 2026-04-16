#include "core.h"
#include "raylib.h"
#include "raygui.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
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
    struct {
      v3 v;
      uint16_t first_attached_edge;
    } vertex;

    struct {
      uint16_t verticies[2];
      uint16_t first_attached_face;
      uint16_t next_attached_edge;
    } edge;

    struct {
      uint16_t edges[3];
      uint16_t next_attached_face;
    } face;
  };

} polytope_node;

typedef struct {
  polytope_node *nodes;
  uint16_t *free_list;

  uint16_t last_nodes[NODE_TYPE_COUNT];

  uint16_t node_count;
  uint16_t free_count;
  uint16_t max_nodes;

  uint16_t nearest;
  float nearest_distance;
} polytope;

struct {
  enum {
    STATE_NONE,
    STATE_PICK_NEAREST,
    STATE_NEW_SUPPORT,
    STATE_EXPANSION,
    STATE_FINISHED,
  } state;

  union {
    v3 new_support;
  };

  uint32_t step;
  bool is_collision;
  simplex simplex;
  polytope *polytope;

} simulation_state;

struct {
  bool collapsed;
  bool draw_minkowski;

  float simplex_alpha;
  float polytope_alpha;
} ui_state;

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c, v3 *closest);
float distance_to_line_segment(v3 from, v3 a, v3 b, v3 *closest);
bool gjk_check_intersection_bodies(physics_world *world, body_handle body_1, body_handle body_2, simplex *simplex);
v3 support_bodies(physics_world *world, v3 direction, body_handle body_1, body_handle body_2);

static void render_minkowski_difference(const physics_world *world);
static void render_simplex(const simplex *s);
static void render_polytope(const polytope *p);

body_handle body_1;
body_handle body_2;

static uint32_t polytope_memory_size(uint16_t max_nodes) {
  return sizeof(polytope) + (max_nodes + 1) * sizeof(polytope_node) + max_nodes * sizeof(uint16_t);
}

static polytope *polytope_init(uint8_t *memory, uint16_t max_nodes) {
  polytope *result = (polytope *)memory;
  memset(memory, 0, sizeof(polytope));

  memory += sizeof(polytope);
  result->nodes = (polytope_node *)memory;

  memory += (max_nodes + 1) * sizeof(polytope_node);
  result->free_list = (uint16_t *)memory;

  result->max_nodes = max_nodes;
  result->nearest_distance = FLT_MAX;

  return result;
}

static void polytope_clear(polytope *polytope) {
  polytope->node_count = 0;
  polytope->free_count = 0;

  memset(polytope->last_nodes, 0, NODE_TYPE_COUNT * sizeof(uint16_t));

  polytope->nearest = NIL;
  polytope->nearest_distance = FLT_MAX;
}

static uint16_t polytope_free_index(polytope *polytope) {
  if (polytope->free_count > 0) {
    return polytope->free_list[--polytope->free_count];
  }

  if (polytope->node_count < polytope->max_nodes) {
    return polytope->node_count++ + 1;
  }

  return NIL;
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

static void polytope_attach_edge(polytope *polytope, uint16_t edge, uint16_t v) {
  polytope_node *node = &polytope->nodes[v];
  if (node->vertex.first_attached_edge == NIL) {
    node->vertex.first_attached_edge = edge;
  } else {
    uint16_t edge_index = node->vertex.first_attached_edge;

    while (1) {
      polytope_node *attached_edge = &polytope->nodes[edge_index];
      if (attached_edge->edge.next_attached_edge == NIL) {
        attached_edge->edge.next_attached_edge = edge;
        break;
      }

      edge_index = attached_edge->edge.next_attached_edge;
    }
  }

  polytope->nodes[edge].edge.next_attached_edge = NIL;
}

static void polytope_attach_face(polytope *polytope, uint16_t face, uint16_t edge) {
  polytope_node *node = &polytope->nodes[edge];
  if (node->edge.first_attached_face == NIL) {
    node->edge.first_attached_face = face;
  } else {
    uint16_t face_index = node->edge.first_attached_face;

    while (1) {
      polytope_node *attached_face = &polytope->nodes[face_index];
      if (attached_face->face.next_attached_face == NIL) {
        attached_face->face.next_attached_face = face;
        break;
      }

      face_index = attached_face->face.next_attached_face;
    }
  }

  polytope->nodes[face].face.next_attached_face = NIL;
}

static void polytope_get_face_vericies(const polytope *polytope, uint16_t face, v3 *v1, v3 *v2, v3 *v3) {
  polytope_node node = polytope->nodes[face];
  uint16_t e1 = node.face.edges[0];
  uint16_t e2 = node.face.edges[1];

  *v1 = polytope->nodes[polytope->nodes[e1].edge.verticies[0]].vertex.v;
  *v2 = polytope->nodes[polytope->nodes[e1].edge.verticies[1]].vertex.v;

  *v3 = polytope->nodes[e2].edge.verticies[1] != polytope->nodes[e1].edge.verticies[1]
            ? polytope->nodes[polytope->nodes[e2].edge.verticies[1]].vertex.v
            : polytope->nodes[polytope->nodes[e2].edge.verticies[0]].vertex.v;
}

static void polytope_get_edge_vertices(const polytope *polytope, uint16_t edge, v3 *v1, v3 *v2) {
  polytope_node node = polytope->nodes[edge];
  uint16_t vertex_1 = node.edge.verticies[0];
  uint16_t vertex_2 = node.edge.verticies[1];

  *v1 = polytope->nodes[vertex_1].vertex.v;
  *v2 = polytope->nodes[vertex_2].vertex.v;
}

static uint16_t polytope_add_vertex(polytope *polytope, v3 v) {
  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_VERTEX;
  node->vertex.v = v;
  node->vertex.first_attached_edge = NIL;
  node->nearest_point = v;
  node->distance = lensq(v);

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_edge(polytope *polytope, uint16_t v1, uint16_t v2) {
  if (v1 == NIL || v2 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_EDGE;
  node->edge.verticies[0] = v1;
  node->edge.verticies[1] = v2;
  node->edge.first_attached_face = NIL;

  v3 nearest_point;
  node->distance =
      distance_to_line_segment(zero(), polytope->nodes[v1].vertex.v, polytope->nodes[v2].vertex.v, &nearest_point);
  node->nearest_point = nearest_point;

  polytope_attach_edge(polytope, index, v1);
  polytope_attach_edge(polytope, index, v2);

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_face(polytope *polytope, uint16_t e1, uint16_t e2, uint16_t e3) {
  if (e1 == NIL || e2 == NIL || e3 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_FACE;
  node->face.edges[0] = e1;
  node->face.edges[1] = e2;
  node->face.edges[2] = e3;

  v3 v1, v2, v3;
  polytope_get_face_vericies(polytope, index, &v1, &v2, &v3);

  node->distance = distance_to_triangle(zero(), v1, v2, v3, &node->nearest_point);

  polytope_attach_face(polytope, index, e1);
  polytope_attach_face(polytope, index, e2);
  polytope_attach_face(polytope, index, e3);

  polytope_add_node(polytope, node, index);

  return index;
}

void polytope_delete_face(polytope *polytope, uint16_t face) {
  if (face == NIL) {
    return;
  }

  polytope->free_list[polytope->free_count++] = face;
  polytope->node_count -= 1;
}

static bool polytope_from_simplex(polytope *polytope, const simplex *s) {
  polytope_clear(polytope);

  uint16_t verts[4];
  uint16_t edges[6];

  for (uint32_t i = 0; i < 4; ++i) {
    verts[i] = polytope_add_vertex(polytope, s->points[i]);
  }

  edges[0] = polytope_add_edge(polytope, verts[0], verts[1]);
  edges[1] = polytope_add_edge(polytope, verts[1], verts[2]);
  edges[2] = polytope_add_edge(polytope, verts[2], verts[0]);
  edges[3] = polytope_add_edge(polytope, verts[1], verts[3]);
  edges[4] = polytope_add_edge(polytope, verts[3], verts[2]);
  edges[5] = polytope_add_edge(polytope, verts[0], verts[3]);

  if (polytope_add_face(polytope, edges[0], edges[1], edges[2]) == NIL ||
      polytope_add_face(polytope, edges[0], edges[3], edges[5]) == NIL ||
      polytope_add_face(polytope, edges[2], edges[5], edges[4]) == NIL ||
      polytope_add_face(polytope, edges[1], edges[4], edges[3]) == NIL) {
    return false;
  }

  return true;
}

static void reset_simulation(physics_world *world) {
  simulation_state.state = STATE_NONE;
  simulation_state.step = 0;

  polytope_clear(simulation_state.polytope);
  simulation_state.is_collision = gjk_check_intersection_bodies(world, body_1, body_2, &simulation_state.simplex);
}

static void advance_simulation(physics_world *world) {
  if (!simulation_state.is_collision) {
    return;
  }

  v3 closest;
  polytope_node closest_node = simulation_state.polytope->nodes[simulation_state.polytope->nearest];
  v3 direction = closest_node.nearest_point;
  float tolerance = physics_edit_config(world)->epa_tolerance;
  switch (simulation_state.state) {
    case STATE_NONE:
      simulation_state.state = STATE_PICK_NEAREST;

      if (!polytope_from_simplex(simulation_state.polytope, &simulation_state.simplex)) {
        TraceLog(LOG_FATAL, "Polytope capacity exceeded");
      }
      break;

    case STATE_PICK_NEAREST:
      simulation_state.state = STATE_NEW_SUPPORT;
      simulation_state.new_support = support_bodies(world, direction, body_1, body_2);
      break;

    case STATE_NEW_SUPPORT:
      if (closest_node.type == NODE_VERTEX) {
        simulation_state.state = STATE_FINISHED;
        return;
      }

      float distance = dot(direction, simulation_state.new_support);
      if (distance - closest_node.distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        return;
      }

      v3 a, b, c;
      if (closest_node.type == NODE_EDGE) {
        polytope_get_edge_vertices(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b);
        distance = distance_to_line_segment(simulation_state.new_support, a, b, &closest);
      } else {
        polytope_get_face_vericies(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b, &c);
        distance = distance_to_triangle(simulation_state.new_support, a, b, c, &closest);
      }

      if (distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        return;
      }

      simulation_state.state = STATE_EXPANSION;
      break;

    default:
      break;
  }
}

void scenario_initialize(program_config *config, physics_config *physics_config) {
  config->window_title = "EPA";
  config->draw_ground = false;
  config->camera_position = (v3){0, 5, -10};
  config->camera_target = (v3){0, 5, 10};

  ui_state.simplex_alpha = 0.4;
  ui_state.polytope_alpha = 0.5;

  ui_state.draw_minkowski = true;
}

void scenario_setup_scene(physics_world *world) {
  body b1 = physics_add_box_dynamic(world, 2, vec3(1, 1, 1));
  body s2 = physics_add_box_dynamic(world, 2, vec3(1, 1.5, 2));

  *b1.position = vec3(1, 3, 0);
  *s2.position = vec3(0, 5, 0);

  body_1 = b1.handle;
  body_2 = s2.handle;

  register_gizmo(b1.position, b1.rotation);
  register_gizmo(s2.position, s2.rotation);

  uint32_t memory_size = polytope_memory_size(POLYTOPE_MAX_NODES);
  uint8_t *polytope_memory = malloc(memory_size);

  simulation_state.polytope = polytope_init(polytope_memory, POLYTOPE_MAX_NODES);
  reset_simulation(world);
}

void scenario_teardown() { free(simulation_state.polytope); }

void scenario_draw_scene(physics_world *world) {
  render_minkowski_difference(world);
  if (simulation_state.state == STATE_NONE) {
    render_simplex(&simulation_state.simplex);
  } else {
    render_polytope(simulation_state.polytope);
  }

  if (simulation_state.state == STATE_NEW_SUPPORT) {
    v3 closest = simulation_state.polytope->nodes[simulation_state.polytope->nearest].nearest_point;
    DrawSphere(simulation_state.new_support, 0.05, ORANGE);
    DrawLine3D(closest, simulation_state.new_support, YELLOW);
  }
}

static void render_minkowski_difference(const physics_world *world) {
  if (!ui_state.draw_minkowski) {
    return;
  }

  count_t n;

  v3 s1 = physics_get_shapes(world, body_1, &n)[0].box.size;
  v3 s2 = physics_get_shapes(world, body_2, &n)[0].box.size;

  v3 p1 = physics_get_position(world, body_1);
  v3 p2 = physics_get_position(world, body_2);

  v3 center = sub(p1, p2);
  v3 size = add(s1, s2);

  DrawCubeWires(center, size.x, size.y, size.z, RED);
}

static void render_simplex(const simplex *s) {
  if (!simulation_state.is_collision) {
    return;
  }

  const uint8_t alpha = (uint8_t)(ui_state.simplex_alpha * 255);

  Color color = GREEN;
  color.a = alpha;

  BeginBlendMode(BLEND_ALPHA);
  DrawTriangle3D(s->points[2], s->points[0], s->points[1], color);
  DrawTriangle3D(s->points[0], s->points[2], s->points[3], color);
  DrawTriangle3D(s->points[0], s->points[3], s->points[1], color);
  DrawTriangle3D(s->points[3], s->points[2], s->points[1], color);
  EndBlendMode();

  DrawLine3D(s->points[0], s->points[1], BLACK);
  DrawLine3D(s->points[0], s->points[2], BLACK);
  DrawLine3D(s->points[0], s->points[3], BLACK);
  DrawLine3D(s->points[1], s->points[2], BLACK);
  DrawLine3D(s->points[1], s->points[3], BLACK);
  DrawLine3D(s->points[2], s->points[3], BLACK);
}

static void render_triangle(v3 v_1, v3 v_2, v3 v_3, v3 reference, Color color) {
  v3 normal = cross(sub(v_2, v_1), sub(v_3, v_1));
  if (dot(normal, reference) > 0) {
    DrawTriangle3D(v_1, v_2, v_3, color);
  } else {
    DrawTriangle3D(v_1, v_3, v_2, color);
  }
}

static void render_polytope(const polytope *p) {
  if (!simulation_state.is_collision) {
    return;
  }

  uint16_t last_face_index = p->last_nodes[NODE_FACE];
  bool highlight = simulation_state.state == STATE_PICK_NEAREST || simulation_state.state == STATE_NEW_SUPPORT;

  uint16_t face_index = last_face_index;
  polytope_node *face = &p->nodes[last_face_index];
  while (face_index != NIL) {
    Color color = PINK;
    if (highlight && p->nearest == face_index) {
      color = GREEN;
    }

    v3 v1, v2, v3;
    polytope_get_face_vericies(p, face_index, &v1, &v2, &v3);
    render_triangle(v1, v2, v3, face->nearest_point, color);

    DrawLine3D(v1, v2, BLACK);
    DrawLine3D(v2, v3, BLACK);
    DrawLine3D(v3, v1, BLACK);

    face_index = face->prev;
    face = &p->nodes[face->prev];
  }

  if (highlight) {
    DrawSphere(p->nodes[p->nearest].nearest_point, 0.05, GREEN);
  }
}

void scenario_handle_input(physics_world *world, Camera *camera) {}

void scenario_simulate(physics_world *world, float dt) {}

void scenario_build_ui(physics_world *world) {
  ui_begin_area("EPA", &ui_state.collapsed);

  ui_checkbox("Draw Minkowski", &ui_state.draw_minkowski);

  ui_value_float("Simplex alpha", &ui_state.simplex_alpha, 0.0, 1.0);
  ui_value_float("Polytope alpha", &ui_state.polytope_alpha, 0.0, 1.0);

  CLAY(CLAY_ID("EPA_spacer"), {.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(20)}}});

  CLAY(CLAY_ID("EPA_title"), {.layout = {.childGap = 30}}) {
    char *s;
    switch (simulation_state.state) {
      case STATE_NONE:
        s = "None";
        break;

      case STATE_PICK_NEAREST:
        s = "Pick nearest";
        break;

      case STATE_NEW_SUPPORT:
        s = "New support";
        break;

      case STATE_EXPANSION:
        s = "Expansion";
        break;

      case STATE_FINISHED:
        s = "Finished";
        break;
    }
    ui_label_int("Step", simulation_state.step);
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}});
    ui_label_string("State", s);
  }

  CLAY(CLAY_ID("EPA_space"), {.layout = {.sizing = {.height = CLAY_SIZING_GROW(), .width = CLAY_SIZING_GROW()}}})

  CLAY(CLAY_ID("EPA_buttons"), {.layout = {
                                    .padding = CLAY_PADDING_ALL(5),
                                    .sizing = {.width = CLAY_SIZING_GROW()},
                                    .childAlignment = {.y = CLAY_ALIGN_Y_CENTER},
                                    .childGap = 10,
                                }}) {
    CLAY(CLAY_ID("EPA_buttons_left"), {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}});

    if (ui_button("Reset")) {
      reset_simulation(world);
    }

    if (ui_button("Next")) {
      advance_simulation(world);
    }

    CLAY(CLAY_ID("EPA_buttons_right"), {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}});
  }

  ui_end_area();
}
