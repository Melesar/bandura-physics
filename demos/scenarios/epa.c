#include "scenario-core.h"
#include "raylib.h"
#include "raygui.h"
#include "raymath.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>

#define NIL 0
#define POLYTOPE_MAX_NODES 1024

#define MAX_VISIBLE_FACES 16
#define MAX_ATTACHED_EDGES 2
#define MAX_ATTACHED_FACES 2

#define polytope_for_each_node(p, index, type)                                                                         \
  for (count_t index = p->last_nodes[type]; index != NIL; index = p->nodes[index].prev)

typedef enum {
  NODE_VERTEX,
  NODE_EDGE,
  NODE_FACE,

  NODE_TYPE_COUNT,
} node_type;

typedef enum {
  FLAG_FOR_REMOVAL = 1,
  FLAG_BORDER_EDGE = 2,
} node_flags;

typedef enum {
  BODIES_SPHERES,
  BODIES_BOXES,
  BODIES_CYLINDERS,
} body_types;

typedef struct {
  v3 v;
  v3 v1;
  v3 v2;
} support_point;

typedef struct {
  support_point points[4];
  count_t size;
} simplex;

typedef struct {
  node_type type;
  uint8_t flags;

  uint16_t prev;

  union {
    struct {
      support_point v;
      uint16_t first_attached_edge;
    } vertex;

    struct {
      uint16_t verticies[2];
      uint16_t next_attached_edges[2];
      uint16_t attached_faces[MAX_ATTACHED_FACES];
    } edge;

    struct {
      uint16_t edges[3];
      v3 normal;
      float distance;
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

  support_point new_support;

  struct {
    v3 point;
    v3 normal;
    float depth;
  } contact;

  contact_t ccd_contact;

  uint32_t step;
  bool is_collision;
  bool realtime;
  simplex simplex;
  polytope *polytope;
  body_types bodies;
} simulation_state;

struct {
  bool collapsed;
  bool draw_minkowski;

  float simplex_alpha;
  float polytope_alpha;

  bool dropdown_active;
  count_t dropdown_selected;
} ui_state;

float distance_to_triangle(v3 from, v3 a, v3 b, v3 c, v3 *closest);
float distance_to_line_segment(v3 from, v3 a, v3 b, v3 *closest);
bool gjk_check_intersection_bodies(physics_world *world, body_handle body_1, body_handle body_2, simplex *simplex);
support_point support_bodies(physics_world *world, v3 direction, body_handle body_1, body_handle body_2);
bool epa_for_bodies(physics_world *world, body_handle body_1, body_handle body_2, contact_t *out_contact);

static void dump_polytope(const polytope *polytope, const char *title);
static void render_minkowski_difference(const physics_world *world);
static void render_simplex(const simplex *s);
static void render_polytope(const polytope *p);

int gizmo_1;
int gizmo_2;

body_handle body_1;
body_handle body_2;

static float visibility_epsilon = 0.25;

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
  uint16_t last_node_index = polytope->last_nodes[node->type];

  node->prev = last_node_index;

  polytope->last_nodes[node->type] = index;
}

static void polytope_remove_node(polytope *polytope, uint16_t index) {
  polytope_node node = polytope->nodes[index];

  uint16_t node_index = polytope->last_nodes[node.type];
  polytope_node *current_node = &polytope->nodes[node_index];

  if (node_index == index) {
    polytope->last_nodes[node.type] = current_node->prev;
  } else {
    while (current_node->prev != index) {
      node_index = current_node->prev;
      current_node = &polytope->nodes[node_index];
    }

    current_node->prev = node.prev;
  }

  polytope->free_list[polytope->free_count++] = index;
}

static void polytope_attach_edge(polytope *polytope, uint16_t edge, uint16_t vertex) {
  polytope_node *vertex_node = &polytope->nodes[vertex];
  if (vertex_node->vertex.first_attached_edge == NIL) {
    vertex_node->vertex.first_attached_edge = edge;
  } else {
    uint16_t attached_edge = vertex_node->vertex.first_attached_edge;
    polytope_node *edge_node;
    int i;
    do {
      edge_node = &polytope->nodes[attached_edge];
      i = edge_node->edge.verticies[0] == vertex ? 0 : 1;
      attached_edge = edge_node->edge.next_attached_edges[i];
    } while (attached_edge != NIL);

    edge_node->edge.next_attached_edges[i] = edge;
  }
}

static void polytope_attach_face(polytope *polytope, uint16_t face, uint16_t edge) {
  polytope_node *edge_node = &polytope->nodes[edge];
  for (count_t i = 0; i < MAX_ATTACHED_FACES; ++i) {
    if (edge_node->edge.attached_faces[i] == NIL) {
      edge_node->edge.attached_faces[i] = face;
      break;
    }
  }
}

static void polytope_detach_face(polytope *polytope, uint16_t face, uint16_t edge) {
  polytope_node *edge_node = &polytope->nodes[edge];

  uint16_t *attached_faces = edge_node->edge.attached_faces;
  for (count_t i = 0; i < 2; ++i) {
    if (attached_faces[i] == face) {
      attached_faces[i] = NIL;
    }
  }
}

static void polytope_detach_edge(polytope *polytope, uint16_t edge, uint16_t vertex) {
  polytope_node *vertex_node = &polytope->nodes[vertex];

  uint16_t next_edge = vertex_node->vertex.first_attached_edge;
  polytope_node *next_edge_node = &polytope->nodes[next_edge];
  int i = next_edge_node->edge.verticies[0] == vertex ? 0 : 1;
  if (next_edge == edge) {
    vertex_node->vertex.first_attached_edge = next_edge_node->edge.next_attached_edges[i];
    if (vertex_node->vertex.first_attached_edge == NIL) {
      vertex_node->flags |= FLAG_FOR_REMOVAL;
    }
    return;
  }

  next_edge = next_edge_node->edge.next_attached_edges[i];

  while (next_edge != edge) {
    next_edge_node = &polytope->nodes[next_edge];
    i = next_edge_node->edge.verticies[0] == vertex ? 0 : 1;
    next_edge = next_edge_node->edge.next_attached_edges[i];
  }

  polytope_node *current_node = next_edge_node;
  int current_i = i;

  next_edge_node = &polytope->nodes[next_edge];
  i = next_edge_node->edge.verticies[0] == vertex ? 0 : 1;

  current_node->edge.next_attached_edges[current_i] = next_edge_node->edge.next_attached_edges[i];
}

static void polytope_get_face_verticies(const polytope *polytope, uint16_t face, v3 *v1, v3 *v2, v3 *v3) {
  polytope_node node = polytope->nodes[face];
  uint16_t e1 = node.face.edges[0];
  uint16_t e2 = node.face.edges[1];
  uint16_t *edge_verts_1 = polytope->nodes[e1].edge.verticies;
  uint16_t *edge_verts_2 = polytope->nodes[e2].edge.verticies;

  *v1 = polytope->nodes[edge_verts_1[0]].vertex.v.v;
  *v2 = polytope->nodes[edge_verts_1[1]].vertex.v.v;

  *v3 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
            ? polytope->nodes[edge_verts_2[1]].vertex.v.v
            : polytope->nodes[edge_verts_2[0]].vertex.v.v;
}

static void polytope_get_edge_vertices(const polytope *polytope, uint16_t edge, v3 *v1, v3 *v2) {
  polytope_node node = polytope->nodes[edge];
  uint16_t vertex_1 = node.edge.verticies[0];
  uint16_t vertex_2 = node.edge.verticies[1];

  *v1 = polytope->nodes[vertex_1].vertex.v.v;
  *v2 = polytope->nodes[vertex_2].vertex.v.v;
}

static uint16_t polytope_add_vertex(polytope *polytope, support_point p) {
  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_VERTEX;
  node->flags = 0;
  node->vertex.v = p;
  node->vertex.first_attached_edge = NIL;

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
  node->flags = 0;
  node->edge.verticies[0] = v1;
  node->edge.verticies[1] = v2;

  memset(node->edge.attached_faces, 0, MAX_ATTACHED_FACES * sizeof(uint16_t));
  memset(node->edge.next_attached_edges, 0, MAX_ATTACHED_EDGES * sizeof(uint16_t));

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
  node->flags = 0;
  node->face.edges[0] = e1;
  node->face.edges[1] = e2;
  node->face.edges[2] = e3;

  v3 v1, v2, v3;
  polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

  node->face.distance = distance_to_triangle(zero(), v1, v2, v3, &node->face.normal);

  polytope_attach_face(polytope, index, e1);
  polytope_attach_face(polytope, index, e2);
  polytope_attach_face(polytope, index, e3);

  polytope_add_node(polytope, node, index);

  return index;
}

static void polytope_remove_face(polytope *polytope, uint16_t face) {
  if (face == NIL) {
    return;
  }

  polytope_node *face_node = &polytope->nodes[face];
  uint16_t e1 = face_node->face.edges[0];
  uint16_t e2 = face_node->face.edges[1];
  uint16_t e3 = face_node->face.edges[2];

  polytope_detach_face(polytope, face, e1);
  polytope_detach_face(polytope, face, e2);
  polytope_detach_face(polytope, face, e3);

  polytope_remove_node(polytope, face);
}

static void polytope_remove_edge(polytope *polytope, uint16_t edge) {
  if (edge == NIL) {
    return;
  }

  polytope_node *edge_node = &polytope->nodes[edge];
  if (edge_node->edge.attached_faces[0] != NIL || edge_node->edge.attached_faces[1] != NIL) {
    TraceLog(LOG_FATAL, "Removing face with attached face");
    return;
  }

  uint16_t v1 = edge_node->edge.verticies[0];
  uint16_t v2 = edge_node->edge.verticies[1];

  polytope_detach_edge(polytope, edge, v1);
  polytope_detach_edge(polytope, edge, v2);

  polytope_remove_node(polytope, edge);
}

static void polytope_remove_vertex(polytope *polytope, uint16_t vertex) {}

static void polytope_update_nearest(polytope *polytope) {
  polytope->nearest_distance = FLT_MAX;

  uint16_t index = polytope->last_nodes[NODE_FACE];

  while (index != NIL) {
    polytope_node node = polytope->nodes[index];

    float distance = node.face.distance;
    if (distance < polytope->nearest_distance) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }

    index = node.prev;
  }
}

static bool polytope_is_face_visible(const polytope_node *face, v3 support_point) {
  return dot(normalize(face->face.normal), normalize(support_point)) > visibility_epsilon;
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

  polytope_update_nearest(polytope);

  return true;
}

static void epa_calculate_contact(physics_world *world, polytope *polytope) {}

static void epa_update_visible_faces(polytope *polytope) {
  uint16_t stack_ptr = 1;
  uint16_t stack[MAX_VISIBLE_FACES] = {polytope->nearest};

  polytope_node *face_node = &polytope->nodes[polytope->nearest];
  face_node->flags |= FLAG_FOR_REMOVAL;

  while (stack_ptr > 0) {
    uint16_t face_index = stack[--stack_ptr];
    face_node = &polytope->nodes[face_index];

    for (count_t i = 0; i < 3; ++i) {
      uint16_t edge_index = face_node->face.edges[i];
      polytope_node *edge_node = &polytope->nodes[edge_index];

      count_t visible_count = 0;
      for (count_t j = 0; j < 2; ++j) {
        uint16_t adjasent_face_index = edge_node->edge.attached_faces[j];
        face_node = &polytope->nodes[adjasent_face_index];

        if (face_node->flags & FLAG_FOR_REMOVAL) {
          visible_count += 1;
          continue;
        }

        if (polytope_is_face_visible(face_node, simulation_state.new_support.v)) {
          visible_count += 1;
          face_node->flags |= FLAG_FOR_REMOVAL;
          stack[stack_ptr++] = adjasent_face_index;
        }
      }

      if (visible_count == 2) {
        edge_node->flags |= FLAG_FOR_REMOVAL;
      } else {
        edge_node->flags |= FLAG_BORDER_EDGE;
      }
    }
  }
}

static void epa_expand_polytope(polytope *polytope, support_point p) {
  epa_update_visible_faces(polytope);

  polytope_for_each_node(polytope, index, NODE_FACE) {
    if (polytope->nodes[index].flags & FLAG_FOR_REMOVAL) {
      polytope_remove_face(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, NODE_EDGE) {
    if (polytope->nodes[index].flags & FLAG_FOR_REMOVAL) {
      polytope_remove_edge(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, NODE_VERTEX) {
    if (polytope->nodes[index].flags & FLAG_FOR_REMOVAL) {
      polytope_remove_vertex(polytope, index);
    }
  }

  uint16_t new_vertex = polytope_add_vertex(polytope, simulation_state.new_support);

  polytope_for_each_node(polytope, index, NODE_EDGE) {
    polytope_node *edge_node = &polytope->nodes[index];
    if ((edge_node->flags & FLAG_BORDER_EDGE) == 0) {
      continue;
    }

    uint16_t edge_index = index;
    uint16_t first_connected_vertex_index = edge_node->edge.verticies[0];
    uint16_t first_new_edge = polytope_add_edge(polytope, new_vertex, first_connected_vertex_index);
    uint16_t prev_edge = first_new_edge;

    uint16_t connected_vertex_index = edge_node->edge.verticies[1];
    while (connected_vertex_index != first_connected_vertex_index) {
      polytope->nodes[edge_index].flags = 0;

      uint16_t new_edge = polytope_add_edge(polytope, new_vertex, connected_vertex_index);
      polytope_add_face(polytope, prev_edge, new_edge, edge_index);

      polytope_node *connected_vertex_node = &polytope->nodes[connected_vertex_index];
      uint16_t attached_edge_index = connected_vertex_node->vertex.first_attached_edge;
      while (attached_edge_index != NIL) {
        polytope_node attached_edge_node = polytope->nodes[attached_edge_index];
        count_t i = attached_edge_node.edge.verticies[0] == connected_vertex_index ? 0 : 1;

        if (attached_edge_index == new_edge || attached_edge_index == edge_index) {
          attached_edge_index = attached_edge_node.edge.next_attached_edges[i];
          continue;
        }

        if (attached_edge_node.flags & FLAG_BORDER_EDGE) {
          edge_index = attached_edge_index;
          connected_vertex_index = attached_edge_node.edge.verticies[1 - i];
          break;
        }

        attached_edge_index = attached_edge_node.edge.next_attached_edges[i];
      }

      prev_edge = new_edge;
    }

    polytope_add_face(polytope, first_new_edge, prev_edge, edge_index);

    break;
  }

  polytope_update_nearest(polytope);
}

static void reset_simulation(physics_world *world) {
  simulation_state.state = STATE_NONE;
  simulation_state.step = 0;
  simulation_state.realtime = false;

  polytope_clear(simulation_state.polytope);
  simulation_state.is_collision = gjk_check_intersection_bodies(world, body_1, body_2, &simulation_state.simplex);

  v3 p1 = physics_get_position(world, body_1);
  v3 p2 = physics_get_position(world, body_2);
  quat rot1 = physics_get_rotation(world, body_1);
  quat rot2 = physics_get_rotation(world, body_2);

  printf("Body 1: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f)\n", p1.x, p1.y, p1.z, rot1.x, rot1.y, rot1.z, rot1.w);
  printf("Body 2: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f)\n", p2.x, p2.y, p2.z, rot2.x, rot2.y, rot2.z, rot2.w);
}

static void advance_simulation(physics_world *world) {
  if (!simulation_state.is_collision) {
    return;
  }

  v3 closest;
  polytope_node closest_face = simulation_state.polytope->nodes[simulation_state.polytope->nearest];
  v3 direction = closest_face.face.normal;
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
      if (closest_face.type == NODE_VERTEX) {
        simulation_state.state = STATE_FINISHED;
        epa_calculate_contact(world, simulation_state.polytope);
        return;
      }

      float distance = dot(direction, simulation_state.new_support.v);
      if (distance - closest_face.face.distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        epa_calculate_contact(world, simulation_state.polytope);
        return;
      }

      v3 a, b, c;
      if (closest_face.type == NODE_EDGE) {
        polytope_get_edge_vertices(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b);
        distance = distance_to_line_segment(simulation_state.new_support.v, a, b, &closest);
      } else {
        polytope_get_face_verticies(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b, &c);
        distance = distance_to_triangle(simulation_state.new_support.v, a, b, c, &closest);
      }

      if (distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        epa_calculate_contact(world, simulation_state.polytope);
        return;
      }

      epa_update_visible_faces(simulation_state.polytope);
      simulation_state.state = STATE_EXPANSION;
      break;

    case STATE_EXPANSION:
      epa_expand_polytope(simulation_state.polytope, simulation_state.new_support);

      simulation_state.step += 1;
      simulation_state.state = STATE_PICK_NEAREST;
      break;

    default:
      break;
  }
}

static void reset_bodies(physics_world *world) {
  physics_reset(world);

  unregister_gizmo(gizmo_1);
  unregister_gizmo(gizmo_2);

  body b1, b2;
  switch (simulation_state.bodies) {
    case BODIES_SPHERES:
      b1 = physics_add_sphere_dynamic(world, 2, 1);
      b2 = physics_add_sphere_dynamic(world, 2, 0.7);
      break;

    case BODIES_BOXES:
      b1 = physics_add_box_dynamic(world, 2, vec3(1.3, 1.3, 1.3));
      b2 = physics_add_box_dynamic(world, 2, vec3(1.3, 1.3, 1.3));
      break;

    default:
      return;
  }

  *b2.position = vec3(0, 7, 0);
  *b1.position = vec3(0, 6.41, 0);
  *b2.rotation = (quat){.x = 0.0582429692, .y = 0.0582429692, .z = 0.0582429692, .w = 0.994898676};
  *b1.rotation = (quat){.x = 0.00147654035, .y = 0.00147654035, .z = 0.00147654035, .w = 0.999996721};

  body_1 = b1.handle;
  body_2 = b2.handle;

  gizmo_1 = register_gizmo(b1.position, b1.rotation);
  gizmo_2 = register_gizmo(b2.position, b2.rotation);
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
  simulation_state.bodies = BODIES_BOXES;
  reset_bodies(world);

  uint32_t memory_size = polytope_memory_size(POLYTOPE_MAX_NODES);
  uint8_t *polytope_memory = malloc(memory_size);

  simulation_state.polytope = polytope_init(polytope_memory, POLYTOPE_MAX_NODES);
  reset_simulation(world);
}

void scenario_teardown() { free(simulation_state.polytope); }

void scenario_draw_scene(physics_world *world) {
  render_minkowski_difference(world);
  DrawSphere(zero(), 0.02, BLUE);

  if (simulation_state.realtime) {
    simulation_state.is_collision = gjk_check_intersection_bodies(world, body_1, body_2, &simulation_state.simplex);

    if (!simulation_state.is_collision) {
      return;
    }

    polytope_from_simplex(simulation_state.polytope, &simulation_state.simplex);

    float tolerance = physics_edit_config(world)->epa_tolerance;
    while (1) {
      polytope_node closest_face = simulation_state.polytope->nodes[simulation_state.polytope->nearest];
      v3 direction = closest_face.face.normal;

      support_point support = support_bodies(world, direction, body_1, body_2);
      if (closest_face.type == NODE_VERTEX) {
        epa_calculate_contact(world, simulation_state.polytope);
        break;
      }

      float distance = dot(direction, support.v);
      if (distance - closest_face.face.distance < tolerance) {
        epa_calculate_contact(world, simulation_state.polytope);
        break;
      }

      v3 a, b, c, closest;
      if (closest_face.type == NODE_EDGE) {
        polytope_get_edge_vertices(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b);
        distance = distance_to_line_segment(simulation_state.new_support.v, a, b, &closest);
      } else {
        polytope_get_face_verticies(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b, &c);
        distance = distance_to_triangle(support.v, a, b, c, &closest);
      }

      if (distance < tolerance) {
        epa_calculate_contact(world, simulation_state.polytope);
        break;
      }

      epa_expand_polytope(simulation_state.polytope, support);
    }

    render_polytope(simulation_state.polytope);
    draw_arrow(simulation_state.contact.point, scale(simulation_state.contact.normal, 1.3), ORANGE);
    draw_arrow(simulation_state.ccd_contact.point, scale(simulation_state.ccd_contact.normal, 1.3), VIOLET);
  } else {
    if (simulation_state.state == STATE_NONE) {
      render_simplex(&simulation_state.simplex);
    } else {
      render_polytope(simulation_state.polytope);
    }

    if (simulation_state.state == STATE_NEW_SUPPORT) {
      v3 vt1, vt2, vt3;
      polytope_get_face_verticies(simulation_state.polytope, simulation_state.polytope->nearest, &vt1, &vt2, &vt3);

      v3 center = scale(add(vt1, add(vt2, vt3)), 1.0 / 3);
      DrawLine3D(center, simulation_state.new_support.v, YELLOW);
      DrawSphere(simulation_state.new_support.v, 0.05, ORANGE);
    }

    if (simulation_state.state == STATE_FINISHED) {
      draw_arrow(simulation_state.contact.point, scale(simulation_state.contact.normal, 1.3), ORANGE);
      draw_arrow(simulation_state.ccd_contact.point, scale(simulation_state.ccd_contact.normal, 1.3), VIOLET);
    }
  }
}

static void render_minkowski_difference(const physics_world *world) {
  if (!ui_state.draw_minkowski) {
    return;
  }

  count_t n;

  body_shape s1 = physics_get_shapes(world, body_1, &n)[0];
  body_shape s2 = physics_get_shapes(world, body_2, &n)[0];
  v3 p1 = physics_get_position(world, body_1);
  v3 p2 = physics_get_position(world, body_2);

  float r1, r2;
  v3 e1, e2, center;
  switch (simulation_state.bodies) {
    case BODIES_SPHERES:
      r1 = s1.sphere.radius;
      r2 = s2.sphere.radius;

      center = sub(p1, p2);
      float size = r1 + r2;

      DrawSphereWires(center, size, 32, 32, BLACK);
      break;

    case BODIES_BOXES:
      e1 = s1.box.size;
      e2 = s2.box.size;

      center = sub(p1, p2);
      v3 ss = add(e1, e2);

      DrawCubeWires(center, ss.x, ss.y, ss.z, RED);
      break;

    default:
      break;
  }
}

static void render_simplex(const simplex *s) {
  if (!simulation_state.is_collision) {
    return;
  }

  const uint8_t alpha = (uint8_t)(ui_state.simplex_alpha * 255);

  Color color = GREEN;
  color.a = alpha;

  BeginBlendMode(BLEND_ALPHA);
  DrawTriangle3D(s->points[2].v, s->points[0].v, s->points[1].v, color);
  DrawTriangle3D(s->points[0].v, s->points[2].v, s->points[3].v, color);
  DrawTriangle3D(s->points[0].v, s->points[3].v, s->points[1].v, color);
  DrawTriangle3D(s->points[3].v, s->points[2].v, s->points[1].v, color);
  EndBlendMode();

  DrawLine3D(s->points[0].v, s->points[1].v, BLACK);
  DrawLine3D(s->points[0].v, s->points[2].v, BLACK);
  DrawLine3D(s->points[0].v, s->points[3].v, BLACK);
  DrawLine3D(s->points[1].v, s->points[2].v, BLACK);
  DrawLine3D(s->points[1].v, s->points[3].v, BLACK);
  DrawLine3D(s->points[2].v, s->points[3].v, BLACK);
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
    } else if (simulation_state.state == STATE_EXPANSION) {
      if (simulation_state.polytope->nodes[face_index].flags & FLAG_FOR_REMOVAL) {
        color = GOLD;
      }
    }

    color.a = (unsigned char)(255 * ui_state.polytope_alpha);

    v3 v1, v2, vv3;
    polytope_get_face_verticies(p, face_index, &v1, &v2, &vv3);
    render_triangle(v1, v2, vv3, face->face.normal, color);

    DrawLine3D(v1, v2, BLACK);
    DrawLine3D(v2, vv3, BLACK);
    DrawLine3D(vv3, v1, BLACK);

    face_index = face->prev;
    face = &p->nodes[face->prev];
  }
}

void scenario_handle_input(physics_world *world, Camera *camera) {}

void scenario_simulate(physics_world *world, float dt) {}

void scenario_build_ui(physics_world *world) {
  ui_begin_area("EPA", &ui_state.collapsed);

  ui_checkbox("Draw Minkowski", &ui_state.draw_minkowski);
  ui_checkbox("Realtime detection", &simulation_state.realtime);

  char *values[] = {"Spheres", "Boxes", "Cylinders"};
  if (ui_dropdown("Bodies", values, 3, &simulation_state.bodies, &ui_state.dropdown_active)) {
    reset_bodies(world);
  }

  ui_value_float("Simplex alpha", &ui_state.simplex_alpha, 0.0, 1.0);
  ui_value_float("Polytope alpha", &ui_state.polytope_alpha, 0.0, 1.0);
  ui_value_float("Visibility epsilon", &visibility_epsilon, 0.001, 0.5);

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

  if (simulation_state.is_collision) {
    if (simulation_state.realtime || simulation_state.state == STATE_FINISHED) {
      ui_label_v3("Normal", simulation_state.contact.normal);
      ui_label_float("Depth", simulation_state.contact.depth);
      ui_label_v3("Point", simulation_state.contact.point);
    } else if (simulation_state.state != STATE_NONE) {
      polytope_node nearest_node = simulation_state.polytope->nodes[simulation_state.polytope->nearest];
      ui_label_float("Nearest distance", simulation_state.polytope->nearest_distance);
      ui_label_int("Nearest node type", nearest_node.type);
    }
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

static void dump_polytope(const polytope *polytope, const char *title) {
  printf("%s\n", title != NULL ? title : "Polytope");

  if (polytope == NULL) {
    printf("--- Vertices ---\n");
    printf("<null>\n");
    printf("--- Edges ---\n");
    printf("<null>\n");
    printf("--- Faces ---\n");
    printf("<null>\n");
    return;
  }

  printf("--- Vertices ---\n");
  for (uint16_t index = polytope->last_nodes[NODE_VERTEX]; index != NIL; index = polytope->nodes[index].prev) {
    v3 vertex = polytope->nodes[index].vertex.v.v;
    printf("%u: (%.6f, %.6f, %.6f)\n", index, vertex.x, vertex.y, vertex.z);
  }

  printf("--- Edges ---\n");
  for (uint16_t index = polytope->last_nodes[NODE_EDGE]; index != NIL; index = polytope->nodes[index].prev) {
    uint16_t *vertices = polytope->nodes[index].edge.verticies;
    printf("%u: (%u, %u)\n", index, vertices[0], vertices[1]);
  }

  printf("--- Faces ---\n");
  for (uint16_t index = polytope->last_nodes[NODE_FACE]; index != NIL; index = polytope->nodes[index].prev) {
    uint16_t *edges = polytope->nodes[index].face.edges;
    v3 v1, v2, v3;
    polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

    printf("%u: (%u, %u, %u)\n", index, edges[0], edges[1], edges[2]);
    printf("  v1: (%.6f, %.6f, %.6f)\n", v1.x, v1.y, v1.z);
    printf("  v2: (%.6f, %.6f, %.6f)\n", v2.x, v2.y, v2.z);
    printf("  v3: (%.6f, %.6f, %.6f)\n", v3.x, v3.y, v3.z);
  }
}
