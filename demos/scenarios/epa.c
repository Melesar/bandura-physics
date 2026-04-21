#include "core.h"
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

#define MAX_ATTACHED_EDGES 2
#define MAX_ATTACHED_FACES 2

typedef enum {
  NODE_VERTEX,
  NODE_EDGE,
  NODE_FACE,

  NODE_TYPE_COUNT,
} node_type;

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

  v3 nearest_point;
  float distance;

  uint16_t next;
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

  uint32_t step;
  bool is_collision;
  bool realtime;
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
support_point support_bodies(physics_world *world, v3 direction, body_handle body_1, body_handle body_2);

static void dump_polytope(const polytope *polytope, const char *title);
static void render_minkowski_difference(const physics_world *world);
static void render_simplex(const simplex *s);
static void render_polytope(const polytope *p);

body_handle body_1;
body_handle body_2;

static int compare_vertex_distance(const void *a, const void *b) {
  uint16_t i1 = *(uint16_t *)a;
  uint16_t i2 = *(uint16_t *)b;

  polytope *polytope = simulation_state.polytope;
  polytope_node v1 = polytope->nodes[i1];
  polytope_node v2 = polytope->nodes[i2];

  if (fabsf(v1.distance - v2.distance) < EPSILON) {
    return 0;
  } else if (v1.distance > v2.distance) {
    return 1;
  } else {
    return -1;
  }
}





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
  polytope_node *last_node = &polytope->nodes[last_node_index];

  node->prev = last_node_index;
  node->next = NIL;

  polytope->last_nodes[node->type] = index;
  last_node->next = index;
}

static void polytope_remove_node(polytope *polytope, uint16_t index) {
  polytope_node node = polytope->nodes[index];

  uint16_t node_index = polytope->last_nodes[node.type];
  polytope_node *current_node = &polytope->nodes[node_index];

  if (node_index == index) {
    polytope->last_nodes[node.type] = current_node->prev;
    if (current_node->prev != NIL) {
      polytope->nodes[current_node->prev].next = NIL;
    }
  } else {
    while (current_node->prev != index) {
      node_index = current_node->prev;
      current_node = &polytope->nodes[node_index];
    }

    current_node->prev = node.prev;
    if (current_node->prev != NIL) {
      polytope->nodes[current_node->prev].next = node_index;
    }
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
  node->vertex.v = p;
  node->vertex.first_attached_edge = NIL;
  node->nearest_point = p.v;
  node->distance = lensq(p.v);

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
  node->distance = distance_to_line_segment(zero(), polytope->nodes[v1].vertex.v.v, polytope->nodes[v2].vertex.v.v,
                                            &node->nearest_point);

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
  node->face.edges[0] = e1;
  node->face.edges[1] = e2;
  node->face.edges[2] = e3;

  v3 v1, v2, v3;
  polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

  node->distance = distance_to_triangle(zero(), v1, v2, v3, &node->nearest_point);

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

static void polytope_update_nearest_for_type(polytope *polytope, uint16_t node_type) {
  uint16_t index = polytope->last_nodes[node_type];

  while (index != NIL) {
    polytope_node node = polytope->nodes[index];

    float distance = node.distance;
    if (distance < polytope->nearest_distance) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }

    index = node.prev;
  }
}

static void polytope_update_nearest(polytope *polytope) {
  polytope->nearest_distance = FLT_MAX;

  polytope_update_nearest_for_type(polytope, NODE_VERTEX);
  polytope_update_nearest_for_type(polytope, NODE_EDGE);
  polytope_update_nearest_for_type(polytope, NODE_FACE);
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

static void epa_calculate_contact(polytope *polytope) {
  polytope_node nearest_node = polytope->nodes[polytope->nearest];
  simulation_state.contact.depth = sqrt(nearest_node.distance);
  simulation_state.contact.normal = normalize(nearest_node.nearest_point);

  // The following algorithm for computing the contact point is taken from the
  // libccd by Daniel Fiser.
  //
  // https://github.com/danfis/libccd/blob/master/src/ccd.c#L133

  // We re-use the free_list array for storing the indicies of verticies.
  // Since EPA is finished already, the polytope won't expand and we don't need to
  // keep track of free indicies anymore.

  count_t vertex_count = 0;

  count_t node_index = polytope->last_nodes[NODE_VERTEX];
  while (node_index != NIL) {
    polytope_node *vertex = &polytope->nodes[node_index];

    polytope->free_list[vertex_count++] = node_index;
    node_index = vertex->prev;
  }

  qsort(polytope->free_list, vertex_count, sizeof(uint16_t), compare_vertex_distance);

  if (vertex_count % 2 == 1) {
    vertex_count += 1;
  }

  v3 point = zero();
  float div = 0;
  for (count_t i = 0; i < vertex_count / 2; ++i) {
    uint16_t index = polytope->free_list[i];
    polytope_node node = polytope->nodes[index];

    point = add(point, node.vertex.v.v1);
    point = add(point, node.vertex.v.v2);
    div += 2.0;
  }

  simulation_state.contact.point = scale(point, 1 / div);
}

static void epa_expand_polytope(polytope *polytope, support_point p) {
  polytope_node closest_node = polytope->nodes[polytope->nearest];
  if (closest_node.type == NODE_FACE) {
    uint16_t edges[6];
    uint16_t verts[5];
    memcpy(edges, closest_node.face.edges, 3 * sizeof(uint16_t));
    memcpy(verts, polytope->nodes[edges[0]].edge.verticies, 2 * sizeof(uint16_t));
    memcpy(verts + 2, polytope->nodes[edges[1]].edge.verticies, 2 * sizeof(uint16_t));

    if (verts[2] != verts[1] && verts[3] != verts[1]) {
      edges[3] = edges[1];
      edges[1] = edges[2];
      edges[2] = edges[3];
    }

    if (verts[3] != verts[0] && verts[3] != verts[1]) {
      verts[2] = verts[3];
    }

    polytope_remove_face(polytope, polytope->nearest);

    verts[3] = polytope_add_vertex(polytope, p);
    edges[3] = polytope_add_edge(polytope, verts[3], verts[0]);
    edges[4] = polytope_add_edge(polytope, verts[3], verts[1]);
    edges[5] = polytope_add_edge(polytope, verts[3], verts[2]);

    if (polytope_add_face(polytope, edges[3], edges[4], edges[0]) == NIL ||
      polytope_add_face(polytope, edges[4], edges[5], edges[1]) == NIL ||
      polytope_add_face(polytope, edges[5], edges[3], edges[2]) == NIL) {

        TraceLog(LOG_FATAL, "Polytope capacity exceeded");
      }

    polytope_update_nearest(polytope);

  } else if (closest_node.type == NODE_EDGE) {
    uint16_t faces[2];
    uint16_t edges[8];
    uint16_t vertices[5];

    vertices[0] = closest_node.edge.verticies[0];
    vertices[2] = closest_node.edge.verticies[1];

    memcpy(faces, closest_node.edge.attached_faces, 2 * sizeof(uint16_t));
    memcpy(edges, polytope->nodes[faces[0]].face.edges, 3 * sizeof(uint16_t));

    if (edges[0] == polytope->nearest) {
      edges[0] = edges[2];
    } else if (edges[1] == polytope->nearest) {
      edges[1] = edges[2];
    }

    polytope_node e = polytope->nodes[edges[0]];
    vertices[1] = e.edge.verticies[0];
    vertices[3] = e.edge.verticies[1];

    if (vertices[1] != vertices[0] && vertices[3] != vertices[0]) {
      edges[2] = edges[0];
      edges[0] = edges[1];
      edges[1] = edges[2];

      if (vertices[1] == vertices[2]) {
        vertices[1] = vertices[3];
      }
    } else if (vertices[1] == vertices[0]) {
      vertices[1] = vertices[3];
    }

    memcpy(edges + 2, polytope->nodes[faces[1]].face.edges, 3 * sizeof(uint16_t));

    if (edges[2] == polytope->nearest) {
      edges[2] = edges[4];
    } else if (edges[3] == polytope->nearest) {
      edges[3] = edges[4];
    }

    memcpy(vertices + 3, polytope->nodes[edges[2]].edge.verticies, 2 * sizeof(uint16_t));

    if (vertices[3] != vertices[2] && vertices[4] != vertices[2]) {
      edges[4] = edges[2];
      edges[2] = edges[3];
      edges[3] = edges[4];
      if (vertices[3] == vertices[0]) {
        vertices[3] = vertices[4];
      }
    } else if (vertices[3] == vertices[2]) {
      vertices[3] = vertices[4];
    }

    vertices[4] = polytope_add_vertex(polytope, simulation_state.new_support);

    polytope_remove_face(polytope, faces[0]);
    polytope_remove_face(polytope, faces[1]);
    polytope_remove_edge(polytope, polytope->nearest);

    edges[4] = polytope_add_edge(polytope, vertices[4], vertices[2]);
    edges[5] = polytope_add_edge(polytope, vertices[4], vertices[0]);
    edges[6] = polytope_add_edge(polytope, vertices[4], vertices[1]);
    edges[7] = polytope_add_edge(polytope, vertices[4], vertices[3]);

    if (polytope_add_face(polytope, edges[1], edges[4], edges[6]) == NIL ||
      polytope_add_face(polytope, edges[0], edges[6], edges[5]) == NIL ||
      polytope_add_face(polytope, edges[3], edges[5], edges[7]) == NIL ||
      polytope_add_face(polytope, edges[4], edges[7], edges[2]) == NIL) {
        TraceLog(LOG_FATAL, "Polytope capacity exceeded");
      }

    polytope_update_nearest(polytope);
  }
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
        epa_calculate_contact(simulation_state.polytope);
        return;
      }

      float distance = dot(direction, simulation_state.new_support.v);
      if (distance - closest_node.distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        epa_calculate_contact(simulation_state.polytope);
        return;
      }

      v3 a, b, c;
      if (closest_node.type == NODE_EDGE) {
        polytope_get_edge_vertices(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b);
        distance = distance_to_line_segment(simulation_state.new_support.v, a, b, &closest);
      } else {
        polytope_get_face_verticies(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b, &c);
        distance = distance_to_triangle(simulation_state.new_support.v, a, b, c, &closest);
      }

      if (distance < tolerance) {
        simulation_state.state = STATE_FINISHED;
        epa_calculate_contact(simulation_state.polytope);
        return;
      }

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
  body b1 = physics_add_sphere_dynamic(world, 2, 1);
  body b2 = physics_add_sphere_dynamic(world, 2, 0.7);

  *b1.position = vec3(0.477, 2.598, 0.535);
  *b2.position = vec3(-0.125, 3.442, -0.378);

  body_1 = b1.handle;
  body_2 = b2.handle;

  register_gizmo(b1.position, b1.rotation);
  register_gizmo(b2.position, b2.rotation);

  uint32_t memory_size = polytope_memory_size(POLYTOPE_MAX_NODES);
  uint8_t *polytope_memory = malloc(memory_size);

  simulation_state.polytope = polytope_init(polytope_memory, POLYTOPE_MAX_NODES);
  reset_simulation(world);
}

void scenario_teardown() { free(simulation_state.polytope); }

void scenario_draw_scene(physics_world *world) {
  render_minkowski_difference(world);

  if (simulation_state.realtime) {
    simulation_state.is_collision = gjk_check_intersection_bodies(world, body_1, body_2, &simulation_state.simplex);

    if (!simulation_state.is_collision) {
      return;
    }

    polytope_from_simplex(simulation_state.polytope, &simulation_state.simplex);

    float tolerance = physics_edit_config(world)->epa_tolerance;
    while(1) {
      polytope_node closest_node = simulation_state.polytope->nodes[simulation_state.polytope->nearest];
      v3 direction = closest_node.nearest_point;

      support_point support = support_bodies(world, direction, body_1, body_2);
      if (closest_node.type == NODE_VERTEX) {
        epa_calculate_contact(simulation_state.polytope);
        break;
      }

      float distance = dot(direction, support.v);
      if (distance - closest_node.distance < tolerance) {
        epa_calculate_contact(simulation_state.polytope);
        break;
      }

      v3 a, b, c, closest;
      if (closest_node.type == NODE_EDGE) {
        polytope_get_edge_vertices(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b);
        distance = distance_to_line_segment(simulation_state.new_support.v, a, b, &closest);
      } else {
        polytope_get_face_verticies(simulation_state.polytope, simulation_state.polytope->nearest, &a, &b, &c);
        distance = distance_to_triangle(support.v, a, b, c, &closest);
      }

      if (distance < tolerance) {
        epa_calculate_contact(simulation_state.polytope);
        break;
      }

      epa_expand_polytope(simulation_state.polytope, support);
    }

    render_polytope(simulation_state.polytope);
  } else {
    if (simulation_state.state == STATE_NONE) {
      render_simplex(&simulation_state.simplex);
    } else {
      render_polytope(simulation_state.polytope);
    }

    if (simulation_state.state == STATE_NEW_SUPPORT) {
      v3 closest = simulation_state.polytope->nodes[simulation_state.polytope->nearest].nearest_point;
      DrawSphere(simulation_state.new_support.v, 0.05, ORANGE);
      DrawLine3D(closest, simulation_state.new_support.v, YELLOW);
    }

    if (simulation_state.state == STATE_FINISHED) {
      draw_arrow(simulation_state.contact.point, scale(simulation_state.contact.normal, 0.3), ORANGE);
    } else {
      DrawSphere(zero(), 0.02, BLUE);
    }

  }

}

static void render_minkowski_difference(const physics_world *world) {
  if (!ui_state.draw_minkowski) {
    return;
  }

  count_t n;

  float r1 = physics_get_shapes(world, body_1, &n)[0].sphere.radius;
  float r2 = physics_get_shapes(world, body_2, &n)[0].sphere.radius;

  v3 p1 = physics_get_position(world, body_1);
  v3 p2 = physics_get_position(world, body_2);

  v3 center = sub(p1, p2);
  float size = r1 + r2;

  DrawSphereWires(center, size, 32, 32, BLACK);
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
    }

    v3 v1, v2, v3;
    polytope_get_face_verticies(p, face_index, &v1, &v2, &v3);
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
  ui_checkbox("Realtime detection", &simulation_state.realtime);

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
