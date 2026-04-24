#include "bnd-core.h"
#include <string.h>
#include <float.h>
#include <stdlib.h>

#define NIL 0

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
      support_point v;
      uint16_t first_attached_edge;
    } vertex;

    struct {
      uint16_t verticies[2];
      uint16_t next_attached_edges[2];
      uint16_t attached_faces[2];
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

polytope *pt;

static int compare_vertex_distance(const void *a, const void *b) {
  uint16_t i1 = *(uint16_t *)a;
  uint16_t i2 = *(uint16_t *)b;

  polytope_node v1 = pt->nodes[i1];
  polytope_node v2 = pt->nodes[i2];

  if (fabsf(v1.distance - v2.distance) < EPSILON) {
    return 0;
  } else if (v1.distance > v2.distance) {
    return 1;
  } else {
    return -1;
  }
}

static uint32_t polytope_memory_size(count_t max_nodes) {
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
  for (count_t i = 0; i < 2; ++i) {
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

  memset(node->edge.attached_faces, 0, 2 * sizeof(uint16_t));
  memset(node->edge.next_attached_edges, 0, 2 * sizeof(uint16_t));

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

  polytope_node current_nearest = polytope->nodes[polytope->nearest];
  while (index != NIL) {
    polytope_node node = polytope->nodes[index];

    float distance = node.distance;
    bool closer_than_same_type = node.type == current_nearest.type && distance < polytope->nearest_distance;
    bool much_closer_than_other_type = node.type != current_nearest.type && distance - polytope->nearest_distance < -0.01;
    if (closer_than_same_type || much_closer_than_other_type) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
      current_nearest = node;
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

static void epa_calculate_contact(polytope *polytope, contact *out_contact) {
  polytope_node nearest_node = polytope->nodes[polytope->nearest];
  out_contact->depth = sqrt(nearest_node.distance);
  out_contact->normal = negate(normalize(nearest_node.nearest_point));

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

  out_contact->point = scale(point, 1 / div);
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

      // TODO report error properly
      // TraceLog(LOG_FATAL, "Polytope capacity exceeded");
      return;
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

    vertices[4] = polytope_add_vertex(polytope, p);

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
      // TODO report error properly
      // TraceLog(LOG_FATAL, "Polytope capacity exceeded");
      return;
    }

    polytope_update_nearest(polytope);
  }
}

void epa_init(const physics_config *config) {
  count_t memory_size = polytope_memory_size(config->epa_max_nodes);
  uint8_t *memory = malloc(memory_size);
  pt = polytope_init(memory, config->epa_max_nodes);
}

void epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance,
                     contact *contact) {
  polytope_from_simplex(pt, simplex);

  while (1) {
    polytope_node closest_node = pt->nodes[pt->nearest];
    if (closest_node.type == NODE_VERTEX) {
      epa_calculate_contact(pt, contact);
      break;
    }

    v3 direction = closest_node.nearest_point;

    support_point p = support(ctx, normalize(direction));

    float distance = dot(direction, p.v);
    if (distance - closest_node.distance < tolerance) {
      epa_calculate_contact(pt, contact);
      break;
    }

    v3 a, b, c, closest;
    if (closest_node.type == NODE_EDGE) {
      polytope_get_edge_vertices(pt, pt->nearest, &a, &b);
      distance = distance_to_line_segment(p.v, a, b, &closest);
    } else {
      polytope_get_face_verticies(pt, pt->nearest, &a, &b, &c);
      distance = distance_to_triangle(p.v, a, b, c, &closest);
    }

    if (distance < tolerance) {
      epa_calculate_contact(pt, contact);
      break;
    }

    epa_expand_polytope(pt, p);
  }
}
