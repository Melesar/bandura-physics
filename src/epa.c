#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <string.h>
#include <float.h>

#define NIL 0
#define EPA_MAX_ATTEMPTS 128
#define VISIBLE_FACES_STACK_SIZE 16

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

typedef struct {
  body_support v;
  uint16_t first_attached_edge;
} vertex;

typedef struct {
  uint16_t verticies[2];
  uint16_t next_attached_edges[2];
  uint16_t attached_faces[2];
} edge;

typedef struct {
  uint16_t edges[3];
  bnd_v3 normal;
  float distance;
} face;

typedef union {
  vertex vertex;
  edge edge;
  face face;
} polytope_node_type;

typedef struct {
  node_type type;
  polytope_node_type value;

  uint16_t prev;
} polytope_node;

typedef struct {
  polytope_node *nodes;
  uint8_t *flags;
  uint16_t *free_list;

  uint16_t last_nodes[NODE_TYPE_COUNT];

  uint16_t node_count;
  uint16_t free_count;
  uint16_t max_nodes;

  uint16_t nearest;
  float nearest_distance;
} polytope;

const float visibility_epsilon = 0.25;
static polytope *pt;

static uint32_t polytope_flags_size(uint16_t max_nodes) {
  return max_nodes + 1 + (max_nodes % 2 == 0);
}

uint32_t polytope_memory_size(uint16_t max_nodes) {
  return sizeof(polytope) + (max_nodes + 1) * sizeof(polytope_node) + polytope_flags_size(max_nodes) + max_nodes * sizeof(uint16_t);
}

static polytope *polytope_init(uint8_t *memory, uint16_t max_nodes) {
  polytope *result = (polytope *)memory;
  memset(memory, 0, sizeof(polytope));

  memory += sizeof(polytope);
  result->nodes = (polytope_node *)memory;

  memory += (max_nodes + 1) * sizeof(polytope_node);
  result->flags = memory;

  uint32_t flags_size = polytope_flags_size(max_nodes);
  memset(memory, 0, flags_size);

  memory += flags_size;
  result->free_list = (uint16_t *)memory;

  result->max_nodes = max_nodes;
  result->nearest_distance = FLT_MAX;

  return result;
}

static void polytope_clear(polytope *polytope) {
  polytope->node_count = 0;
  polytope->free_count = 0;

  memset(polytope->last_nodes, 0, NODE_TYPE_COUNT * sizeof(uint16_t));
  memset(polytope->flags, 0, polytope_flags_size(polytope->max_nodes));

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
  if (vertex_node->value.vertex.first_attached_edge == NIL) {
    vertex_node->value.vertex.first_attached_edge = edge;
  } else {
    uint16_t attached_edge = vertex_node->value.vertex.first_attached_edge;
    polytope_node *edge_node;
    int i;
    do {
      edge_node = &polytope->nodes[attached_edge];
      i = edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
      attached_edge = edge_node->value.edge.next_attached_edges[i];
    } while (attached_edge != NIL);

    edge_node->value.edge.next_attached_edges[i] = edge;
  }
}

static void polytope_attach_face(polytope *polytope, uint16_t face, uint16_t edge) {
  polytope_node *edge_node = &polytope->nodes[edge];
  for (count_t i = 0; i < 2; ++i) {
    if (edge_node->value.edge.attached_faces[i] == NIL) {
      edge_node->value.edge.attached_faces[i] = face;
      break;
    }
  }
}

static void polytope_detach_face(polytope *polytope, uint16_t face, uint16_t edge) {
  polytope_node *edge_node = &polytope->nodes[edge];

  uint16_t *attached_faces = edge_node->value.edge.attached_faces;
  for (count_t i = 0; i < 2; ++i) {
    if (attached_faces[i] == face) {
      attached_faces[i] = NIL;
    }
  }
}

static void polytope_detach_edge(polytope *polytope, uint16_t edge, uint16_t vertex) {
  polytope_node *vertex_node = &polytope->nodes[vertex];

  uint16_t next_edge = vertex_node->value.vertex.first_attached_edge;
  polytope_node *next_edge_node = &polytope->nodes[next_edge];
  int i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
  if (next_edge == edge) {
    vertex_node->value.vertex.first_attached_edge = next_edge_node->value.edge.next_attached_edges[i];
    if (vertex_node->value.vertex.first_attached_edge == NIL) {
      polytope->flags[vertex] |= FLAG_FOR_REMOVAL;
    }
    return;
  }

  next_edge = next_edge_node->value.edge.next_attached_edges[i];

  while (next_edge != edge) {
    next_edge_node = &polytope->nodes[next_edge];
    i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
    next_edge = next_edge_node->value.edge.next_attached_edges[i];
  }

  polytope_node *current_node = next_edge_node;
  int current_i = i;

  next_edge_node = &polytope->nodes[next_edge];
  i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;

  current_node->value.edge.next_attached_edges[current_i] = next_edge_node->value.edge.next_attached_edges[i];
}

static void polytope_get_face_verticies(const polytope *polytope, uint16_t face, bnd_v3 *v1, bnd_v3 *v2, bnd_v3 *v3) {
  polytope_node node = polytope->nodes[face];
  uint16_t e1 = node.value.face.edges[0];
  uint16_t e2 = node.value.face.edges[1];
  uint16_t *edge_verts_1 = polytope->nodes[e1].value.edge.verticies;
  uint16_t *edge_verts_2 = polytope->nodes[e2].value.edge.verticies;

  *v1 = polytope->nodes[edge_verts_1[0]].value.vertex.v.p;
  *v2 = polytope->nodes[edge_verts_1[1]].value.vertex.v.p;

  *v3 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
    ? polytope->nodes[edge_verts_2[1]].value.vertex.v.p
    : polytope->nodes[edge_verts_2[0]].value.vertex.v.p;
}

static uint16_t polytope_add_vertex(polytope *polytope, body_support p) {
  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  polytope_node *node = &polytope->nodes[index];
  node->type = NODE_VERTEX;
  node->value.vertex.v = p;
  node->value.vertex.first_attached_edge = NIL;

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
  node->value.edge.verticies[0] = v1;
  node->value.edge.verticies[1] = v2;

  memset(node->value.edge.attached_faces, 0, 2 * sizeof(uint16_t));
  memset(node->value.edge.next_attached_edges, 0, 2 * sizeof(uint16_t));

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
  node->value.face.edges[0] = e1;
  node->value.face.edges[1] = e2;
  node->value.face.edges[2] = e3;

  bnd_v3 v1, v2, v3;
  polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

  if (bnd_v3_distancesqr(v1, v2) < EPSILON || bnd_v3_distancesqr(v2, v3) < EPSILON || bnd_v3_distancesqr(v3, v1) < EPSILON) {
    polytope->free_list[polytope->free_count++] = index;
    return NIL;
  }

  node->value.face.distance = distance_to_triangle(bnd_v3_zero(), v1, v2, v3, &node->value.face.normal);

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
  uint16_t e1 = face_node->value.face.edges[0];
  uint16_t e2 = face_node->value.face.edges[1];
  uint16_t e3 = face_node->value.face.edges[2];

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
  if (edge_node->value.edge.attached_faces[0] != NIL || edge_node->value.edge.attached_faces[1] != NIL) {
    return;
  }

  uint16_t v1 = edge_node->value.edge.verticies[0];
  uint16_t v2 = edge_node->value.edge.verticies[1];

  polytope_detach_edge(polytope, edge, v1);
  polytope_detach_edge(polytope, edge, v2);

  polytope_remove_node(polytope, edge);
}

static void polytope_remove_vertex(polytope *polytope, uint16_t vertex) {
  if (vertex == NIL) {
    return;
  }

  polytope_remove_node(polytope, vertex);
}

static void polytope_clear_flags(polytope *polytope) {
  memset(polytope->flags, 0, polytope->node_count);
}

static void polytope_update_nearest(polytope *polytope) {
  polytope->nearest_distance = FLT_MAX;

  polytope_for_each_node(polytope, index, NODE_FACE) {
    polytope_node node = polytope->nodes[index];

    float distance = node.value.face.distance;
    if (distance < polytope->nearest_distance) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }
  }
}

static bool polytope_is_face_visible(const polytope_node *face, bnd_v3 support_point) {
  return bnd_v3_dot(bnd_v3_normalize(face->value.face.normal), bnd_v3_normalize(support_point)) > visibility_epsilon;
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
  polytope_clear_flags(polytope);

  return true;
}

static void epa_invalid_contact(body_support p, contact *contact) {
  contact->point = bnd_v3_scale(bnd_v3_add(p.p1.point, p.p2.point), 0.5);
  contact->normal = bnd_v3_up();
  contact->depth = 0.1;
}

static void epa_calculate_contact(const polytope *polytope, contact *contact) {
  polytope_node node = polytope->nodes[polytope->nearest];
  uint16_t e1 = node.value.face.edges[0];
  uint16_t e2 = node.value.face.edges[1];
  uint16_t *edge_verts_1 = polytope->nodes[e1].value.edge.verticies;
  uint16_t *edge_verts_2 = polytope->nodes[e2].value.edge.verticies;

  body_support v1 = polytope->nodes[edge_verts_1[0]].value.vertex.v;
  body_support v2 = polytope->nodes[edge_verts_1[1]].value.vertex.v;

  body_support vv3 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
    ? polytope->nodes[edge_verts_2[1]].value.vertex.v
    : polytope->nodes[edge_verts_2[0]].value.vertex.v;

  bnd_v3 barycenter = bnd_v3_barycentric(bnd_v3_zero(), v1.p, v2.p, vv3.p);
  bnd_v3 p1 = bnd_v3_add(bnd_v3_scale(v1.p1.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v2.p1.point, barycenter.y), bnd_v3_scale(vv3.p1.point, barycenter.z)));
  bnd_v3 p2 = bnd_v3_add(bnd_v3_scale(v1.p2.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v2.p2.point, barycenter.y), bnd_v3_scale(vv3.p2.point, barycenter.z)));

  contact->point = bnd_v3_scale(bnd_v3_add(p1, p2), 0.5);
  contact->depth = sqrt(node.value.face.distance);

  float length = bnd_v3_len(node.value.face.normal);
  if (length > EPSILON) {
    contact->normal = bnd_v3_scale(node.value.face.normal, -1.0 / length);
  } else {
    contact->normal = bnd_v3_up();
  }
}


void epa_get_final_points(bnd_v3 *points) {
  const polytope_node *node = &pt->nodes[pt->nearest];

  uint16_t e1 = node->value.face.edges[0];
  uint16_t e2 = node->value.face.edges[1];
  uint16_t *edge_verts_1 = pt->nodes[e1].value.edge.verticies;
  uint16_t *edge_verts_2 = pt->nodes[e2].value.edge.verticies;

  body_support *v1 = &pt->nodes[edge_verts_1[0]].value.vertex.v;
  body_support *v2 = &pt->nodes[edge_verts_1[1]].value.vertex.v;

  body_support *v3 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
    ? &pt->nodes[edge_verts_2[1]].value.vertex.v
    : &pt->nodes[edge_verts_2[0]].value.vertex.v;

  points[0] = v1->p1.point;
  points[1] = v2->p1.point;
  points[2] = v3->p1.point;

  points[3] = v1->p2.point;
  points[4] = v2->p2.point;
  points[5] = v3->p2.point;
}

static void epa_update_visible_faces(polytope *polytope, body_support p) {
  uint16_t stack_ptr = 1;
  uint16_t stack[VISIBLE_FACES_STACK_SIZE] = { polytope->nearest };

  polytope_node *face_node = &polytope->nodes[polytope->nearest];
  polytope->flags[polytope->nearest] |= FLAG_FOR_REMOVAL;

  while (stack_ptr > 0) {
    uint16_t face_index = stack[--stack_ptr];
    face_node = &polytope->nodes[face_index];

    for (count_t i = 0; i < 3; ++i) {
      uint16_t edge_index = face_node->value.face.edges[i];
      polytope_node *edge_node = &polytope->nodes[edge_index];

      count_t visible_count = 0;
      for (count_t j = 0; j < 2; ++j) {
        uint16_t adjasent_face_index = edge_node->value.edge.attached_faces[j];
        const polytope_node *adjasent_face_node = &polytope->nodes[adjasent_face_index];

        if (polytope->flags[adjasent_face_index] & FLAG_FOR_REMOVAL) {
          visible_count += 1;
        } else if (polytope_is_face_visible(adjasent_face_node, p.p)) {
          visible_count += 1;
          polytope->flags[adjasent_face_index] |= FLAG_FOR_REMOVAL;
          stack[stack_ptr++] = adjasent_face_index;
        }
      }

      if (visible_count == 2) {
        polytope->flags[edge_index] |= FLAG_FOR_REMOVAL;
      } else if (visible_count == 1) {
        polytope->flags[edge_index] |= FLAG_BORDER_EDGE;
      }
    }
  }
}

static bool epa_expand_polytope(polytope *polytope, body_support p) {
  epa_update_visible_faces(polytope, p);

  polytope_for_each_node(polytope, index, NODE_FACE) {
    if (polytope->flags[index] & FLAG_FOR_REMOVAL) {
      polytope_remove_face(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, NODE_EDGE) {
    if (polytope->flags[index] & FLAG_FOR_REMOVAL) {
      polytope_remove_edge(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, NODE_VERTEX) {
    if (polytope->flags[index] & FLAG_FOR_REMOVAL) {
      polytope_remove_vertex(polytope, index);
    }
  }

  uint16_t new_vertex = polytope_add_vertex(polytope, p);

  polytope_for_each_node(polytope, index, NODE_EDGE) {
    polytope_node *edge_node = &polytope->nodes[index];
    if ((polytope->flags[index] & FLAG_BORDER_EDGE) == 0) {
      continue;
    }

    uint16_t edge_index = index;
    uint16_t first_connected_vertex_index = edge_node->value.edge.verticies[0];
    uint16_t first_new_edge = polytope_add_edge(polytope, new_vertex, first_connected_vertex_index);
    uint16_t prev_edge = first_new_edge;

    uint16_t connected_vertex_index = edge_node->value.edge.verticies[1];
    while (connected_vertex_index != first_connected_vertex_index) {
      /**
       * Sometimes, in rare cases, this routine might add a duplicate edge - the one which is identical to some other
       * one. I saw this happen when spawning objects at the same position, so that they overlap. Need to investigate
       * this further, but for now in this case we just bail out and return some bogus contact data.
       */
      uint16_t new_edge = polytope_add_edge(polytope, new_vertex, connected_vertex_index);
      if (polytope_add_face(polytope, prev_edge, new_edge, edge_index) == NIL) {
        return false;
      }

      polytope_node *connected_vertex_node = &polytope->nodes[connected_vertex_index];
      uint16_t attached_edge_index = connected_vertex_node->value.vertex.first_attached_edge;
      while (attached_edge_index != NIL) {
        polytope_node attached_edge_node = polytope->nodes[attached_edge_index];
        count_t i = attached_edge_node.value.edge.verticies[0] == connected_vertex_index ? 0 : 1;

        if (attached_edge_index == new_edge || attached_edge_index == edge_index) {
          attached_edge_index = attached_edge_node.value.edge.next_attached_edges[i];
          continue;
        }

        if (polytope->flags[attached_edge_index] & FLAG_BORDER_EDGE) {
          edge_index = attached_edge_index;
          connected_vertex_index = attached_edge_node.value.edge.verticies[1 - i];
          break;
        }

        attached_edge_index = attached_edge_node.value.edge.next_attached_edges[i];
      }

      prev_edge = new_edge;
    }

    if (polytope_add_face(polytope, first_new_edge, prev_edge, edge_index) == NIL) {
      return false;
    }

    break;
  }

  polytope_update_nearest(polytope);
  polytope_clear_flags(polytope);

  return true;
}

bnd_error epa_init(bnd_world *world) {
  bnd_allocator allocator = world->allocator;
  uint32_t memory_size = polytope_memory_size(world->config.advanced.epa_max_nodes);
  uint8_t *memory;

  ALLOC_BUFFER8(memory, memory_size);

  pt = polytope_init(memory, world->config.advanced.epa_max_nodes);

  return OK;
}

void epa_get_contact(const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact) {
  PROFILE_FUNCTION

  if (!polytope_from_simplex(pt, simplex)) {
    epa_invalid_contact(simplex->points[0], contact);
    return;
  }

  count_t attempts = 0;
  body_support support_point;
  while (attempts++ < EPA_MAX_ATTEMPTS) {
    polytope_node closest_face = pt->nodes[pt->nearest];
    bnd_v3 direction = closest_face.value.face.normal;

    support_point = support(ctx, bnd_v3_normalize(direction));
    float distance = bnd_v3_dot(direction, support_point.p);
    if (distance - closest_face.value.face.distance < tolerance) {
      epa_calculate_contact(pt, contact);
      return;
    }

    bnd_v3 a, b, c, closest;
    polytope_get_face_verticies(pt, pt->nearest, &a, &b, &c);
    distance = distance_to_triangle(support_point.p, a, b, c, &closest);

    if (distance < tolerance) {
      epa_calculate_contact(pt, contact);
      return;
    }

    if (!epa_expand_polytope(pt, support_point)) {
      break;
    }
  }

  epa_invalid_contact(support_point, contact);
}
