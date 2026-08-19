#include "bnd-core.h"
#include "bnd-math.h"
#include "profiler.h"

#include <string.h>
#include <float.h>

#define NIL 0
#define EPA_MAX_ATTEMPTS 128
#define VISIBLE_NODES_STACK_SIZE 16

#define polytope_for_each_node(p, index, type)                                                                         \
  for (count_t index = p->last_nodes[type]; index != NIL; index = p->nodes[index].prev)

typedef enum {
  EPA_STATUS_OK,
  EPA_STATUS_NOT_RUN,
  EPA_STATUS_CONVERGED,
  EPA_STATUS_INVALID_POLYTOPE,
  EPA_STATUS_EXPANSION_FAILED,
  EPA_STATUS_ITERATION_LIMIT,
} epa_status;

const float visibility_epsilon = 0.25;

typedef void (*epa_iteration_observer)(const epa_polytope *polytope, body_support support_point, count_t iteration, void *user_data);

typedef struct {
  epa_status status;
  count_t iterations;
} epa_run_result;

static uint32_t polytope_flags_size(uint16_t max_nodes) {
  return max_nodes + 1 + (max_nodes % 2 == 0);
}

uint32_t polytope_memory_size(uint16_t max_nodes) {
  return (max_nodes + 1) * sizeof(epa_polytope_node) + polytope_flags_size(max_nodes) + max_nodes * sizeof(uint16_t);
}

static void polytope_clear(epa_polytope *polytope) {
  polytope->node_count = 0;
  polytope->free_count = 0;

  memset(polytope->last_nodes, 0, EPA_NODE_TYPE_COUNT * sizeof(uint16_t));
  memset(polytope->flags, 0, polytope_flags_size(polytope->max_nodes));

  polytope->nearest = NIL;
  polytope->nearest_distance = FLT_MAX;
}

static uint16_t polytope_free_index(epa_polytope *polytope) {
  if (polytope->free_count > 0) {
    return polytope->free_list[--polytope->free_count];
  }

  if (polytope->node_count < polytope->max_nodes) {
    return polytope->node_count++ + 1;
  }

  return NIL;
}

static void polytope_add_node(epa_polytope *polytope, epa_polytope_node *node, uint16_t index) {
  uint16_t last_node_index = polytope->last_nodes[node->type];

  node->prev = last_node_index;

  polytope->last_nodes[node->type] = index;
}

static void polytope_remove_node(epa_polytope *polytope, uint16_t index) {
  epa_polytope_node node = polytope->nodes[index];

  uint16_t node_index = polytope->last_nodes[node.type];
  epa_polytope_node *current_node = &polytope->nodes[node_index];

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

static void polytope_attach_edge(epa_polytope *polytope, uint16_t edge, uint16_t vertex) {
  epa_polytope_node *vertex_node = &polytope->nodes[vertex];
  if (vertex_node->value.vertex.first_attached_edge == NIL) {
    vertex_node->value.vertex.first_attached_edge = edge;
  } else {
    uint16_t attached_edge = vertex_node->value.vertex.first_attached_edge;
    epa_polytope_node *edge_node;
    int i;
    do {
      edge_node = &polytope->nodes[attached_edge];
      i = edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
      attached_edge = edge_node->value.edge.next_attached_edges[i];
    } while (attached_edge != NIL);

    edge_node->value.edge.next_attached_edges[i] = edge;
  }
}

static void polytope_attach_face(epa_polytope *polytope, uint16_t face, uint16_t edge) {
  epa_polytope_node *edge_node = &polytope->nodes[edge];
  for (count_t i = 0; i < 2; ++i) {
    if (edge_node->value.edge.attached_faces[i] == NIL) {
      edge_node->value.edge.attached_faces[i] = face;
      break;
    }
  }
}

static void polytope_detach_face(epa_polytope *polytope, uint16_t face, uint16_t edge) {
  epa_polytope_node *edge_node = &polytope->nodes[edge];

  uint16_t *attached_faces = edge_node->value.edge.attached_faces;
  for (count_t i = 0; i < 2; ++i) {
    if (attached_faces[i] == face) {
      attached_faces[i] = NIL;
    }
  }
}

static void polytope_detach_edge(epa_polytope *polytope, uint16_t edge, uint16_t vertex) {
  epa_polytope_node *vertex_node = &polytope->nodes[vertex];

  uint16_t next_edge = vertex_node->value.vertex.first_attached_edge;
  epa_polytope_node *next_edge_node = &polytope->nodes[next_edge];
  int i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
  if (next_edge == edge) {
    vertex_node->value.vertex.first_attached_edge = next_edge_node->value.edge.next_attached_edges[i];
    if (vertex_node->value.vertex.first_attached_edge == NIL) {
      polytope->flags[vertex] |= EPA_FLAG_FOR_REMOVAL;
    }
    return;
  }

  next_edge = next_edge_node->value.edge.next_attached_edges[i];

  while (next_edge != edge) {
    next_edge_node = &polytope->nodes[next_edge];
    i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;
    next_edge = next_edge_node->value.edge.next_attached_edges[i];
  }

  epa_polytope_node *current_node = next_edge_node;
  int current_i = i;

  next_edge_node = &polytope->nodes[next_edge];
  i = next_edge_node->value.edge.verticies[0] == vertex ? 0 : 1;

  current_node->value.edge.next_attached_edges[current_i] = next_edge_node->value.edge.next_attached_edges[i];
}

static void polytope_get_edge_verticies(const epa_polytope *polytope, uint16_t edge, bnd_v3 *v1, bnd_v3 *v2) {
  epa_polytope_node node = polytope->nodes[edge];
  uint16_t vertex_1 = node.value.edge.verticies[0];
  uint16_t vertex_2 = node.value.edge.verticies[1];

  *v1 = polytope->nodes[vertex_1].value.vertex.v.p;
  *v2 = polytope->nodes[vertex_2].value.vertex.v.p;
}

static void polytope_get_face_verticies(const epa_polytope *polytope, uint16_t face, bnd_v3 *v1, bnd_v3 *v2, bnd_v3 *v3) {
  epa_polytope_node node = polytope->nodes[face];
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

static uint16_t polytope_add_vertex(epa_polytope *polytope, body_support p) {
  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_VERTEX;
  node->value.vertex.v = p;
  node->value.vertex.first_attached_edge = NIL;

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_edge(epa_polytope *polytope, uint16_t v1, uint16_t v2) {
  if (v1 == NIL || v2 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_EDGE;
  node->value.edge.verticies[0] = v1;
  node->value.edge.verticies[1] = v2;

  bnd_v3 closest;
  node->distance = sqr_distance_to_line_segment(bnd_v3_zero(), polytope->nodes[v1].value.vertex.v.p, polytope->nodes[v2].value.vertex.v.p, &closest);
  node->normal = closest;

  memset(node->value.edge.attached_faces, 0, 2 * sizeof(uint16_t));
  memset(node->value.edge.next_attached_edges, 0, 2 * sizeof(uint16_t));

  polytope_attach_edge(polytope, index, v1);
  polytope_attach_edge(polytope, index, v2);

  polytope_add_node(polytope, node, index);

  return index;
}

static uint16_t polytope_add_face(epa_polytope *polytope, uint16_t e1, uint16_t e2, uint16_t e3) {
  if (e1 == NIL || e2 == NIL || e3 == NIL) {
    return NIL;
  }

  uint16_t index = polytope_free_index(polytope);
  if (index == NIL) {
    return NIL;
  }

  epa_polytope_node *node = &polytope->nodes[index];
  node->type = EPA_NODE_FACE;
  node->value.face.edges[0] = e1;
  node->value.face.edges[1] = e2;
  node->value.face.edges[2] = e3;

  bnd_v3 v1, v2, v3, p;
  polytope_get_face_verticies(polytope, index, &v1, &v2, &v3);

  if (bnd_v3_distancesqr(v1, v2) < EPSILON || bnd_v3_distancesqr(v2, v3) < EPSILON || bnd_v3_distancesqr(v3, v1) < EPSILON) {
    polytope->free_list[polytope->free_count++] = index;
    return NIL;
  }

  bnd_v3 normal = bnd_v3_cross(bnd_v3_sub(v3, v1), bnd_v3_sub(v2, v1));
  if (bnd_v3_dot(normal, v1) < 0) {
    normal = bnd_v3_negate(normal);
  }

  node->distance = sqr_distance_to_triangle(bnd_v3_zero(), v1, v2, v3, &p);
  node->normal = normal;

  polytope_attach_face(polytope, index, e1);
  polytope_attach_face(polytope, index, e2);
  polytope_attach_face(polytope, index, e3);

  polytope_add_node(polytope, node, index);

  return index;
}

static void polytope_remove_face(epa_polytope *polytope, uint16_t face) {
  if (face == NIL) {
    return;
  }

  epa_polytope_node *face_node = &polytope->nodes[face];
  uint16_t e1 = face_node->value.face.edges[0];
  uint16_t e2 = face_node->value.face.edges[1];
  uint16_t e3 = face_node->value.face.edges[2];

  polytope_detach_face(polytope, face, e1);
  polytope_detach_face(polytope, face, e2);
  polytope_detach_face(polytope, face, e3);

  polytope_remove_node(polytope, face);
}

static void polytope_remove_edge(epa_polytope *polytope, uint16_t edge) {
  if (edge == NIL) {
    return;
  }

  epa_polytope_node *edge_node = &polytope->nodes[edge];
  if (edge_node->value.edge.attached_faces[0] != NIL || edge_node->value.edge.attached_faces[1] != NIL) {
    return;
  }

  uint16_t v1 = edge_node->value.edge.verticies[0];
  uint16_t v2 = edge_node->value.edge.verticies[1];

  polytope_detach_edge(polytope, edge, v1);
  polytope_detach_edge(polytope, edge, v2);

  polytope_remove_node(polytope, edge);
}

static void polytope_remove_vertex(epa_polytope *polytope, uint16_t vertex) {
  if (vertex == NIL) {
    return;
  }

  polytope_remove_node(polytope, vertex);
}

static void polytope_clear_flags(epa_polytope *polytope) {
  memset(polytope->flags, 0, polytope->node_count + 1);
}

static void polytope_update_nearest(epa_polytope *polytope) {
  polytope->nearest_distance = FLT_MAX;

  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    epa_polytope_node node = polytope->nodes[index];

    float distance = node.distance;
    if (distance < polytope->nearest_distance) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    epa_polytope_node node = polytope->nodes[index];
    epa_polytope_node current_nearest = polytope->nodes[polytope->nearest];

    float distance = node.distance;
    if (distance < polytope->nearest_distance ||
        (distance == polytope->nearest_distance && current_nearest.type == EPA_NODE_FACE)) {
      polytope->nearest_distance = distance;
      polytope->nearest = index;
    }
  }
}

static bool polytope_is_face_visible(const epa_polytope_node *face, bnd_v3 support_point) {
  return bnd_v3_dot(bnd_v3_normalize(face->normal), bnd_v3_normalize(support_point)) > visibility_epsilon;
}

static bool polytope_contains_vertex(const epa_polytope *polytope, bnd_v3 point) {
  polytope_for_each_node(polytope, index, EPA_NODE_VERTEX) {
    if (bnd_v3_distancesqr(polytope->nodes[index].value.vertex.v.p, point) < EPSILON) {
      return true;
    }
  }

  return false;
}

static bool polytope_from_simplex(epa_polytope *polytope, const simplex *s) {
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

  contact->features.witness_a = p.p1.point;
  contact->features.witness_b = p.p2.point;
  contact->features.normal = contact->normal;
}

static void epa_calculate_contact(const epa_polytope *polytope, contact *contact) {
  epa_polytope_node node = polytope->nodes[polytope->nearest];
  bnd_v3 p1, p2;
  if (node.type == EPA_NODE_FACE) {
    uint16_t e1 = node.value.face.edges[0];
    uint16_t e2 = node.value.face.edges[1];
    uint16_t *edge_verts_1 = polytope->nodes[e1].value.edge.verticies;
    uint16_t *edge_verts_2 = polytope->nodes[e2].value.edge.verticies;

    body_support v0 = polytope->nodes[edge_verts_1[0]].value.vertex.v;
    body_support v1 = polytope->nodes[edge_verts_1[1]].value.vertex.v;

    body_support v2 = edge_verts_2[1] != edge_verts_1[1] && edge_verts_2[1] != edge_verts_1[0]
      ? polytope->nodes[edge_verts_2[1]].value.vertex.v
      : polytope->nodes[edge_verts_2[0]].value.vertex.v;

    bnd_v3 barycenter = bnd_v3_barycentric(bnd_v3_zero(), v0.p, v1.p, v2.p);
    p1 = bnd_v3_add(bnd_v3_scale(v0.p1.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v1.p1.point, barycenter.y), bnd_v3_scale(v2.p1.point, barycenter.z)));
    p2 = bnd_v3_add(bnd_v3_scale(v0.p2.point, barycenter.x), bnd_v3_add(bnd_v3_scale(v1.p2.point, barycenter.y), bnd_v3_scale(v2.p2.point, barycenter.z)));
  } else if (node.type == EPA_NODE_EDGE) {
    body_support v0 = polytope->nodes[node.value.edge.verticies[0]].value.vertex.v;
    body_support v1 = polytope->nodes[node.value.edge.verticies[1]].value.vertex.v;

    bnd_v3 d = bnd_v3_sub(v1.p, v0.p);
    float t = -1.0 * bnd_v3_dot(v0.p, d) / bnd_v3_lensqr(d);
    p1 = bnd_v3_add(v0.p1.point, bnd_v3_scale(bnd_v3_sub(v1.p1.point, v0.p1.point), t));
    p2 = bnd_v3_add(v0.p2.point, bnd_v3_scale(bnd_v3_sub(v1.p2.point, v0.p2.point), t));
  } else {
    return;
  }

  contact->point = bnd_v3_scale(bnd_v3_add(p1, p2), 0.5);
  contact->depth = sqrt(node.distance);

  float length = bnd_v3_len(node.normal);
  if (length > EPSILON) {
    contact->normal = bnd_v3_scale(node.normal, -1.0 / length);
  } else {
    contact->normal = bnd_v3_up();
  }

  contact->features.witness_a = p1;
  contact->features.witness_b = p2;
  contact->features.normal = contact->normal;
}

static void mark_edge_for_removal(epa_polytope *polytope, uint16_t edge_index, body_support p, uint16_t *stack, uint16_t *stack_ptr) {
  epa_polytope_node *edge_node = &polytope->nodes[edge_index];

  count_t visible_count = 0;
  for (count_t j = 0; j < 2; ++j) {
    uint16_t adjasent_face_index = edge_node->value.edge.attached_faces[j];
    const epa_polytope_node *adjasent_face_node = &polytope->nodes[adjasent_face_index];

    if (polytope->flags[adjasent_face_index] & EPA_FLAG_FOR_REMOVAL) {
      visible_count += 1;
    } else if (polytope_is_face_visible(adjasent_face_node, p.p)) {
      visible_count += 1;
      polytope->flags[adjasent_face_index] |= EPA_FLAG_FOR_REMOVAL;
      stack[*stack_ptr] = adjasent_face_index;
      *stack_ptr += 1;
    }
  }

  if (visible_count == 2) {
    polytope->flags[edge_index] |= EPA_FLAG_FOR_REMOVAL;
  } else if (visible_count == 1) {
    polytope->flags[edge_index] |= EPA_FLAG_BORDER_EDGE;
  }
}

static void epa_update_visible_nodes(epa_polytope *polytope, body_support p) {
  uint16_t stack_ptr = 1;
  uint16_t stack[VISIBLE_NODES_STACK_SIZE] = { polytope->nearest };

  epa_polytope_node *node = &polytope->nodes[polytope->nearest];
  polytope->flags[polytope->nearest] |= EPA_FLAG_FOR_REMOVAL;

  while (stack_ptr > 0) {
    uint16_t node_index = stack[--stack_ptr];
    node = &polytope->nodes[node_index];

    if (node->type == EPA_NODE_FACE) {
      for (count_t i = 0; i < 3; ++i) {
        uint16_t edge_index = node->value.face.edges[i];
        mark_edge_for_removal(polytope, edge_index, p, stack, &stack_ptr);
      }
    } else if (node->type == EPA_NODE_EDGE) {
      mark_edge_for_removal(polytope, node_index, p, stack, &stack_ptr);
    }
  }
}

static bool epa_expand_polytope(epa_polytope *polytope, body_support p) {
  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_face(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_edge(polytope, index);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_VERTEX) {
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      polytope_remove_vertex(polytope, index);
    }
  }

  uint16_t new_vertex = polytope_add_vertex(polytope, p);

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    epa_polytope_node *edge_node = &polytope->nodes[index];
    if ((polytope->flags[index] & EPA_FLAG_BORDER_EDGE) == 0) {
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

      epa_polytope_node *connected_vertex_node = &polytope->nodes[connected_vertex_index];
      uint16_t attached_edge_index = connected_vertex_node->value.vertex.first_attached_edge;
      while (attached_edge_index != NIL) {
        epa_polytope_node attached_edge_node = polytope->nodes[attached_edge_index];
        count_t i = attached_edge_node.value.edge.verticies[0] == connected_vertex_index ? 0 : 1;

        if (attached_edge_index == new_edge || attached_edge_index == edge_index) {
          attached_edge_index = attached_edge_node.value.edge.next_attached_edges[i];
          continue;
        }

        if (polytope->flags[attached_edge_index] & EPA_FLAG_BORDER_EDGE) {
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
  epa_polytope *polytope = &world->epa_polytope;

  memset(polytope, 0, sizeof(epa_polytope));

  count_t max_nodes = world->config.advanced.epa_max_nodes;
  ALLOC_BUFFER8(polytope->nodes, (max_nodes + 1) * sizeof(epa_polytope_node));
  ALLOC_BUFFER1(polytope->flags, polytope_flags_size(max_nodes));
  ALLOC_BUFFER2(polytope->free_list, max_nodes * sizeof(uint16_t));

  polytope->max_nodes = max_nodes;
  polytope->nearest_distance = FLT_MAX;

  return OK;
}

void epa_teardown(bnd_world *world) {
  epa_polytope *polytope = &world->epa_polytope;
  world->allocator.free(polytope->nodes, (polytope->max_nodes + 1) * sizeof(epa_polytope_node));
  world->allocator.free(polytope->flags, polytope_flags_size(polytope->max_nodes));
  world->allocator.free(polytope->free_list, polytope->max_nodes * sizeof(uint16_t));
}

static epa_status epa_run(epa_polytope *polytope, const collision_detection_context *ctx, body_support *support_point, float tolerance) {
  epa_polytope_node closest_node = polytope->nodes[polytope->nearest];
  bnd_v3 direction = closest_node.normal;
  bnd_v3 normal_direction = bnd_v3_normalize(direction);

  *support_point = support(ctx, normal_direction);

  float support_distance = bnd_v3_dot(normal_direction, support_point->p);
  float face_distance = sqrtf(closest_node.distance);
  if (support_distance - face_distance < tolerance) {
    return EPA_STATUS_CONVERGED;
  }

  float distance;
  bnd_v3 a, b, c, closest;
  if (closest_node.type == EPA_NODE_FACE) {
    polytope_get_face_verticies(polytope, polytope->nearest, &a, &b, &c);
    distance = sqr_distance_to_triangle(support_point->p, a, b, c, &closest);
  } else if (closest_node.type == EPA_NODE_EDGE) {
    polytope_get_edge_verticies(polytope, polytope->nearest, &a, &b);
    distance = sqr_distance_to_line_segment(support_point->p, a, b, &closest);
  } else {
    return EPA_STATUS_INVALID_POLYTOPE;
  }

  if (distance < tolerance) {
    return EPA_STATUS_CONVERGED;
  }

  // TODO don't check this every time, just when producing a zero-length edge.
  if (polytope_contains_vertex(polytope, support_point->p)) {
    return EPA_STATUS_CONVERGED;
  }

  epa_update_visible_nodes(polytope, *support_point);

  if (!epa_expand_polytope(polytope, *support_point)) {
    return EPA_STATUS_EXPANSION_FAILED;
  }

  return EPA_STATUS_OK;
}


count_t epa_get_contact(bnd_world *world, const collision_detection_context *ctx, const simplex *simplex, float tolerance, contact *contact) {
  PROFILER_FUNCTION_START

  epa_polytope *polytope = &world->epa_polytope;
  body_support support_point = simplex->points[0];
  if (!polytope_from_simplex(polytope, simplex)) {
    epa_invalid_contact(support_point, contact);
    PROFILER_FUNCTION_END
    return 0;
  }

  count_t attempts = 1;
  for (; attempts <= EPA_MAX_ATTEMPTS; ++attempts) {
    epa_status result = epa_run(polytope, ctx, &support_point, tolerance);
    switch(result) {
      case EPA_STATUS_OK:
        continue;

      case EPA_STATUS_CONVERGED:
        epa_calculate_contact(polytope, contact);
        PROFILER_FUNCTION_END
        return attempts;

      default:
        epa_invalid_contact(support_point, contact);
        PROFILER_FUNCTION_END
        return attempts;
    }
  }

  PROFILER_FUNCTION_END
  return attempts;
}

#if defined(BND_DEBUG)

static void epa_debug_render_iteration(const epa_polytope *polytope, body_support support_point, bnd_debug_draw_epa_callbacks callbacks, void *user_data) {
  polytope_for_each_node(polytope, index, EPA_NODE_FACE) {
    const epa_polytope_node *face = &polytope->nodes[index];
    const bnd_v3 normal = face->normal;

    bnd_v3 a, b, c;
    polytope_get_face_verticies(polytope, index, &a, &b, &c);

    bnd_v3 winding = bnd_v3_cross(bnd_v3_sub(b, a), bnd_v3_sub(c, a));
    if (bnd_v3_dot(winding, normal) < 0) {
      bnd_v3 tmp = b;
      b = c;
      c = tmp;
    }

    bnd_debug_epa_flags flags = DEBUG_EPA_NONE;
    if (index == polytope->nearest) {
      flags |= DEBUG_EPA_FACE_NEAREST;
    }
    if (polytope->flags[index] & EPA_FLAG_FOR_REMOVAL) {
      flags |= DEBUG_EPA_FACE_REMOVED;
    }

    if (callbacks.draw_face != NULL) {
      callbacks.draw_face(a, b, c, flags, user_data);
    }

    if (callbacks.draw_normal != NULL) {
      flags |= DEBUG_EPA_NORMAL_FACE;
      bnd_v3 center = bnd_v3_scale(bnd_v3_add(bnd_v3_add(a, b), c), 0.333);
      callbacks.draw_normal(center, bnd_v3_normalize(normal), flags, user_data);
    }
  }

  polytope_for_each_node(polytope, index, EPA_NODE_EDGE) {
    const epa_polytope_node *edge = &polytope->nodes[index];

    bnd_v3 a, b;
    polytope_get_edge_verticies(polytope, index, &a, &b);

    bnd_debug_epa_flags flags = DEBUG_EPA_NORMAL_EDGE;
    if (index == polytope->nearest) {
      flags |= DEBUG_EPA_NORMAL_NEAREST;
    }

    if (callbacks.draw_normal != NULL) {
      bnd_v3 center = bnd_v3_scale(bnd_v3_add(a, b), 0.5);
      callbacks.draw_normal(center, bnd_v3_normalize(edge->normal), flags, user_data);
    }
  }

  if (callbacks.draw_support != NULL) {
    callbacks.draw_support(support_point.p, user_data);
  }
}

bool epa_debug_draw(bnd_world *world, const epa_debug_status *debug_status, bnd_debug_draw_epa_callbacks callbacks, void *user_data) {
  epa_polytope *polytope = &world->epa_polytope;
  if (!polytope_from_simplex(polytope, &debug_status->s)) {
    return false;
  }

  body_support support_point = debug_status->s.points[0];
  if (debug_status->target_iteration == 0) {
    epa_debug_render_iteration(polytope, support_point, callbacks, user_data);
    return true;
  }

  epa_status status = EPA_STATUS_OK;
  for (int iteration = 1; iteration < EPA_MAX_ATTEMPTS; ++iteration) {
    if (iteration > debug_status->target_iteration) {
      return false;
    }

    status = epa_run(polytope, &debug_status->ctx, &support_point, world->config.advanced.epa_tolerance);
    if (status != EPA_STATUS_OK && status != EPA_STATUS_CONVERGED) {
      return false;
    }

    if (iteration == debug_status->target_iteration) {
      bnd_v3 direction = bnd_v3_normalize(polytope->nodes[polytope->nearest].normal);
      body_support next_support = support(&debug_status->ctx, direction);

      epa_debug_render_iteration(polytope, next_support, callbacks, user_data);
      return true;
    }
  }

  return false;
}

#endif
