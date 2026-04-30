#include "bnd-core.h"
#include <stdlib.h>
#include <string.h>

#define DEFAULT_VERTEX_PER_MESH 512
#define DEFAULT_FACE_PER_MESH 256

static bool is_mesh_convex(const bnd_mesh_data *data) { return true; }

static void resize_buffer(void **buffer, count_t count, count_t *capacity, count_t element_size) {
  if (count <= *capacity) {
    return;
  }

  while (count > *capacity) {
    *capacity *= 2;
  }

  *buffer = realloc(*buffer, *capacity * element_size);
}

static void import_buffer(const bnd_mesh_buffer *buffer, void **target, count_t *target_count, count_t *target_capacity, count_t target_size) {
  count_t new_count = *target_count + buffer->elemenets_count;
  resize_buffer(target, new_count, target_capacity, buffer->element_size);

  count_t target_stride = 0;
  if (buffer->element_size > target_size) {
    target_stride = buffer->element_size - target_size;
  }

  uint8_t *to = (uint8_t *)*target;
  to += *target_count * buffer->element_size;

  if (buffer->stride == 0 && target_stride == 0) {
    memcpy(to, buffer->buffer, buffer->elemenets_count * buffer->element_size);
  } else {
    uint8_t *from = (uint8_t *)buffer->buffer;
    for (count_t i = 0; i < buffer->elemenets_count; ++i) {
      memcpy(to, from, buffer->element_size);

      from += buffer->element_size + buffer->stride;
      to += buffer->element_size + target_stride;
    }
  }

  *target_count += buffer->elemenets_count;
}

void meshes_init(bnd_world *world) {
  count_t num_meshes = world->config.memory.meshes_capacity;

  mesh_storage *meshes = &world->meshes;
  meshes->submeshes = malloc(num_meshes * sizeof(submesh));
  meshes->submesh_capacity = num_meshes;
  meshes->submesh_count = 0;

  meshes->verticies = malloc(num_meshes * DEFAULT_VERTEX_PER_MESH * sizeof(v3));
  meshes->vertex_capacity = num_meshes * DEFAULT_VERTEX_PER_MESH;
  meshes->vertex_count = 0;

  meshes->indicies = malloc(num_meshes * DEFAULT_FACE_PER_MESH * 3 * sizeof(uint32_t));
  meshes->index_capacity = num_meshes * DEFAULT_FACE_PER_MESH * 3;
  meshes->index_count = 0;
}

bnd_mesh_handle bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data) {
  if (!is_mesh_convex(data)) {
    raise_error(BND_ERROR_MESH_IS_CONCAVE, NULL, "Concave meshes are not properly supported at the moment");
  }

  mesh_storage *meshes = &world->meshes;

  submesh sm;
  sm.vertex_offset = meshes->vertex_count;
  sm.index_offset = meshes->index_count;
  sm.vertex_count = data->vertex_buffer.elemenets_count;
  sm.index_count = data->index_buffer.elemenets_count;

  import_buffer(&data->vertex_buffer, (void **)&meshes->verticies, &meshes->vertex_count, &meshes->vertex_capacity, sizeof(v3));
  import_buffer(&data->index_buffer, (void **)&meshes->indicies, &meshes->index_count, &meshes->index_capacity, sizeof(uint32_t));

  resize_buffer((void **)&meshes->submeshes, meshes->submesh_count + 1, &meshes->submesh_capacity, sizeof(submesh));

  count_t submesh_offset = meshes->submesh_count;
  meshes->submeshes[meshes->submesh_count++] = sm;

  return (bnd_mesh_handle){submesh_offset, 1};
}
