#include "bandura.h"
#include "bnd-core.h"
#include "bnd-math.h"

#include <float.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static inline bnd_error mesh_validation_error(char *message) {
  return (bnd_error) { BND_ERROR_INVALID_MESH, message };
}

static inline bnd_error realloc_error() {
  return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Allocator.realloc failed to re-allocate mesh buffer" };
}

static bnd_error resize_buffer(bnd_allocator allocator, void **buffer, count_t alignment, count_t count, count_t *capacity, count_t element_size) {
  count_t old_capacity = *capacity;
  if (count <= old_capacity) {
    return OK;
  }

  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Mesh buffer is full and Allocator.realloc is NULL" };
  }

  count_t new_capacity = old_capacity;
  while (count > new_capacity) {
    new_capacity *= 2;
  }

  void *new_buffer = allocator.realloc(*buffer, alignment, old_capacity * element_size, new_capacity * element_size);
  if (new_buffer == NULL) {
    return realloc_error();
  }

  *buffer = new_buffer;
  *capacity = new_capacity;

  return OK;
}

static bnd_error ensure_meshes_capacity(bnd_allocator allocator, mesh_storage *meshes) {
  if (meshes->mesh_count + 1 < meshes->mesh_capacity) {
    return OK;
  }

  if (allocator.realloc == NULL) {
    return (bnd_error) { BND_ERROR_NO_SPACE_AVAILABLE, "Mesh buffer is full and Allocator.realloc is NULL" };
  }

  count_t old_capacity = meshes->mesh_capacity;
  while (meshes->mesh_count + 1 > meshes->mesh_capacity) {
    meshes->mesh_capacity *= 2;
  }

  REALLOC_BUFFER4(meshes->meshes, allocator, sizeof(bnd_mesh), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->inertias, allocator, sizeof(bnd_m3), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->volumes, allocator, sizeof(float), old_capacity, meshes->mesh_capacity);
  REALLOC_BUFFER4(meshes->aabbs, allocator, sizeof(bnd_aabb), old_capacity, meshes->mesh_capacity);

  return OK;
}

static bnd_v3 read_vertex(const bnd_mesh_buffer *buffer, count_t i) {
  bnd_v3 vertex = bnd_v3_zero();
  uint8_t *verticies = buffer->buffer;
  uint32_t step = buffer->element_size + buffer->stride;

  memcpy(&vertex, &verticies[i * step], buffer->element_size);

  return vertex;
}

static uint32_t read_index(const bnd_mesh_buffer *buffer, count_t i) {
  uint8_t *indices = buffer->buffer;
  uint8_t *src = &indices[i * (buffer->element_size + buffer->stride)];

  switch (buffer->element_size) {
    case 1:
      return src[0];

    case 2: {
      uint16_t index;
      memcpy(&index, src, sizeof(index));
      return index;
    }

    case 4: {
      uint32_t index;
      memcpy(&index, src, sizeof(index));
      return index;
    }

    default:
      return 0;
  }
}

static bnd_error import_verticies(bnd_allocator allocator, const bnd_mesh_buffer *buffer, mesh_storage *meshes, bnd_v3 com) {
  count_t new_count = buffer->elements_count + meshes->vertex_count;
  bnd_error err = resize_buffer(allocator, (void **)&meshes->verticies, 4, new_count, &meshes->vertex_capacity, sizeof(bnd_v3));
  if (err.type != BND_OK) {
    return err;
  }

  bnd_v3 *dest = &meshes->verticies[meshes->vertex_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    bnd_v3 v = read_vertex(buffer, i);
    dest[i] = bnd_v3_sub(v, com);
  }

  meshes->vertex_count = new_count;

  return OK;
}

static bnd_error import_indicies(bnd_allocator allocator, const bnd_mesh_buffer *buffer, mesh_storage *meshes) {
  count_t new_count = meshes->index_count + buffer->elements_count;
  bnd_error err = resize_buffer(allocator, (void **)&meshes->indicies, 4, new_count, &meshes->index_capacity, sizeof(uint32_t));
  if (err.type != BND_OK) {
    return err;
  }

  uint32_t *dest = &meshes->indicies[meshes->index_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    uint32_t index = read_index(buffer, i);
    dest[i] = index + meshes->index_count;
  }

  meshes->index_count = new_count;
  return OK;
}

static float tetr_inertia_moment(bnd_m3 m, count_t i) {
  return m.m0[i] * m.m0[i] + m.m1[i] * m.m2[i] + m.m1[i] * m.m1[i] + m.m0[i] * m.m2[i] + m.m2[i] * m.m2[i] + m.m0[i] * m.m1[i];
}

static float tetr_inertia_product(bnd_m3 m, count_t i, count_t j) {
  return 2.0 * m.m0[i] * m.m0[j] + m.m1[i] * m.m2[j] + m.m2[i] * m.m1[j] +
    2.0 * m.m1[i] * m.m1[j] + m.m0[i] * m.m2[j] + m.m2[i] * m.m0[j] +
    2.0 * m.m2[i] * m.m2[j] + m.m0[i] * m.m1[j] + m.m1[i] * m.m0[j];
}

static bool is_mesh_convex(const bnd_mesh_data *data) {
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&data->index_buffer, i + 0);
    count_t i1 = read_index(&data->index_buffer, i + 1);
    count_t i2 = read_index(&data->index_buffer, i + 2);

    bnd_v3 v0 = read_vertex(&data->vertex_buffer, i0);
    bnd_v3 v1 = read_vertex(&data->vertex_buffer, i1);
    bnd_v3 v2 = read_vertex(&data->vertex_buffer, i2);

    bnd_v3 n = bnd_v3_cross(bnd_v3_sub(v2, v0), bnd_v3_sub(v1, v0));
    float d = -bnd_v3_dot(n, v2);

    bool has_sign = false;
    float s = 0;
    for (count_t j = 0; j < data->vertex_buffer.elements_count; ++j) {
      if (j == i0 || j == i1 || j == i2) {
        continue;
      }

      bnd_v3 v = read_vertex(&data->vertex_buffer, j);
      float sv = bnd_v3_dot(n, v) + d;
      if (fabsf(sv) < EPSILON) {
        continue;
      }

      if (!has_sign) {
        s = sv;
        has_sign = true;
      } else if ((sv < 0 && s > 0) || (sv > 0 && s < 0)) {
        return false;
      }
    }
  }

  return true;
}

static bnd_error validate_mesh(const bnd_mesh_data *data) {
  bnd_mesh_buffer index_buffer = data->index_buffer;
  if (index_buffer.buffer == NULL) {
    return mesh_validation_error("Mesh index buffer is NULL");
  }

  if (index_buffer.element_size != 1 && index_buffer.element_size != 2 && index_buffer.element_size != 4) {
    return mesh_validation_error("Unsupported index buffer element size. Supported sizes are 1, 2 or 4 bytes");
  }

  if (index_buffer.elements_count < 12) {
    return mesh_validation_error("Mesh index buffer must contain at least 12 elements");
  }

  if (index_buffer.elements_count % 3 != 0) {
    return mesh_validation_error("Mesh contains indicies count non-divisible by 3");
  }

  bnd_mesh_buffer vertex_buffer = data->vertex_buffer;
  if (vertex_buffer.buffer == NULL) {
    return mesh_validation_error("Mesh vertex buffer is NULL");
  }

  if (vertex_buffer.elements_count < 4) {
    return mesh_validation_error("Mesh vertex buffer must contain at least 4 elements");
  }

  if (vertex_buffer.element_size != 3 * sizeof(float)) {
    return mesh_validation_error("Vertex buffer is required to have elements composed of 3 floats");
  }

  count_t vertex_count = vertex_buffer.elements_count;
  for (count_t i = 0; i + 2 < index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&index_buffer, i + 0);
    count_t i1 = read_index(&index_buffer, i + 1);
    count_t i2 = read_index(&index_buffer, i + 2);

    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count) {
      return mesh_validation_error("One of mesh's indices is out of bounds");
    }

    // TODO properly check for degenerate faces and perhaps fix them.
  }

  return OK;
}

static void calculate_mass_properties(const bnd_mesh_data *data, bnd_m3 *inertia, bnd_v3 *com, float *volume) {
  /**
   * This function is a rewrite of SkComputeInertia3x3 from
   *
   * https://github.com/blackedout01/simkn
   */

  float ia = 0, ib = 0, ic = 0, iap = 0, ibp = 0, icp = 0;

  *volume = 0;
  *com = bnd_v3_zero();
  *inertia = (bnd_m3){ 0 };
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    bnd_v3 v0 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 0));
    bnd_v3 v1 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 1));
    bnd_v3 v2 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 2));

    bnd_m3 m = { { v0.x, v0.y, v0.z }, { v1.x, v1.y, v1.z }, { v2.x, v2.y, v2.z } };

    float det = bnd_v3_dot(v0, bnd_v3_cross(v1, v2));
    float tetr_volume = det / 6.0;

    bnd_v3 tetr_com = v0;
    tetr_com = bnd_v3_add(tetr_com, v1);
    tetr_com = bnd_v3_add(tetr_com, v2);
    tetr_com = bnd_v3_scale(tetr_com, 0.25);

    float v100 = tetr_inertia_moment(m, 0);
    float v010 = tetr_inertia_moment(m, 1);
    float v001 = tetr_inertia_moment(m, 2);

    ia += det * (v010 + v001);
    ib += det * (v100 + v001);
    ic += det * (v100 + v010);
    iap += det * tetr_inertia_product(m, 1, 2);
    ibp += det * tetr_inertia_product(m, 0, 1);
    icp += det * tetr_inertia_product(m, 0, 2);

    tetr_com = bnd_v3_scale(tetr_com, tetr_volume);
    *com = bnd_v3_add(*com, tetr_com);
    *volume += tetr_volume;
  }

  *com = bnd_v3_scale(*com, 1.0 / *volume);
  ia = ia / 60.0 - *volume * (com->y * com->y + com->z * com->z);
  ib = ib / 60.0 - *volume * (com->x * com->x + com->z * com->z);
  ic = ic / 60.0 - *volume * (com->x * com->x + com->y * com->y);
  iap = iap / 120.0 - *volume * (com->y * com->z);
  ibp = ibp / 120.0 - *volume * (com->x * com->y);
  icp = icp / 120.0 - *volume * (com->x * com->z);

  inertia->m0[0] = ia;
  inertia->m1[1] = ib;
  inertia->m2[2] = ic;
  inertia->m0[1] = inertia->m1[0] = -ibp;
  inertia->m0[2] = inertia->m2[0] = -icp;
  inertia->m1[2] = inertia->m2[1] = -iap;
}

static bnd_aabb calculate_aabb(const mesh_storage *meshes, submesh submesh) {
  count_t vertex_start = submesh.vertex_offset;
  count_t vertex_end = vertex_start + submesh.vertex_count;

  bnd_v3 min = (bnd_v3){FLT_MAX, FLT_MAX, FLT_MAX};
  bnd_v3 max = bnd_v3_negate(min);
  for (count_t i = vertex_start; i < vertex_end; ++i) {
    bnd_v3 v = meshes->verticies[i];

    min = bnd_v3_min(min, v);
    max = bnd_v3_max(max, v);
  }

  return (bnd_aabb) {
    .center = bnd_v3_scale(bnd_v3_add(min, max), 0.5),
    .half_extents = bnd_v3_scale(bnd_v3_sub(max, min), 0.5),
  };
}

bnd_error meshes_init(bnd_world *world) {
  mesh_storage *meshes = &world->meshes;
  count_t num_meshes = world->config.memory.meshes_capacity;
  bnd_allocator allocator = world->allocator;

  ALLOC_BUFFER4(meshes->submeshes, num_meshes * sizeof(submesh));
  meshes->submesh_capacity = num_meshes;
  meshes->submesh_count = 0;

  ALLOC_BUFFER4(meshes->meshes, num_meshes * sizeof(bnd_mesh));
  meshes->mesh_capacity = num_meshes;
  meshes->mesh_count = 0;

  ALLOC_BUFFER4(meshes->verticies, num_meshes * DEFAULT_VERTEX_PER_MESH * sizeof(bnd_v3));
  meshes->vertex_capacity = num_meshes * DEFAULT_VERTEX_PER_MESH;
  meshes->vertex_count = 0;

  ALLOC_BUFFER4(meshes->indicies, num_meshes * DEFAULT_FACE_PER_MESH * 3 * sizeof(uint32_t));
  meshes->index_capacity = num_meshes * DEFAULT_FACE_PER_MESH * 3;
  meshes->index_count = 0;

  ALLOC_BUFFER4(meshes->inertias, num_meshes * sizeof(bnd_m3));
  ALLOC_BUFFER4(meshes->volumes, num_meshes * sizeof(float));
  ALLOC_BUFFER4(meshes->aabbs, num_meshes * sizeof(bnd_aabb));

  return OK;
}

void meshes_teardown(bnd_world *world) {
  mesh_storage meshes = world->meshes;

  world->allocator.free(meshes.submeshes, meshes.submesh_capacity * sizeof(submesh));
  world->allocator.free(meshes.meshes, meshes.mesh_capacity * sizeof(bnd_mesh));
  world->allocator.free(meshes.verticies, meshes.vertex_capacity * sizeof(bnd_v3));
  world->allocator.free(meshes.indicies, meshes.index_capacity * sizeof(uint32_t));
  world->allocator.free(meshes.inertias, meshes.mesh_capacity * sizeof(bnd_m3));
  world->allocator.free(meshes.volumes, meshes.mesh_capacity * sizeof(float));
  world->allocator.free(meshes.aabbs, meshes.mesh_capacity * sizeof(bnd_aabb));
}

bnd_error bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass) {
  bnd_error e = validate_mesh(data);
  if (e.type != BND_OK) {
    return e;
  }

  if (!is_mesh_convex(data)) {
    e.type = BND_ERROR_MESH_IS_CONCAVE;
    e.message = "Concave meshes are not properly supported at the moment";

    return e;
  }

  bnd_m3 inertia;
  float volume;
  calculate_mass_properties(data, &inertia, center_of_mass, &volume);

  mesh_storage *meshes = &world->meshes;

  submesh sm;
  sm.vertex_offset = meshes->vertex_count;
  sm.index_offset = meshes->index_count;
  sm.vertex_count = data->vertex_buffer.elements_count;
  sm.index_count = data->index_buffer.elements_count;

  bnd_allocator allocator = world->allocator;
  e = import_verticies(allocator, &data->vertex_buffer, meshes, *center_of_mass);
  if (e.type != BND_OK) {
    return e;
  }

  e = import_indicies(allocator, &data->index_buffer, meshes);
  if (e.type != BND_OK) {
    return e;
  }

  e = resize_buffer(allocator, (void **)&meshes->submeshes, 4, meshes->submesh_count + 1, &meshes->submesh_capacity, sizeof(submesh));
  if (e.type != BND_OK) {
    return e;
  }

  e = ensure_meshes_capacity(allocator, meshes);
  if (e.type != BND_OK) {
    return e;
  }

  count_t submesh_offset = meshes->submesh_count++;
  meshes->submeshes[submesh_offset] = sm;

  *handle = meshes->mesh_count++;
  meshes->meshes[*handle] = (bnd_mesh){ submesh_offset, 1 };
  meshes->inertias[*handle] = inertia;
  meshes->volumes[*handle] = volume;
  meshes->aabbs[*handle] = calculate_aabb(meshes, sm);

  return OK;
}
