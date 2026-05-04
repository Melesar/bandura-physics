#include "bandura.h"
#include "bnd-core.h"
#include <stdlib.h>
#include <string.h>

#define DEFAULT_VERTEX_PER_MESH 512
#define DEFAULT_FACE_PER_MESH 256

static void resize_buffer(void **buffer, count_t count, count_t *capacity, count_t element_size) {
  if (count <= *capacity) {
    return;
  }

  while (count > *capacity) {
    *capacity *= 2;
  }

  *buffer = realloc(*buffer, *capacity * element_size);
}

static void ensure_meshes_capacity(mesh_storage *meshes) {
  if (meshes->mesh_count + 1 < meshes->mesh_capacity) {
    return;
  }

  while (meshes->mesh_count + 1 > meshes->mesh_capacity) {
    meshes->mesh_capacity *= 2;
  }

  meshes->meshes = realloc(meshes->meshes, meshes->mesh_capacity * sizeof(bnd_mesh));
  meshes->inertias = realloc(meshes->inertias, meshes->mesh_capacity * sizeof(m3));
  meshes->volumes = realloc(meshes->volumes, meshes->mesh_capacity * sizeof(float));
}


static v3 read_vertex(const bnd_mesh_buffer *buffer, count_t i) {
  v3 vertex = zero();
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

static void import_verticies(const bnd_mesh_buffer *buffer, mesh_storage *meshes, v3 com) {
  count_t new_count = buffer->elements_count + meshes->vertex_count;
  resize_buffer((void **)&meshes->verticies, new_count, &meshes->vertex_capacity, sizeof(v3));

  v3 *dest = &meshes->verticies[meshes->vertex_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    v3 v = read_vertex(buffer, i);
    dest[i] = sub(v, com);
  }

  meshes->vertex_count = new_count;
}

static void import_indicies(const bnd_mesh_buffer *buffer, mesh_storage *meshes) {
  count_t new_count = meshes->index_count + buffer->elements_count;
  resize_buffer((void **)&meshes->indicies, new_count, &meshes->index_capacity, sizeof(uint32_t));

  uint32_t *dest = &meshes->indicies[meshes->index_count];
  for (count_t i = 0; i < buffer->elements_count; ++i) {
    uint32_t index = read_index(buffer, i);
    dest[i] = index + meshes->index_count;
  }

  meshes->index_count = new_count;
}

static float tetr_inertia_moment(m3 m, count_t i) {
  return m.m0[i] * m.m0[i] + m.m1[i] * m.m2[i] + m.m1[i] * m.m1[i] + m.m0[i] * m.m2[i] + m.m2[i] * m.m2[i] +
         m.m0[i] * m.m1[i];
}

static float tetr_inertia_product(m3 m, count_t i, count_t j) {
  return 2.0 * m.m0[i] * m.m0[j] + m.m1[i] * m.m2[j] + m.m2[i] * m.m1[j] + 2.0 * m.m1[i] * m.m1[j] + m.m0[i] * m.m2[j] +
         m.m2[i] * m.m0[j] + 2.0 * m.m2[i] * m.m2[j] + m.m0[i] * m.m1[j] + m.m1[i] * m.m0[j];
}

static bool is_mesh_convex(const bnd_mesh_data *data) {
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&data->index_buffer, i + 0);
    count_t i1 = read_index(&data->index_buffer, i + 1);
    count_t i2 = read_index(&data->index_buffer, i + 2);

    v3 v0 = read_vertex(&data->vertex_buffer, i0);
    v3 v1 = read_vertex(&data->vertex_buffer, i1);
    v3 v2 = read_vertex(&data->vertex_buffer, i2);

    v3 n = cross(sub(v2, v0), sub(v1, v0));
    float d = -dot(n, v2);

    bool has_sign = false;
    float s = 0;
    for (count_t j = 0; j < data->vertex_buffer.elements_count; ++j) {
      if (j == i0 || j == i1 || j == i2) {
        continue;
      }

      v3 v = read_vertex(&data->vertex_buffer, j);
      float sv = dot(n, v) + d;
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

static bool validate_mesh(const bnd_mesh_data *data) {
  bnd_mesh_buffer index_buffer = data->index_buffer;
  if (index_buffer.buffer == NULL) {
    raise_error(BND_ERROR_MESH_INVALID, (void *) data, "Mesh index buffer is NULL");
    return false;
  }

  if (index_buffer.element_size != 1 && index_buffer.element_size != 2 && index_buffer.element_size != 4) {
    raise_error(BND_ERROR_MESH_INVALID, (void *) data, "Unsupported index buffer element size. Supported sizes are 1, 2 or 4 bytes");
    return false;
  }

  if (index_buffer.elements_count == 0) {
    raise_error(BND_ERROR_MESH_INVALID, (void *) data, "Mesh index buffer is empty");
    return false;
  }

  if (index_buffer.elements_count % 3 != 0) {
    raise_error(BND_ERROR_MESH_INVALID, (void *)data, "Mesh contains %u indicies (non-divisible by 3)", index_buffer.elements_count);
    return false;
  }

  bnd_mesh_buffer vertex_buffer = data->vertex_buffer;
  if (vertex_buffer.element_size != 3 * sizeof(float)) {
    raise_error(BND_ERROR_MESH_INVALID, (void *) data, "Vertex buffer is required to have elements composed of 3 floats. Current element size: %u", vertex_buffer.element_size);
    return false;
  }

  count_t vertex_count = vertex_buffer.elements_count;
  for (count_t i = 0; i + 2 < index_buffer.elements_count; i += 3) {
    count_t i0 = read_index(&index_buffer, i + 0);
    count_t i1 = read_index(&index_buffer, i + 1);
    count_t i2 = read_index(&index_buffer, i + 2);

    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count) {
      raise_error(BND_ERROR_MESH_INVALID, (void *) data, "Face #%u contains index which is out of bounds: (%u, %u, %u) while vertex count is %u", i / 3, i0, i1, i2, vertex_count);
      return false;
    }

    // TODO properly check for degenerate faces and perhaps fix them.
  }

  return true;
}

static void calculate_mass_properties(const bnd_mesh_data *data, m3 *inertia, v3 *com, float *volume) {
  /**
   * This function is a rewrite of SkComputeInertia3x3 from
   *
   * https://github.com/blackedout01/simkn
   */

  float ia = 0, ib = 0, ic = 0, iap = 0, ibp = 0, icp = 0;

  *volume = 0;
  *com = zero();
  *inertia = (m3){ 0 };
  for (count_t i = 0; i + 2 < data->index_buffer.elements_count; i += 3) {
    v3 v0 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 0));
    v3 v1 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 1));
    v3 v2 = read_vertex(&data->vertex_buffer, read_index(&data->index_buffer, i + 2));

    m3 m = { { v0.x, v0.y, v0.z }, { v1.x, v1.y, v1.z }, { v2.x, v2.y, v2.z } };

    float det = dot(v0, cross(v1, v2));
    float tetr_volume = det / 6.0;

    v3 tetr_com = v0;
    tetr_com = add(tetr_com, v1);
    tetr_com = add(tetr_com, v2);
    tetr_com = scale(tetr_com, 0.25);

    float v100 = tetr_inertia_moment(m, 0);
    float v010 = tetr_inertia_moment(m, 1);
    float v001 = tetr_inertia_moment(m, 2);

    ia += det * (v010 + v001);
    ib += det * (v100 + v001);
    ic += det * (v100 + v010);
    iap += det * tetr_inertia_product(m, 1, 2);
    ibp += det * tetr_inertia_product(m, 0, 1);
    icp += det * tetr_inertia_product(m, 0, 2);

    tetr_com = scale(tetr_com, tetr_volume);
    *com = add(*com, tetr_com);
    *volume += tetr_volume;
  }

  *com = scale(*com, 1.0 / *volume);
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

void meshes_init(bnd_world *world) {
  count_t num_meshes = world->config.memory.meshes_capacity;

  mesh_storage *meshes = &world->meshes;
  meshes->submeshes = malloc(num_meshes * sizeof(submesh));
  meshes->submesh_capacity = num_meshes;
  meshes->submesh_count = 0;

  meshes->meshes = malloc(num_meshes * sizeof(bnd_mesh));
  meshes->mesh_capacity = num_meshes;
  meshes->mesh_count = 0;

  meshes->verticies = malloc(num_meshes * DEFAULT_VERTEX_PER_MESH * sizeof(v3));
  meshes->vertex_capacity = num_meshes * DEFAULT_VERTEX_PER_MESH;
  meshes->vertex_count = 0;

  meshes->indicies = malloc(num_meshes * DEFAULT_FACE_PER_MESH * 3 * sizeof(uint32_t));
  meshes->index_capacity = num_meshes * DEFAULT_FACE_PER_MESH * 3;
  meshes->index_count = 0;

  meshes->inertias = malloc(num_meshes * sizeof(m3));
  meshes->volumes = malloc(num_meshes * sizeof(float));
}

void meshes_teardown(bnd_world *world) {
  mesh_storage meshes = world->meshes;

  free(meshes.submeshes);
  free(meshes.meshes);
  free(meshes.verticies);
  free(meshes.indicies);
  free(meshes.inertias);
  free(meshes.volumes);
}

bool bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, v3 *center_of_mass) {
  if (!validate_mesh(data)) {
    return false;
  }

  if (!is_mesh_convex(data)) {
    raise_error(BND_ERROR_MESH_IS_CONCAVE, (void *) data, "Concave meshes are not properly supported at the moment");
    return false;
  }

  m3 inertia;
  float volume;
  calculate_mass_properties(data, &inertia, center_of_mass, &volume);

  mesh_storage *meshes = &world->meshes;

  submesh sm;
  sm.vertex_offset = meshes->vertex_count;
  sm.index_offset = meshes->index_count;
  sm.vertex_count = data->vertex_buffer.elements_count;
  sm.index_count = data->index_buffer.elements_count;

  import_verticies(&data->vertex_buffer, meshes, *center_of_mass);
  import_indicies(&data->index_buffer, meshes);

  resize_buffer((void **)&meshes->submeshes, meshes->submesh_count + 1, &meshes->submesh_capacity, sizeof(submesh));
  ensure_meshes_capacity(meshes);

  count_t submesh_offset = meshes->submesh_count++;
  meshes->submeshes[submesh_offset] = sm;

  *handle = meshes->mesh_count++;
  meshes->meshes[*handle] = (bnd_mesh){ submesh_offset, 1 };
  meshes->inertias[*handle] = inertia;
  meshes->volumes[*handle] = volume;

  return true;
}
