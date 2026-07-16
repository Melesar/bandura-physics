#include "bnd-core.h"
#include "bnd-math.h"

#include <string.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>

typedef struct {
  float x;
  float y;
} contact_filter_point;

static float cross_2d(contact_filter_point a, contact_filter_point b, contact_filter_point c) {
  contact_filter_point ab = { b.x - a.x, b.y - a.y };
  contact_filter_point ac = { c.x - a.x, c.y - a.y };
  return ab.x * ac.y - ab.y * ac.x;
}

static void sort_points(contact_filter_point *points) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    contact_filter_point value = points[i];
    count_t j = i;
    while (j > 0) {
      contact_filter_point previous = points[j - 1];
      if (previous.x < value.x || (previous.x == value.x && previous.y <= value.y)) {
        break;
      }

      points[j] = previous;
      --j;
    }
    points[j] = value;
  }
}

static float contact_set_area(contact *contacts, const count_t *indices, bnd_v3 origin, bnd_v3 tangent_x, bnd_v3 tangent_y) {
  contact_filter_point points[MAX_CONTACTS_PER_PAIR];
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    bnd_v3 offset = bnd_v3_sub(contacts[indices[i]].point, origin);
    points[i] = (contact_filter_point){
      .x = bnd_v3_dot(offset, tangent_x),
      .y = bnd_v3_dot(offset, tangent_y),
    };
  }

  sort_points(points);

  contact_filter_point hull[MAX_CONTACTS_PER_PAIR * 2];
  count_t hull_count = 0;

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    while (hull_count >= 2 && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  count_t lower_count = hull_count;
  for (count_t i = MAX_CONTACTS_PER_PAIR - 1; i < MAX_CONTACTS_PER_PAIR; --i) {
    while (hull_count > lower_count && cross_2d(hull[hull_count - 2], hull[hull_count - 1], points[i]) <= EPSILON) {
      --hull_count;
    }
    hull[hull_count++] = points[i];
  }

  if (hull_count <= 3) {
    return 0;
  }

  --hull_count;

  float area = 0;
  for (count_t i = 0; i < hull_count; ++i) {
    contact_filter_point a = hull[i];
    contact_filter_point b = hull[(i + 1) % hull_count];
    area += a.x * b.y - a.y * b.x;
  }

  return fabsf(area) * 0.5f;
}

static float contact_set_depth(contact *contacts, const count_t *indices) {
  float depth = 0;
  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    depth += contacts[indices[i]].depth;
  }

  return depth;
}

static bool better_contact_set(float area, float depth, float best_area, float best_depth) {
  if (area > best_area + EPSILON) {
    return true;
  }

  if (fabsf(area - best_area) <= EPSILON && depth > best_depth + EPSILON) {
    return true;
  }

  return false;
}

static void sort_indices(count_t *indices) {
  for (count_t i = 1; i < MAX_CONTACTS_PER_PAIR; ++i) {
    count_t value = indices[i];
    count_t j = i;
    while (j > 0 && indices[j - 1] > value) {
      indices[j] = indices[j - 1];
      --j;
    }
    indices[j] = value;
  }
}

void contacts_filter_largest_surface_area(contact *contacts, count_t contact_count, count_t *selected_indices) {
  count_t deepest = 0;
  for (count_t i = 1; i < contact_count; ++i) {
    if (contacts[i].depth > contacts[deepest].depth) {
      deepest = i;
    }
  }

  bnd_v3 normal = contacts[deepest].normal;
  bnd_v3 tangent_seed = fabsf(normal.y) < 0.70710678f ? bnd_v3_up() : bnd_v3_right();
  bnd_v3 tangent_x = bnd_v3_cross(tangent_seed, normal);
  if (bnd_v3_lensqr(tangent_x) <= EPSILON * EPSILON) {
    tangent_x = bnd_v3_cross(bnd_v3_forward(), normal);
  }
  tangent_x = bnd_v3_normalize(tangent_x);
  bnd_v3 tangent_y = bnd_v3_normalize(bnd_v3_cross(normal, tangent_x));
  bnd_v3 origin = contacts[deepest].point;

  float best_area = -FLT_MAX;
  float best_depth = -FLT_MAX;

  for (count_t i = 0; i < contact_count; ++i) {
    if (i == deepest) {
      continue;
    }

    for (count_t j = i + 1; j < contact_count; ++j) {
      if (j == deepest) {
        continue;
      }

      for (count_t k = j + 1; k < contact_count; ++k) {
        if (k == deepest) {
          continue;
        }

        count_t indices[MAX_CONTACTS_PER_PAIR] = { deepest, i, j, k };
        float area = contact_set_area(contacts, indices, origin, tangent_x, tangent_y);
        float depth = contact_set_depth(contacts, indices);

        if (better_contact_set(area, depth, best_area, best_depth)) {
          memcpy(selected_indices, indices, sizeof(indices));
          best_area = area;
          best_depth = depth;
        }
      }
    }
  }

  // Since will move the elements within the same buffer, having the indices in ascending order will prevent data corruption.
  sort_indices(selected_indices);

  for (count_t i = 0; i < MAX_CONTACTS_PER_PAIR; ++i) {
    contacts[i] = contacts[selected_indices[i]];
  }
}
