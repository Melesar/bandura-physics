#include "scenario-core.h"
#include "bnd-math.h"

bool collapsed;
float sphere_radius;
bnd_v3 arena_size;
float projectile_interval;

float max_inclination;
float min_impulse;
float max_impulse;
float explosion_radius;
float explosion_impulse;

float timer;

bool collapsed;
bool has_tracked;
bnd_body_handle tracked;

imported_mesh cone_mesh;
bnd_body_handle projectiles[128];
count_t active_projectile_count;

static float random_next_float() {
  return (float) GetRandomValue(0, 1024) / 1024;
}

static bnd_body_handle shoot_projectile(bnd_world *world) {
  if (active_projectile_count >= 128) {
    TraceLog(LOG_WARNING, "Max projectiles reached");
    return (bnd_body_handle) { 0 };
  }

  bnd_body_handle projectile = bnd_add_sphere_dynamic(world, 1, 0.5);
  bnd_set_position(world, projectile, (bnd_v3){0, sphere_radius + 1, 0});

  float incline = random_next_float() * max_inclination;
  float azimuth = random_next_float() * 2 * PI;

  bnd_v3 direction = (bnd_v3){sinf(incline) * cosf(azimuth), cosf(incline), sinf(incline) * sinf(azimuth)};
  float impulse = min_impulse + (max_impulse - min_impulse) * random_next_float();

  bnd_apply_impulse(world, projectile, bnd_v3_scale(direction, impulse));
  bnd_event_subscribe(world, projectile, BND_EVENT_COLLISION);

  projectiles[active_projectile_count++] = projectile;

  return projectile;
}

static void add_ground_tile(bnd_world *world, bnd_v3 anchor, bnd_v3 direction, bnd_v3 size) {
  bnd_v3 center = bnd_v3_add(anchor, (bnd_v3){direction.x * size.x * 0.5, direction.y * size.y * 0.5, direction.z * size.z * 0.5});
  bnd_body_handle b = bnd_add_box_static(world, size);
  bnd_set_position(world, b, center);
}

void scenario_configure(program_config *config, bnd_config *physics_config) {
  config->window_title = "Sandbox";
  config->camera_position = (bnd_v3){ 22.542, 11.645, 20.752 };
  config->camera_target = (bnd_v3){ 0, 0, 0 };
  config->draw_ground = false;

  physics_config->memory.contacts_capacity = 512;
  physics_config->memory.dynamics_capacity = 256;
  physics_config->memory.statics_capacity = 96;
  physics_config->memory.events_capacity = 128;
}

void scenario_initialize(bnd_world *world) {
  sphere_radius = 2;
  arena_size = (bnd_v3){60, 0.25, 65};
  projectile_interval = 1;

  max_inclination = PI * 0.25;
  min_impulse = 3;
  max_impulse = 25;
  explosion_radius = 2;
  explosion_impulse = 10;

  cone_mesh.success = import_raylib_mesh(world, GenMeshCone(2, 3, 16), &cone_mesh.mesh);
}

void scenario_setup_scene(bnd_world *world) {
  timer = 0;
  active_projectile_count = 0;

  bnd_add_sphere_static(world, sphere_radius);

  add_ground_tile(world, (bnd_v3){sphere_radius, 0, 0}, (bnd_v3){1, 0, 0}, (bnd_v3){arena_size.x * 0.5 - sphere_radius, arena_size.y, arena_size.z});
  add_ground_tile(world, (bnd_v3){-sphere_radius, 0, 0}, (bnd_v3){-1, 0, 0}, (bnd_v3){arena_size.x * 0.5 - sphere_radius, arena_size.y, arena_size.z});
  add_ground_tile(world, (bnd_v3){0, 0, sphere_radius}, (bnd_v3){0, 0, 1}, (bnd_v3){2 * sphere_radius, arena_size.y, arena_size.z * 0.5 - sphere_radius});
  add_ground_tile(world, (bnd_v3){0, 0, -sphere_radius}, (bnd_v3){0, 0, -1}, (bnd_v3){2 * sphere_radius, arena_size.y, arena_size.z * 0.5 - sphere_radius});

  for (count_t i = 0; i < 6; i++) {
    bnd_body_handle box = bnd_add_box_dynamic(world, 5, bnd_v3_one());
    bnd_set_position(world, box, (bnd_v3){-sphere_radius - 15, 0.5 + i, sphere_radius + 7});
    bnd_put_to_sleep(world, box);
  }

  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){0.25, 5, arena_size.z}), (bnd_v3){arena_size.x * 0.5, 2.5, 0});
  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){arena_size.x, 5, 0.2}), (bnd_v3){0, 2.5, -arena_size.z * 0.5});
  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){0.25, 5, arena_size.z}), (bnd_v3){-arena_size.x * 0.5, 2.5, 0});

  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){0.25, 3, 5}), (bnd_v3){3, 1.5, -sphere_radius - 15});
  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){0.25, 3, 5}), (bnd_v3){-3, 1.5, -sphere_radius - 15});
  bnd_set_position(world, bnd_add_box_static(world, (bnd_v3){6, 3, 0.25}), (bnd_v3){0, 1.5, -sphere_radius - 17.5});

  for (count_t i = 0; i < 25; ++i) {
    bnd_body_handle s = bnd_add_sphere_dynamic(world, 3, 0.5);
    bnd_set_position(world, s, (bnd_v3){GetRandomValue(-2, 2), 2.0 * (i / 5), -sphere_radius - 13.5 + GetRandomValue(1, 4)});
  }

  if (cone_mesh.success) {
    for (count_t i = 0; i < 4; ++i) {
      for (count_t j = 0; j < 4; ++j) {
        bnd_body_handle cone = bnd_add_mesh_dynamic(world, 10, cone_mesh.mesh);
        bnd_set_position(world, cone, (bnd_v3){sphere_radius + 10 + i * 5, 5, j * 5});
      }
    }
  }
}

void scenario_handle_input(bnd_world *world, Camera *camera) {
  Ray r = GetScreenToWorldRay(GetMousePosition(), *camera);
  bnd_raycast_hit hit;

  bnd_ray ray = {
    .origin = r.position,
    .direction = r.direction,
    .max_distance = 100,
  };
  has_tracked = bnd_raycast_closest(world, ray, &hit);
  tracked = hit.body;
}

void scenario_simulate(bnd_world *world, float dt) {
  bnd_simulate(world, dt);

  for (int i = active_projectile_count - 1; i >= 0; --i) {
    bnd_event_enumerator enumerator;
    bnd_v3 pos = bnd_get_position(world, projectiles[i]);

    bool any_collisions = bnd_event_enumerate(world, projectiles[i], &enumerator);
    bool fallen = pos.y < -2;
    if (any_collisions || fallen) {
      bnd_remove_body(world, projectiles[i]);

      bnd_body_handle overlaps[5];
      count_t overlap_count = bnd_overlap(world, pos, explosion_radius, overlaps, 5);

      for (count_t j = 0; j < overlap_count; ++j) {
        bnd_v3 body_pos = bnd_get_position(world, overlaps[j]);
        bnd_apply_impulse(world, overlaps[j], bnd_v3_scale(bnd_v3_normalize(bnd_v3_sub(body_pos, pos)), explosion_impulse));
      }

      if (active_projectile_count > 0) {
        projectiles[i] = projectiles[--active_projectile_count];
      }
    }
  }

  timer += dt;
  if (timer >= projectile_interval) {
    timer -= projectile_interval;
    shoot_projectile(world);
  }

}

void scenario_draw_scene(bnd_world *world) { }

void scenario_build_ui(bnd_world *world) {
  ui_begin_area("Sandbox", &collapsed);

  ui_value_float("Sphere radius", &sphere_radius, 0.1, 10);
  ui_value_float("Arena width", &arena_size.x, sphere_radius + 2, 100);
  ui_value_float("Arena depth", &arena_size.z, sphere_radius + 2, 100);
  ui_value_float("Arena height", &arena_size.y, 0.1, 10);
  ui_value_float("Projectile interval", &projectile_interval, 0.1, 10);
  ui_value_float("Max inclination", &max_inclination, 0, PI);
  ui_value_float("Min impulse", &min_impulse, 1, 100);
  ui_value_float("Max impulse", &max_impulse, 1, 100);
  ui_value_float("Explosion radius", &explosion_radius, 0.1, 10);
  ui_value_float("Explosion impulse", &explosion_impulse, 1, 100);

  ui_end_area();

  ui_begin_area("Tracking", &collapsed);

  if (has_tracked) {
    ui_label_int("Index", tracked.index);
  }

  ui_end_area();
}

void scenario_teardown() { }
