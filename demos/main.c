#include "scenario-core.h"
#include "raygui.h"
#include "raymath.h"
#include "rlgl.h"
#include "string.h"

#include <stdio.h>
#include <math.h>

#define RLIGHTS_IMPLEMENTATION
#include "rcamera.h"
#include "shaders/rlights.h"

const int ui_font_size = 14;
const int screen_width = 1920;
const int screen_height = 1080;
const int frame_rate = 60;
const int simulation_rate = 120;

const float simulation_step = 1.0 / simulation_rate;

typedef struct {
  float movement_speed;
  float rotation_sensitivity;
} camera_settings;

void init_debugging();
void init_gizmos();
void manipulate_gizmos(Camera *camera);
void draw_gizmos();

static void draw_custom_grid(int slices, float spacing);
static void init_physics(const program_config *program_config);
static Shader setup_lighting();
static Camera setup_camera(program_config config);
static void update_camera(Camera *camera, float deltaTime);
static void draw_scene(program_config config, Camera camera, Shader shader, float dt);
static void build_ui();
static void process_inputs(Camera *camera);
static void reset();

extern void scenario_configure(program_config *config, bnd_config *bandura_config);
extern void scenario_initialize(bnd_world *world);
extern void scenario_setup_scene(bnd_world *world);
extern void scenario_handle_input(bnd_world *world, Camera *camera);
extern void scenario_simulate(bnd_world *world, float dt);
extern void scenario_draw_scene(bnd_world *world);
extern void scenario_build_ui(bnd_world *world);
extern void scenario_teardown();

camera_settings cam_settings = {
  .movement_speed = 10.0f,
  .rotation_sensitivity = 0.1f,
};

bnd_debug_draw_callbacks debug_callbacks = {
  .draw_shape = draw_shape,
  .draw_aabb = draw_aabb,
  .draw_contact = draw_contact,
  .draw_joint = draw_joint,
};

bool edit_mode = false;
bool simulation_running = true;
bool step_forward = false;

master_widget_state widget_state = { 0 };

extern Model groundModel;
static bnd_world *world;
static bnd_config config;

int main(int argc, char **argv) {
  program_config program_config = { 0 };
  program_config.draw_ground = true;
  widget_state.draw_bodies = true;

  config = bnd_default_config();

  scenario_configure(&program_config, &config);

  InitWindow(screen_width, screen_height, program_config.window_title);
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetExitKey(KEY_F10);
  SetTargetFPS(frame_rate);
  SetTraceLogLevel(LOG_DEBUG);

  Camera camera = setup_camera(program_config);
  Shader shader = setup_lighting();

  ui_initialize();
  init_debugging();
  init_gizmos();
  init_physics(&program_config);

  setup_scene(shader);
  scenario_initialize(world);
  scenario_setup_scene(world);

  if (argc > 1 && !strncmp(argv[1], "-p", 2)) {
    simulation_running = false;
  }

  float accum = 0;
  float deltaTime = 0;
  while (!WindowShouldClose()) {
    update_camera(&camera, GetFrameTime());

    process_inputs(&camera);

    int sim_count = 0;
    if (!edit_mode) {
      accum += deltaTime;
      sim_count = (int)(fminf(accum / simulation_step, 10));

      for (int i = 0; i < sim_count; i++) {
        if (!simulation_running && !step_forward) {
          break;
        }

        scenario_simulate(world, simulation_step);

        step_forward = false;
      }
    } else {
      manipulate_gizmos(&camera);
    }

    draw_scene(program_config, camera, shader, deltaTime);

    accum -= sim_count * simulation_step;
    deltaTime = GetFrameTime();
  }

  ui_teardown();
  scenario_teardown();
  bnd_teardown(world);

  UnloadShader(shader);
  CloseWindow();

  return 0;
}

static void process_inputs(Camera *camera) {
  if (IsKeyPressed(KEY_SPACE)) {
    simulation_running = !simulation_running;
  }

  if (IsKeyPressed(KEY_PERIOD)) {
    step_forward = true;
  }

  if (IsKeyPressed(KEY_R)) {
    reset();
  }

  if (!edit_mode && IsKeyPressed(KEY_ESCAPE)) {
    edit_mode = true;
  }

  if (edit_mode && IsKeyPressed(KEY_P)) {
    edit_mode = false;
  }

  if (!edit_mode)
    scenario_handle_input(world, camera);
}


static void draw_scene(program_config program_config, Camera camera, Shader shader, float dt) {
  BeginDrawing();

  ClearBackground(COLOR_BACKGROUND);

  BeginMode3D(camera);

  BeginShaderMode(shader);

  if (widget_state.draw_bodies) {
    bnd_debug_draw(world, BND_DEBUG_DRAW_SHAPES, debug_callbacks, &widget_state);
  }

  scenario_draw_scene(world);

  // Draw ground plane
  if (program_config.draw_ground) {
    DrawModel(groundModel, (Vector3){ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
  }
  EndShaderMode();

  if (program_config.draw_ground) {
    draw_custom_grid(32, 1.0f);
  }

  if (edit_mode) {
    draw_gizmos();
  }

  bnd_debug_draw_flags flags = BND_DEBUG_DRAW_NONE;
  if (widget_state.draw_bounding_boxes) {
    flags |= BND_DEBUG_DRAW_AABBS;
  }
  if (widget_state.draw_collisions) {
    flags |= BND_DEBUG_DRAW_CONTACTS;
  }
  if (widget_state.draw_joints) {
    flags |= BND_DEBUG_DRAW_JOINTS;
  }
  bnd_debug_draw(world, flags, debug_callbacks, &widget_state);

  EndMode3D();

  ui_begin();
  build_ui();
  ui_end(dt);

  DrawFPS(1800, 1050);

  EndDrawing();
}

static void update_camera(Camera *camera, float deltaTime) {
  Vector3 forward = GetCameraForward(camera);
  Vector3 right = GetCameraRight(camera);

  Vector3 movement = { 0 };

  if (IsKeyDown(KEY_W))
    movement = Vector3Add(movement, Vector3Scale(forward, cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_S))
    movement = Vector3Add(movement, Vector3Scale(forward, -cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_A))
    movement = Vector3Add(movement, Vector3Scale(right, -cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_D))
    movement = Vector3Add(movement, Vector3Scale(right, cam_settings.movement_speed * deltaTime));

  camera->position = Vector3Add(camera->position, movement);
  camera->target = Vector3Add(camera->target, movement);

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    Vector2 mouseDelta = GetMouseDelta();
    Vector3 rotation = { 0 };
    rotation.x = -mouseDelta.x * cam_settings.rotation_sensitivity; // Yaw
    rotation.y = -mouseDelta.y * cam_settings.rotation_sensitivity; // Pitch

    UpdateCameraPro(camera, (Vector3){ 0 }, rotation, 0.0f);
  }
}

static Camera setup_camera(program_config program_config) {
  Camera3D camera = { 0 };
  camera.position = program_config.camera_position;
  camera.target = program_config.camera_target;
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  return camera;
}

static void draw_custom_grid(int slices, float spacing) {
  int halfSlices = slices / 2;
  Color mainColor = COLOR_GRID_MAIN;
  Color subColor = COLOR_GRID_SUB;

  for (int i = -halfSlices; i <= halfSlices; i++) {
    Color lineColor = (i % 10 == 0) ? mainColor : subColor;

    DrawLine3D((Vector3){ i * spacing, 0.03f, -halfSlices * spacing },
        (Vector3){ i * spacing, 0.03f, halfSlices * spacing }, lineColor);

    DrawLine3D((Vector3){ -halfSlices * spacing, 0.03f, i * spacing },
        (Vector3){ halfSlices * spacing, 0.03f, i * spacing }, lineColor);
  }
}


static void reset() {
  bnd_reset_world(world);
  scenario_setup_scene(world);
}

static void init_physics(const program_config *program_config) {
  if (program_config->custom_malloc == NULL) {
    world = bnd_init(config);
  } else {
    bnd_result_world result = bnd_init_with_allocator(config, (bnd_allocator) {
      program_config->custom_malloc,
      program_config->custom_realloc,
      program_config->custom_free
    });

    if (result.error.type != BND_OK) {
      TraceLog(LOG_FATAL, "Failed to initialize the world: %s", result.error.message);
    }

    world = result.value;
  }

}

static void build_ui() {
  CLAY(CLAY_ID("Container"), {
    .layout = {
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .padding = CLAY_PADDING_ALL(15),
      .childGap = 15,
  }}) {
    bool ui_debug = widget_state.show_ui_debug;
    if (ui_begin_area("Debug widget", &widget_state.is_collapsed)) {
      ui_checkbox("UI debug", &widget_state.show_ui_debug);
      ui_checkbox("Physics config", &widget_state.show_physics_config_widget);
      ui_checkbox("World stats", &widget_state.show_physics_world_stats);
      ui_checkbox("Draw bodies", &widget_state.draw_bodies);
      ui_checkbox("Bodies as wireframe", &widget_state.bodies_as_wireframe);
      ui_checkbox("Draw collisions", &widget_state.draw_collisions);
      ui_checkbox("Draw bounding boxes", &widget_state.draw_bounding_boxes);
      ui_checkbox("Draw joints", &widget_state.draw_joints);
    }

    ui_end_area();

    if (ui_debug != widget_state.show_ui_debug) {
      ui_set_debug(widget_state.show_ui_debug);
    }

    if (widget_state.show_physics_config_widget) {
      bnd_config *physics_config = bnd_edit_config(world);
      if (ui_begin_area("Physics config", &widget_state.physics_config_collapsed)) {
        ui_value_float("Linear damping", &physics_config->simulation.linear_drag, 0, 1);
        ui_value_float("Angular damping", &physics_config->simulation.angular_drag, 0, 1);
        ui_value_float("Restitution", &physics_config->simulation.bounciness, 0, 2);
        ui_value_float("Friction", &physics_config->simulation.friction, 0, 1);
        ui_value_int("Max GJK iterations", (int *)&physics_config->advanced.max_gjk_iterations, 1, 1000);
        ui_value_float("EPA tolerance", &physics_config->advanced.epa_tolerance, 0, 1);
        ui_value_int("Iterations factor", (int *)&physics_config->advanced.resolution_attempts_factor, 1, 20);
        ui_value_float("Penetration epsilon", &physics_config->advanced.penetration_epsilon, 0.001, 0.5);
        ui_value_float("Velocity epsilon", &physics_config->advanced.velocity_epsilon, 0.001, 0.5);
        ui_value_float("Sleep base bias", &physics_config->simulation.sleep_base_bias, 0, 1);
        ui_value_float("Sleep threshold", &physics_config->simulation.sleep_threshold, 0, 10);
        ui_value_float("Restitution damping epsilon", &physics_config->simulation.min_bounce_velocity, 0, 1);
      }

      ui_end_area();
    }

    scenario_build_ui(world);
  }

  if (widget_state.show_physics_world_stats) {
    CLAY(CLAY_ID("Stats"), {
      .layout = {
        .layoutDirection = CLAY_LEFT_TO_RIGHT,
        .childGap = 10,
        .padding = CLAY_PADDING_ALL(3),
      }}) {
      bnd_world_stats stats = bnd_stats(world);

      ui_label_stat("Body count", stats.body_count);
      ui_label_stat("Contacts count", stats.contacts_count);
      ui_label_stat("Incomplete resolutions", stats.incomplete_resolutions);
    }
  }
}

static Shader setup_lighting() {
  char *vs_shader_path = "demos/shaders/lighting_fog.vs";
  char *fs_shader_path = "demos/shaders/lighting_fog.fs";

  Shader shader = LoadShader(vs_shader_path, fs_shader_path);

  Light keyLight = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 10.0f, 20.0f, 10.0f }, Vector3Zero(),
      WHITE, // #ffffff
      shader);
  keyLight.enabled = 1;
  UpdateLightValues(shader, keyLight);

  Light rimLight = CreateLight(LIGHT_POINT, (Vector3){ -10.0f, 10.0f, -10.0f }, Vector3Zero(),
      (Color){ 0x44, 0x44, 0xff, 0xff }, // Blue rim light
      shader);
  rimLight.enabled = 1;
  UpdateLightValues(shader, rimLight);

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(
      shader, ambientLoc, (float[4]){ 0x40 / 255.0f, 0x40 / 255.0f, 0x40 / 255.0f, 1.0f }, SHADER_UNIFORM_VEC4);

  int fogColorLoc = GetShaderLocation(shader, "fogColor");
  int fogStartLoc = GetShaderLocation(shader, "fogStart");
  int fogEndLoc = GetShaderLocation(shader, "fogEnd");

  SetShaderValue(shader, fogColorLoc, (float[3]){ 0x12 / 255.0f, 0x12 / 255.0f, 0x14 / 255.0f }, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, fogStartLoc, (float[1]){ 20.0f }, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader, fogEndLoc, (float[1]){ 100.0f }, SHADER_UNIFORM_FLOAT);

  return shader;
}
