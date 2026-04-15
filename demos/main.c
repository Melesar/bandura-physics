#include "core.h"
#include "raygui.h"
#include "raymath.h"
#include "rlgl.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

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
static void setup_scene(Shader shader);
static void init_physics();
static Shader setup_lighting();
static Camera setup_camera(program_config config);
static void update_camera(Camera *camera, float deltaTime);
static void draw_scene(Camera camera, Shader shader, float dt);
static void build_ui();
static void draw_physics_bodies();
static void process_inputs(Camera *camera);
static void reset();

extern void scenario_initialize(program_config *config,
                                physics_config *physics_config);
extern void scenario_setup_scene(physics_world *world);
extern void scenario_handle_input(physics_world *world, Camera *camera);
extern void scenario_simulate(physics_world *world, float dt);
extern void scenario_draw_scene(physics_world *world);
extern void scenario_build_ui(physics_world *world);
extern void scenario_teardown();

camera_settings cam_settings = {
    .movement_speed = 10.0f,
    .rotation_sensitivity = 0.1f,
};

Color colors[] = {BROWN,    YELLOW,     GREEN, MAROON, MAGENTA,
                  RAYWHITE, DARKPURPLE, LIME,  PINK,   ORANGE,
                  BROWN,    YELLOW,     GREEN, MAROON, MAGENTA,
                  RAYWHITE, DARKPURPLE, LIME,  PINK,   ORANGE};
Material materials[20];
Mesh meshes[20];

bool edit_mode = false;
bool simulation_running = true;
bool step_forward = false;

struct {
  bool is_collapsed;
  bool physics_config_collapsed;

  bool show_ui_debug;
  bool show_physics_world_stats;
  bool show_physics_config_widget;
  bool draw_collisions;
} master_widget_state;

static Model groundModel;
static physics_world *world;
static physics_config config;

int main(int argc, char **argv) {
  program_config program_config = {0};
  config = physics_default_config();

  scenario_initialize(&program_config, &config);

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
  init_physics();
  setup_scene(shader);
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
      sim_count = (int)(accum / simulation_step);

      for (int i = 0; i < sim_count; i++) {
        if (!simulation_running && !step_forward)
          break;

        scenario_simulate(world, simulation_step);

        step_forward = false;
      }
    } else {
      manipulate_gizmos(&camera);
    }

    draw_scene(camera, shader, deltaTime);

    accum -= sim_count * simulation_step;
    deltaTime = GetFrameTime();
  }

  ui_teardown();
  scenario_teardown();
  physics_teardown(world);

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

static void draw_physics_bodies_typed(body_type type) {
  body_enumerator_typed enumerator;
  physics_enumerate_bodies_typed(world, type, &enumerator);

  count_t i = 0;
  while (physics_body_next_typed(world, &enumerator)) {
    v3 position = physics_get_position(world, enumerator.handle);
    quat rotation = physics_get_rotation(world, enumerator.handle);

    count_t shapes_count;
    body_shape *shapes =
        physics_get_shapes(world, enumerator.handle, &shapes_count);

    m4 scale;
    m4 transform =
        MatrixMultiply(QuaternionToMatrix(rotation),
                       MatrixTranslate(position.x, position.y, position.z));
    Material material = materials[enumerator.handle.index % 20];

    for (count_t k = 0; k < shapes_count; ++k) {
      body_shape shape = shapes[k];
      m4 shape_transform =
          mul(as_matrix(shape.rotation),
              MatrixTranslate(shape.offset.x, shape.offset.y, shape.offset.z));
      m4 full_transform = mul(shape_transform, transform);

      switch (shape.type) {
      case SHAPE_BOX:
        scale =
            MatrixScale(shape.box.size.x, shape.box.size.y, shape.box.size.z);
        DrawMesh(meshes[SHAPE_BOX], material, mul(scale, full_transform));
        break;

      case SHAPE_SPHERE:
        scale = MatrixScale(shape.sphere.radius, shape.sphere.radius,
                            shape.sphere.radius);
        DrawMesh(meshes[SHAPE_SPHERE], material, mul(scale, full_transform));
        break;

      case SHAPE_CYLINDER:
        scale = MatrixScale(shape.cylinder.radius, shape.cylinder.height,
                            shape.cylinder.radius);
        DrawMesh(meshes[SHAPE_CYLINDER], material, mul(scale, full_transform));
        break;

      default:
        break;
      }
    }
  }
}

static void draw_physics_bodies() {
  draw_physics_bodies_typed(BODY_DYNAMIC);
  draw_physics_bodies_typed(BODY_STATIC);
}

static void draw_scene(Camera camera, Shader shader, float dt) {
  BeginDrawing();

  ClearBackground(COLOR_BACKGROUND);

  BeginMode3D(camera);

  BeginShaderMode(shader);

  draw_physics_bodies();
  scenario_draw_scene(world);

  // Draw ground plane
  DrawModel(groundModel, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
  draw_custom_grid(32, 1.0f);

  if (edit_mode)
    draw_gizmos();
  if (master_widget_state.draw_collisions)
    physics_draw_collisions(world);

  EndShaderMode();

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

  Vector3 movement = {0};

  if (IsKeyDown(KEY_W))
    movement =
        Vector3Add(movement, Vector3Scale(forward, cam_settings.movement_speed *
                                                       deltaTime));
  if (IsKeyDown(KEY_S))
    movement = Vector3Add(
        movement,
        Vector3Scale(forward, -cam_settings.movement_speed * deltaTime));
  if (IsKeyDown(KEY_A))
    movement =
        Vector3Add(movement, Vector3Scale(right, -cam_settings.movement_speed *
                                                     deltaTime));
  if (IsKeyDown(KEY_D))
    movement = Vector3Add(
        movement, Vector3Scale(right, cam_settings.movement_speed * deltaTime));

  camera->position = Vector3Add(camera->position, movement);
  camera->target = Vector3Add(camera->target, movement);

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    Vector2 mouseDelta = GetMouseDelta();
    Vector3 rotation = {0};
    rotation.x = -mouseDelta.x * cam_settings.rotation_sensitivity; // Yaw
    rotation.y = -mouseDelta.y * cam_settings.rotation_sensitivity; // Pitch

    UpdateCameraPro(camera, (Vector3){0}, rotation, 0.0f);
  }
}

static Camera setup_camera(program_config program_config) {
  Camera3D camera = {0};
  camera.position = program_config.camera_position;
  camera.target = program_config.camera_target;
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
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

    DrawLine3D((Vector3){i * spacing, 0.03f, -halfSlices * spacing},
               (Vector3){i * spacing, 0.03f, halfSlices * spacing}, lineColor);

    DrawLine3D((Vector3){-halfSlices * spacing, 0.03f, i * spacing},
               (Vector3){halfSlices * spacing, 0.03f, i * spacing}, lineColor);
  }
}

static void setup_scene(Shader shader) {
  meshes[SHAPE_BOX] = GenMeshCube(1, 1, 1);
  meshes[SHAPE_SPHERE] = GenMeshSphere(1, 16, 16);
  meshes[SHAPE_PLANE] = GenMeshPlane(200.0f, 200.0f, 1, 1);

  Mesh cylinder = GenMeshCylinder(1, 1, 32);
  for (int i = 0; i < cylinder.vertexCount; ++i) {
    cylinder.vertices[i * 3 + 1] -= 0.5;
  }
  UpdateMeshBuffer(cylinder, RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION,
                   cylinder.vertices, 3 * cylinder.vertexCount * sizeof(float),
                   0);

  meshes[SHAPE_CYLINDER] = cylinder;

  for (size_t i = 0; i < 20; ++i) {
    Material m = LoadMaterialDefault();
    m.shader = shader;
    m.maps[MATERIAL_MAP_ALBEDO].color = colors[i];

    materials[i] = m;
  }

  groundModel = LoadModelFromMesh(meshes[SHAPE_PLANE]);

  groundModel.materials[0].shader = shader;
  groundModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = COLOR_GROUND;
}

static void reset() {
  physics_reset(world);
  physics_add_plane(world, zero(), up());
  scenario_setup_scene(world);
}

static void init_physics() {
  world = physics_init(&config);
  physics_add_plane(world, zero(), up());
}

static void build_ui() {
  CLAY(CLAY_ID("Container"), {.layout = {
                                  .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                  .padding = CLAY_PADDING_ALL(15),
                                  .childGap = 15,
                              }}) {
    bool ui_debug = master_widget_state.show_ui_debug;
    if (ui_begin_area("Debug widget", &master_widget_state.is_collapsed)) {
      ui_checkbox("UI debug", &master_widget_state.show_ui_debug);
      ui_checkbox("Physics config",
                  &master_widget_state.show_physics_config_widget);
      ui_checkbox("World stats", &master_widget_state.show_physics_world_stats);
      ui_checkbox("Draw collisions", &master_widget_state.draw_collisions);
    }

    ui_end_area();

    if (ui_debug != master_widget_state.show_ui_debug) {
      ui_set_debug(master_widget_state.show_ui_debug);
    }

    if (master_widget_state.show_physics_config_widget) {
      physics_config *physics_config = physics_edit_config(world);
      if (ui_begin_area("Physics config",
                        &master_widget_state.physics_config_collapsed)) {
        ui_value_float("Linear damping", &physics_config->linear_damping, 0, 1);
        ui_value_float("Angular damping", &physics_config->angular_damping, 0,
                       1);
        ui_value_float("Restitution", &physics_config->restitution, 0, 2);
        ui_value_float("Friction", &physics_config->friction, 0, 1);
        ui_value_int("Iterations factor",
                     (int *)&physics_config->resolution_attempts_factor, 1, 20);
        // ui_value_int("Max penetration iterations",
        // (int*)&physics_config->max_penentration_iterations, 1, 500);
        // ui_value_int("Max velocity iterations",
        // (int*)&physics_config->max_velocity_iterations, 1, 500);
        ui_value_float("Penetration epsilon",
                       &physics_config->penetration_epsilon, 0.001, 0.5);
        ui_value_float("Velocity epsilon", &physics_config->velocity_epsilon,
                       0.001, 0.5);
        ui_value_float("Sleep base bias", &physics_config->sleep_base_bias, 0,
                       1);
        ui_value_float("Sleep threshold", &physics_config->sleep_threshold, 0,
                       10);
        ui_value_float("Restitution damping epsilon",
                       &physics_config->restitution_damping_limit, 0, 1);
      }

      ui_end_area();
    }

    scenario_build_ui(world);
  }

  if (master_widget_state.show_physics_world_stats) {
    CLAY(CLAY_ID("Stats"), {.layout = {
                                .layoutDirection = CLAY_LEFT_TO_RIGHT,
                                .childGap = 10,
                                .padding = CLAY_PADDING_ALL(3),
                            }}) {
      physics_world_stats stats = physics_get_stats(world);

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

  Light keyLight = CreateLight(LIGHT_DIRECTIONAL,
                               (Vector3){10.0f, 20.0f, 10.0f}, Vector3Zero(),
                               WHITE, // #ffffff
                               shader);
  keyLight.enabled = 1;
  UpdateLightValues(shader, keyLight);

  Light rimLight =
      CreateLight(LIGHT_POINT, (Vector3){-10.0f, 10.0f, -10.0f}, Vector3Zero(),
                  (Color){0x44, 0x44, 0xff, 0xff}, // Blue rim light
                  shader);
  rimLight.enabled = 1;
  UpdateLightValues(shader, rimLight);

  int ambientLoc = GetShaderLocation(shader, "ambient");
  SetShaderValue(shader, ambientLoc,
                 (float[4]){0x40 / 255.0f, 0x40 / 255.0f, 0x40 / 255.0f, 1.0f},
                 SHADER_UNIFORM_VEC4);

  int fogColorLoc = GetShaderLocation(shader, "fogColor");
  int fogStartLoc = GetShaderLocation(shader, "fogStart");
  int fogEndLoc = GetShaderLocation(shader, "fogEnd");

  SetShaderValue(shader, fogColorLoc,
                 (float[3]){0x12 / 255.0f, 0x12 / 255.0f, 0x14 / 255.0f},
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(shader, fogStartLoc, (float[1]){20.0f}, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader, fogEndLoc, (float[1]){100.0f}, SHADER_UNIFORM_FLOAT);

  return shader;
}
