#include <benchmark/benchmark.h>

#include <bandura.h>

#include <cmath>
#include <ctime>
#include <string>
#include <vector>

namespace {

constexpr float kStepDt = 1.0f / 60.0f;
constexpr int kStepsPerBatch = 120;
constexpr bnd_quat kIdentityRotation = {0.0f, 0.0f, 0.0f, 1.0f};

enum class SceneKind {
  SparseAwakeGrid,
  DenseSettlingPile,
  CompoundCrowd,
  DrivenJointLattice,
};

struct SceneDefinition {
  const char *name;
  SceneKind kind;
  unsigned int primary_size;
  unsigned int grid_x;
  unsigned int grid_y;
  unsigned int grid_z;
};

struct SceneInstance {
  bnd_world *world = nullptr;
  std::vector<bnd_body_handle> anchors;
  unsigned int expected_dynamics = 0;
  unsigned int expected_statics = 0;
  unsigned int configured_shapes_per_compound = 0;
  unsigned int configured_joints = 0;
  unsigned int warmup_contacts = 0;
  unsigned int warmup_awake = 0;
  bool joint_contacts_seen = false;
  std::string error;
};

bool Check(SceneInstance *instance, bnd_error error, const char *operation) {
  if (error.type == BND_OK) {
    return true;
  }

  instance->error = operation;
  if (error.message != nullptr) {
    instance->error += ": ";
    instance->error += error.message;
  }
  return false;
}

bool Check(SceneInstance *instance, bnd_result_handle result,
           const char *operation, bnd_body_handle *handle) {
  if (!Check(instance, result.error, operation)) {
    return false;
  }

  *handle = result.value;
  return true;
}

bool Check(SceneInstance *instance, bnd_result_u32 result,
           const char *operation) {
  return Check(instance, result.error, operation);
}

bool AddBoxStaticAt(SceneInstance *instance, bnd_v3 size, bnd_v3 position) {
  bnd_body_handle handle;
  return Check(instance, bnd_add_box_static(instance->world, size),
               "bnd_add_box_static", &handle) &&
         Check(instance, bnd_set_position(instance->world, handle, position),
               "bnd_set_position");
}

bool AddStaticEnclosure(SceneInstance *instance, float half_extent,
                        float height) {
  constexpr float wall_thickness = 0.5f;
  const float wall_center = half_extent + wall_thickness * 0.5f;

  return AddBoxStaticAt(instance,
                        {2.0f * (half_extent + wall_thickness), wall_thickness,
                         2.0f * (half_extent + wall_thickness)},
                        {0.0f, -wall_thickness * 0.5f, 0.0f}) &&
         AddBoxStaticAt(
             instance,
             {wall_thickness, height, 2.0f * (half_extent + wall_thickness)},
             {wall_center, height * 0.5f, 0.0f}) &&
         AddBoxStaticAt(
             instance,
             {wall_thickness, height, 2.0f * (half_extent + wall_thickness)},
             {-wall_center, height * 0.5f, 0.0f}) &&
         AddBoxStaticAt(
             instance,
             {2.0f * (half_extent + wall_thickness), height, wall_thickness},
             {0.0f, height * 0.5f, wall_center}) &&
         AddBoxStaticAt(
             instance,
             {2.0f * (half_extent + wall_thickness), height, wall_thickness},
             {0.0f, height * 0.5f, -wall_center});
}

unsigned int RoundUpTo64(unsigned int value) { return (value + 63U) & ~63U; }

bnd_config MakeConfig(const SceneDefinition &definition) {
  bnd_config config = bnd_default_config();
  const unsigned int dynamic_count =
      definition.kind == SceneKind::SparseAwakeGrid ||
              definition.kind == SceneKind::CompoundCrowd
          ? definition.primary_size
      : definition.kind == SceneKind::DenseSettlingPile
          ? definition.primary_size * definition.primary_size *
                definition.primary_size
          : definition.primary_size * definition.primary_size;
  const unsigned int static_count =
      definition.kind == SceneKind::DrivenJointLattice ? definition.primary_size
      : definition.kind == SceneKind::SparseAwakeGrid  ? 0U
                                                       : 5U;
  const unsigned int joint_count =
      definition.kind == SceneKind::DrivenJointLattice
          ? 2U * definition.primary_size * (definition.primary_size - 1U) +
                definition.primary_size
          : 0U;

  // These bounds cover the deterministic local-neighbour contact patterns in
  // the pile/crowd layouts and leave the timed simulation free of growth.
  const unsigned int contacts_capacity =
      definition.kind == SceneKind::DenseSettlingPile ? 32768U
      : definition.kind == SceneKind::CompoundCrowd   ? 65536U
      : definition.kind == SceneKind::DrivenJointLattice
          ? RoundUpTo64(joint_count + 64U)
          : 64U;

  config.memory.dynamics_capacity = dynamic_count + 16U;
  config.memory.statics_capacity = static_count + 16U;
  config.memory.contacts_capacity = contacts_capacity;
  config.memory.joints_capacity = joint_count + 16U;
  config.advanced.shapes_brackets_capacity[0] =
      RoundUpTo64(dynamic_count + static_count + 16U);
  config.advanced.shapes_brackets_capacity[2] =
      definition.kind == SceneKind::CompoundCrowd
          ? RoundUpTo64(dynamic_count + 16U)
          : 64U;
  config.advanced.contacts_cache.buffer_capacity = contacts_capacity;
  config.advanced.contacts_cache.hash_table_capacity = contacts_capacity * 2U;

  if (definition.kind == SceneKind::SparseAwakeGrid ||
      definition.kind == SceneKind::DrivenJointLattice) {
    config.simulation.gravity = {0.0f, 0.0f, 0.0f};
  }

  return config;
}

bool BuildSparseAwakeGrid(SceneInstance *instance,
                          const SceneDefinition &definition) {
  for (unsigned int y = 0; y < definition.grid_y; ++y) {
    for (unsigned int z = 0; z < definition.grid_z; ++z) {
      for (unsigned int x = 0; x < definition.grid_x; ++x) {
        bnd_body_handle body;
        if (!Check(
                instance,
                bnd_add_box_dynamic(instance->world, 1.0f, {1.0f, 1.0f, 1.0f}),
                "bnd_add_box_dynamic", &body) ||
            !Check(instance,
                   bnd_set_position(instance->world, body,
                                    {4.0f * static_cast<float>(x),
                                     4.0f * static_cast<float>(y),
                                     4.0f * static_cast<float>(z)}),
                   "bnd_set_position") ||
            !Check(
                instance,
                bnd_set_velocity(instance->world, body, {0.70f, 0.20f, -0.30f}),
                "bnd_set_velocity") ||
            !Check(instance,
                   bnd_set_angular_momentum(instance->world, body,
                                            {0.20f, -0.10f, 0.15f}),
                   "bnd_set_angular_momentum")) {
          return false;
        }
      }
    }
  }

  instance->expected_dynamics = definition.primary_size;
  return true;
}

bool BuildDenseSettlingPile(SceneInstance *instance, unsigned int side) {
  constexpr float box_size = 1.0f;
  constexpr float spacing = 0.92f;
  const float half_extent =
      0.5f * static_cast<float>(side - 1U) * spacing + box_size;
  if (!AddStaticEnclosure(instance, half_extent,
                          static_cast<float>(side) * spacing + 2.0f)) {
    return false;
  }

  for (unsigned int y = 0; y < side; ++y) {
    for (unsigned int z = 0; z < side; ++z) {
      for (unsigned int x = 0; x < side; ++x) {
        bnd_body_handle body;
        const bnd_v3 position = {
            (static_cast<float>(x) - 0.5f * static_cast<float>(side - 1U)) *
                spacing,
            0.5f * box_size + static_cast<float>(y) * spacing,
            (static_cast<float>(z) - 0.5f * static_cast<float>(side - 1U)) *
                spacing,
        };
        if (!Check(instance,
                   bnd_add_box_dynamic(instance->world, 1.0f,
                                       {box_size, box_size, box_size}),
                   "bnd_add_box_dynamic", &body) ||
            !Check(instance, bnd_set_position(instance->world, body, position),
                   "bnd_set_position")) {
          return false;
        }
      }
    }
  }

  instance->expected_dynamics = side * side * side;
  instance->expected_statics = 5;
  return true;
}

bool BuildCompoundCrowd(SceneInstance *instance,
                        const SceneDefinition &definition) {
  constexpr float box_size = 0.60f;
  constexpr float shape_offset = 0.35f;
  constexpr float spacing = 1.10f;
  const float half_extent =
      0.5f * static_cast<float>(definition.grid_x - 1U) * spacing + 1.0f;
  if (!AddStaticEnclosure(instance, half_extent,
                          static_cast<float>(definition.grid_y) * spacing +
                              2.0f)) {
    return false;
  }

  for (unsigned int y = 0; y < definition.grid_y; ++y) {
    for (unsigned int z = 0; z < definition.grid_z; ++z) {
      for (unsigned int x = 0; x < definition.grid_x; ++x) {
        bnd_body_shape shapes[4] = {
            {BND_BOX,
             {{{box_size, box_size, box_size}}},
             {shape_offset, 0.0f, 0.0f},
             kIdentityRotation},
            {BND_BOX,
             {{{box_size, box_size, box_size}}},
             {-shape_offset, 0.0f, 0.0f},
             kIdentityRotation},
            {BND_BOX,
             {{{box_size, box_size, box_size}}},
             {0.0f, 0.0f, shape_offset},
             kIdentityRotation},
            {BND_BOX,
             {{{box_size, box_size, box_size}}},
             {0.0f, 0.0f, -shape_offset},
             kIdentityRotation},
        };
        float masses[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        bnd_body_handle body;
        const bnd_v3 position = {
            (static_cast<float>(x) -
             0.5f * static_cast<float>(definition.grid_x - 1U)) *
                spacing,
            0.5f * box_size + static_cast<float>(y) * spacing,
            (static_cast<float>(z) -
             0.5f * static_cast<float>(definition.grid_z - 1U)) *
                spacing,
        };
        if (!Check(instance,
                   bnd_add_compound_body_dynamic(instance->world, shapes,
                                                 masses, 4),
                   "bnd_add_compound_body_dynamic", &body) ||
            !Check(instance, bnd_set_position(instance->world, body, position),
                   "bnd_set_position")) {
          return false;
        }
      }
    }
  }

  instance->expected_dynamics = definition.primary_size;
  instance->expected_statics = 5;
  instance->configured_shapes_per_compound = 4;
  return true;
}

bool BuildDrivenJointLattice(SceneInstance *instance, unsigned int side) {
  constexpr float radius = 0.15f;
  constexpr float spacing = 1.20f;
  constexpr float max_joint_distance = 1.00f;
  constexpr float anchor_distance = 0.40f;
  std::vector<bnd_body_handle> bodies(side * side);
  instance->anchors.reserve(side);

  for (unsigned int y = 0; y < side; ++y) {
    for (unsigned int x = 0; x < side; ++x) {
      bnd_body_handle body;
      if (!Check(instance,
                 bnd_add_sphere_dynamic(instance->world, 1.0f, radius),
                 "bnd_add_sphere_dynamic", &body) ||
          !Check(instance,
                 bnd_set_position(instance->world, body,
                                  {static_cast<float>(x) * spacing,
                                   static_cast<float>(y) * spacing, 0.0f}),
                 "bnd_set_position")) {
        return false;
      }
      bodies[y * side + x] = body;
    }
  }

  for (unsigned int x = 0; x < side; ++x) {
    bnd_body_handle anchor;
    if (!Check(instance, bnd_add_sphere_static(instance->world, 0.01f),
               "bnd_add_sphere_static", &anchor) ||
        !Check(instance,
               bnd_set_position(
                   instance->world, anchor,
                   {static_cast<float>(x) * spacing,
                    static_cast<float>(side - 1U) * spacing + anchor_distance,
                    0.0f}),
               "bnd_set_position")) {
      return false;
    }
    instance->anchors.push_back(anchor);
  }

  for (unsigned int y = 0; y < side; ++y) {
    for (unsigned int x = 0; x < side; ++x) {
      const bnd_body_handle body = bodies[y * side + x];
      if (x + 1U < side &&
          !Check(instance,
                 bnd_add_joint(instance->world, body, bodies[y * side + x + 1U],
                               {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
                               max_joint_distance),
                 "bnd_add_joint")) {
        return false;
      }
      if (y + 1U < side &&
          !Check(instance,
                 bnd_add_joint(instance->world, body,
                               bodies[(y + 1U) * side + x], {0.0f, 0.0f, 0.0f},
                               {0.0f, 0.0f, 0.0f}, max_joint_distance),
                 "bnd_add_joint")) {
        return false;
      }
    }
  }

  for (unsigned int x = 0; x < side; ++x) {
    if (!Check(instance,
               bnd_add_joint(instance->world, bodies[(side - 1U) * side + x],
                             instance->anchors[x], {0.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f}, 0.20f),
               "bnd_add_joint")) {
      return false;
    }
  }

  instance->expected_dynamics = side * side;
  instance->expected_statics = side;
  instance->configured_joints = 2U * side * (side - 1U) + side;
  return true;
}

bool BuildScene(SceneInstance *instance, const SceneDefinition &definition) {
  instance->world = bnd_init(MakeConfig(definition));
  if (instance->world == nullptr) {
    instance->error = "bnd_init returned null";
    return false;
  }

  switch (definition.kind) {
  case SceneKind::SparseAwakeGrid:
    return BuildSparseAwakeGrid(instance, definition);
  case SceneKind::DenseSettlingPile:
    return BuildDenseSettlingPile(instance, definition.primary_size);
  case SceneKind::CompoundCrowd:
    return BuildCompoundCrowd(instance, definition);
  case SceneKind::DrivenJointLattice:
    return BuildDrivenJointLattice(instance, definition.primary_size);
  }

  instance->error = "unknown scene kind";
  return false;
}

bool ValidateCounts(SceneInstance *instance) {
  if (bnd_body_count(instance->world, BND_BODY_DYNAMIC) !=
          instance->expected_dynamics ||
      bnd_body_count(instance->world, BND_BODY_STATIC) !=
          instance->expected_statics) {
    instance->error = "scene body count differs from its configured tier";
    return false;
  }
  return true;
}

bool WarmUp(SceneInstance *instance, const SceneDefinition &definition) {
  if (!ValidateCounts(instance)) {
    return false;
  }

  bnd_simulate(instance->world, kStepDt);
  instance->warmup_contacts = bnd_collisions_count(instance->world);
  instance->warmup_awake = bnd_awake_count(instance->world);

  if (definition.kind == SceneKind::SparseAwakeGrid &&
      (instance->warmup_contacts != 0 ||
       instance->warmup_awake != instance->expected_dynamics)) {
    instance->error = "sparse grid generated contacts or put an awake body to "
                      "sleep during warm-up";
    return false;
  }
  if ((definition.kind == SceneKind::DenseSettlingPile ||
       definition.kind == SceneKind::CompoundCrowd) &&
      instance->warmup_contacts == 0) {
    instance->error =
        "contact-heavy scene generated no contacts during warm-up";
    return false;
  }
  return true;
}

bool DriveJointLattice(SceneInstance *instance, unsigned int step) {
  constexpr float spacing = 1.20f;
  constexpr float anchor_distance = 0.40f;
  const unsigned int side = static_cast<unsigned int>(instance->anchors.size());
  const float phase = static_cast<float>(step) * 0.19f;
  for (unsigned int x = 0; x < side; ++x) {
    const float anchor_phase = phase + static_cast<float>(x) * 0.37f;
    const bnd_v3 position = {
        static_cast<float>(x) * spacing + 0.12f * std::sin(anchor_phase),
        static_cast<float>(side - 1U) * spacing + anchor_distance +
            0.10f * std::sin(anchor_phase * 0.7f),
        0.10f * std::cos(anchor_phase),
    };
    if (!Check(
            instance,
            bnd_set_position(instance->world, instance->anchors[x], position),
            "bnd_set_position")) {
      return false;
    }
  }
  return true;
}

bool ValidateAfterBatch(SceneInstance *instance,
                        const SceneDefinition &definition) {
  if (!ValidateCounts(instance)) {
    return false;
  }

  if (definition.kind == SceneKind::SparseAwakeGrid &&
      (bnd_collisions_count(instance->world) != 0 ||
       bnd_awake_count(instance->world) != instance->expected_dynamics)) {
    instance->error =
        "sparse grid contact or awake-count invariant failed after the batch";
    return false;
  }
  if (definition.kind == SceneKind::DrivenJointLattice &&
      !instance->joint_contacts_seen) {
    instance->error = "driven joint lattice generated no joint contacts during "
                      "the measured sequence";
    return false;
  }
  return true;
}

double ProcessCpuSeconds() {
  return static_cast<double>(std::clock()) /
         static_cast<double>(CLOCKS_PER_SEC);
}

void AddCounters(benchmark::State &state, const SceneInstance &instance) {
  const bnd_world_stats stats = bnd_stats(instance.world);
  state.counters["dynamic_bodies"] = instance.expected_dynamics;
  state.counters["static_bodies"] = instance.expected_statics;
  state.counters["shapes_per_compound"] =
      instance.configured_shapes_per_compound;
  state.counters["joints"] = instance.configured_joints;
  state.counters["contacts_after_warmup"] = instance.warmup_contacts;
  state.counters["contacts_after_batch"] = bnd_collisions_count(instance.world);
  state.counters["awake_after_warmup"] = instance.warmup_awake;
  state.counters["awake_after_batch"] = bnd_awake_count(instance.world);
  state.counters["incomplete_resolutions"] = stats.incomplete_resolutions;
  state.counters["incomplete_collision_detections"] =
      stats.incomplete_collision_detections;
  state.counters["steps_per_batch"] = kStepsPerBatch;
}

void RunScene(benchmark::State &state, const SceneDefinition *definition) {
  for (auto _ : state) {
    (void)_;
    SceneInstance instance;
    if (!BuildScene(&instance, *definition) ||
        !WarmUp(&instance, *definition)) {
      if (instance.world != nullptr) {
        bnd_teardown(instance.world);
      }
      state.SkipWithError(instance.error.c_str());
      break;
    }

    // Google Benchmark receives CPU time normalized to a single simulated
    // step. Each state iteration still constructs and runs one 120-step batch.
    double elapsed = 0.0;
    for (unsigned int step = 0; step < kStepsPerBatch; ++step) {
      if (definition->kind == SceneKind::DrivenJointLattice &&
          !DriveJointLattice(&instance, step)) {
        break;
      }
      const double begin = ProcessCpuSeconds();
      bnd_simulate(instance.world, kStepDt);
      elapsed += ProcessCpuSeconds() - begin;
      if (definition->kind == SceneKind::DrivenJointLattice &&
          bnd_collisions_count(instance.world) != 0) {
        instance.joint_contacts_seen = true;
      }
    }
    if (!instance.error.empty()) {
      bnd_teardown(instance.world);
      state.SkipWithError(instance.error.c_str());
      break;
    }
    state.SetIterationTime(elapsed / static_cast<double>(kStepsPerBatch));

    const bool valid = ValidateAfterBatch(&instance, *definition);
    AddCounters(state, instance);
    bnd_teardown(instance.world);
    if (!valid) {
      state.SkipWithError(instance.error.c_str());
      break;
    }
  }
}

const SceneDefinition kSparse128 = {
    "SparseAwakeGrid_128", SceneKind::SparseAwakeGrid, 128, 8, 2, 8};
const SceneDefinition kSparse256 = {
    "SparseAwakeGrid_256", SceneKind::SparseAwakeGrid, 256, 8, 4, 8};
const SceneDefinition kSparse512 = {
    "SparseAwakeGrid_512", SceneKind::SparseAwakeGrid, 512, 8, 8, 8};
const SceneDefinition kPile6 = {
    "DenseSettlingPile_6", SceneKind::DenseSettlingPile, 6, 0, 0, 0};
const SceneDefinition kPile8 = {
    "DenseSettlingPile_8", SceneKind::DenseSettlingPile, 8, 0, 0, 0};
const SceneDefinition kPile10 = {
    "DenseSettlingPile_10", SceneKind::DenseSettlingPile, 10, 0, 0, 0};
const SceneDefinition kCrowd32 = {
    "CompoundCrowd_32", SceneKind::CompoundCrowd, 32, 4, 2, 4};
const SceneDefinition kCrowd64 = {
    "CompoundCrowd_64", SceneKind::CompoundCrowd, 64, 4, 4, 4};
const SceneDefinition kCrowd128 = {
    "CompoundCrowd_128", SceneKind::CompoundCrowd, 128, 4, 8, 4};
const SceneDefinition kLattice8 = {
    "DrivenJointLattice_8", SceneKind::DrivenJointLattice, 8, 0, 0, 0};
const SceneDefinition kLattice12 = {
    "DrivenJointLattice_12", SceneKind::DrivenJointLattice, 12, 0, 0, 0};
const SceneDefinition kLattice16 = {
    "DrivenJointLattice_16", SceneKind::DrivenJointLattice, 16, 0, 0, 0};

#define REGISTER_SCENE(name, definition)                                       \
  BENCHMARK_CAPTURE(RunScene, name, &definition)                               \
      ->UseManualTime()                                                        \
      ->Unit(benchmark::kMillisecond)

REGISTER_SCENE(SparseAwakeGrid_128, kSparse128);
REGISTER_SCENE(SparseAwakeGrid_256, kSparse256);
REGISTER_SCENE(SparseAwakeGrid_512, kSparse512);
REGISTER_SCENE(DenseSettlingPile_6, kPile6);
REGISTER_SCENE(DenseSettlingPile_8, kPile8);
REGISTER_SCENE(DenseSettlingPile_10, kPile10);
REGISTER_SCENE(CompoundCrowd_32, kCrowd32);
REGISTER_SCENE(CompoundCrowd_64, kCrowd64);
REGISTER_SCENE(CompoundCrowd_128, kCrowd128);
REGISTER_SCENE(DrivenJointLattice_8, kLattice8);
REGISTER_SCENE(DrivenJointLattice_12, kLattice12);
REGISTER_SCENE(DrivenJointLattice_16, kLattice16);

#undef REGISTER_SCENE

} // namespace

BENCHMARK_MAIN();
