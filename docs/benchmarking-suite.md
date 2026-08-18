# Bandura benchmarking suite

## Purpose

This document defines the first Bandura performance benchmark suite. Its purpose
is to make regressions visible and to provide repeatable, subsystem-oriented
workloads for profiling changes to the physics library.

The suite is inspired by the useful parts of Box2D's benchmark application:
named deterministic scene builders, a fresh world for each run, a fixed time
step, an untimed warm-up, and aggregated measurements across repeated runs. It
does not attempt to reproduce Box2D's scenes or its multithread scaling tests.
Bandura is currently single-threaded and has different physics features, so its
workloads must reflect Bandura's storage, broad phase, collision pipeline,
contacts, sleeping, compounds, and distance joints.

The initial suite is report-only. It produces comparable local results but does
not fail CI on a performance threshold. Timing results are meaningful only when
the machine, compiler, operating-system power policy, and command line are
recorded with the result.

## Scope and non-goals

The first version measures simulation workloads through `bnd_simulate`. It does
not benchmark rendering, debug drawing, demo UI code, allocator throughput,
body creation/removal, raycasts, overlaps, mesh import, or mesh-specific
collision. Those are separate workloads that may be added later without
changing the runner model below.

The benchmark executable must use the public `bandura.h` API for scene creation
and stepping. It must not access `bnd_world_t` or other internal structures.
This keeps the executable representative of a library consumer and prevents a
benchmark from depending on fragile storage details.

The existing `BND_PROFILING` facility is not the benchmark timer. It is useful
after a regression has been found, but it is currently oriented around the demo
workflow and asynchronous CSV output. The benchmark runner owns its timing and
reporting.

## Runner and build integration

Create a headless `benchmarks/` directory and a `zig build bench` target. The
target will use Google Benchmark after that dependency has been integrated by
the project owner.

The executable consists of:

- A small C++ Google Benchmark entry point that registers fixtures and exposes
  normal Google Benchmark command-line flags.
- C or C-compatible scene helpers that create worlds and use the Bandura public
  API. Keeping the workload definitions independent of the harness makes them
  easy to reuse in a future visual inspection tool.
- A shared scene contract, conceptually containing `create`, optional
  `drive_step`, `validate`, and `destroy` callbacks plus static metadata.

Google Benchmark must build and run in `ReleaseFast`. Results are written with
Google Benchmark's normal console report and JSON output. Generated benchmark
results belong under ignored build output and must not be committed as a source
of truth. A result capture should include the Git revision, compiler/version,
optimization mode, operating system, CPU model, and the exact command line.

No Bandura public API additions are required. The build target is the only
required production integration point.

## Measurement protocol

### Unit of measurement

One Google Benchmark sample is one complete, fixed-duration scene run:

1. Construct a fresh world and populate the selected scene outside timed code.
2. Validate construction and execute one untimed `bnd_simulate` warm-up step.
3. Execute exactly 120 timed calls to `bnd_simulate` with `dt = 1.0f / 60.0f`.
4. Collect counters, validate the resulting world state, and tear the world
   down outside timed code.

Thus every sample represents two simulated seconds. Google Benchmark repeats
these identical batches and reports distribution statistics over batch times;
the primary normalized figure is nanoseconds per simulated step.

The runner must create a new world for every measured batch. Reusing one world
would allow sleep state, contact cache age, and scene drift from one sample to
influence the next sample, making the reported distribution hard to interpret.

### Timed boundary

Only `bnd_simulate` is timed. World allocation, body/shape/joint creation,
scene setup, warm-up, validation, counter collection, and teardown are outside
the timed section.

Some scenes need deterministic external motion to remain representative. Their
`drive_step` callback runs immediately before the relevant simulation step and
is outside timing. Google Benchmark timing is paused while the callback runs,
then resumed for the call to `bnd_simulate`. This deliberately measures physics
cost rather than gameplay code that applies forces or moves a kinematic/static
driver.

### Determinism and capacity rules

- Use no random generator, wall-clock input, user input, or runtime-selected
  layout. All dimensions, masses, velocities, and driver positions are fixed.
- Use `bnd_default_config()` as the starting point. Do not set
  `sleep_threshold` to zero: sleeping is a real Bandura optimization and must
  be represented by the suite.
- Derive `dynamics_capacity`, `statics_capacity`, `contacts_capacity`,
  `joints_capacity`, and compound shape-bracket capacities from the selected
  tier before calling `bnd_init`. Provide headroom for the scene's known maximum
  contact count so allocation or reallocation cannot occur during a timed step.
- Treat any returned `bnd_error`, capacity failure, invalid handle, or failed
  scene invariant as a skipped/failed benchmark rather than a timing result.
- Use CPU time rather than wall time for the primary benchmark result. The
  benchmark remains single-threaded; system noise is reported through Google
  Benchmark repetitions rather than hidden by choosing a best run.

### Counters and diagnostics

Every result includes at least these custom counters:

- dynamic body count and static body count;
- configured shape count per compound where applicable;
- configured joint count where applicable;
- contact count after the warm-up and after the 120th step;
- awake dynamic count after the warm-up and after the 120th step;
- `incomplete_resolutions` and `incomplete_collision_detections` from
  `bnd_stats`.

The counters are diagnostic metadata, not additional benchmark assertions
except where a scene contract explicitly requires active constraints or
contacts. They make a time change explainable: for example, a faster settling
pile may simply have slept more bodies rather than improving the solver.

## Initial scene suite

All tiers below are separate Google Benchmark registrations. The small tier is
the smoke-test workload; medium and large tiers are the normal regression
comparison workloads.

### 1. Sparse awake grid

**Goal:** establish the baseline cost of awake dynamic bodies without narrow
phase contacts or contact solving.

- Spawn 128, 256, or 512 one-metre dynamic boxes on a three-dimensional grid
  with four metres between centres.
- Use zero gravity and deterministic low linear/angular velocities. During a
  120-step batch, the separation remains safely larger than the bodies' AABBs.
- Velocities remain above the default sleep threshold, so bodies stay awake by
  normal engine behavior rather than by disabling sleeping.
- Add no static geometry, joints, compound shapes, or collision subscriptions.

This scene measures integration, awake AABB regeneration, and the current
dynamic-dynamic pairwise AABB scan. It intentionally establishes the
broad-phase cost that also exists underneath the other dynamic-body workloads.
Validation requires zero generated contacts and an unchanged awake count.

### 2. Dense settling pile

**Goal:** measure persistent primitive contacts, convex narrow phase, contact
cache/manifold work, iterative resolution, and natural sleep transitions.

- Create a floor and four enclosing static walls.
- Spawn 6 cubed, 8 cubed, or 10 cubed dynamic half-metre boxes on a slightly
  compressed cubic lattice above the floor. Use normal gravity and default
  material settings from `bnd_default_config()`.
- Do not drive the scene after setup. The pile is free to collide, settle, and
  sleep naturally over the batch.

Box-box collision takes Bandura's convex GJK/EPA path. The scene will normally
start contact-heavy and become cheaper as bodies settle; that evolution is a
feature of this workload, not noise. Validation requires contacts after warm-up
and records the awake/contact transition through the batch rather than requiring
all bodies to remain awake.

### 3. Compound crowd

**Goal:** measure compound-shape storage and shape-pair expansion in addition
to convex contact work.

- Reuse the dense pile's static enclosure.
- Spawn 32, 64, or 128 dynamic compound bodies. Each body is a fixed four-box
  cross: four equal boxes at local offsets along positive/negative X and Z.
  Supply equal per-shape masses and identity local rotations.
- Place compound bodies on a deterministic compressed lattice above the floor;
  use normal gravity and no per-step driver.
- Configure the appropriate power-of-two compound shape bracket tier and enough
  contact capacity for all expected body-pair/shape-pair contacts before world
  creation.

This workload covers the public compound-body path, `shape_brackets` access,
and the multiplicative collision work caused by multiple shapes on each body.
As with the pile, sleeping remains enabled and its effect is reported in
counters. Validation requires compound creation to succeed and contact activity
after warm-up.

### 4. Driven joint lattice

**Goal:** measure lowering distance joints into solver contact rows and solving
them under sustained deterministic motion.

- Build 8 by 8, 12 by 12, or 16 by 16 grids of small dynamic spheres. Sphere
  radii and grid spacing must ensure dynamic bodies never physically collide;
  the workload's contacts must therefore come from joints.
- Join horizontal and vertical neighbours with `bnd_add_joint`. Attach the top
  row to static anchor bodies through static distance joints.
- Before every timed simulation step, move all top anchors through a fixed
  phase-shifted sinusoidal path. The drive callback is outside timing.
- Use zero gravity and default sleep settings. Bodies disturbed by the anchors
  stay active naturally; any unaffected bodies may sleep.

Joint rows are deliberately kept active by the moving anchors, so this scene
continues to exercise `joints_generate_contacts` and contact resolution for the
entire batch. Validation requires nonzero joint-generated contacts during the
measured sequence. The sparse grid is the control workload for interpreting the
pairwise broad-phase component that joint scenes still incur.

## Acceptance and operating procedure

Before using a benchmark result for an optimization decision:

1. Build the benchmark target in `ReleaseFast` and run every small tier.
2. Confirm every scene passes its construction and post-batch validation.
3. Run the full suite at least twice with the same command line and compare
   Google Benchmark variation plus the Bandura counters.
4. If a regression appears, use the existing profiling build on the same scene
   shape to inspect phase-level cost; do not infer a specific subsystem solely
   from a total benchmark time.

Performance gates are intentionally out of scope for the first release. Add
them only after the suite has accumulated stable baselines on controlled runner
hardware.

## Future additions

The runner is designed to accept later scene families without changing the
measurement protocol:

- batched raycast and overlap-query throughput;
- body creation, removal, reset, and handle-reuse throughput;
- mesh import and mesh-vs-mesh / mesh-vs-plane collision workloads;
- event subscription and collision-event delivery;
- allocator behavior using `bnd_init_with_allocator`.

Each future scene must state its timed boundary, fixed batch length, capacity
rules, expected physical state, and counters in the same way as the initial
four scenes.
