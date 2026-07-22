# Library Map

## Public Boundary

- `include/bandura.h`: public C99 ABI. Defines math-compatible value types, errors/results, allocators, body/mesh handles, configuration, contact/event types, the opaque `bnd_world`, and exported `bnd_*` functions.
- `include/bnd-math.h`: optional vector, quaternion, and matrix helpers. Inline vector/quaternion operations live here; non-inline matrix operations live in `src/math.c`.
- `include/profiler.h`: compile-time instrumentation contract used internally. Its macros disappear unless `BND_PROFILING` is defined.

## Internal Hub

`src/bnd-core.h` owns the shared internal vocabulary:

- error/result and allocation macros;
- `common_data`, `dynamic_bodies`, `static_bodies`, and `bnd_world_t`;
- body-shape brackets, contacts, joints, meshes, events, and the contact cache;
- collision support types (`shape_context`, `simplex`, `body_support`);
- cross-module function declarations.

Changes to this header often require coordinated changes in several `.c` files. In particular, body buffers are governed by the lifecycle described in [body-storage.md](body-storage.md).

## Module Ownership

| File | Primary responsibility |
|---|---|
| `src/core.c` | Default allocator/configuration, required-memory calculation, world initialization and teardown |
| `src/world.c` | Body creation/removal/access, handle mappings, body reordering, sleeping, integration, AABBs, and the simulation entry point |
| `src/shapes.c` | Power-of-two compound-shape bracket storage |
| `src/collision_detection.c` | AABB broad phase, analytic pair tests, support mappings, collision dispatch, and contact manifold/cache integration |
| `src/gjk.c` | Convex intersection test in Minkowski-difference space |
| `src/epa.c` | Penetration depth, normal, and witnesses after GJK intersection |
| `src/contacts.c` | Contact/cache storage, generation ordering, event emission, and manifold reduction |
| `src/contacts_resolution.c` | Iterative interpenetration and velocity/friction resolution |
| `src/joints.c` | Distance-joint storage and conversion to solver contacts |
| `src/queries.c` | Raycasts and spherical overlap queries |
| `src/mesh.c` | Convex-mesh validation, import/storage, mass properties, and mesh AABBs |
| `src/events.c` | Per-body subscriptions and per-frame linked event lists |
| `src/debug.c` | Debug-draw adapters and collision/EPA debug support |
| `src/math.c` | Matrix and rigid-body inertia operations |

## World Composition

`bnd_world_t` owns all library state:

- separate structure-of-arrays stores for dynamic and static bodies;
- frame contacts and persistent contact features;
- joints;
- imported meshes;
- per-frame events;
- compound-shape brackets;
- configuration, statistics, allocator, structural generation, and simulation age.

The public `bnd_world` remains opaque so internal storage can change without exposing layout through the ABI.

## Simulation Pipeline

`bnd_simulate` executes in this order:

1. Integrate force, impulse, velocity, angular momentum, rotation, and position for awake dynamics.
2. Recalculate AABBs for awake dynamics and dirty statics.
3. Reset frame contacts and events.
4. Generate dynamic-dynamic collision contacts, then dynamic joint constraints.
5. Generate dynamic-static collision contacts, then static joint constraints.
6. Prepare and iteratively resolve penetration, velocity, restitution, and friction.
7. Repartition awake and sleeping dynamics.
8. Clear transient force/impulse/acceleration buffers.
9. Prune aged cached contact features and advance world age.

Ordering is structural. AABBs depend on post-integration transforms; events expose only collision contacts generated before joint rows; the solver uses the dynamic-contact boundary to decide whether body B is movable.

## Collision Architecture

The broad phase is a pairwise AABB scan over dynamic-dynamic and dynamic-static pairs. Each overlapping body pair expands into all combinations of its constituent shapes.

The narrow phase uses:

- analytic tests for selected primitive pairs and shape-plane pairs;
- support mappings plus GJK for convex intersection;
- EPA for penetration data after GJK;
- a dispatch table that canonicalizes shape order and records whether the result must be inverted;
- a feature cache and four-contact manifold reduction for stable multi-frame contact sets.

See [contacts.md](contacts.md) for contact ordering and orientation invariants.
