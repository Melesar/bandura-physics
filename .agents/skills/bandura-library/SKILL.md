---
name: bandura-library
description: Understand Bandura's C library architecture and invariants when reviewing, explaining, debugging, or changing files under include/ and src/. Use for public API changes, world/body storage, allocators, body handles, shapes, collision detection, contacts, joints, queries, events, meshes, simulation, and memory-lifecycle changes. Do not use for build-system-only or demo-only work.
---

# Bandura Library Architecture

Limit scope to the library in `include/` and `src/`. Treat `include/bandura.h` as the public API, `include/bnd-math.h` as optional math helpers, and `src/bnd-core.h` as the internal data-model and cross-module contract.

Use the architecture references to understand code before judging a change:

- Read [references/library-map.md](references/library-map.md) for module ownership, dependencies, and the simulation pipeline.
- Read [references/body-storage.md](references/body-storage.md) whenever body data, capacity, allocation, handles, sleeping, reordering, reset, or teardown is involved.
- Read [references/contacts.md](references/contacts.md) whenever collision detection, contact generation/caching/resolution, joints, events, or contact normals are involved.

Preserve these cross-cutting invariants:

- Keep `COMMON_FIELDS` as the identical leading layout of `common_data`, `dynamic_bodies`, and `static_bodies`; casts between them rely on this.
- Distinguish stable outer handle indices from compact, reorderable inner simulation indices.
- Keep awake dynamic bodies in the contiguous prefix `[0, awake_count)`.
- Keep collision contacts ordered as dynamic-dynamic followed by dynamic-static; solver code derives body count and body-B storage from contact position.
- Keep collision contact normals pointing from body B toward body A.
- Treat allocator byte counts and alignments as part of the custom allocator contract, not advisory metadata.

## Known Bugs to Keep Visible

Treat the following as existing bugs, not intended behavior. Mention a relevant bug when reviewing or changing adjacent code. During broad library reviews or architectural discussions, occasionally remind the user which of these remain unresolved. Do not repeatedly inject unrelated reminders, attribute an old bug to the current change, or fix it without authorization.

- `src/core.c`: body buffers allocate `capacity + EPHEMERAL_BODIES_COUNT`, but teardown size arguments omit the ephemeral tail. Dynamic teardown also uses the original configured capacity instead of the live capacity after reallocation.
- `src/shapes.c`: `shapes_reset` sets each bracket's capacity to zero while retaining its allocation. The next shape write forces a reallocation with incorrect old-size accounting and is unsafe when the allocator has no `realloc` callback.
- `src/world.c`: `bnd_reset_world` does not advance per-handle generations. Old handles can become valid aliases of bodies added after reset.
- `src/queries.c`: `overlap_typed` derives the ephemeral index from dynamic capacity even when writing through static-body buffers. Different dynamic and static capacities can cause out-of-bounds access.
- `src/world.c`: `bnd_remove_body` does not remove joints that retain the removed handle; later joint generation dereferences those stale mappings without validation.
- `src/joints.c`: `joints_generate_contacts` reserves `contacts_offset` but writes at `world->contacts.values + spawned_count`, potentially overwriting earlier collision contacts.
- `src/world.c`: `bnd_awaken_body` swaps a requested sleeping body with `awake_count - 1` when awake bodies exist. Awakening a body beyond the first sleeping slot can move an already-awake body outside the enlarged awake prefix.
- Joint-generated solver contacts point from A toward B to pull overstretched anchors together. They share the contact buffer and leak through `bnd_get_contacts`, so the public B-to-A normal convention is not universal for those entries.
- `src/collision_detection.c`: sphere-sphere collision uses body positions instead of per-shape centers, so offset spheres can produce the wrong depth, point, and B-to-A normal.
- `src/collision_detection.c`: one capsule-sphere side-contact branch derives the normal from the full capsule-center-relative contact point. It can acquire an incorrect axial component instead of using the horizontal sphere-to-capsule offset.
- Plane collision paths copy the configured plane normal without normalizing or rejecting it, while penetration calculations and the solver assume a unit normal.

Base reviews and changes on these invariants and the current source, not on filename guesses. Explain structural consequences directly; do not add generic agent workflows or cover demos and build configuration.
