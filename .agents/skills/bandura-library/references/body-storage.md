# Body Storage and Lifecycle

## Contents

- [Layout domains](#layout-domains)
- [Buffer classification](#buffer-classification)
- [Capacity and allocation](#capacity-and-allocation)
- [Body creation](#body-creation)
- [Reordering and identity](#reordering-and-identity)
- [Removal](#removal)
- [Frame clearing and world reset](#frame-clearing-and-world-reset)
- [Teardown](#teardown)
- [Lifecycle coupling for new buffers](#lifecycle-coupling-for-new-buffers)

## Layout Domains

`COMMON_FIELDS` is a physical-layout contract. It is the first member sequence of `common_data`, `dynamic_bodies`, and `static_bodies`; `as_common` and direct casts depend on pointer compatibility at offset zero.

Body storage has two orthogonal index domains:

- **Inner index:** dense array position used by simulation data. It changes when bodies move, sleep, wake, or are removed.
- **Outer index:** stable public-handle identity. `outer_lookup[outer].index` resolves to the current inner index; `inner_lookup[inner]` resolves back to the outer index.

`generations[outer]` invalidates a removed handle when its outer slot is reused. `free_list` stores reusable outer indices. `outer_lookup` nodes also form the ascending linked list used by typed body enumeration.

## Buffer Classification

### Common inner-indexed body data

These values describe a particular body and must move with it:

- `positions`
- `rotations`
- `shapes`
- `aabbs`
- `event_masks`
- `event_links`
- `inner_lookup` (the moved body's outer identity)

### Outer-indexed identity data

These values describe stable handle slots and must not be swapped or moved with inner body data:

- `generations`
- `outer_lookup`
- `free_list`
- `first_outer_node`

### Dynamic inner-indexed data

- persistent physical state: `inv_masses`, `velocities`, `angular_momenta`, `inv_inertia_tensors`, `motion_avgs`;
- derived state: `inv_intertias`, `accelerations`;
- frame accumulators: `forces`, `torques`, `impulses`, `angular_impulses`.

All dynamic arrays share the same inner index as the common prefix.

## Capacity and Allocation

`capacity` is the logical maximum number of real bodies before growth. Every common and dynamic body array is physically allocated for:

```text
total_capacity = capacity + EPHEMERAL_BODIES_COUNT
```

Real bodies use `[0, capacity)`. Temporary query bodies may use `[capacity, capacity + EPHEMERAL_BODIES_COUNT)`. The reserved tail must be included consistently in required-memory calculations, allocation, reallocation, and allocator size arguments during teardown.

`bnd_required_memory` sums each common and dynamic element size and adds explicit worst-case alignment padding. Its alignment count is currently a hand-maintained constant, so a new allocation can require both an element-size update and another alignment allowance.

`init_commons` initializes counters and allocates all common arrays. `bnd_init_internal` allocates dynamic-only arrays. `realloc_data` doubles the logical capacity and reallocates every corresponding array using old and new total capacities.

Zero logical capacity is unsupported by the current doubling loop: shifting zero never grows it.

## Body Creation

Common initialization writes position, rotation, shape-storage reference, event mask/link, and initial AABB. Dynamic initialization additionally writes every dynamic field, including zeroing frame accumulators and derived acceleration.

A newly created dynamic body must be awake. If sleeping bodies already exist, `insert_new_dynamic_body` first moves the body at `awake_count` to the previous end, inserts the new body at the awake/sleep boundary, repairs both lookup directions, then increments `awake_count` and `count`.

Static creation appends an inner slot and marks static AABBs dirty. Compound bodies store their actual `bnd_body_shape` values in `shape_brackets`; the body array stores only a bracket/offset/count descriptor.

## Reordering and Identity

`swap_bodies` handles general reordering for sleeping, waking, and removal:

- swap all common inner-indexed body arrays;
- swap every dynamic array for dynamic bodies;
- repair `outer_lookup` through the swapped `inner_lookup` values;
- increment `world->generation` to invalidate active enumerators.

`move_body` copies a dynamic body into an unused inner slot when insertion must preserve the awake prefix. It copies every common per-body field except lookup metadata, plus every dynamic field. The caller repairs lookup mappings.

Any new inner-indexed buffer omitted from either operation silently attaches its data to the wrong body after a reorder.

The awake partition is:

```text
[0, awake_count)       awake dynamics
[awake_count, count)   sleeping dynamics
```

Only awake dynamics are integrated and have their AABBs refreshed every frame.

## Removal

`bnd_remove_body` performs these coupled state changes:

- validate the handle;
- increment `generations[outer]` and push the outer slot to `free_list`;
- release the body's compound-shape slot;
- compact the inner arrays while preserving the awake/sleep partition;
- decrement body and possibly awake counts;
- remove the outer node from the enumeration list;
- repair lookup mappings through `swap_bodies`;
- increment `world->generation`.

The removed slot's bytes are not cleared; counts and mappings make them unreachable. External structures containing body handles must either be removed or validate their handles before resolving inner indices.

## Frame Clearing and World Reset

After simulation, `clear_forces` zeroes forces, torques, impulses, angular impulses, and accelerations for all existing dynamics. Persistent velocity, momentum, transform, mass, and inertia state remains.

`bnd_reset_world` is a logical reuse operation, not deallocation:

- reset dynamic/static counts and free-list counts;
- reset `awake_count` and enumeration-list roots;
- reset age and selected statistics;
- clear logical contacts, shape occupancy, joints, and cached contact features;
- retain body-buffer allocations, imported meshes, allocator, configuration, and most stored bytes.

Because body generations are not advanced, reset currently fails to guarantee that pre-reset handles remain invalid.

## Teardown

`bnd_teardown` releases common arrays, dynamic arrays, auxiliary world stores, and finally the world object. The allocator API receives a byte size for every free. A size-sensitive allocator therefore requires the exact current allocation size, including capacity growth and ephemeral slots.

The current code violates this requirement for body arrays: common frees omit the ephemeral tail, and dynamic frees use configured initial capacity rather than live capacity. Preserve this as a known defect until it is explicitly fixed.

## Lifecycle Coupling for New Buffers

A common inner-indexed body buffer participates in all of these architectural locations:

- `COMMON_FIELDS` declaration and prefix layout;
- common element size in `bnd_required_memory` and alignment allowance;
- `init_commons` allocation;
- `realloc_data` common section;
- per-body initialization if old bytes cannot be inherited;
- `swap_bodies`;
- `move_body` when dynamic bodies carry the field;
- frame clear or world reset if its semantics require clearing;
- `teardown_commons` with live total capacity.

A dynamic-only buffer participates in:

- `dynamic_bodies` declaration;
- dynamic element size and alignment in `bnd_required_memory`;
- `bnd_init_internal` allocation;
- `realloc_data` dynamic section;
- `init_body_dynamic`;
- both `swap_bodies` and `move_body`;
- frame clearing/reset according to persistence;
- dynamic teardown with live total capacity.

Outer-indexed metadata follows allocation/reallocation/teardown but not inner-body initialization, swapping, or moving. Mixing these categories corrupts stable handles.
