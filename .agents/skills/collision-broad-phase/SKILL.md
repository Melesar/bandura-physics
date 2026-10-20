---
name: collision-broad-phase
description: Work on or review Bandura's AABB broad-phase collision contacts, including compound-shape contact chains, contact hashing, free-list lifecycle, and broad-contact storage. Use for run_broad_phase, broad_phase_contact, broad_contacts_set, or broad contact hash-table changes; not for narrow-phase geometry alone.
---

# Collision Broad Phase

Use this skill for the persistent AABB broad-phase contact system in `src/collision_detection.c` and the supporting storage in `src/contacts.c` / `include/bnd-core.h`. It is deliberately narrower than `bandura-library`: load that skill too when a change also affects general body storage, public APIs, solver contacts, or narrow-phase collision behavior.

## Scope and reading

Before changing or reviewing this system, read:

- `broad_contacts_invariants.md` for the source contact-set contract;
- the `broad_phase_contact`, `broad_contacts_set`, and `contacts` declarations in `include/bnd-core.h`;
- `run_broad_phase_typed`, its list/hash helpers, and `contacts_init`, `contacts_reset`, and `hash_table_resize_if_needed`.

Do not conflate these persistent broad-phase contacts with narrow-phase manifolds or solver `contact` rows.

## Contact-set model

There are two independent `broad_contacts_set`s:

- `dynamics` contains dynamic--dynamic body pairs;
- `statics` contains dynamic--static body pairs.

Every `broad_phase_contact` represents one potentially overlapping *shape pair*. Several shape contacts may belong to one body pair:

- The first shape contact is the **body contact** (the root).
- Later shape contacts form a `next` chain from that root.
- `first` and `last` must point only to roots, never to attached shape contacts.
- `next_body` links roots only; attached contacts have no body-list role.
- All linked-list sentinels are `UINT32_MAX`.

When a root is removed but has an attached shape contact, promote the next shape contact to root: preserve its remaining `next` chain, transfer `next_body`, repair predecessor/`first`/`last`, and repoint the hash-table index. When removing a whole body pair, unlink the root from the body list before freeing every shape contact.

## Hash-table contract

`contacts.keys` and `contacts.indices` are parallel arrays shared by both contact sets. A table entry must point only to a body-contact root. Its shape contacts are reachable only through `next`.

- `0` is empty; `0x7FFFFFFFFFFFFFFF` is a tombstone; live keys have bit 63 set.
- `hash_table_create_key` includes the body type, stable outer indices, and generations. Dynamic--dynamic pairs must be canonicalized by stable outer index so their key survives inner-index reordering.
- Track `hash_table_entry_count` as the number of live body-pair entries: increment only on root creation, decrement only when the body pair is removed, and reset it with the table.
- Probing uses `capacity - 1` masking, so the configured broad hash capacity must be nonzero and a power of two. Keep `bnd_required_memory` consistent with any capacity normalization or validation.
- Hash resizing must rebuild entries by traversing each set's root `next_body` list, never by treating attached shape contacts as table entries.

## Allocation and free-list lifecycle

`next` is the next unallocated slot cursor; live storage is indexed from zero. Allocate from `free_list` first; otherwise allocate `next++`, resizing before `next == capacity` would write out of bounds.

Initialize and reset both contact sets consistently:

- `first = last = free_list = UINT32_MAX`;
- `next = 0`;
- clear every `keys` entry with `sizeof(uint64_t)`;
- set `hash_table_entry_count = 0`.

When freeing a contact, its `next` becomes the free-list link. Therefore save the original shape-chain successor *before* appending it to the free list, and never dereference `contacts[UINT32_MAX]` after reaching the end of a chain.

## Broad-phase update rules

The broad phase first checks body AABBs, then expands an overlapping body pair into all shape-pair AABB checks. Per-shape AABBs must use each shape's transformed offset and rotation.

- Dynamic--dynamic scanning must visit each unordered pair once; dynamic--static scans dynamic A against every static B.
- Run dynamic contacts before static contacts to preserve the downstream category boundary.
- On an overlapping shape pair, create a contact only when that exact `(shape_a, shape_b)` is absent. Do not duplicate existing contacts.
- On a non-overlapping shape pair, remove only the matching contact. Maintain the current body-pair existence and hash slot after creation, full deletion, or root promotion; nested shape loops continue after each of those transitions.
- If body AABBs cease to overlap, remove the entire body pair and all of its shape contacts.

## Known limitations to preserve visibly

- Collision-layer filtering currently continues before looking up/removing an existing broad contact. Changing a body's layer or disabling a layer pair can therefore leave stale broad contacts. The setters deliberately carry a TODO; do not describe this as resolved unless the invalidation path is implemented.
- `run_broad_phase` returns `bnd_error`; callers must preserve error handling when changing allocation or hash behavior.

## Review checklist

For any change, trace these cases on paper before declaring it sound:

1. First contact in an empty dynamic or static set.
2. Two overlapping shape pairs for one compound-body pair.
3. Removing an attached shape contact.
4. Removing a root with attached contacts (root promotion).
5. Removing the final shape contact for a body pair.
6. Removing a non-first body pair with multiple shape contacts.
7. Reusing freed slots and resizing both a contact set and the hash table.
8. World reset followed by first insertion.

