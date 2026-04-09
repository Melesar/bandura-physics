# `ccd` Rewrite Review: Moving From Heap Allocations To Arena/Preallocated Memory

## Executive Summary

The library is already split into two very different memory models:

- GJK and MPR are effectively stack-only and do not need a rewrite for your allocation policy.
- EPA/polytope handling is the part that allocates dynamically. It creates and destroys graph nodes (`vertex`, `edge`, `face`) during the query.

The important consequence is that a straight `malloc -> arena_alloc` substitution is not enough. The current EPA code assumes:

- individual object allocation,
- individual object deletion,
- stable pointers between connected graph nodes,
- the ability to fail part-way through a topology rewrite and then just abandon the whole temporary polytope.

If you want an arena-based model that still behaves well, the safest redesign is:

1. Preallocate a query workspace at setup time.
2. Make EPA use that workspace explicitly.
3. Replace raw heap object lifetime with either:
   - a monotonic scratch arena plus whole-query reset, or
   - fixed-capacity pools/freelists allocated from an arena during setup.

For this codebase, the best fit is usually a hybrid:

- allocate the backing storage once at setup,
- reset it per query,
- recycle `vertex`/`edge`/`face` slots within the query via freelists.

That preserves your “no dynamic allocations after setup” rule without making EPA capacity explode on long runs.

## Where The Current Code Allocates

### Global allocation abstraction

All allocation macros are hard-wired to `realloc()` in [`ccd/alloc.h`](./alloc.h):

- `CCD_ALLOC(type)` at `ccd/alloc.h:36`
- `CCD_ALLOC_ARR(type, num_elements)` at `ccd/alloc.h:40`

There is no allocator hook or workspace object in the public API today.

### EPA/polytope node allocation

The actual per-query heap allocations happen here:

- `ccdPtAddVertex()` allocates one `ccd_pt_vertex_t` in [`ccd/polytope.c`](./polytope.c):101
- `ccdPtAddEdge()` allocates one `ccd_pt_edge_t` in [`ccd/polytope.c`](./polytope.c):131
- `ccdPtAddFace()` allocates one `ccd_pt_face_t` in [`ccd/polytope.c`](./polytope.c):167

Those nodes are individually freed here:

- `ccdPtDelVertex()` in [`ccd/polytope.h`](./polytope.h):187
- `ccdPtDelEdge()` in [`ccd/polytope.h`](./polytope.h):204
- `ccdPtDelFace()` in [`ccd/polytope.h`](./polytope.h):226
- `ccdPtDestroy()` walks all lists and frees everything in [`ccd/polytope.c`](./polytope.c):74

### Temporary allocation in penetration position computation

`penEPAPos()` allocates a temporary array of vertex pointers, sorts it with `qsort()`, then frees it:

- allocation in [`ccd/ccd.c`](./ccd.c):129
- free in [`ccd/ccd.c`](./ccd.c):150

This is the only non-polytope dynamic allocation I found.

### What does not allocate

These paths are already compatible with your policy:

- `ccd_simplex_t` is a fixed-size stack object in [`ccd/simplex.h`](./simplex.h):24
- GJK uses that fixed simplex only in [`ccd/ccd.c`](./ccd.c):186
- MPR works entirely on a fixed simplex as well in [`ccd/mpr.c`](./mpr.c)

## Key Design Decisions In The Current Author’s Implementation

These are the main author choices that matter for your rewrite.

### 1. EPA polytope is a mutable pointer graph, not an indexed mesh

The polytope stores:

- intrusive lists of all vertices/edges/faces,
- edge -> vertex references,
- edge -> face references,
- vertex -> list of incident edges.

See the type layout in [`ccd/polytope.h`](./polytope.h):38-92.

This is a very pointer-heavy design. It is convenient for heap allocation, but it is not the most natural fit for a fixed-capacity scratch workspace. In your project, index handles are likely a better long-term representation than raw pointers.

### 2. Topology rewrites rely on delete-and-recreate

`expandPolytope()` does not mutate the hull minimally. It removes old faces/edges and allocates replacement objects:

- edge expansion path in [`ccd/ccd.c`](./ccd.c):777-865
- face expansion path in [`ccd/ccd.c`](./ccd.c):867-900

This is the core gotcha for an arena rewrite. A pure bump arena cannot reclaim those deleted nodes during the query.

### 3. Failure handling assumes “temporary structure, throw it away”

Several constructors build partial topology and only check for failure at the end:

- `simplexToPolytope4()` in [`ccd/ccd.c`](./ccd.c):564-585
- `simplexToPolytope3()` in [`ccd/ccd.c`](./ccd.c):639-666
- `simplexToPolytope2()` in [`ccd/ccd.c`](./ccd.c):736-770

This works because on `-2` the caller abandons the polytope and later destroys it. That is acceptable for heap allocations and still acceptable for a per-query scratch workspace, but it is not compatible with “partially mutate a long-lived structure and continue”.

### 4. Nearest-element tracking is cached but invalidated by deletion

`ccd_pt_t` caches the nearest element. Deletes clear the cache if the deleted element was cached, and a later call rescans all lists:

- nearest renew in [`ccd/polytope.c`](./polytope.c):30-59
- cache invalidation in delete helpers in [`ccd/polytope.h`](./polytope.h):196, [`ccd/polytope.h`](./polytope.h):218, [`ccd/polytope.h`](./polytope.h):243

If you switch to tombstones or inactive slots, the nearest scan must skip dead entries.

### 5. EPA iteration count is not explicitly bounded

`__ccdGJKEPA()` loops until `nextSupport()` says no more progress:

- main EPA loop in [`ccd/ccd.c`](./ccd.c):271-282

That is numerically reasonable, but it is a poor fit for fixed-capacity memory unless you introduce an explicit EPA node/iteration budget.

### 6. `penEPAPos()` uses a sort-based temporary buffer

The penetration position code computes a heuristic from sorted vertices:

- [`ccd/ccd.c`](./ccd.c):118-152

For your rewrite, this is low risk compared with EPA topology, but it still violates your allocation rule as written.

## Main Gotchas For The Rewrite

### Gotcha 1: A pure bump arena is not enough by itself

If you only replace `CCD_ALLOC()` with `arena_alloc()` and replace `free()` with no-op:

- deleted faces/edges/vertices will keep consuming slots,
- repeated EPA expansion can exhaust the arena even when the active hull stays small,
- nearest scans can accidentally see stale nodes unless you unlink or mark them inactive.

That means you need one of:

- per-query arena reset plus large enough capacity for all temporary churn, or
- per-query fixed-capacity pools with freelist reuse.

For this code, freelist reuse is the more robust choice.

### Gotcha 2: `expandPolytope()` currently mutates before it knows replacement allocation succeeded

Example:

- old faces are deleted first in [`ccd/ccd.c`](./ccd.c):840-844 and [`ccd/ccd.c`](./ccd.c):887-888
- new edges/faces are allocated afterward

With the current API that is acceptable because failure aborts the whole query. If you want stronger invariants, redesign expansion as:

1. reserve all needed new slots,
2. only after successful reservation unlink old topology,
3. publish new topology,
4. otherwise roll back using a workspace mark.

That is cleaner for arena-style code and easier to reason about in debug builds.

### Gotcha 3: Pointer identity is deeply baked in

Many branches compare addresses to determine topology relationships. For example:

- edge/face sorting logic in [`ccd/ccd.c`](./ccd.c):799-835
- face vertex recovery in [`ccd/polytope.c`](./polytope.c):176-186

If you move to index handles, you must preserve the same logical identity rules. This is a good change, but it is not mechanical.

### Gotcha 4: Capacity planning is impossible without introducing limits

The current code can keep expanding until tolerances stop it. With no heap fallback, you need an explicit answer to:

- maximum active vertices?
- maximum active edges?
- maximum active faces?
- maximum EPA expansions per query?

Without those limits, arena exhaustion just becomes a runtime failure you cannot size confidently.

### Gotcha 5: Touching-contact and degenerate paths still build temporary topology

The 2-simplex and 3-simplex conversion code has special touching-contact branches:

- triangle touching case in [`ccd/ccd.c`](./ccd.c):625-636
- segment touching case in [`ccd/ccd.c`](./ccd.c):727-734

Those cases still allocate nodes today. If you want the rewrite to be clean, they must use the same workspace path as the general EPA code, not a special heap fallback.

### Gotcha 6: `ccdPtDestroy()` semantics should probably disappear from the hot path

Today destruction means walking lists and freeing every node. In an arena model, the hot-path equivalent should become:

- unlink/reset lists,
- clear counters,
- optionally reset a workspace marker.

If you keep the old destroy semantics and merely stub out `free()`, you will preserve work you no longer need.

## Recommended Allocation Model

Do not implement EPA on top of a pure “allocate-only, never reuse until end-of-query” arena unless you are comfortable with overprovisioning heavily.

The better model is:

### Setup-time allocation

Allocate once during program setup:

- `max_vertices` slots
- `max_edges` slots
- `max_faces` slots
- optional scratch arrays for sorting / temporary index lists

Those backing arrays can come from your engine arena during setup.

### Per-query execution

For each collision query:

- reset the workspace counts/freelists,
- run GJK using stack-only simplex,
- if EPA is needed, allocate polytope nodes from the workspace pools,
- when nodes are deleted during expansion, return them to per-query freelists,
- when the query finishes, reset the whole workspace.

This gives you:

- no OS allocations after setup,
- deterministic memory use,
- bounded failure mode (`workspace exhausted`),
- stable internal storage during the query.

### Why not just use a single monotonic arena?

Because EPA does real delete/create churn inside one query. The active hull size and the total number of allocations are not the same thing. A monotonic arena tracks total churn, not active topology.

## Concrete API Shape I Would Use

Add an explicit workspace object instead of hiding allocator state inside `ccd_t`.

Suggested direction:

```c
typedef struct {
    ccd_pt_vertex_t *vertices;
    ccd_pt_edge_t   *edges;
    ccd_pt_face_t   *faces;

    uint32_t max_vertices;
    uint32_t max_edges;
    uint32_t max_faces;

    uint32_t vertex_count;
    uint32_t edge_count;
    uint32_t face_count;

    uint32_t free_vertex_head;
    uint32_t free_edge_head;
    uint32_t free_face_head;

    ccd_pt_vertex_t **sorted_vertices_tmp;
    uint32_t sorted_vertices_tmp_cap;
} ccd_workspace_t;
```

Then expose query variants that require it:

```c
int ccdGJKSeparateWithWorkspace(
    const void *obj1,
    const void *obj2,
    const ccd_t *ccd,
    ccd_workspace_t *ws,
    ccd_vec3_t *sep);

int ccdGJKPenetrationWithWorkspace(
    const void *obj1,
    const void *obj2,
    const ccd_t *ccd,
    ccd_workspace_t *ws,
    ccd_real_t *depth,
    ccd_vec3_t *dir,
    ccd_vec3_t *pos);
```

Reasons for keeping workspace out of `ccd_t`:

- `ccd_t` is algorithm configuration, not temporary state.
- separate workspaces make parallel queries possible.
- ownership becomes explicit.
- it matches your “setup allocates, runtime reuses” philosophy.

## What I Would Change In The Internal Design

### Option A: Minimal rewrite, preserve most existing code

Keep the current struct shapes and intrusive lists, but change allocation like this:

- replace `CCD_ALLOC()` calls with `ccdWsAllocVertex/Edge/Face()`,
- replace `free()` in delete helpers with `ccdWsFreeVertex/Edge/Face()`,
- store a back-pointer to `ccd_workspace_t` in `ccd_pt_t`,
- replace `penEPAPos()` temp allocation with a workspace scratch array.

This is the shortest path and likely the fastest way to get working code.

The main downside is that you keep the pointer-heavy graph and its complexity.

### Option B: Better rewrite, move to index-based fixed-capacity pools

Represent nodes by indices, not raw pointers:

- vertices/edges/faces live in arrays,
- references are integer handles,
- delete means “return slot to freelist”,
- invalid handles are easy to assert on,
- debug instrumentation becomes easier.

This is more work, but it is a better long-term fit for a deterministic physics codebase.

For your project philosophy, this is the design I would prefer unless you need a very small diff.

## How To Convert The Existing Allocation Model

## Step 1: Introduce a workspace and thread it through EPA code

Make `ccd_pt_t` hold workspace access:

```c
typedef struct {
    ccd_list_t vertices;
    ccd_list_t edges;
    ccd_list_t faces;

    ccd_pt_el_t *nearest;
    ccd_real_t nearest_dist;
    int nearest_type;

    ccd_workspace_t *ws;
} ccd_pt_t;
```

Change `ccdPtInit()` to accept the workspace.

## Step 2: Replace raw alloc/free helpers with typed pool helpers

Instead of:

```c
vert = CCD_ALLOC(ccd_pt_vertex_t);
free(v);
```

use:

```c
vert = ccdWsAllocVertex(pt->ws);
ccdWsFreeVertex(pt->ws, v);
```

The alloc helpers should:

- pop from freelist if available,
- otherwise use the next unused slot,
- return `NULL` on exhaustion.

The free helpers should:

- unlink the node from intrusive lists first,
- poison/debug-clear it in debug builds,
- push the slot back to the freelist.

## Step 3: Remove `realloc`-style allocator abstraction from the hot path

`ccd/alloc.h` is not the right abstraction anymore. For your rewrite:

- keep it only for setup-time helpers if needed, or
- remove it from EPA internals entirely.

The current macro layer is too weak because it cannot express:

- typed pool allocation,
- per-query reset,
- freelist recycling,
- capacity checks.

## Step 4: Replace `penEPAPos()` temporary heap buffer

Rewrite [`ccd/ccd.c`](./ccd.c):118-152 to use:

- a preallocated `ccd_pt_vertex_t *tmp[]` buffer in the workspace, or
- an index array if you move to handle-based topology.

If the vertex count exceeds the temp capacity, return a deterministic failure code.

## Step 5: Add explicit EPA limits

Add at least one of:

- `ccd->epa_max_iterations`
- `ws->max_vertices / max_edges / max_faces`

I would add both. The current code has a bounded GJK loop but not a bounded EPA expansion loop.

## Step 6: Decide your failure policy

You need a project-level answer for workspace exhaustion:

- return `-2` like current allocation failure,
- optionally emit a debug trace,
- reset the query workspace and fail deterministically.

That preserves existing caller expectations while removing runtime heap usage.

## The Single Biggest Design Choice

The most important choice is this:

- Do you want a monotonic arena per query?
- Or do you want fixed-capacity pools/freelists per query?

For this library, I recommend fixed-capacity pools/freelists allocated from an arena during setup.

Reason:

- EPA deletes and recreates nodes inside one query.
- That makes freelist reuse naturally match the algorithm.
- A pure bump arena is simple, but it turns algorithmic churn into memory pressure.

If you insist on a pure arena with no within-query reuse, then you should also rewrite EPA so that expansion is append-only or rebuild-based. That is a substantially larger algorithmic rewrite than the current code needs.

## Suggested Rewrite Order

1. Leave GJK and MPR alone.
2. Introduce `ccd_workspace_t`.
3. Convert `ccdPtAdd*` / `ccdPtDel*` to workspace-backed pools.
4. Convert `penEPAPos()` temp allocation to workspace scratch.
5. Add EPA capacity/iteration limits.
6. Add debug validation for hull consistency after each expansion.
7. Only then consider a second-pass refactor to handle/index-based topology.

## Validation Checklist

After the rewrite, I would specifically test:

- repeated `ccdGJKPenetration()` calls with the same workspace reset each frame,
- worst-case deep penetration cases that force many EPA expansions,
- touching contacts where simplex size is 2 or 3,
- degenerate shapes that trigger zero-area / zero-volume branches,
- deterministic failure when workspace capacity is intentionally too small,
- parity against the current implementation for depth/direction/position.

## Bottom Line

The library does not need a full algorithmic rewrite everywhere. The heap dependence is concentrated in EPA polytope management and one small temporary array.

The part you should change is not just “which allocator function gets called”. The real rewrite is changing EPA from:

- heap-allocated mutable graph with individual frees

to:

- setup-time preallocated workspace with per-query reset and in-query slot reuse.

That is the design that matches your project philosophy without fighting the algorithm on every expansion step.
