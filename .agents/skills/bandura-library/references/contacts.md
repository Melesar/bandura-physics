# Contacts, Constraints, and Normal Orientation

## Contact Ownership and Ordering

Every internal `contact` stores inner indices. Body A is always dynamic. Body B is dynamic or static according to which contiguous section contains the contact:

```text
[0, contacts.dynamic_count)              dynamic-dynamic
[contacts.dynamic_count, contacts.count) dynamic-static
```

The solver, public contact conversion, and wake propagation infer body count and body-B storage from this boundary. Reordering generation stages or inserting another contact category without preserving the boundary changes their meaning.

`contacts_generate` intends this sequence:

1. dynamic-dynamic collision contacts;
2. dynamic-dynamic joint constraints;
3. dynamic-static collision contacts;
4. dynamic-static joint constraints.

Collision events are emitted immediately after each collision-detection pass, before joint rows are appended.

## Collision Normal Convention

For collision contacts, always define:

```text
normal = unit direction from body B toward body A
```

Consequences:

- position correction moves A along `+normal` and movable B along `-normal`;
- velocity resolution initially applies impulse along `+normal` to A, then negates it for B;
- plane collisions use the plane's outward normal because the plane is body B;
- sphere-sphere uses `center_a - center_b`;
- public `bnd_contact.body_a`, `body_b`, and `normal` preserve this relationship;
- collision events preserve the same A/B ordering and normal even when delivered to body B.

Do not flip the event normal per subscriber. Consumers can negate it themselves when they need the normal relative to body B.

The convention describes intended collision-contact semantics, not a guarantee that every current narrow-phase implementation is correct. Known violations include sphere-sphere ignoring shape offsets and a capsule-sphere branch introducing an axial normal component. Plane paths also rely on callers supplying a unit normal even though the API does not enforce or document that requirement.

## Dispatch Inversion

The collision dispatch table chooses a primary orientation for each shape pair. When a pair arrives in the opposite shape order, `ctx_inverse` swaps bodies and shapes for the narrow-phase function. After detection, the caller restores the canonical body indices and negates:

- `contact.normal`;
- cached witness A/B;
- cached feature normal.

Any new shape-pair entry must set its `primary` flag consistently with the orientation expected by its function.

## GJK, EPA, and Cached Features

The support mapping constructs the Minkowski difference `supportA(direction) - supportB(-direction)`. GJK determines whether that difference encloses the origin. EPA obtains the nearest polytope face and negates its face normal when producing the collision normal, yielding B-to-A orientation.

Cacheable contacts retain:

- witness point on A;
- witness point on B;
- B-to-A normal.

Before storage, witnesses are transformed into each body's local frame and the normal into body A's local frame. Reconstruction returns all three to world space. A cached feature is retained while its witnesses remain close enough and its separation along the reconstructed normal stays below the configured threshold.

At most four contacts survive per body pair. When more exist, manifold reduction preserves the deepest point and selects the four-point set with the largest projected surface area, using total depth as a tie-breaker.

## Solver Interpretation

`prepare_contacts` builds a contact-space basis with the normal as its Y axis and computes relative velocity as velocity A minus velocity B. Resolution is iterative:

- choose the greatest remaining penetration, correct positions/rotations, and update every affected contact depth;
- choose the greatest desired velocity correction, apply normal/friction impulses, and update every affected contact velocity.

`resolution_attempts_factor * contacts.count` bounds each phase. The contact order boundary decides whether one or two inverse masses/inertias participate.

## Joint Constraint Exception

Distance joints are lowered into the same `contact` representation, but they correct overstretch rather than penetration. Their generated normal currently points from anchor A toward anchor B so `+normal` pulls A toward B and `-normal` pulls B toward A.

Treat these as internal constraint rows, not as examples for collision-normal construction. They currently share debug drawing and `bnd_get_contacts`, which exposes an inconsistency with the public B-to-A collision convention.

Two additional known defects affect joint rows:

- `joints_generate_contacts` receives `contacts_offset` and reserves capacity for that region but writes from the start of `world->contacts.values`, overwriting prior rows.
- Body removal leaves joints containing stale handles, and joint generation resolves them without calling `bnd_handle_valid`.

## Known Collision-Normal Defects

- `sphere_sphere_collision` reads `data->positions` rather than `body_a_center` and `body_b_center`. Compound or otherwise offset sphere shapes therefore use body origins for separation and normal construction.
- In the capsule-sphere side-overlap case where the sphere center is outside the capsule radius but the shapes overlap, `closest` includes the sphere's local Y coordinate before `normal = normalize(-closest)`. The B-to-A direction should come from the rotated negative horizontal offset; the current value tilts toward the capsule origin when the sphere is above or below its local midplane.
- Shape-plane routines use `shape.value.plane.normal` directly for signed distance, depth, contact normal, and solver basis. A non-unit configured normal scales penetration and violates the solver's unit-normal assumption.
- Degenerate coincident centers require an arbitrary fallback direction; do not classify those mathematically undefined cases as orientation bugs. EPA's invalid-contact fallback likewise emits explicitly bogus placeholder data rather than a reliable geometric normal.
