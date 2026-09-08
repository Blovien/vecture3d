# Large Worlds (Double Precision) {#large-worlds}

Vecture3D always uses double precision world positions. At a coordinate of 1e7 meters,
a float has a step of about one meter, so it cannot retain small changes in absolute
position. Double precision world coordinates preserve those changes.

Velocities, forces, local shapes, contact manifolds, solver calculations and the broad
phase tree retain float precision. Positions are subtracted in double precision before
local deltas are converted to floats.

There is no precision build option or required compiler definition. The public headers
and library always use the same world position types. `b3IsDoublePrecision()` remains
available for compatibility and always returns true. The existing
`b3CreateWorldDoublePrecision` symbol is retained, including its `b3CreateWorld` alias,
so applications cannot accidentally link these headers to an older float library.

## The two world-position types

World coordinates use two distinct types:

- `b3Pos` stores a world position as three doubles.
- `b3WorldTransform` combines a `b3Pos` translation with a float `b3Quat` rotation.

`b3Vec3`, `b3Quat`, `b3Transform` and `b3AABB` retain float components.
`b3Transform` describes local and relative frames. `b3WorldTransform` describes world
space. Public body positions, world transforms, query origins and world hit points use
the corresponding world types.

`b3Pos` and `b3Vec3` are distinct structs. Callers converting between absolute positions
and local vectors use these helpers:

```c
b3Pos  p = b3ToPos( v );        // float vector -> world position
b3Vec3 v = b3ToVec3( p );       // world position -> float vector (lossy far from origin)
b3Vec3 d = b3SubPos( a, b );    // a - b, demoted to float (the precision boundary)
b3Pos  q = b3OffsetPos( p, d ); // p + d
float      x = b3RoundDownFloat( p.x );  // conservative narrowing, pair with b3RoundUpFloat to
                                         // build a float box that always contains double bounds
```

## Operating range

Body integration, contact solving, joints and continuous collision use local float
calculations around double precision world positions. The large world tests compare
stack settling, bullet collision and queries at the origin and at 1e7 meters.

The practical limit comes from the broad phase, which stays float and stores conservative
(outward-rounded) float bounds. Far from the origin the float bound quantization grows: about one
meter at 1e7, sixteen meters at 1e8. Overlapping shapes always still produce a pair, so correctness
is preserved, but beyond roughly 1e7 to 1e8 meters the broad phase reports extra false pairs and
loses some margin hysteresis, which costs performance. Stay within about ±1e7 to ±1e8 meters.

## Queries far from the origin

Every spatial query takes a caller supplied `b3Pos` origin and re-differences each shape against a
nearby base at full precision, so hit points and fractions stay accurate far from the world origin.
The one shared limit is the broad phase: the tree is traversed in conservative outward rounded float,
so it never misses a pair, but a cast that grazes a shape by less than a coordinate float ULP far
from the origin can still miss at the tree level. Only the explosion is a pure float carve-out.

- `b3World_OverlapShape`, `b3World_CastShape`, `b3World_CastMover`, and `b3World_CollideMover` take a
  `b3Pos` origin. Their proxy, mover, and returned planes are relative to that origin and each shape
  is re-differenced against it in float, so a query around a mover at 1e7 is as precise as one at the
  origin. Shape cast hit points come back as `b3Pos`.
- `b3World_CastRay` / `b3World_CastRayClosest` take a `b3Pos` origin and re-difference each shape
  against its body in full precision, so hit points and fractions stay accurate far from the origin.
  The tree traversal itself is float (see the broad phase limit above). Hit points come back as
  `b3Pos`.
- `b3Shape_RayCast` takes a `b3Pos` origin and a translation and returns a `b3WorldCastOutput` whose
  hit point is a world `b3Pos`, re-centered on the origin for full precision.
- `b3World_Explode` resolves the per-shape impulse in float around the explosion position.

The character controller (`b3World_CastMover` / `b3World_CollideMover`) drives this: pass the
character's world position as the origin and the mover stays precise even at 1e7. The only remaining
float carve-out is the broad phase tree traversal, which Box2D shares.

## Debug drawing

`b3World_Draw` hands every callback world coordinates in the double-capable `b3Pos` and
`b3WorldTransform` types, so the engine stays camera agnostic. The host shifts into its own camera
frame inside the callbacks: keep a draw origin near the camera and difference against it in double
before the coordinates demote to float, and a distant scene draws crisply instead of snapping to the
coarse float grid around the absolute origin. A zero origin reproduces absolute coordinates and leaves
a near-origin scene unchanged. The sample app sets the draw origin from the camera eye each frame, and
the Large World sample uses it to render a stack at 1e7 with no jitter.

## Determinism

`DeterminismTest` checks the existing double precision sleep steps and state hashes
across worker counts. Recordings retain the double precision wire format. Snapshot
loading rejects records marked as single precision, as well as incompatible versions
and layouts.

## SIMD

Double precision is orthogonal to SIMD. The wide types used in the broad phase and mesh collision
stay 4-wide float, and the contact solver retains its float calculations. There is no double-precision
SIMD path and none is needed: the hot interior never sees an absolute world coordinate.
