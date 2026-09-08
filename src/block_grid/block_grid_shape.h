// SPDX-License-Identifier: MIT
//
// Engine-side adapters for BlockGrid shapes. Each one narrows the
// field with a grid query, then defers to the same convex function any other
// shape would use, so results stay identical to an equivalent compound.
//
// Every adapter takes its scratch from the caller. The world owns that scratch
// and sizes it for the largest attached grid when a grid shape is attached or
// replaced, so no query and no step allocates.

#pragma once

#include "block_grid.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <stdbool.h>

typedef struct v3BlockGridData v3BlockGridData;

// One candidate Hitbox of a sweep, kept as the ordering pair the traversal sorts.
typedef struct v3BlockGridSweepCandidate
{
	// Fraction of the sweep below which the fast shape provably cannot touch this
	// Hitbox. Conservative: it never exceeds the true first contact fraction.
	float entryBound;
	int hitboxIndex;
} v3BlockGridSweepCandidate;

// World-owned scratch for every BlockGrid query and sweep. Sized by the largest
// attached grid: one visited bit per Hitbox for the query walk, one candidate
// entry per Hitbox for the sweep ordering.
typedef struct v3BlockGridScratch
{
	v3BlockGridQueryScratch query;
	v3BlockGridSweepCandidate* candidates;
	int candidateCapacity;
} v3BlockGridScratch;

// Grows the scratch so it serves any query or sweep against this grid. Called when a
// grid shape is attached or replaced, never inside a step or a query. Returns false
// only when the allocator fails, leaving the previous capacity intact.
bool v3BlockGridScratchReserve( v3BlockGridScratch* scratch, const v3BlockGridData* grid );
void v3BlockGridScratchDestroy( v3BlockGridScratch* scratch );

// True when the scratch is large enough to answer for this grid.
bool v3BlockGridScratchFits( const v3BlockGridScratch* scratch, const v3BlockGridData* grid );

typedef struct v3BlockGridTOIResult
{
	b3TOIOutput output;
	int hitboxIndex;

	// The cap stopped the traversal while an unvisited candidate could still be
	// reached before the best impact found, so the sweep has no answer. The caller
	// must hold the fast body at holdFraction with its velocity untouched rather
	// than accept output, which may describe a later Hitbox than the one skipped.
	bool capExhausted;
	float holdFraction;
} v3BlockGridTOIResult;

b3AABB v3ComputeBlockGridAABB( const v3BlockGridData* grid, b3Transform transform );

b3BoxHull v3MakeBlockGridHitboxHull( const v3BlockGridData* grid, int hitboxIndex );

// Mass properties from logical unit cells, independent of collision culling and merging.
b3MassData v3ComputeBlockGridMass( const v3BlockGridData* grid, float density );

b3CastOutput v3RayCastBlockGrid( const v3BlockGridData* grid, const b3RayCastInput* input, v3BlockGridScratch* scratch );
b3CastOutput v3ShapeCastBlockGrid( const v3BlockGridData* grid, const b3ShapeCastInput* input, v3BlockGridScratch* scratch );
bool v3OverlapBlockGrid( const v3BlockGridData* grid, b3Transform transform, const b3ShapeProxy* proxy,
						 v3BlockGridScratch* scratch );
int v3CollideMoverAndBlockGrid( b3PlaneResult* planes, int planeCapacity, const v3BlockGridData* grid, const b3Capsule* mover,
								v3BlockGridScratch* scratch );

void v3QueryBlockGridPairs( const v3BlockGridData* grid, b3AABB localAABB, b3TreeQueryCallbackFcn* callback, void* context,
							v3BlockGridScratch* scratch );

// Continuous collision of one fast convex shape against one BlockGrid.
//
// Candidates come out of the AABB query as (entry lower bound, Hitbox index) pairs,
// are sorted ascending with the Hitbox index as the tie-break, and are evaluated in
// that order. The traversal stops at the first candidate whose bound is at or beyond
// the best impact found, because nothing after it can be reached sooner.
//
// candidateCap bounds the number of evaluations; zero means unlimited. Reaching it
// while an unvisited candidate is still reachable sooner than the best impact sets
// capExhausted with holdFraction at that candidate's bound.
v3BlockGridTOIResult v3TimeOfImpactBlockGrid( const v3BlockGridData* grid, const b3TOIInput* input, b3AABB localBounds,
											  b3Vec3 localCentroidB, float fallbackRadius, v3BlockGridScratch* scratch,
											  int candidateCap );
