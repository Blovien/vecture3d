// SPDX-License-Identifier: MIT
//
// Engine-side adapters for BlockGrid shapes.
//
// The grid itself knows nothing about shapes, bodies, or contacts: it answers
// AABB overlap, ray casts, and occupancy. This file turns those
// answers into the results the engine's shape paths expect, by resolving a
// candidate hitbox into an ordinary box hull and deferring to the existing
// convex functions. Doing it that way keeps the manifold, warm starting, and
// solver math identical to any other convex child.
//
// Query scratch is taken from the caller's lifetime, never from the payload, so
// the grid stays immutable and shareable between shapes and threads. The caller is
// the world: it sizes one scratch per worker and one for its own query entry points
// when a grid shape is attached or replaced, so no query and no step allocates here.

#include "block_grid_shape.h"

#include "compound.h"
#include "core.h"
#include "math_internal.h"
#include "shape.h"
#include "block_grid.h"
#include "block_grid_internal.h"

#include "box3d/collision.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

bool v3BlockGridScratchFits( const v3BlockGridScratch* scratch, const v3BlockGridData* grid )
{
	if ( scratch == NULL || grid == NULL )
	{
		return false;
	}
	return scratch->query.visitedWordCapacity >= v3BlockGrid_GetQueryScratchWordCount( grid ) &&
		   scratch->candidateCapacity >= v3BlockGrid_GetHitboxCount( grid );
}

bool v3BlockGridScratchReserve( v3BlockGridScratch* scratch, const v3BlockGridData* grid )
{
	if ( scratch == NULL || grid == NULL )
	{
		return false;
	}

	int wordCount = v3BlockGrid_GetQueryScratchWordCount( grid );
	if ( wordCount > scratch->query.visitedWordCapacity )
	{
		uint64_t* words = b3TryAlloc( (size_t)wordCount * sizeof( uint64_t ) );
		if ( words == NULL )
		{
			return false;
		}

		// The query clears only the words it touched, so a fresh buffer starts zeroed
		// and stays that way between queries.
		memset( words, 0, (size_t)wordCount * sizeof( uint64_t ) );
		if ( scratch->query.visitedHitboxes != NULL )
		{
			b3Free( scratch->query.visitedHitboxes, (size_t)scratch->query.visitedWordCapacity * sizeof( uint64_t ) );
		}
		scratch->query.visitedHitboxes = words;
		scratch->query.visitedWordCapacity = wordCount;
	}

	// One candidate entry per Hitbox, so an unlimited cap can evaluate every candidate
	// the query returns and the list is never truncated.
	int hitboxCount = v3BlockGrid_GetHitboxCount( grid );
	if ( hitboxCount > scratch->candidateCapacity )
	{
		v3BlockGridSweepCandidate* candidates = b3TryAlloc( (size_t)hitboxCount * sizeof( v3BlockGridSweepCandidate ) );
		if ( candidates == NULL )
		{
			return false;
		}
		if ( scratch->candidates != NULL )
		{
			b3Free( scratch->candidates, (size_t)scratch->candidateCapacity * sizeof( v3BlockGridSweepCandidate ) );
		}
		scratch->candidates = candidates;
		scratch->candidateCapacity = hitboxCount;
	}

	return true;
}

void v3BlockGridScratchDestroy( v3BlockGridScratch* scratch )
{
	if ( scratch == NULL )
	{
		return;
	}

	if ( scratch->query.visitedHitboxes != NULL )
	{
		b3Free( scratch->query.visitedHitboxes, (size_t)scratch->query.visitedWordCapacity * sizeof( uint64_t ) );
	}
	if ( scratch->candidates != NULL )
	{
		b3Free( scratch->candidates, (size_t)scratch->candidateCapacity * sizeof( v3BlockGridSweepCandidate ) );
	}
	*scratch = (v3BlockGridScratch){ 0 };
}

// Builds the box hull for one hitbox, already placed at its own centre.
b3BoxHull v3MakeBlockGridHitboxHull( const v3BlockGridData* grid, int hitboxIndex )
{
	b3Vec3 center, halfExtent;
	v3BlockGrid_GetHitboxBounds( grid, hitboxIndex, &center, &halfExtent );
	return b3MakeAxisAlignedBoxHull( halfExtent, center );
}

// Logical cells define mass independently of collision culling and merging. Each cell
// is a unit cube of uniform density. The center of mass is the mean cell center, and
// inertia sums each cube's tensor and its parallel-axis contribution.
//
// Calculations use normalized stored cells. The returned centroid restores the cook
// origin to match the caller's local frame. Inertia about the centroid is translation invariant.
b3MassData v3ComputeBlockGridMass( const v3BlockGridData* grid, float density )
{
	b3MassData massData = { 0 };
	if ( grid == NULL || grid->blockCount <= 0 || density <= 0.0f )
	{
		return massData;
	}

	const v3BlockGridBlockData* blocks = v3BlockGridBlocks( grid );
	int blockCount = grid->blockCount;

	// A block at lattice (x, y, z) spans that unit cell, so its centre is offset
	// by half a block on each axis.
	b3Vec3 sum = b3Vec3_zero;
	for ( int i = 0; i < blockCount; ++i )
	{
		sum.x += (float)blocks[i].x + 0.5f;
		sum.y += (float)blocks[i].y + 0.5f;
		sum.z += (float)blocks[i].z + 0.5f;
	}

	float inverseCount = 1.0f / (float)blockCount;
	b3Vec3 center = { sum.x * inverseCount, sum.y * inverseCount, sum.z * inverseCount };

	// One unit cube weighs the same as any other, so the whole assembly is
	// count times density and each block contributes the same local tensor.
	float blockMass = density;
	b3Vec3 half = { -0.5f, -0.5f, -0.5f };
	b3Vec3 halfUpper = { 0.5f, 0.5f, 0.5f };
	b3Matrix3 blockInertia = b3BoxInertia( blockMass, half, halfUpper );

	b3Matrix3 inertia = b3Mat3_zero;
	for ( int i = 0; i < blockCount; ++i )
	{
		b3Vec3 offset = {
			(float)blocks[i].x + 0.5f - center.x,
			(float)blocks[i].y + 0.5f - center.y,
			(float)blocks[i].z + 0.5f - center.z,
		};
		inertia = b3AddMM( inertia, b3AddMM( blockInertia, b3Steiner( blockMass, offset ) ) );
	}

	massData.mass = blockMass * (float)blockCount;
	// Stored cells omit the cook origin, but shape mass data uses the caller's local frame.
	massData.center = b3Add( center, v3BlockGridWorldOrigin( grid ) );
	massData.inertia = inertia;
	return massData;
}

// Query the grid and run the convex function on each candidate as it is visited.
// Processing candidates directly avoids truncation by a fixed collection buffer.

b3CastOutput v3RayCastBlockGrid( const v3BlockGridData* grid, const b3RayCastInput* input, v3BlockGridScratch* scratch )
{
	b3CastOutput output = { 0 };

	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		return output;
	}

	b3Vec3 p1 = input->origin;
	b3Vec3 p2 = b3MulAdd( input->origin, input->maxFraction, input->translation );
	v3BlockGridRayResult hit = v3BlockGrid_RayCast( grid, p1, p2, &scratch->query );

	if ( hit.status != v3_blockGridRayHit )
	{
		return output;
	}

	output.hit = true;
	output.fraction = hit.fraction * input->maxFraction;
	output.normal = hit.faceNormal;
	output.point = b3MulAdd( input->origin, output.fraction, input->translation );
	output.childIndex = hit.hitboxIndex;
	output.materialIndex = (int)v3BlockGrid_GetHitboxMaterial( grid, hit.hitboxIndex );
	return output;
}

// The swept bounds of a shape cast, used to narrow the field before running the
// convex casts for every hitbox.
static b3AABB v3BlockGridCastBounds( const b3ShapeCastInput* input )
{
	b3AABB aabb = { input->proxy.points[0], input->proxy.points[0] };
	for ( int i = 1; i < input->proxy.count; ++i )
	{
		aabb.lowerBound = b3Min( aabb.lowerBound, input->proxy.points[i] );
		aabb.upperBound = b3Max( aabb.upperBound, input->proxy.points[i] );
	}

	b3Vec3 end = b3MulSV( input->maxFraction, input->translation );
	aabb.lowerBound = b3Min( aabb.lowerBound, b3Add( aabb.lowerBound, end ) );
	aabb.upperBound = b3Max( aabb.upperBound, b3Add( aabb.upperBound, end ) );

	b3Vec3 r = { input->proxy.radius, input->proxy.radius, input->proxy.radius };
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );
	return aabb;
}

typedef struct v3BlockGridCastWork
{
	const v3BlockGridData* grid;
	const b3ShapeCastInput* input;
	b3CastOutput* output;
	bool hit;
} v3BlockGridCastWork;

static bool v3BlockGridCastCallback( int hitboxIndex, void* context )
{
	v3BlockGridCastWork* work = context;
	b3BoxHull box = v3MakeBlockGridHitboxHull( work->grid, hitboxIndex );

	// Carry the best fraction forward so later candidates give up early.
	b3ShapeCastInput localInput = *work->input;
	localInput.maxFraction = work->output->fraction;

	b3CastOutput candidate = b3ShapeCastHull( &box.base, &localInput );
	if ( candidate.hit && candidate.fraction <= work->output->fraction )
	{
		candidate.childIndex = hitboxIndex;
		candidate.materialIndex = (int)v3BlockGrid_GetHitboxMaterial( work->grid, hitboxIndex );
		*work->output = candidate;
		work->hit = true;
	}
	return true;
}

b3CastOutput v3ShapeCastBlockGrid( const v3BlockGridData* grid, const b3ShapeCastInput* input, v3BlockGridScratch* scratch )
{
	b3CastOutput output = { 0 };
	output.fraction = input->maxFraction;

	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		return (b3CastOutput){ 0 };
	}

	v3BlockGridCastWork work = { .grid = grid, .input = input, .output = &output, .hit = false };
	v3BlockGrid_QueryAABB( grid, v3BlockGridCastBounds( input ), &scratch->query, v3BlockGridCastCallback, &work );

	if ( work.hit == false )
	{
		return (b3CastOutput){ 0 };
	}
	return output;
}

typedef struct v3BlockGridOverlapWork
{
	const v3BlockGridData* grid;
	const b3ShapeProxy* proxy;
	bool touching;
} v3BlockGridOverlapWork;

static bool v3BlockGridOverlapCallback( int hitboxIndex, void* context )
{
	v3BlockGridOverlapWork* work = context;
	b3BoxHull box = v3MakeBlockGridHitboxHull( work->grid, hitboxIndex );
	b3DistanceInput distanceInput = {
		.proxyB = *work->proxy,
		.transform = b3Transform_identity,
		.useRadii = true,
	};
	distanceInput.proxyA.points = b3GetHullPoints( &box.base );
	distanceInput.proxyA.count = box.base.vertexCount;
	distanceInput.proxyA.radius = 0.0f;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );
	if ( distanceOutput.distance < 10.0f * FLT_EPSILON )
	{
		work->touching = true;

		// One touch settles the question, so stop the query here.
		return false;
	}
	return true;
}

bool v3OverlapBlockGrid( const v3BlockGridData* grid, b3Transform transform, const b3ShapeProxy* proxy,
						 v3BlockGridScratch* scratch )
{
	// Hitboxes are stored in the grid's frame, so the proxy comes to them rather
	// than the other way round. b3ShapeCastShape localises its proxy before
	// dispatch; overlap dispatches with the world transform still attached, so
	// the grid has to do it here.
	b3Vec3 localPoints[B3_MAX_SHAPE_CAST_POINTS];
	int count = b3MinInt( proxy->count, B3_MAX_SHAPE_CAST_POINTS );
	if ( count <= 0 )
	{
		return false;
	}

	for ( int i = 0; i < count; ++i )
	{
		localPoints[i] = b3InvTransformPoint( transform, proxy->points[i] );
	}

	b3ShapeProxy localProxy = { localPoints, count, proxy->radius };

	b3AABB aabb = { localPoints[0], localPoints[0] };
	for ( int i = 1; i < count; ++i )
	{
		aabb.lowerBound = b3Min( aabb.lowerBound, localPoints[i] );
		aabb.upperBound = b3Max( aabb.upperBound, localPoints[i] );
	}
	b3Vec3 r = { proxy->radius, proxy->radius, proxy->radius };
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );

	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		return false;
	}

	// An AABB overlap is only a candidate. Confirm with the exact proxy so a
	// grazing box does not report contact it does not have.
	v3BlockGridOverlapWork work = { .grid = grid, .proxy = &localProxy, .touching = false };
	v3BlockGrid_QueryAABB( grid, aabb, &scratch->query, v3BlockGridOverlapCallback, &work );
	return work.touching;
}

typedef struct v3BlockGridMoverWork
{
	const v3BlockGridData* grid;
	const b3Capsule* mover;
	b3PlaneResult* planes;
	int capacity;
	int count;
} v3BlockGridMoverWork;

static bool v3BlockGridMoverCallback( int hitboxIndex, void* context )
{
	v3BlockGridMoverWork* work = context;

	// The caller's plane array is the real limit here, so stop when it is full
	// rather than when some arbitrary candidate count is reached. One hitbox can
	// contribute several planes, hence the room check before each.
	if ( work->count >= work->capacity )
	{
		return false;
	}
	b3BoxHull box = v3MakeBlockGridHitboxHull( work->grid, hitboxIndex );
	work->count += b3CollideMoverAndHull( work->planes + work->count, &box.base, work->mover );
	return work->count < work->capacity;
}

int v3CollideMoverAndBlockGrid( b3PlaneResult* planes, int planeCapacity, const v3BlockGridData* grid, const b3Capsule* mover,
								v3BlockGridScratch* scratch )
{
	b3AABB aabb;
	aabb.lowerBound = b3Min( mover->center1, mover->center2 );
	aabb.upperBound = b3Max( mover->center1, mover->center2 );
	b3Vec3 r = { mover->radius, mover->radius, mover->radius };
	aabb.lowerBound = b3Sub( aabb.lowerBound, r );
	aabb.upperBound = b3Add( aabb.upperBound, r );

	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		return 0;
	}

	v3BlockGridMoverWork work = { .grid = grid, .mover = mover, .planes = planes, .capacity = planeCapacity, .count = 0 };
	v3BlockGrid_QueryAABB( grid, aabb, &scratch->query, v3BlockGridMoverCallback, &work );
	return work.count;
}

typedef struct v3BlockGridPairContext
{
	b3TreeQueryCallbackFcn* callback;
	void* context;
	bool proceed;
} v3BlockGridPairContext;

static bool v3BlockGridPairCallback( int hitboxIndex, void* context )
{
	v3BlockGridPairContext* pairs = context;

	// The pair path keys a contact on this index, so it must stay the hitbox
	// index and nothing else. Within one cooked grid it is stable, which is what
	// lets warm starting match the same hitbox across frames.
	pairs->proceed = pairs->callback( hitboxIndex, (uint64_t)hitboxIndex, pairs->context );
	return pairs->proceed;
}

void v3QueryBlockGridPairs( const v3BlockGridData* grid, b3AABB localAABB, b3TreeQueryCallbackFcn* callback, void* context,
							v3BlockGridScratch* scratch )
{
	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		return;
	}

	v3BlockGridPairContext pairs = { .callback = callback, .context = context, .proceed = true };
	v3BlockGrid_QueryAABB( grid, localAABB, &scratch->query, v3BlockGridPairCallback, &pairs );
}

// Continuous collision against the grid. The engine's compound path descends a
// child tree, which the grid does not have, so this narrows with one query and
// runs the ordinary convex time of impact per candidate. Without it a fast body
// tunnels straight through.
//
// Candidates are not evaluated as the query finds them. The query order is the
// grid's storage order, which says nothing about when the sweep reaches a Hitbox,
// so a cap applied to that order would cut off exactly the candidates a fast body
// is about to hit. Instead every candidate is collected with a conservative entry
// lower bound, the list is sorted by that bound, and the traversal walks it in
// order, stopping as soon as the remaining bounds are at or beyond the best impact
// found. Ordering only reorders work: the impact it accepts is the same one an
// exhaustive traversal accepts.

typedef struct v3BlockGridTOIWork
{
	const v3BlockGridData* grid;

	// The entry bound of a Hitbox is the fraction below which the fast shape's
	// bounding sphere provably cannot reach it. See v3BlockGridEntryBound.
	b3Vec3 startCentroid;
	float sweepRate;
	float reachRadius;

	v3BlockGridSweepCandidate* candidates;
	int candidateCapacity;
	int candidateCount;
	bool overflowed;
} v3BlockGridTOIWork;

// Half the rotation angle carried by a sweep, as sine and its chord companion.
// For unit quaternions |dot(q1, q2)| is cos(theta / 2), so a point r from the centre
// of mass moves at most 2 * r * sin(theta / 2) over the whole sweep.
static float v3BlockGridSweepSinHalfAngle( b3Quat q1, b3Quat q2 )
{
	float cosHalf = q1.v.x * q2.v.x + q1.v.y * q2.v.y + q1.v.z * q2.v.z + q1.s * q2.s;
	cosHalf = b3AbsFloat( cosHalf );
	cosHalf = b3MinFloat( cosHalf, 1.0f );
	return sqrtf( b3MaxFloat( 0.0f, 1.0f - cosHalf * cosHalf ) );
}

// Distance from a point to an axis aligned box, both in the grid's local frame.
static float v3BlockGridHitboxDistance( const v3BlockGridData* grid, int hitboxIndex, b3Vec3 point )
{
	b3Vec3 center, halfExtent;
	v3BlockGrid_GetHitboxBounds( grid, hitboxIndex, &center, &halfExtent );

	b3Vec3 delta = {
		b3MaxFloat( 0.0f, b3AbsFloat( point.x - center.x ) - halfExtent.x ),
		b3MaxFloat( 0.0f, b3AbsFloat( point.y - center.y ) - halfExtent.y ),
		b3MaxFloat( 0.0f, b3AbsFloat( point.z - center.z ) - halfExtent.z ),
	};
	return b3Length( delta );
}

// The entry bound of one Hitbox: the largest fraction that is provably below any
// fraction at which the fast shape can touch it.
//
// Write the fast shape's centroid in the grid's local frame as q(t). The shape lies
// inside the ball of radius reachRadius about q(t), and the local frame carries the
// Hitbox unchanged, so a touch at t needs dist(q(t), hitbox) <= reachRadius. The
// caller bounds |q(t) - q(0)| by t * sweepRate, so
//
//     dist(q(0), hitbox) - t * sweepRate <= reachRadius
//
// is necessary for a touch, and solving for t gives the bound below. A sweep whose
// relative motion is nil cannot reach anything it does not already touch, which is
// the sweepRate == 0 branch.
static float v3BlockGridEntryBound( const v3BlockGridTOIWork* work, int hitboxIndex )
{
	float distance = v3BlockGridHitboxDistance( work->grid, hitboxIndex, work->startCentroid );
	float reach = distance - work->reachRadius;
	if ( reach <= 0.0f )
	{
		return 0.0f;
	}

	if ( work->sweepRate <= FLT_MIN )
	{
		return 1.0f;
	}

	float bound = reach / work->sweepRate;
	return bound >= 1.0f ? 1.0f : bound;
}

static bool v3BlockGridTOICollectCallback( int hitboxIndex, void* context )
{
	v3BlockGridTOIWork* work = context;
	if ( work->candidateCount >= work->candidateCapacity )
	{
		// The world sizes the list for the largest attached grid, so this is
		// unreachable unless that sizing was skipped. Dropping candidates would be a
		// silent tunnel, so stop and let the caller report an incomplete sweep.
		work->overflowed = true;
		return false;
	}

	work->candidates[work->candidateCount] = (v3BlockGridSweepCandidate){
		.entryBound = v3BlockGridEntryBound( work, hitboxIndex ),
		.hitboxIndex = hitboxIndex,
	};
	work->candidateCount += 1;
	return true;
}

// Ascending entry bound, ties by ascending hitbox index. The tie-break makes this a
// total order over distinct candidates, so the sorted sequence is unique and the
// traversal order does not depend on the sort implementation.
static int v3BlockGridCompareCandidates( const void* a, const void* b )
{
	const v3BlockGridSweepCandidate* left = a;
	const v3BlockGridSweepCandidate* right = b;

	if ( left->entryBound < right->entryBound )
	{
		return -1;
	}
	if ( left->entryBound > right->entryBound )
	{
		return 1;
	}
	return left->hitboxIndex < right->hitboxIndex ? -1 : ( left->hitboxIndex > right->hitboxIndex ? 1 : 0 );
}

// The rotation-invariant radius of the fast shape about its own centroid, inflated
// so it also covers the fallback proxy and the speculative margin the solve targets.
// A bound computed from it is conservative for both evaluation paths below.
static float v3BlockGridReachRadius( const b3ShapeProxy* proxy, b3Vec3 localCentroid, float fallbackRadius )
{
	float radius = 0.0f;
	for ( int i = 0; i < proxy->count; ++i )
	{
		radius = b3MaxFloat( radius, b3Distance( proxy->points[i], localCentroid ) );
	}
	radius += proxy->radius;
	radius = b3MaxFloat( radius, fallbackRadius + B3_LINEAR_SLOP );
	return radius + B3_SPECULATIVE_DISTANCE;
}

v3BlockGridTOIResult v3TimeOfImpactBlockGrid( const v3BlockGridData* grid, const b3TOIInput* input, b3AABB localBounds,
											  b3Vec3 localCentroidB, float fallbackRadius, v3BlockGridScratch* scratch,
											  int candidateCap )
{
	v3BlockGridTOIResult result = {
		.output = { .state = b3_toiStateSeparated, .fraction = input != NULL ? input->maxFraction : 0.0f },
		.hitboxIndex = -1,
	};
	if ( grid == NULL || input == NULL )
	{
		return result;
	}

	if ( v3BlockGridScratchFits( scratch, grid ) == false )
	{
		// Without scratch the sweep cannot enumerate anything, and reporting a miss
		// would be a tunnel. Hold the fast body where it started instead.
		result.capExhausted = true;
		result.holdFraction = 0.0f;
		return result;
	}

	// The bound is built from the relative motion, so it holds for the general sweep
	// contract where either body may move. Under the current finalize ordering the grid
	// body's own sweep is always stationary when a BlockGrid sweep runs, so every
	// grid-side term below is zero on the reachable paths. The grid's local frame
	// carries the Hitboxes, and q(t) below is the fast shape's centroid written in
	// that frame.
	b3Transform gridStart = b3GetSweepTransform( &input->sweepA, 0.0f );
	b3Transform fastStart = b3GetSweepTransform( &input->sweepB, 0.0f );
	b3Vec3 worldCentroid = b3TransformPoint( fastStart, localCentroidB );

	// Both centres of mass travel a straight line, so their contribution to
	// |q(t) - q(0)| grows linearly in t at this rate.
	float translationRate = b3Distance( input->sweepB.c1, input->sweepB.c2 ) + b3Distance( input->sweepA.c1, input->sweepA.c2 );

	// Rotation contributes a bounded displacement rather than a rate: the fast shape's
	// centroid swings about its own centre of mass, and the whole local frame swings
	// about the grid's. Both are charged in full from t = 0, which only loosens the
	// bound. The grid swing guards the general contract and is zero under the current
	// finalize ordering.
	float fastSwing = 2.0f * b3Distance( localCentroidB, input->sweepB.localCenter ) *
					  v3BlockGridSweepSinHalfAngle( input->sweepB.q1, input->sweepB.q2 );
	float gridSwing =
		2.0f * b3Distance( worldCentroid, input->sweepA.c1 ) * v3BlockGridSweepSinHalfAngle( input->sweepA.q1, input->sweepA.q2 );
	float swing = fastSwing + gridSwing;

	v3BlockGridTOIWork work = {
		.grid = grid,
		.startCentroid = b3InvTransformPoint( gridStart, worldCentroid ),
		.sweepRate = translationRate,
		.reachRadius = v3BlockGridReachRadius( &input->proxyB, localCentroidB, fallbackRadius ) + swing,
		.candidates = scratch->candidates,
		.candidateCapacity = scratch->candidateCapacity,
	};

	// The query bounds are the fast shape's swept box written in the grid's frame at
	// t = 0. A rotating grid moves that frame under the box, so grow it by the same
	// swing the entry bound charges, or the query would drop a Hitbox the sweep
	// reaches. This inflation guards the general contract: a grid that does not rotate
	// grows by nothing, which is every path the current finalize ordering reaches.
	if ( gridSwing > 0.0f )
	{
		b3Vec3 margin = { gridSwing, gridSwing, gridSwing };
		localBounds.lowerBound = b3Sub( localBounds.lowerBound, margin );
		localBounds.upperBound = b3Add( localBounds.upperBound, margin );
	}

	v3BlockGrid_QueryAABB( grid, localBounds, &scratch->query, v3BlockGridTOICollectCallback, &work );
	if ( work.overflowed )
	{
		result.capExhausted = true;
		result.holdFraction = 0.0f;
		return result;
	}

	if ( work.candidateCount > 1 )
	{
		qsort( work.candidates, (size_t)work.candidateCount, sizeof( v3BlockGridSweepCandidate ), v3BlockGridCompareCandidates );
	}

	// Zero is the world's unlimited sentinel. Every candidate the query returned may
	// then be evaluated, which the list is sized for.
	int evaluationLimit = candidateCap > 0 ? candidateCap : work.candidateCount;
	int evaluatedCount = 0;

	for ( int i = 0; i < work.candidateCount; ++i )
	{
		const v3BlockGridSweepCandidate* candidate = work.candidates + i;

		// Sorted ascending, so no candidate from here on can be reached before the
		// impact already found. The rest of the list is settled without touching it.
		if ( candidate->entryBound >= result.output.fraction )
		{
			break;
		}

		if ( evaluatedCount >= evaluationLimit )
		{
			// This untested candidate may be reached before the best impact found.
			// The cap prevents testing it, so hold the body at its entry fraction.
			result.capExhausted = true;
			result.holdFraction = candidate->entryBound;
			break;
		}

		evaluatedCount += 1;
		b3BoxHull box = v3MakeBlockGridHitboxHull( grid, candidate->hitboxIndex );

		b3TOIInput candidateInput = *input;
		candidateInput.proxyA.points = b3GetHullPoints( &box.base );
		candidateInput.proxyA.count = box.base.vertexCount;
		candidateInput.proxyA.radius = 0.0f;
		candidateInput.maxFraction = result.output.fraction;

		b3TOIOutput output = b3TimeOfImpact( &candidateInput );
		if ( 0.0f < output.fraction && output.fraction < result.output.fraction )
		{
			result.output = output;
			result.hitboxIndex = candidate->hitboxIndex;
			continue;
		}

		if ( output.fraction != 0.0f )
		{
			continue;
		}

		// A zero fraction means the sweep started already touching this box, which
		// the general solve cannot advance past. Retry against a point proxy so the
		// mover still gets a usable impact rather than nothing.
		b3TOIInput fallbackInput = candidateInput;
		fallbackInput.proxyB = (b3ShapeProxy){ &localCentroidB, 1, fallbackRadius + B3_LINEAR_SLOP };
		output = b3TimeOfImpact( &fallbackInput );
		if ( 0.0f < output.fraction && output.fraction < result.output.fraction )
		{
			result.output = output;
			result.output.usedFallback = true;
			result.hitboxIndex = candidate->hitboxIndex;
		}
	}

	return result;
}

b3AABB v3ComputeBlockGridAABB( const v3BlockGridData* grid, b3Transform transform )
{
	b3AABB local = v3BlockGrid_GetBounds( grid );

	// Hitboxes are axis aligned in grid coordinates. Transform all eight corners
	// because the body transform may include rotation.
	b3Vec3 lower = local.lowerBound;
	b3Vec3 upper = local.upperBound;
	b3AABB result = { { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };
	for ( int i = 0; i < 8; ++i )
	{
		b3Vec3 corner = {
			( i & 1 ) ? upper.x : lower.x,
			( i & 2 ) ? upper.y : lower.y,
			( i & 4 ) ? upper.z : lower.z,
		};
		b3Vec3 p = b3TransformPoint( transform, corner );
		result.lowerBound = b3Min( result.lowerBound, p );
		result.upperBound = b3Max( result.upperBound, p );
	}
	return result;
}
