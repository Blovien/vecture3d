// SPDX-License-Identifier: MIT

#include "block_grid_pair.h"

#include "block_grid_internal.h"

typedef struct v3BlockGridPairTraversalContext
{
	const v3BlockGridData* gridA;
	const v3BlockGridData* gridB;
	b3Transform outerToInner;
	v3BlockGridQueryScratch* innerScratch;
	v3BlockGridPairCandidateFcn* callback;
	void* callbackContext;
	v3BlockGridPairTraversalResult* result;
	uint64_t candidateCapacity;
	bool outerIsA;
	bool stopped;
	float inflation;
} v3BlockGridPairTraversalContext;

static b3AABB v3BlockGridPairInflateBounds( b3AABB bounds, float inflation )
{
	b3Vec3 margin = { inflation, inflation, inflation };
	return (b3AABB){ b3Sub( bounds.lowerBound, margin ), b3Add( bounds.upperBound, margin ) };
}

static bool v3BlockGridPairScratchFits( const v3BlockGridData* grid, const v3BlockGridQueryScratch* scratch )
{
	int wordCount = v3BlockGrid_GetQueryScratchWordCount( grid );
	return scratch != NULL && scratch->visitedHitboxes != NULL && scratch->visitedWordCapacity >= wordCount;
}

static bool v3BlockGridPairScratchIsDisjoint( const v3BlockGridData* gridA, const v3BlockGridQueryScratch* scratchA,
											  const v3BlockGridData* gridB, const v3BlockGridQueryScratch* scratchB )
{
	size_t bytesA = (size_t)v3BlockGrid_GetQueryScratchWordCount( gridA ) * sizeof( uint64_t );
	size_t bytesB = (size_t)v3BlockGrid_GetQueryScratchWordCount( gridB ) * sizeof( uint64_t );
	uintptr_t beginA = (uintptr_t)scratchA->visitedHitboxes;
	uintptr_t beginB = (uintptr_t)scratchB->visitedHitboxes;
	if ( bytesA > UINTPTR_MAX - beginA || bytesB > UINTPTR_MAX - beginB )
	{
		return false;
	}
	return beginA + bytesA <= beginB || beginB + bytesB <= beginA;
}

static bool v3BlockGridPairMeasureQuery( const v3BlockGridData* grid, b3AABB bounds, v3BlockGridQueryCost* cost )
{
	// A prepared query cannot visit more groups than the validated cooked grid owns.
	uint64_t groupCount = (uint64_t)grid->direct.groupsX * (uint64_t)grid->direct.groupsY * (uint64_t)grid->direct.groupsZ;
	v3BlockGridQueryCostStatus status = v3BlockGridMeasureDirectQueryCost( grid, bounds, groupCount, cost );
	return status != v3_blockGridQueryCostInvalid && status != v3_blockGridQueryCostWorkRefused;
}

static bool v3BlockGridPairAddCount( uint64_t* total, uint64_t amount )
{
	if ( amount > UINT64_MAX - *total )
	{
		return false;
	}
	*total += amount;
	return true;
}

static bool v3BlockGridPairAddQueryWork( v3BlockGridPairTraversalResult* result, v3BlockGridQueryCost cost )
{
	return v3BlockGridPairAddCount( &result->visitedGroups, cost.groups ) &&
		   v3BlockGridPairAddCount( &result->markedReferences, cost.references );
}

typedef struct v3BlockGridPairInnerContext
{
	v3BlockGridPairTraversalContext* traversal;
	int outerHitboxIndex;
} v3BlockGridPairInnerContext;

static bool v3BlockGridPairInnerCandidate( int innerHitboxIndex, void* contextPointer )
{
	v3BlockGridPairInnerContext* inner = contextPointer;
	v3BlockGridPairTraversalContext* context = inner->traversal;
	v3BlockGridPairTraversalResult* result = context->result;
	if ( result->candidatePairs >= context->candidateCapacity || v3BlockGridPairAddCount( &result->candidatePairs, 1 ) == false )
	{
		context->stopped = true;
		return false;
	}
	int hitboxIndexA = context->outerIsA ? inner->outerHitboxIndex : innerHitboxIndex;
	int hitboxIndexB = context->outerIsA ? innerHitboxIndex : inner->outerHitboxIndex;
	if ( context->callback( hitboxIndexA, hitboxIndexB, context->callbackContext ) == false )
	{
		context->stopped = true;
		return false;
	}
	return true;
}

static bool v3BlockGridPairOuterCandidate( int outerHitboxIndex, void* contextPointer )
{
	v3BlockGridPairTraversalContext* context = contextPointer;
	const v3BlockGridData* outer = context->outerIsA ? context->gridA : context->gridB;
	const v3BlockGridData* inner = context->outerIsA ? context->gridB : context->gridA;

	b3Vec3 center, halfExtent;
	v3BlockGrid_GetHitboxBounds( outer, outerHitboxIndex, &center, &halfExtent );
	b3AABB outerBounds = { b3Sub( center, halfExtent ), b3Add( center, halfExtent ) };
	b3AABB innerBounds = b3AABB_Transform( context->outerToInner, outerBounds );
	innerBounds = v3BlockGridPairInflateBounds( innerBounds, context->inflation );

	v3BlockGridQueryCost cost;
	if ( v3BlockGridPairMeasureQuery( inner, innerBounds, &cost ) == false )
	{
		context->stopped = true;
		return false;
	}
	// Measuring reads the group offsets once, then the admitted query walks the groups again
	if ( v3BlockGridPairAddCount( &context->result->visitedGroups, cost.groups ) == false ||
		 v3BlockGridPairAddQueryWork( context->result, cost ) == false )
	{
		context->stopped = true;
		return false;
	}
	v3BlockGridPairInnerContext innerContext = { context, outerHitboxIndex };
	v3BlockGridQueryStatus status =
		v3BlockGrid_QueryAABB( inner, innerBounds, context->innerScratch, v3BlockGridPairInnerCandidate, &innerContext );
	if ( status == v3_blockGridQueryInvalid )
	{
		context->stopped = true;
		return false;
	}
	return status == v3_blockGridQueryCompleted;
}

static bool v3BlockGridPairDirectionIsAToB( v3BlockGridQueryCost costA, v3BlockGridQueryCost costB )
{
	// References are the main marking cost, while group count breaks ties for empty space
	if ( costA.references != costB.references )
	{
		return costA.references < costB.references;
	}
	if ( costA.groups != costB.groups )
	{
		return costA.groups < costB.groups;
	}
	return true;
}

v3BlockGridPairTraversalResult v3BlockGridPairEnumerateCandidates( const v3BlockGridData* gridA, b3Transform transformA,
																   const v3BlockGridData* gridB, b3Transform transformB,
																   v3BlockGridPairTraversalScratch* scratch,
																   v3BlockGridPairCandidateFcn* callback, void* context,
																   float admissionDistance )
{
	v3BlockGridPairTraversalResult result = { .status = v3_blockGridPairTraversalInvalid };
	if ( gridA == NULL || gridB == NULL || scratch == NULL || callback == NULL || b3IsValidTransform( transformA ) == false ||
		 b3IsValidTransform( transformB ) == false || !b3IsValidFloat( admissionDistance ) || admissionDistance < 0.0f )
	{
		return result;
	}
	if ( v3BlockGridPairScratchFits( gridA, &scratch->queryA ) == false ||
		 v3BlockGridPairScratchFits( gridB, &scratch->queryB ) == false ||
		 v3BlockGridPairScratchIsDisjoint( gridA, &scratch->queryA, gridB, &scratch->queryB ) == false )
	{
		return result;
	}

	b3Transform bToA = b3InvMulTransforms( transformA, transformB );
	b3Transform aToB = b3InvMulTransforms( transformB, transformA );
	const float inflation = admissionDistance;
	b3AABB windowA = v3BlockGridPairInflateBounds( b3AABB_Transform( bToA, v3BlockGrid_GetBounds( gridB ) ), inflation );
	b3AABB windowB = v3BlockGridPairInflateBounds( b3AABB_Transform( aToB, v3BlockGrid_GetBounds( gridA ) ), inflation );
	v3BlockGridQueryCost costA, costB;
	if ( v3BlockGridPairMeasureQuery( gridA, windowA, &costA ) == false ||
		 v3BlockGridPairAddCount( &result.visitedGroups, costA.groups ) == false )
	{
		return result;
	}
	if ( v3BlockGridPairMeasureQuery( gridB, windowB, &costB ) == false ||
		 v3BlockGridPairAddCount( &result.visitedGroups, costB.groups ) == false )
	{
		return result;
	}

	v3BlockGridPairTraversalContext traversal = {
		.gridA = gridA,
		.gridB = gridB,
		.callback = callback,
		.callbackContext = context,
		.result = &result,
		// TODO: Bound worst-case traversal work without discarding valid contacts.
		// Dense valid grids can make a simulation step impractically slow.
		// Cooking admits at most 65,534 hitboxes per grid. Their product is the exact maximum
		// number of distinct hitbox pairs this traversal can emit.
		.candidateCapacity = (uint64_t)gridA->hitboxCount * (uint64_t)gridB->hitboxCount,
		.inflation = inflation,
	};

	bool outerIsA = v3BlockGridPairDirectionIsAToB( costA, costB );
	traversal.outerToInner = outerIsA ? aToB : bToA;
	traversal.innerScratch = outerIsA ? &scratch->queryB : &scratch->queryA;
	traversal.outerIsA = outerIsA;

	v3BlockGridQueryCost outerCost = outerIsA ? costA : costB;
	// The outer walk and each inner walk are bounded by the cooked group and reference arrays.
	// Keep checked meter arithmetic without imposing a smaller cutoff that can discard contact.
	if ( v3BlockGridPairAddQueryWork( &result, outerCost ) == false )
	{
		return result;
	}
	const v3BlockGridData* outer = outerIsA ? gridA : gridB;
	v3BlockGridQueryScratch* outerScratch = outerIsA ? &scratch->queryA : &scratch->queryB;
	v3BlockGridQueryStatus queryStatus =
		v3BlockGrid_QueryAABB( outer, outerIsA ? windowA : windowB, outerScratch, v3BlockGridPairOuterCandidate, &traversal );

	if ( traversal.stopped == false && queryStatus == v3_blockGridQueryCompleted )
	{
		result.status = v3_blockGridPairTraversalCompleted;
	}
	return result;
}
