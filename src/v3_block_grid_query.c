// SPDX-License-Identifier: MIT
//
// Read-only queries on a cooked grid.
//
// All queries first move the world coordinates into the frame of the grid and
// stop if there is no overlap. The occupancy count then uses only the group
// masks. The overlap query makes a block range and collects the candidates in
// the scratch buffer.
//
// A ray instead moves through the projection grid and reads only the cells that
// it crosses. It needs no scratch buffer. If the field has no projection grid,
// the ray uses the same group walk as the overlap query.
//
// The two ray paths always agree. This includes the selection between two boxes
// that touch on the same face, because the caller uses the result to identify
// the contact.

#include "ctz.h"
#include "platform.h"
#include "v3_block_grid_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#if B3_DEBUG

// Ladder counters for the occupancy narrowing cost model. Atomic because pair
// enumeration runs inside parallel broad-phase tasks.
static b3AtomicInt s_invocations;
static b3AtomicInt s_groupsWalked;
static b3AtomicInt s_refsMarked;
static b3AtomicInt s_wordsSpanned;
static b3AtomicInt s_bitsTested;
static b3AtomicInt s_overlapsFired;

#endif

static bool v3BlockGridIsValidAABB( b3AABB bounds, bool allowDegenerate )
{
	if ( b3IsValidVec3( bounds.lowerBound ) == false || b3IsValidVec3( bounds.upperBound ) == false )
	{
		return false;
	}
	if ( allowDegenerate )
	{
		return bounds.lowerBound.x <= bounds.upperBound.x && bounds.lowerBound.y <= bounds.upperBound.y &&
			   bounds.lowerBound.z <= bounds.upperBound.z;
	}
	return bounds.lowerBound.x < bounds.upperBound.x && bounds.lowerBound.y < bounds.upperBound.y &&
		   bounds.lowerBound.z < bounds.upperBound.z;
}

static bool v3BlockGridIsValidScratch( const v3BlockGridData* grid, const v3BlockGridQueryScratch* scratch )
{
	int wordCount = v3BlockGrid_GetQueryScratchWordCount( grid );
	return scratch != NULL && scratch->visitedHitboxes != NULL && scratch->visitedWordCapacity >= wordCount;
}

typedef enum v3BlockGridQueryPreparation
{
	v3_blockGridQueryPreparationInvalid,
	v3_blockGridQueryPreparationEmpty,
	v3_blockGridQueryPreparationReady,
} v3BlockGridQueryPreparation;

static v3BlockGridQueryPreparation v3BlockGridPrepareQuery( const v3BlockGridData* grid, b3AABB bounds, b3AABB* localBounds,
															int range[6] )
{
	if ( grid == NULL || v3BlockGridIsValidAABB( bounds, true ) == false )
	{
		return v3_blockGridQueryPreparationInvalid;
	}

	*localBounds = v3BlockGridToLocalBounds( grid, bounds );
	if ( b3AABB_Overlaps( grid->bounds, *localBounds ) == false ||
		 v3BlockGridComputeBlockRange( &grid->frame, *localBounds, v3_blockGridRangeTouching, range ) == false )
	{
		return v3_blockGridQueryPreparationEmpty;
	}
	return v3_blockGridQueryPreparationReady;
}

v3BlockGridQueryCostStatus v3BlockGridMeasureDirectQueryCost( const v3BlockGridData* grid, b3AABB bounds, uint64_t maxGroups,
															  v3BlockGridQueryCost* cost )
{
	if ( cost == NULL )
	{
		return v3_blockGridQueryCostInvalid;
	}
	*cost = (v3BlockGridQueryCost){ 0 };

	b3AABB localBounds;
	int range[6];
	v3BlockGridQueryPreparation preparation = v3BlockGridPrepareQuery( grid, bounds, &localBounds, range );
	if ( preparation == v3_blockGridQueryPreparationInvalid )
	{
		return v3_blockGridQueryCostInvalid;
	}
	if ( preparation == v3_blockGridQueryPreparationEmpty )
	{
		return v3_blockGridQueryCostEmpty;
	}

	uint64_t groupsX = (uint64_t)( ( range[3] >> 2 ) - ( range[0] >> 2 ) + 1 );
	uint64_t groupsY = (uint64_t)( ( range[4] >> 2 ) - ( range[1] >> 2 ) + 1 );
	uint64_t groupsZ = (uint64_t)( ( range[5] >> 2 ) - ( range[2] >> 2 ) + 1 );
	cost->groups = groupsX * groupsY * groupsZ;
	if ( cost->groups > maxGroups )
	{
		return v3_blockGridQueryCostWorkRefused;
	}

	const uint32_t* hitboxOffsets = v3BlockGridHitboxOffsets( grid );
	for ( int gz = range[2] >> 2; gz <= range[5] >> 2; ++gz )
	{
		for ( int gy = range[1] >> 2; gy <= range[4] >> 2; ++gy )
		{
			for ( int gx = range[0] >> 2; gx <= range[3] >> 2; ++gx )
			{
				int group = v3BlockGridGroupIndex( &grid->direct, gx, gy, gz );
				cost->references += hitboxOffsets[group + 1] - hitboxOffsets[group];
			}
		}
	}
	return v3_blockGridQueryCostReady;
}

// The inclusive span of scratch words a query dirtied, empty when last < first.
// Without it, walking and clearing would each cost a pass over the whole bitmap,
// so a point query would pay for every hitbox in the grid.
typedef struct v3BlockGridDirtyWords
{
	int first;
	int last;
} v3BlockGridDirtyWords;

static const v3BlockGridDirtyWords s_noDirtyWords = { .first = 0, .last = -1 };

static void v3BlockGridClearDirtyWords( v3BlockGridQueryScratch* scratch, v3BlockGridDirtyWords dirty )
{
	if ( dirty.last < dirty.first )
	{
		return;
	}
	memset( scratch->visitedHitboxes + dirty.first, 0, (size_t)( dirty.last - dirty.first + 1 ) * sizeof( uint64_t ) );
}

// Marks every hitbox reachable from the groups this range covers and reports
// the words touched. Marking is idempotent, which is what collapses a merged box
// listed by several groups.
// NOTE: Requires scratch clean on entry.
static v3BlockGridDirtyWords v3BlockGridMarkCandidates( const v3BlockGridData* grid, const int range[6],
														v3BlockGridQueryScratch* scratch )
{
	const uint32_t* hitboxOffsets = v3BlockGridHitboxOffsets( grid );
	const uint32_t* hitboxIndices = v3BlockGridHitboxIndices( grid );
	v3BlockGridDirtyWords dirty = s_noDirtyWords;
#if B3_DEBUG
	b3AtomicFetchAddInt( &s_invocations, 1 );
#endif
	// Groups are 4x4x4 blocks, so block coordinates shift down by two.
	for ( int gz = range[2] >> 2; gz <= range[5] >> 2; ++gz )
	{
		for ( int gy = range[1] >> 2; gy <= range[4] >> 2; ++gy )
		{
			for ( int gx = range[0] >> 2; gx <= range[3] >> 2; ++gx )
			{
#if B3_DEBUG
				b3AtomicFetchAddInt( &s_groupsWalked, 1 );
#endif
				int group = v3BlockGridGroupIndex( &grid->direct, gx, gy, gz );
				for ( uint32_t ref = hitboxOffsets[group]; ref < hitboxOffsets[group + 1]; ++ref )
				{
					uint32_t hitbox = hitboxIndices[ref];
#if B3_DEBUG
					b3AtomicFetchAddInt( &s_refsMarked, 1 );
#endif
					int word = (int)( hitbox >> 6 );
					if ( dirty.last < dirty.first )
					{
						dirty.first = word;
						dirty.last = word;
					}
					else
					{
						dirty.first = word < dirty.first ? word : dirty.first;
						dirty.last = word > dirty.last ? word : dirty.last;
					}
					scratch->visitedHitboxes[word] |= (uint64_t)1 << ( hitbox & 63 );
				}
			}
		}
	}
	return dirty;
}

v3BlockGridQueryStatus v3BlockGrid_QueryAABB( const v3BlockGridData* grid, b3AABB bounds, v3BlockGridQueryScratch* scratch,
											  v3BlockGridQueryFcn* callback, void* context )
{
	if ( grid == NULL || callback == NULL || v3BlockGridIsValidScratch( grid, scratch ) == false )
	{
		return v3_blockGridQueryInvalid;
	}

	int range[6];
	b3AABB localBounds;
	v3BlockGridQueryPreparation preparation = v3BlockGridPrepareQuery( grid, bounds, &localBounds, range );
	if ( preparation == v3_blockGridQueryPreparationInvalid )
	{
		return v3_blockGridQueryInvalid;
	}
	if ( preparation == v3_blockGridQueryPreparationEmpty )
	{
		return v3_blockGridQueryCompleted;
	}
	bounds = localBounds;

	v3BlockGridDirtyWords dirty = v3BlockGridMarkCandidates( grid, range, scratch );

	const v3BlockGridHitboxData* hitboxes = v3BlockGridHitboxes( grid );

	// Hitboxes are cooked in ascending ID order, so index order is ID order.
	// Walking words upward and bits from the low end delivers the documented
	// callback order without sorting candidates.
#if B3_DEBUG
	if ( dirty.last >= dirty.first )
	{
		b3AtomicFetchAddInt( &s_wordsSpanned, dirty.last - dirty.first + 1 );
	}
#endif
	for ( int word = dirty.first; word <= dirty.last; ++word )
	{
		uint64_t bits = scratch->visitedHitboxes[word];
		while ( bits != 0 )
		{
			int hitbox = ( word << 6 ) + (int)b3CTZ64( bits );
			bits &= bits - 1;
#if B3_DEBUG
			b3AtomicFetchAddInt( &s_bitsTested, 1 );
#endif
			if ( hitbox < grid->hitboxCount && b3AABB_Overlaps( v3BlockGridHitboxBounds( hitboxes + hitbox ), bounds ) )
			{
#if B3_DEBUG
				b3AtomicFetchAddInt( &s_overlapsFired, 1 );
#endif
				if ( callback( hitbox, context ) == false )
				{
					// Clean on this path too, so a first-hit search can reuse the
					// buffer without clearing.
					v3BlockGridClearDirtyWords( scratch, dirty );
					return v3_blockGridQueryStopped;
				}
			}
		}
	}
	v3BlockGridClearDirtyWords( scratch, dirty );
	return v3_blockGridQueryCompleted;
}

static bool v3BlockGridHitboxRaySlab( const v3BlockGridHitboxData* hitbox, b3Vec3 p1, b3Vec3 direction, float maxFraction,
									  float* fraction, int* axis, float* sign )
{
	float center[3] = { hitbox->center.x, hitbox->center.y, hitbox->center.z };
	float half[3] = { hitbox->halfExtent.x, hitbox->halfExtent.y, hitbox->halfExtent.z };
	float origin[3] = { p1.x, p1.y, p1.z };
	float delta[3] = { direction.x, direction.y, direction.z };
	float tMin = 0.0f;
	float tMax = maxFraction;
	int enterAxis = -1;
	float enterSign = 0.0f;
	for ( int i = 0; i < 3; ++i )
	{
		float lower = center[i] - half[i];
		float upper = center[i] + half[i];
		if ( fabsf( delta[i] ) < 1.0e-9f )
		{
			if ( origin[i] < lower || origin[i] > upper )
			{
				return false;
			}
			continue;
		}
		float inverse = 1.0f / delta[i];
		float t1 = ( lower - origin[i] ) * inverse;
		float t2 = ( upper - origin[i] ) * inverse;
		float normalSign = -1.0f;
		if ( t1 > t2 )
		{
			// Moving negatively on this axis, so the near plane is the upper face
			// and the outward normal is positive.
			float swap = t1;
			t1 = t2;
			t2 = swap;
			normalSign = 1.0f;
		}
		if ( t1 > tMin )
		{
			tMin = t1;
			enterAxis = i;
			enterSign = normalSign;
		}
		tMax = b3MinFloat( tMax, t2 );
		if ( tMin > tMax )
		{
			return false;
		}
	}
	*fraction = tMin;
	*axis = enterAxis;
	*sign = enterSign;
	return true;
}

// Select the nearest computed fraction. Only exact ties use source identity,
// so traversal order cannot turn a chain of nearby hits into a farther result.
static bool v3BlockGridRayCandidateWins( const v3BlockGridHitboxData* hitboxes, int candidate, float candidateFraction,
										 int incumbent, float incumbentFraction )
{
	if ( incumbent == B3_NULL_INDEX )
	{
		return true;
	}
	if ( candidateFraction == incumbentFraction )
	{
		return hitboxes[candidate].sourceId < hitboxes[incumbent].sourceId;
	}
	return candidateFraction < incumbentFraction;
}

// Keep a conservative margin when visiting shared boundaries. This margin
// affects pruning only; it never makes distinct hit fractions equal.
static inline float v3BlockGridRayTestBound( float incumbentFraction )
{
	return incumbentFraction + 1.0e-6f;
}

// Tells if a ray starts right on a cell edge and moves away from the cell
// behind it. That cell still touches the start point, so the walk must include
// it or a box on the far side of the seam goes untested.
static inline bool v3BlockGridStartsOnEdge( float coordinate, float step )
{
	return coordinate == floorf( coordinate ) && step > 0.0f;
}

// Finds the part of the ray that is inside the grid.
//
// The ray can start far outside the field. The walk thus starts where the ray
// enters the grid and does not step through the empty space before it. Returns
// false if the ray stays outside the grid.
static bool v3BlockGridProjectionEntryRange( const float origin[2], const float delta[2], int cellsU, int cellsV, float* enterOut,
											 float* exitOut )
{
	const float extent[2] = { (float)cellsU, (float)cellsV };
	float enter = 0.0f;
	float exit = 1.0f;
	for ( int axis = 0; axis < 2; ++axis )
	{
		if ( fabsf( delta[axis] ) < 1.0e-9f )
		{
			// The ray is parallel to this axis. It is thus inside the limits
			// for all of its length, or outside them for all of it.
			if ( origin[axis] < 0.0f || origin[axis] >= extent[axis] )
			{
				return false;
			}
			continue;
		}
		float inverse = 1.0f / delta[axis];
		float t1 = ( 0.0f - origin[axis] ) * inverse;
		float t2 = ( extent[axis] - origin[axis] ) * inverse;
		if ( t1 > t2 )
		{
			float swap = t1;
			t1 = t2;
			t2 = swap;
		}
		enter = b3MaxFloat( enter, t1 );
		exit = b3MinFloat( exit, t2 );
		if ( enter > exit )
		{
			return false;
		}
	}
	*enterOut = enter;
	*exitOut = exit;
	return true;
}

// Moves through the projection grid and tests only the hitboxes in the cells
// that the ray crosses.
//
// This is the purpose of the grid. A long ray crosses few cells, but the box
// around the same ray contains most of the field.
//
// The caller cannot see a difference from the group walk. Both give the same
// hitbox, the same distance and the same face, and both use the lower source ID
// for equal distances. The caller reads the hitbox to find the material and to
// identify the contact.
//
// Returns false if the field has no projection grid. The caller must then use
// the group walk.
static bool v3BlockGridProjectionRayCast( const v3BlockGridData* grid, b3Vec3 p1, b3Vec3 direction, int* bestOut,
										  float* bestFractionOut, int* bestAxisOut, float* bestSignOut )
{
	const v3BlockGridProjectionIndex* projection = &grid->projection;
	if ( projection->cellCount <= 0 )
	{
		return false;
	}

	int best = B3_NULL_INDEX;
	float bestFraction = 1.0f;
	int bestAxis = -1;
	float bestSign = 0.0f;

	int axisU, axisV;
	v3BlockGridProjectionAxes( projection->projectionAxis, &axisU, &axisV );
	const float start[3] = { p1.x, p1.y, p1.z };
	const float step[3] = { direction.x, direction.y, direction.z };
	const float origin[2] = { start[axisU], start[axisV] };
	const float delta[2] = { step[axisU], step[axisV] };

	float entry, departure;
	if ( v3BlockGridProjectionEntryRange( origin, delta, projection->cellsU, projection->cellsV, &entry, &departure ) == false )
	{
		*bestOut = best;
		*bestFractionOut = bestFraction;
		*bestAxisOut = bestAxis;
		*bestSignOut = bestSign;
		return true;
	}

	const uint32_t* offsets = v3BlockGridProjectionCellOffsets( grid );
	const uint32_t* indices = v3BlockGridProjectionCellIndices( grid );
	const float* nearBound = v3BlockGridProjectionCellNear( grid );
	const float* farBound = v3BlockGridProjectionCellFar( grid );
	const v3BlockGridHitboxData* hitboxes = v3BlockGridHitboxes( grid );

	// Step by step traversal, as in Amanatides and Woo. Musgrave calls the
	// two dimensional form grid tracing. All distances use the same scale as
	// the ray, so a cell edge and a hit compare directly. The first cell needs
	// a limit, because rounding can put it outside the grid.
	float entryU = origin[0] + entry * delta[0];
	float entryV = origin[1] + entry * delta[1];
	int cellU = b3ClampInt( (int)floorf( entryU ), 0, projection->cellsU - 1 );
	int cellV = b3ClampInt( (int)floorf( entryV ), 0, projection->cellsV - 1 );
	int stepU = delta[0] > 0.0f ? 1 : -1;
	int stepV = delta[1] > 0.0f ? 1 : -1;

	// A ray that begins right on a cell edge belongs to both neighbours, but
	// floor sends it only to the forward one. The box behind that edge touches
	// the start point at distance zero and would go untested, so a cast along a
	// seam reported the wrong box and the group walk disagreed. Stepping back
	// one cell when the ray leaves the edge behind it keeps the pair together
	// for the tie rule to choose between.
	//
	// Only the entry needs this. Once the walk is moving it visits both sides of
	// every later edge anyway.
	if ( v3BlockGridStartsOnEdge( entryU, delta[0] ) && cellU > 0 )
	{
		cellU -= 1;
	}
	if ( v3BlockGridStartsOnEdge( entryV, delta[1] ) && cellV > 0 )
	{
		cellV -= 1;
	}

	// A direction that is almost zero gets an edge distance that the walk never
	// reaches. The walk then moves on the other axis and does not divide by a
	// value near zero.
	const float parallel = 1.0e-9f;
	bool marchesU = fabsf( delta[0] ) >= parallel;
	bool marchesV = fabsf( delta[1] ) >= parallel;
	float deltaU = marchesU ? fabsf( 1.0f / delta[0] ) : FLT_MAX;
	float deltaV = marchesV ? fabsf( 1.0f / delta[1] ) : FLT_MAX;
	float reachU =
		marchesU ? b3MaxFloat( 0.0f, delta[0] > 0.0f ? (float)( cellU + 1 ) - entryU : entryU - (float)cellU ) : FLT_MAX;
	float reachV =
		marchesV ? b3MaxFloat( 0.0f, delta[1] > 0.0f ? (float)( cellV + 1 ) - entryV : entryV - (float)cellV ) : FLT_MAX;
	float nextU = marchesU ? entry + reachU * deltaU : FLT_MAX;
	float nextV = marchesV ? entry + reachV * deltaV : FLT_MAX;

	// Each step leaves the grid or crosses one cell edge. The two axes have a
	// known number of edges. This limit thus stops the walk even if a very
	// small step does not move it forward.
	int crossings = projection->cellsU + projection->cellsV + 2;
	float cellEntry = entry;
	float collapsedStart = start[projection->projectionAxis];
	float collapsedStep = step[projection->projectionAxis];
	for ( int crossing = 0; crossing < crossings; ++crossing )
	{
		float cellExit = b3MinFloat( nextU < nextV ? nextU : nextV, departure );
		size_t cell = (size_t)cellV * projection->cellsU + cellU;

		// The part of the flattened axis that the ray occupies in this cell. If
		// it stays fully above or fully below the contents, no hit is possible.
		// This removes most cells of a flat terrain field.
		float atEntry = collapsedStart + collapsedStep * cellEntry;
		float atExit = collapsedStart + collapsedStep * cellExit;
		float low = b3MinFloat( atEntry, atExit );
		float high = b3MaxFloat( atEntry, atExit );
		if ( low <= farBound[cell] && high >= nearBound[cell] )
		{
			for ( uint32_t reference = offsets[cell]; reference < offsets[cell + 1]; ++reference )
			{
				int hitbox = (int)indices[reference];
				float candidateFraction;
				int candidateAxis;
				float candidateSign;
				if ( v3BlockGridHitboxRaySlab( hitboxes + hitbox, p1, direction, v3BlockGridRayTestBound( bestFraction ),
											   &candidateFraction, &candidateAxis, &candidateSign ) &&
					 v3BlockGridRayCandidateWins( hitboxes, hitbox, candidateFraction, best, bestFraction ) )
				{
					best = hitbox;
					bestFraction = candidateFraction;
					bestAxis = candidateAxis;
					bestSign = candidateSign;
				}
			}
		}

		// A hit in this cell is nearer than all hits in the cells after it,
		// because each later cell starts further along the ray. The limit adds
		// the tolerance because a ray on a shared face hits right on the
		// edge between two cells. Without the tolerance the walk stops too
		// early and does not read the next cell.
		if ( cellExit >= departure || ( best != B3_NULL_INDEX && cellExit > v3BlockGridRayTestBound( bestFraction ) ) )
		{
			break;
		}

		cellEntry = cellExit;
		if ( nextU < nextV )
		{
			nextU += deltaU;
			cellU += stepU;
		}
		else
		{
			nextV += deltaV;
			cellV += stepV;
		}
		if ( cellU < 0 || cellU >= projection->cellsU || cellV < 0 || cellV >= projection->cellsV )
		{
			break;
		}
	}

	*bestOut = best;
	*bestFractionOut = bestFraction;
	*bestAxisOut = bestAxis;
	*bestSignOut = bestSign;
	return true;
}

v3BlockGridRayResult v3BlockGrid_RayCast( const v3BlockGridData* grid, b3Vec3 p1, b3Vec3 p2, v3BlockGridQueryScratch* scratch )
{
	v3BlockGridRayResult result = {
		.status = v3_blockGridRayInvalid,
		.hitboxIndex = B3_NULL_INDEX,
		.fraction = 0.0f,
		.faceNormal = b3Vec3_zero,
	};
	if ( grid == NULL || b3IsValidVec3( p1 ) == false || b3IsValidVec3( p2 ) == false ||
		 v3BlockGridIsValidScratch( grid, scratch ) == false )
	{
		return result;
	}

	b3Vec3 direction = b3Sub( p2, p1 );
	if ( b3IsValidVec3( direction ) == false )
	{
		return result;
	}
	b3Vec3 gridOrigin = v3BlockGridWorldOrigin( grid );
	p1 = b3Sub( p1, gridOrigin );
	p2 = b3Sub( p2, gridOrigin );
	b3AABB rayBounds = { b3Min( p1, p2 ), b3Max( p1, p2 ) };
	if ( b3AABB_Overlaps( grid->bounds, rayBounds ) == false )
	{
		result.status = v3_blockGridRayMiss;
		return result;
	}

	int range[6];
	if ( v3BlockGridComputeBlockRange( &grid->frame, rayBounds, v3_blockGridRangeTouching, range ) == false )
	{
		result.status = v3_blockGridRayMiss;
		return result;
	}
	const v3BlockGridHitboxData* hitboxes = v3BlockGridHitboxes( grid );
	int best = B3_NULL_INDEX;
	float bestFraction = 1.0f;
	int bestAxis = -1;
	float bestSign = 0.0f;

	// The cell march is the fast path and touches no scratch at all, so the group
	// walk below only runs for a field that went without an index.
	if ( v3BlockGridProjectionRayCast( grid, p1, direction, &best, &bestFraction, &bestAxis, &bestSign ) == false )
	{
		v3BlockGridDirtyWords dirty = v3BlockGridMarkCandidates( grid, range, scratch );
		for ( int word = dirty.first; word <= dirty.last; ++word )
		{
			uint64_t bits = scratch->visitedHitboxes[word];
			while ( bits != 0 )
			{
				int hitbox = ( word << 6 ) + (int)b3CTZ64( bits );
				bits &= bits - 1;
				if ( hitbox >= grid->hitboxCount )
				{
					continue;
				}
				float candidateFraction;
				int candidateAxis;
				float candidateSign;
				if ( v3BlockGridHitboxRaySlab( hitboxes + hitbox, p1, direction, v3BlockGridRayTestBound( bestFraction ),
											   &candidateFraction, &candidateAxis, &candidateSign ) &&
					 v3BlockGridRayCandidateWins( hitboxes, hitbox, candidateFraction, best, bestFraction ) )
				{
					best = hitbox;
					bestFraction = candidateFraction;
					bestAxis = candidateAxis;
					bestSign = candidateSign;
				}
			}
		}

		// The ray has one exit path from here, so clear the window once.
		v3BlockGridClearDirtyWords( scratch, dirty );
	}

	if ( best == B3_NULL_INDEX )
	{
		result.status = v3_blockGridRayMiss;
		return result;
	}
	result.status = v3_blockGridRayHit;
	result.hitboxIndex = best;
	result.fraction = bestFraction;
	if ( bestAxis >= 0 )
	{
		float* normal = &result.faceNormal.x;
		normal[bestAxis] = bestSign;
	}
	return result;
}

static uint64_t v3BlockGridXMask( int lower, int upper )
{
	uint64_t nibble = ( ( (uint64_t)1 << ( upper - lower + 1 ) ) - 1 ) << lower;
	return nibble * UINT64_C( 0x1111111111111111 );
}

static uint64_t v3BlockGridYMask( int lower, int upper )
{
	uint64_t layer = 0;
	for ( int y = lower; y <= upper; ++y )
	{
		layer |= UINT64_C( 0xF ) << ( 4 * y );
	}
	return layer | ( layer << 16 ) | ( layer << 32 ) | ( layer << 48 );
}

static uint64_t v3BlockGridZMask( int lower, int upper )
{
	uint64_t mask = 0;
	for ( int z = lower; z <= upper; ++z )
	{
		mask |= UINT64_C( 0xFFFF ) << ( 16 * z );
	}
	return mask;
}

int v3BlockGrid_GetOccupiedBlockCount( const v3BlockGridData* grid, b3AABB bounds )
{
	if ( grid == NULL || v3BlockGridIsValidAABB( bounds, false ) == false )
	{
		return 0;
	}
	bounds = v3BlockGridToLocalBounds( grid, bounds );
	if ( b3AABB_Overlaps( grid->bounds, bounds ) == false )
	{
		return 0;
	}

	// Occupancy measures logical block volume. Unlike collision candidates, a
	// block that only touches the query's upper face is not included.
	int range[6];
	if ( v3BlockGridComputeBlockRange( &grid->frame, bounds, v3_blockGridRangeHalfOpen, range ) == false )
	{
		return 0;
	}

	const uint64_t* occupancyMasks = v3BlockGridOccupancyMasks( grid );
	int count = 0;
	for ( int gz = range[2] >> 2; gz <= range[5] >> 2; ++gz )
	{
		int z0 = gz == ( range[2] >> 2 ) ? range[2] & 3 : 0;
		int z1 = gz == ( range[5] >> 2 ) ? range[5] & 3 : 3;
		for ( int gy = range[1] >> 2; gy <= range[4] >> 2; ++gy )
		{
			int y0 = gy == ( range[1] >> 2 ) ? range[1] & 3 : 0;
			int y1 = gy == ( range[4] >> 2 ) ? range[4] & 3 : 3;
			for ( int gx = range[0] >> 2; gx <= range[3] >> 2; ++gx )
			{
				int x0 = gx == ( range[0] >> 2 ) ? range[0] & 3 : 0;
				int x1 = gx == ( range[3] >> 2 ) ? range[3] & 3 : 3;
				int group = v3BlockGridGroupIndex( &grid->direct, gx, gy, gz );
				uint64_t rangeMask = v3BlockGridXMask( x0, x1 ) & v3BlockGridYMask( y0, y1 ) & v3BlockGridZMask( z0, z1 );
				count += b3PopCount64( occupancyMasks[group] & rangeMask );
			}
		}
	}
	return count;
}

v3BlockGridQueryCounters v3BlockGrid_GetQueryCounters( void )
{
#if B3_DEBUG
	return (v3BlockGridQueryCounters){
		.invocations = b3AtomicLoadInt( &s_invocations ),
		.groupsWalked = b3AtomicLoadInt( &s_groupsWalked ),
		.refsMarked = b3AtomicLoadInt( &s_refsMarked ),
		.wordsSpanned = b3AtomicLoadInt( &s_wordsSpanned ),
		.bitsTested = b3AtomicLoadInt( &s_bitsTested ),
		.overlapsFired = b3AtomicLoadInt( &s_overlapsFired ),
	};
#else
	return (v3BlockGridQueryCounters){ 0 };
#endif
}

void v3BlockGrid_ResetQueryCounters( void )
{
#if B3_DEBUG
	b3AtomicStoreInt( &s_invocations, 0 );
	b3AtomicStoreInt( &s_groupsWalked, 0 );
	b3AtomicStoreInt( &s_refsMarked, 0 );
	b3AtomicStoreInt( &s_wordsSpanned, 0 );
	b3AtomicStoreInt( &s_bitsTested, 0 );
	b3AtomicStoreInt( &s_overlapsFired, 0 );
#endif
}
