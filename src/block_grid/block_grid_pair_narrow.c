// SPDX-License-Identifier: MIT

#include "manifold.h"
#include "math_internal.h"
#include "block_grid_internal.h"
#include "block_grid_pair.h"
#include "block_grid_shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"

#include <float.h>
#include <string.h>

static bool v3BlockGridPairCacheIsValid( const b3SATCache* cache )
{
	if ( cache == NULL || b3IsValidFloat( cache->separation ) == false )
	{
		return false;
	}

	switch ( cache->type )
	{
		case b3_invalidAxis:
			return true;

		case b3_faceAxisA:
			return cache->indexA < v3_blockGridPairBoxFaceCount;

		case b3_faceAxisB:
			return cache->indexB < v3_blockGridPairBoxFaceCount;

		case b3_edgePairAxis:
			return cache->indexA < v3_blockGridPairBoxEdgeCount && cache->indexB < v3_blockGridPairBoxEdgeCount &&
				   ( cache->indexA & 1 ) == 0 && ( cache->indexB & 1 ) == 0;

		default:
			return false;
	}
}

static int v3BlockGridPairEdgeAxis( int edgeIndex )
{
	if ( edgeIndex >= 16 )
	{
		return 0;
	}

	return ( edgeIndex & 2 ) == 0 ? 1 : 2;
}

static b3Vec3 v3BlockGridPairAxis( int axis )
{
	switch ( axis )
	{
		case 0:
			return b3Vec3_axisX;

		case 1:
			return b3Vec3_axisY;

		default:
			return b3Vec3_axisZ;
	}
}

static int v3BlockGridPairAlignedFace( b3Vec3 normal )
{
	int axis = b3MaxElementIndex( b3Abs( normal ) );
	float component = b3GetByIndex( normal, axis );
	if ( b3AbsFloat( component ) < 1.0f - 32.0f * FLT_EPSILON )
	{
		return B3_NULL_INDEX;
	}

	return 2 * axis + ( component > 0.0f );
}

// A face covered by the neighbouring cells of its own grid is not part of the aggregate boundary.
// Whether it is, is a property of the cooked field, so the cook records it as six bits on the
// Hitbox and this reads one of them.
static bool v3BlockGridPairHitboxFaceIsBuried( const v3BlockGridData* grid, int hitboxIndex, int faceIndex )
{
	return ( v3BlockGridHitboxes( grid )[hitboxIndex].faceMask & (uint32_t)( 1u << faceIndex ) ) != 0;
}

static bool v3BlockGridPairHasBuriedShallowFace( const v3BlockGridData* gridA, int hitboxIndexA, const v3BlockGridData* gridB,
												 int hitboxIndexB, b3Transform transformBtoA, const b3LocalManifold* manifold )
{
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		if ( manifold->points[i].separation < -B3_LINEAR_SLOP )
		{
			return false;
		}
	}

	int faceA = v3BlockGridPairAlignedFace( manifold->normal );
	if ( faceA != B3_NULL_INDEX && v3BlockGridPairHitboxFaceIsBuried( gridA, hitboxIndexA, faceA ) )
	{
		return true;
	}

	b3Vec3 normalB = b3Neg( b3InvRotateVector( transformBtoA.q, manifold->normal ) );
	int faceB = v3BlockGridPairAlignedFace( normalB );
	return faceB != B3_NULL_INDEX && v3BlockGridPairHitboxFaceIsBuried( gridB, hitboxIndexB, faceB );
}

static bool v3BlockGridPairMapSatKey( const b3SeparatingAxis* axis, b3Transform transformBtoA, uint8_t* key )
{
	if ( axis->type == b3_faceAxisA && axis->indexA >= 0 && axis->indexA < v3_blockGridPairBoxFaceCount )
	{
		*key = (uint8_t)axis->indexA;
		return true;
	}

	if ( axis->type == b3_faceAxisB && axis->indexB >= 0 && axis->indexB < v3_blockGridPairBoxFaceCount )
	{
		// A-to-B points away from B's reference face
		*key = (uint8_t)( v3_blockGridPairBoxFaceCount + ( axis->indexB ^ 1 ) );
		return true;
	}

	if ( axis->type != b3_edgePairAxis || axis->indexA < 0 || axis->indexA >= v3_blockGridPairBoxEdgeCount || axis->indexB < 0 ||
		 axis->indexB >= v3_blockGridPairBoxEdgeCount || ( axis->indexA & 1 ) != 0 || ( axis->indexB & 1 ) != 0 )
	{
		return false;
	}

	int axisA = v3BlockGridPairEdgeAxis( axis->indexA );
	int axisB = v3BlockGridPairEdgeAxis( axis->indexB );
	b3Matrix3 rotation = b3MakeMatrixFromQuat( transformBtoA.q );
	b3Vec3 axesB[3] = { rotation.cx, rotation.cy, rotation.cz };
	b3Vec3 canonical = b3Cross( v3BlockGridPairAxis( axisA ), axesB[axisB] );
	if ( b3LengthSquared( canonical ) <= FLT_MIN )
	{
		return false;
	}

	float alignment = b3Dot( axis->normal, canonical );
	if ( b3IsValidFloat( alignment ) == false || alignment == 0.0f )
	{
		return false;
	}

	int family = axisA * 3 + axisB;
	int sign = alignment > 0.0f ? 1 : 0;
	*key = (uint8_t)( v3_blockGridPairFaceSatKeyCount + 2 * family + sign );
	return true;
}

static bool v3BlockGridPairBuildAxis( b3LocalManifold* manifold, b3LocalManifoldPoint points[4], b3LocalManifold* footprint,
									  b3LocalManifoldPoint footprintPoints[v3_blockGridPairMaxFootprintPoints], b3SATCache* cache,
									  const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA,
									  b3SeparatingAxis axis, float admissionDistance )
{
	*manifold = (b3LocalManifold){ .points = points };
	*footprint = (b3LocalManifold){ .points = footprintPoints };
	return b3BuildHullManifoldForAxisWithFootprint( manifold, 4, footprint, v3_blockGridPairMaxFootprintPoints, hullA, hullB,
													transformBtoA, axis, cache, admissionDistance );
}

v3BlockGridPairManifoldStatus v3BlockGridPairCollideHitboxes( const v3BlockGridData* gridA, int hitboxIndexA,
															  const v3BlockGridData* gridB, int hitboxIndexB,
															  b3Transform transformBtoA, const b3SATCache* previousCache,
															  v3BlockGridPairManifold* result, float admissionDistance )
{
	if ( result == NULL )
	{
		return v3_blockGridPairManifoldInvalid;
	}
	*result = (v3BlockGridPairManifold){ .directedSatKey = v3_blockGridPairSatKeyInvalid };

	if ( gridA == NULL || gridB == NULL || hitboxIndexA < 0 || hitboxIndexA >= v3BlockGrid_GetHitboxCount( gridA ) ||
		 hitboxIndexB < 0 || hitboxIndexB >= v3BlockGrid_GetHitboxCount( gridB ) ||
		 b3IsValidTransform( transformBtoA ) == false || !b3IsValidFloat( admissionDistance ) || admissionDistance < 0.0f ||
		 ( previousCache != NULL && v3BlockGridPairCacheIsValid( previousCache ) == false ) )
	{
		return v3_blockGridPairManifoldInvalid;
	}

	b3BoxHull boxA = v3MakeBlockGridHitboxHull( gridA, hitboxIndexA );
	b3BoxHull boxB = v3MakeBlockGridHitboxHull( gridB, hitboxIndexB );
	b3AxisQuery query = b3ComputeSeparatingAxis( &boxA.base, &boxB.base, transformBtoA, false );
	b3SeparatingAxis exactAxis = b3GetBestAxis( &query );
	if ( exactAxis.separation > admissionDistance )
	{
		return v3_blockGridPairManifoldEmpty;
	}

	b3LocalManifoldPoint selectedPoints[4] = { 0 };
	b3LocalManifold selectedManifold = { .points = selectedPoints };
	b3LocalManifoldPoint selectedFootprintPoints[v3_blockGridPairMaxFootprintPoints] = { 0 };
	b3LocalManifold selectedFootprint = { .points = selectedFootprintPoints };
	b3SATCache selectedCache;
	if ( b3BuildHullManifoldFromQueryWithFootprint( &selectedManifold, 4, &selectedFootprint, v3_blockGridPairMaxFootprintPoints,
													&boxA.base, &boxB.base, transformBtoA, &query, &selectedCache,
													admissionDistance ) == false )
	{
		return v3_blockGridPairManifoldEmpty;
	}

	b3SeparatingAxis selectedAxis;
	uint8_t selectedKey;
	if ( b3EvaluateHullFeature( &selectedAxis, &boxA.base, &boxB.base, transformBtoA, &selectedCache ) == false ||
		 v3BlockGridPairMapSatKey( &selectedAxis, transformBtoA, &selectedKey ) == false )
	{
		return v3_blockGridPairManifoldInvalid;
	}

	bool retainedPreviousAxis = false;
	if ( previousCache != NULL && previousCache->type != b3_invalidAxis )
	{
		b3SeparatingAxis previousAxis;
		uint8_t previousKey;
		b3LocalManifoldPoint previousPoints[4] = { 0 };
		b3LocalManifold previousManifold;
		b3LocalManifoldPoint previousFootprintPoints[v3_blockGridPairMaxFootprintPoints] = { 0 };
		b3LocalManifold previousFootprint;
		b3SATCache rebuiltCache;

		// Compare the old feature at the current pose, never against its stale cached separation
		if ( b3EvaluateHullFeature( &previousAxis, &boxA.base, &boxB.base, transformBtoA, previousCache ) &&
			 v3BlockGridPairMapSatKey( &previousAxis, transformBtoA, &previousKey ) &&
			 previousAxis.separation >= exactAxis.separation - B3_LINEAR_SLOP &&
			 v3BlockGridPairBuildAxis( &previousManifold, previousPoints, &previousFootprint, previousFootprintPoints,
									   &rebuiltCache, &boxA.base, &boxB.base, transformBtoA, previousAxis, admissionDistance ) )
		{
			selectedManifold = previousManifold;
			memcpy( selectedPoints, previousPoints, (size_t)previousManifold.pointCount * sizeof( b3LocalManifoldPoint ) );
			selectedManifold.points = selectedPoints;
			selectedFootprint = previousFootprint;
			memcpy( selectedFootprintPoints, previousFootprintPoints,
					(size_t)previousFootprint.pointCount * sizeof( b3LocalManifoldPoint ) );
			selectedFootprint.points = selectedFootprintPoints;
			selectedCache = rebuiltCache;
			selectedKey = previousKey;
			retainedPreviousAxis = true;
		}
	}

	if ( selectedManifold.pointCount <= 0 || selectedManifold.pointCount > 4 || selectedFootprint.pointCount <= 0 ||
		 selectedFootprint.pointCount > v3_blockGridPairMaxFootprintPoints )
	{
		return v3_blockGridPairManifoldEmpty;
	}

	if ( selectedKey < v3_blockGridPairFaceSatKeyCount &&
		 v3BlockGridPairHasBuriedShallowFace( gridA, hitboxIndexA, gridB, hitboxIndexB, transformBtoA, &selectedManifold ) )
	{
		return v3_blockGridPairManifoldEmpty;
	}
	result->normal = selectedManifold.normal;
	result->satCache = selectedCache;
	result->pointCount = selectedManifold.pointCount;
	result->directedSatKey = selectedKey;
	result->retainedPreviousAxis = retainedPreviousAxis;
	memcpy( result->points, selectedManifold.points, (size_t)selectedManifold.pointCount * sizeof( b3LocalManifoldPoint ) );
	result->footprint.pointCount = (uint8_t)selectedFootprint.pointCount;
	for ( int i = 0; i < selectedFootprint.pointCount; ++i )
	{
		result->footprint.points[i] = selectedFootprint.points[i].point;
	}
	return v3_blockGridPairManifoldTouching;
}
