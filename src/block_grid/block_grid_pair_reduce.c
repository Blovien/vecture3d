// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "manifold.h"
#include "block_grid_pair.h"

#include "box3d/constants.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>

enum
{
	v3_blockGridPairExtremeCount = 8,
};

typedef struct v3BlockGridPairLeaf
{
	v3BlockGridPairManifold manifold;
	b3Vec2 projectedFootprint[v3_blockGridPairMaxFootprintPoints];
	b3Vec2 lower;
	b3Vec2 upper;
	uint32_t materialPairKey;
	uint32_t hitboxPairKey;
	float normalLower;
	float normalUpper;
	int parent;
	int regionIndex;
} v3BlockGridPairLeaf;

typedef struct v3BlockGridPairReducerPoint
{
	b3LocalManifoldPoint point;
	uint32_t hitboxPairKey;
	uint8_t valid;
	uint8_t reserved[3];
} v3BlockGridPairReducerPoint;

typedef struct v3BlockGridPairRegion
{
	b3Vec3 normal;
	b3Vec3 tangentU;
	b3Vec3 tangentV;
	v3BlockGridPairReducerPoint extremes[v3_blockGridPairExtremeCount];
	v3BlockGridPairReducerPoint deepest;
	uint32_t materialPairKey;
	uint32_t supportRegionKey;
	uint8_t directedSatKey;
	uint8_t reserved[3];
} v3BlockGridPairRegion;

// Overflow summaries omit the derived second tangent so they fit in the existing touching pair
// leaf slots. The full form remains in the builder for the ordinary path.
typedef struct v3BlockGridPairOverflowRegion
{
	b3Vec3 normal;
	b3Vec3 tangentU;
	v3BlockGridPairReducerPoint extremes[v3_blockGridPairExtremeCount];
	v3BlockGridPairReducerPoint deepest;
	uint32_t materialPairKey;
	uint32_t supportRegionKey;
	uint8_t directedSatKey;
	uint8_t reserved[3];
} v3BlockGridPairOverflowRegion;

struct v3BlockGridPairBuilder
{
	v3BlockGridPairLeaf* leaves;
	v3BlockGridPairRegion regions[v3_blockGridPairStandardMaxRegions];
	uint64_t touchingPairCount;
	uint64_t correlationTestCount;
	int leafCount;
	int leafCapacity;
	int regionCount;
	uint8_t status;
	uint8_t reductionEngaged;
	uint8_t retainedRegionCount;
	uint8_t reserved[5];
};

typedef struct v3BlockGridPairPoint2
{
	b3Vec2 point;
	uint64_t identity;
} v3BlockGridPairPoint2;

static b3Vec2 v3BlockGridPairMin2( b3Vec2 a, b3Vec2 b )
{
	return (b3Vec2){ b3MinFloat( a.x, b.x ), b3MinFloat( a.y, b.y ) };
}

static b3Vec2 v3BlockGridPairMax2( b3Vec2 a, b3Vec2 b )
{
	return (b3Vec2){ b3MaxFloat( a.x, b.x ), b3MaxFloat( a.y, b.y ) };
}

static b3Vec2 v3BlockGridPairSub2( b3Vec2 a, b3Vec2 b )
{
	return (b3Vec2){ a.x - b.x, a.y - b.y };
}

static float v3BlockGridPairDot2( b3Vec2 a, b3Vec2 b )
{
	return a.x * b.x + a.y * b.y;
}

static float v3BlockGridPairDistanceSquared2( b3Vec2 a, b3Vec2 b )
{
	b3Vec2 d = v3BlockGridPairSub2( a, b );
	return v3BlockGridPairDot2( d, d );
}

static b3Vec2 v3BlockGridPairMulAdd2( b3Vec2 a, float scale, b3Vec2 b )
{
	return (b3Vec2){ a.x + scale * b.x, a.y + scale * b.y };
}

static bool v3BlockGridPairAddBytes( size_t* cursor, size_t bytes )
{
	if ( *cursor > SIZE_MAX - bytes )
	{
		return false;
	}
	*cursor += bytes;
	return true;
}

uint32_t v3BlockGridPairHitboxPairKey( int hitboxIndexA, int hitboxIndexB )
{
	return (uint32_t)hitboxIndexA << 16 | (uint32_t)hitboxIndexB;
}

static uint64_t v3BlockGridPairPointIdentity( const v3BlockGridPairReducerPoint* point )
{
	return (uint64_t)point->hitboxPairKey << 32 | b3MakeFeatureId( point->point.pair );
}

static bool v3BlockGridPairManifoldIsValid( const v3BlockGridPairManifold* manifold )
{
	if ( manifold == NULL || manifold->pointCount <= 0 || manifold->pointCount > 4 || manifold->footprint.pointCount == 0 ||
		 manifold->footprint.pointCount > v3_blockGridPairMaxFootprintPoints ||
		 manifold->directedSatKey >= v3_blockGridPairSatKeyCount || manifold->retainedPreviousAxis > 1 ||
		 manifold->reserved[0] != 0 || manifold->reserved[1] != 0 || manifold->footprint.reserved[0] != 0 ||
		 manifold->footprint.reserved[1] != 0 || manifold->footprint.reserved[2] != 0 ||
		 b3IsValidVec3( manifold->normal ) == false || b3IsNormalized( manifold->normal ) == false )
	{
		return false;
	}

	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		if ( b3IsValidVec3( manifold->points[i].point ) == false || b3IsValidFloat( manifold->points[i].separation ) == false )
		{
			return false;
		}
	}
	for ( int i = 0; i < manifold->footprint.pointCount; ++i )
	{
		if ( b3IsValidVec3( manifold->footprint.points[i] ) == false )
		{
			return false;
		}
	}
	return true;
}

static int v3BlockGridPairCompareLeafPair( const void* a, const void* b )
{
	const v3BlockGridPairLeaf* leafA = a;
	const v3BlockGridPairLeaf* leafB = b;
	return ( leafA->hitboxPairKey > leafB->hitboxPairKey ) - ( leafA->hitboxPairKey < leafB->hitboxPairKey );
}

static int v3BlockGridPairCompareLeafPartition( const void* a, const void* b )
{
	const v3BlockGridPairLeaf* leafA = a;
	const v3BlockGridPairLeaf* leafB = b;
	if ( leafA->manifold.directedSatKey != leafB->manifold.directedSatKey )
	{
		return ( leafA->manifold.directedSatKey > leafB->manifold.directedSatKey ) -
			   ( leafA->manifold.directedSatKey < leafB->manifold.directedSatKey );
	}
	if ( leafA->materialPairKey != leafB->materialPairKey )
	{
		return ( leafA->materialPairKey > leafB->materialPairKey ) - ( leafA->materialPairKey < leafB->materialPairKey );
	}
	return v3BlockGridPairCompareLeafPair( a, b );
}

static int v3BlockGridPairCompareLeafSweep( const void* a, const void* b )
{
	const v3BlockGridPairLeaf* leafA = a;
	const v3BlockGridPairLeaf* leafB = b;
	if ( leafA->lower.x != leafB->lower.x )
	{
		return ( leafA->lower.x > leafB->lower.x ) - ( leafA->lower.x < leafB->lower.x );
	}
	if ( leafA->upper.x != leafB->upper.x )
	{
		return ( leafA->upper.x > leafB->upper.x ) - ( leafA->upper.x < leafB->upper.x );
	}
	return v3BlockGridPairCompareLeafPair( a, b );
}

static void v3BlockGridPairBuildTangentBasis( b3Vec3 normal, b3Vec3* tangentU, b3Vec3* tangentV )
{
	b3Vec3 seed = b3Vec3_axisX;
	float alignment = b3AbsFloat( normal.x );
	if ( b3AbsFloat( normal.y ) < alignment )
	{
		seed = b3Vec3_axisY;
		alignment = b3AbsFloat( normal.y );
	}
	if ( b3AbsFloat( normal.z ) < alignment )
	{
		seed = b3Vec3_axisZ;
	}

	*tangentU = b3Normalize( b3Cross( seed, normal ) );
	*tangentV = b3Cross( normal, *tangentU );
}

static bool v3BlockGridPairProjectLeaf( v3BlockGridPairLeaf* leaf, b3Vec3 normal, b3Vec3 tangentU, b3Vec3 tangentV )
{
	if ( b3Dot( leaf->manifold.normal, normal ) < 1.0f - 32.0f * FLT_EPSILON )
	{
		return false;
	}

	leaf->lower = (b3Vec2){ FLT_MAX, FLT_MAX };
	leaf->upper = (b3Vec2){ -FLT_MAX, -FLT_MAX };
	leaf->normalLower = FLT_MAX;
	leaf->normalUpper = -FLT_MAX;
	for ( int i = 0; i < leaf->manifold.footprint.pointCount; ++i )
	{
		b3Vec3 point = leaf->manifold.footprint.points[i];
		b3Vec2 projected = { b3Dot( point, tangentU ), b3Dot( point, tangentV ) };
		leaf->projectedFootprint[i] = projected;
		leaf->lower = v3BlockGridPairMin2( leaf->lower, projected );
		leaf->upper = v3BlockGridPairMax2( leaf->upper, projected );
		float height = b3Dot( point, normal );
		leaf->normalLower = b3MinFloat( leaf->normalLower, height );
		leaf->normalUpper = b3MaxFloat( leaf->normalUpper, height );
	}
	return true;
}

static float v3BlockGridPairCross2( b3Vec2 a, b3Vec2 b, b3Vec2 c )
{
	return ( b.x - a.x ) * ( c.y - a.y ) - ( b.y - a.y ) * ( c.x - a.x );
}

static bool v3BlockGridPairPointOnSegment( b3Vec2 point, b3Vec2 a, b3Vec2 b )
{
	return v3BlockGridPairCross2( a, b, point ) == 0.0f && point.x >= b3MinFloat( a.x, b.x ) &&
		   point.x <= b3MaxFloat( a.x, b.x ) && point.y >= b3MinFloat( a.y, b.y ) && point.y <= b3MaxFloat( a.y, b.y );
}

static bool v3BlockGridPairSegmentsIntersect( b3Vec2 a1, b3Vec2 a2, b3Vec2 b1, b3Vec2 b2 )
{
	float aa = v3BlockGridPairCross2( a1, a2, b1 );
	float ab = v3BlockGridPairCross2( a1, a2, b2 );
	float ba = v3BlockGridPairCross2( b1, b2, a1 );
	float bb = v3BlockGridPairCross2( b1, b2, a2 );
	if ( ( ( aa > 0.0f && ab < 0.0f ) || ( aa < 0.0f && ab > 0.0f ) ) &&
		 ( ( ba > 0.0f && bb < 0.0f ) || ( ba < 0.0f && bb > 0.0f ) ) )
	{
		return true;
	}
	return ( aa == 0.0f && v3BlockGridPairPointOnSegment( b1, a1, a2 ) ) ||
		   ( ab == 0.0f && v3BlockGridPairPointOnSegment( b2, a1, a2 ) ) ||
		   ( ba == 0.0f && v3BlockGridPairPointOnSegment( a1, b1, b2 ) ) ||
		   ( bb == 0.0f && v3BlockGridPairPointOnSegment( a2, b1, b2 ) );
}

static float v3BlockGridPairPointSegmentDistanceSquared( b3Vec2 point, b3Vec2 a, b3Vec2 b )
{
	b3Vec2 edge = v3BlockGridPairSub2( b, a );
	float lengthSquared = v3BlockGridPairDot2( edge, edge );
	if ( lengthSquared <= FLT_MIN )
	{
		return v3BlockGridPairDistanceSquared2( point, a );
	}
	float fraction = b3ClampFloat( v3BlockGridPairDot2( v3BlockGridPairSub2( point, a ), edge ) / lengthSquared, 0.0f, 1.0f );
	return v3BlockGridPairDistanceSquared2( point, v3BlockGridPairMulAdd2( a, fraction, edge ) );
}

static float v3BlockGridPairSegmentDistanceSquared( b3Vec2 a1, b3Vec2 a2, b3Vec2 b1, b3Vec2 b2 )
{
	if ( v3BlockGridPairSegmentsIntersect( a1, a2, b1, b2 ) )
	{
		return 0.0f;
	}
	float distance = v3BlockGridPairPointSegmentDistanceSquared( a1, b1, b2 );
	distance = b3MinFloat( distance, v3BlockGridPairPointSegmentDistanceSquared( a2, b1, b2 ) );
	distance = b3MinFloat( distance, v3BlockGridPairPointSegmentDistanceSquared( b1, a1, a2 ) );
	return b3MinFloat( distance, v3BlockGridPairPointSegmentDistanceSquared( b2, a1, a2 ) );
}

static bool v3BlockGridPairPointInPolygon( b3Vec2 point, const b3Vec2* polygon, int count )
{
	if ( count < 3 )
	{
		return false;
	}
	b3Vec2 origin = polygon[0];
	double twiceArea = 0.0;
	for ( int i = 1; i < count - 1; ++i )
	{
		double ax = (double)polygon[i].x - origin.x;
		double ay = (double)polygon[i].y - origin.y;
		double bx = (double)polygon[i + 1].x - origin.x;
		double by = (double)polygon[i + 1].y - origin.y;
		twiceArea += ax * by - ay * bx;
	}
	if ( twiceArea == 0.0 )
	{
		return false;
	}

	int sign = 0;
	for ( int i = 0; i < count; ++i )
	{
		double ax = (double)polygon[( i + 1 ) % count].x - polygon[i].x;
		double ay = (double)polygon[( i + 1 ) % count].y - polygon[i].y;
		double bx = (double)point.x - polygon[i].x;
		double by = (double)point.y - polygon[i].y;
		double cross = ax * by - ay * bx;
		if ( cross == 0.0f )
		{
			continue;
		}
		int nextSign = cross > 0.0f ? 1 : -1;
		if ( sign != 0 && sign != nextSign )
		{
			return false;
		}
		sign = nextSign;
	}
	return true;
}

static bool v3BlockGridPairFootprintsCorrelate( const v3BlockGridPairLeaf* a, const v3BlockGridPairLeaf* b )
{
	float tolerance = B3_OVERLAP_SLOP;
	// Projected footprints remain separate when they belong to different support levels
	if ( a->normalUpper + tolerance < b->normalLower || b->normalUpper + tolerance < a->normalLower )
	{
		return false;
	}

	int countA = a->manifold.footprint.pointCount;
	int countB = b->manifold.footprint.pointCount;
	const b3Vec2* pointsA = a->projectedFootprint;
	const b3Vec2* pointsB = b->projectedFootprint;
	if ( v3BlockGridPairPointInPolygon( pointsA[0], pointsB, countB ) ||
		 v3BlockGridPairPointInPolygon( pointsB[0], pointsA, countA ) )
	{
		return true;
	}

	float toleranceSquared = tolerance * tolerance;
	int edgeCountA = countA == 1 ? 1 : countA;
	int edgeCountB = countB == 1 ? 1 : countB;
	for ( int i = 0; i < edgeCountA; ++i )
	{
		b3Vec2 a1 = pointsA[i];
		b3Vec2 a2 = pointsA[countA == 1 ? i : ( i + 1 ) % countA];
		for ( int j = 0; j < edgeCountB; ++j )
		{
			b3Vec2 b1 = pointsB[j];
			b3Vec2 b2 = pointsB[countB == 1 ? j : ( j + 1 ) % countB];
			if ( v3BlockGridPairSegmentDistanceSquared( a1, a2, b1, b2 ) <= toleranceSquared )
			{
				return true;
			}
		}
	}
	return false;
}

static int v3BlockGridPairFindRoot( v3BlockGridPairLeaf* leaves, int index )
{
	int root = index;
	while ( leaves[root].parent != root )
	{
		root = leaves[root].parent;
	}
	while ( leaves[index].parent != index )
	{
		int parent = leaves[index].parent;
		leaves[index].parent = root;
		index = parent;
	}
	return root;
}

static void v3BlockGridPairUnion( v3BlockGridPairLeaf* leaves, int indexA, int indexB )
{
	int rootA = v3BlockGridPairFindRoot( leaves, indexA );
	int rootB = v3BlockGridPairFindRoot( leaves, indexB );
	if ( rootA == rootB )
	{
		return;
	}
	if ( leaves[rootB].hitboxPairKey < leaves[rootA].hitboxPairKey )
	{
		B3_SWAP( rootA, rootB );
	}
	// The lowest pair key owns the component so union order cannot change its anchor
	leaves[rootB].parent = rootA;
}

static int v3BlockGridPairCompareReducerPoints( const void* a, const void* b )
{
	uint64_t keyA = v3BlockGridPairPointIdentity( a );
	uint64_t keyB = v3BlockGridPairPointIdentity( b );
	return ( keyA > keyB ) - ( keyA < keyB );
}

static bool v3BlockGridPairReducerPointIsLower( const v3BlockGridPairReducerPoint* a, const v3BlockGridPairReducerPoint* b )
{
	return b->valid == 0 || v3BlockGridPairPointIdentity( a ) < v3BlockGridPairPointIdentity( b );
}

static void v3BlockGridPairResetRegion( v3BlockGridPairRegion* region, const v3BlockGridPairLeaf* root )
{
	memset( region, 0, sizeof( *region ) );
	region->normal = root->manifold.normal;
	v3BlockGridPairBuildTangentBasis( region->normal, &region->tangentU, &region->tangentV );
	region->materialPairKey = root->materialPairKey;
	region->supportRegionKey = root->hitboxPairKey;
	region->directedSatKey = root->manifold.directedSatKey;
}

static void v3BlockGridPairAddReducerPoint( v3BlockGridPairRegion* region, v3BlockGridPairReducerPoint point )
{
	const float diagonalScale = 0.70710678118654752440f;
	b3Vec3 axes[4] = {
		region->tangentU,
		region->tangentV,
		b3MulSV( diagonalScale, b3Add( region->tangentU, region->tangentV ) ),
		b3MulSV( diagonalScale, b3Sub( region->tangentU, region->tangentV ) ),
	};

	for ( int i = 0; i < 4; ++i )
	{
		float projection = b3Dot( point.point.point, axes[i] );
		int lowerIndex = 2 * i;
		int upperIndex = lowerIndex + 1;
		v3BlockGridPairReducerPoint* lower = region->extremes + lowerIndex;
		v3BlockGridPairReducerPoint* upper = region->extremes + upperIndex;
		float lowerProjection = lower->valid ? b3Dot( lower->point.point, axes[i] ) : FLT_MAX;
		float upperProjection = upper->valid ? b3Dot( upper->point.point, axes[i] ) : -FLT_MAX;
		if ( projection < lowerProjection ||
			 ( projection == lowerProjection && v3BlockGridPairReducerPointIsLower( &point, lower ) ) )
		{
			*lower = point;
		}
		if ( projection > upperProjection ||
			 ( projection == upperProjection && v3BlockGridPairReducerPointIsLower( &point, upper ) ) )
		{
			*upper = point;
		}
	}

	if ( region->deepest.valid == 0 || point.point.separation < region->deepest.point.separation ||
		 ( point.point.separation == region->deepest.point.separation &&
		   v3BlockGridPairReducerPointIsLower( &point, &region->deepest ) ) )
	{
		region->deepest = point;
	}
}

static int v3BlockGridPairComparePoints2( const void* a, const void* b )
{
	const v3BlockGridPairPoint2* pointA = a;
	const v3BlockGridPairPoint2* pointB = b;
	if ( pointA->point.x != pointB->point.x )
	{
		return ( pointA->point.x > pointB->point.x ) - ( pointA->point.x < pointB->point.x );
	}
	if ( pointA->point.y != pointB->point.y )
	{
		return ( pointA->point.y > pointB->point.y ) - ( pointA->point.y < pointB->point.y );
	}
	return ( pointA->identity > pointB->identity ) - ( pointA->identity < pointB->identity );
}

static float v3BlockGridPairPolygonArea( const v3BlockGridPairRegion* region, const v3BlockGridPairReducerPoint* points,
										 int count )
{
	v3BlockGridPairPoint2 sorted[4];
	for ( int i = 0; i < count; ++i )
	{
		sorted[i] = (v3BlockGridPairPoint2){
			.point = { b3Dot( points[i].point.point, region->tangentU ), b3Dot( points[i].point.point, region->tangentV ) },
			.identity = v3BlockGridPairPointIdentity( points + i ),
		};
	}
	qsort( sorted, (size_t)count, sizeof( sorted[0] ), v3BlockGridPairComparePoints2 );

	b3Vec2 hull[8];
	int hullCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		while ( hullCount >= 2 && v3BlockGridPairCross2( hull[hullCount - 2], hull[hullCount - 1], sorted[i].point ) <= 0.0f )
		{
			hullCount -= 1;
		}
		hull[hullCount++] = sorted[i].point;
	}

	int lowerCount = hullCount;
	for ( int i = count - 2; i >= 0; --i )
	{
		while ( hullCount > lowerCount &&
				v3BlockGridPairCross2( hull[hullCount - 2], hull[hullCount - 1], sorted[i].point ) <= 0.0f )
		{
			hullCount -= 1;
		}
		hull[hullCount++] = sorted[i].point;
	}
	if ( hullCount <= 2 )
	{
		return 0.0f;
	}
	hullCount -= 1;

	b3Vec2 origin = hull[0];
	double twiceArea = 0.0;
	for ( int i = 1; i < hullCount - 1; ++i )
	{
		double ax = (double)hull[i].x - origin.x;
		double ay = (double)hull[i].y - origin.y;
		double bx = (double)hull[i + 1].x - origin.x;
		double by = (double)hull[i + 1].y - origin.y;
		twiceArea += ax * by - ay * bx;
	}
	return (float)( 0.5 * ( twiceArea < 0.0 ? -twiceArea : twiceArea ) );
}

static bool v3BlockGridPairIdentitySetIsLower( const v3BlockGridPairReducerPoint* a, const v3BlockGridPairReducerPoint* b,
											   int count )
{
	for ( int i = 0; i < count; ++i )
	{
		uint64_t keyA = v3BlockGridPairPointIdentity( a + i );
		uint64_t keyB = v3BlockGridPairPointIdentity( b + i );
		if ( keyA != keyB )
		{
			return keyA < keyB;
		}
	}
	return false;
}

static float v3BlockGridPairTangentDistanceSquared( const v3BlockGridPairRegion* region, const v3BlockGridPairReducerPoint* a,
													const v3BlockGridPairReducerPoint* b )
{
	b3Vec3 delta = b3Sub( a->point.point, b->point.point );
	float u = b3Dot( delta, region->tangentU );
	float v = b3Dot( delta, region->tangentV );
	return u * u + v * v;
}

static int v3BlockGridPairGatherReducerPoints( v3BlockGridPairReducerPoint points[v3_blockGridPairExtremeCount],
											   const v3BlockGridPairRegion* region )
{
	int count = 0;
	float toleranceSquared = B3_OVERLAP_SLOP * B3_OVERLAP_SLOP;
	for ( int i = 0; i < v3_blockGridPairExtremeCount; ++i )
	{
		v3BlockGridPairReducerPoint candidate = region->extremes[i];
		if ( candidate.valid == 0 )
		{
			continue;
		}

		int duplicate = B3_NULL_INDEX;
		for ( int j = 0; j < count; ++j )
		{
			if ( v3BlockGridPairTangentDistanceSquared( region, &candidate, points + j ) <= toleranceSquared )
			{
				duplicate = j;
				break;
			}
		}
		if ( duplicate == B3_NULL_INDEX )
		{
			points[count++] = candidate;
		}
		else if ( candidate.point.separation < points[duplicate].point.separation ||
				  ( candidate.point.separation == points[duplicate].point.separation &&
					v3BlockGridPairReducerPointIsLower( &candidate, points + duplicate ) ) )
		{
			points[duplicate] = candidate;
		}
	}
	qsort( points, (size_t)count, sizeof( points[0] ), v3BlockGridPairCompareReducerPoints );
	return count;
}

static int v3BlockGridPairSelectLine( v3BlockGridPairReducerPoint selected[4], const v3BlockGridPairRegion* region,
									  const v3BlockGridPairReducerPoint* points, int count,
									  const v3BlockGridPairReducerPoint* deepest )
{
	float bestDistance = B3_OVERLAP_SLOP * B3_OVERLAP_SLOP;
	bool found = false;
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = i + 1; j < count; ++j )
		{
			float distance = v3BlockGridPairTangentDistanceSquared( region, points + i, points + j );
			v3BlockGridPairReducerPoint candidate[2] = { points[i], points[j] };
			if ( distance > bestDistance ||
				 ( distance == bestDistance && found && v3BlockGridPairIdentitySetIsLower( candidate, selected, 2 ) ) )
			{
				selected[0] = candidate[0];
				selected[1] = candidate[1];
				bestDistance = distance;
				found = true;
			}
		}
	}
	if ( found )
	{
		return 2;
	}
	selected[0] = *deepest;
	return 1;
}

static int v3BlockGridPairSelectReducerPoints( v3BlockGridPairReducerPoint selected[4], const v3BlockGridPairRegion* region )
{
	v3BlockGridPairReducerPoint points[v3_blockGridPairExtremeCount];
	int count = v3BlockGridPairGatherReducerPoints( points, region );
	if ( count <= 1 )
	{
		// The deepest point is only the fallback when the support footprint has collapsed
		selected[0] = region->deepest;
		return 1;
	}

	float areaTolerance = B3_OVERLAP_SLOP * B3_OVERLAP_SLOP;
	if ( count <= 4 )
	{
		float area = count >= 3 ? v3BlockGridPairPolygonArea( region, points, count ) : 0.0f;
		if ( area > areaTolerance )
		{
			memcpy( selected, points, (size_t)count * sizeof( points[0] ) );
			return count;
		}
		return v3BlockGridPairSelectLine( selected, region, points, count, &region->deepest );
	}

	float bestArea = areaTolerance;
	bool found = false;
	for ( int a = 0; a < count - 3; ++a )
	{
		for ( int b = a + 1; b < count - 2; ++b )
		{
			for ( int c = b + 1; c < count - 1; ++c )
			{
				for ( int d = c + 1; d < count; ++d )
				{
					v3BlockGridPairReducerPoint candidate[4] = { points[a], points[b], points[c], points[d] };
					float area = v3BlockGridPairPolygonArea( region, candidate, 4 );
					if ( area > bestArea ||
						 ( area == bestArea && found && v3BlockGridPairIdentitySetIsLower( candidate, selected, 4 ) ) )
					{
						memcpy( selected, candidate, sizeof( candidate ) );
						bestArea = area;
						found = true;
					}
				}
			}
		}
	}
	if ( found )
	{
		return 4;
	}
	return v3BlockGridPairSelectLine( selected, region, points, count, &region->deepest );
}

static int v3BlockGridPairCompareRegions( const void* a, const void* b )
{
	const v3BlockGridPairRegion* regionA = a;
	const v3BlockGridPairRegion* regionB = b;
	if ( regionA->directedSatKey != regionB->directedSatKey )
	{
		return ( regionA->directedSatKey > regionB->directedSatKey ) - ( regionA->directedSatKey < regionB->directedSatKey );
	}
	if ( regionA->materialPairKey != regionB->materialPairKey )
	{
		return ( regionA->materialPairKey > regionB->materialPairKey ) - ( regionA->materialPairKey < regionB->materialPairKey );
	}
	return ( regionA->supportRegionKey > regionB->supportRegionKey ) - ( regionA->supportRegionKey < regionB->supportRegionKey );
}

_Static_assert( sizeof( v3BlockGridPairOverflowRegion ) <= sizeof( v3BlockGridPairLeaf ),
				"overflow region summaries must fit in touching pair scratch" );

static b3Vec3 v3BlockGridPairOverflowSupportAxis( const v3BlockGridPairOverflowRegion* region, int axisIndex )
{
	const float diagonalScale = 0.70710678118654752440f;
	b3Vec3 tangentV = b3Cross( region->normal, region->tangentU );
	if ( axisIndex == 0 )
	{
		return region->tangentU;
	}
	if ( axisIndex == 1 )
	{
		return tangentV;
	}
	if ( axisIndex == 2 )
	{
		return b3MulSV( diagonalScale, b3Add( region->tangentU, tangentV ) );
	}
	return b3MulSV( diagonalScale, b3Sub( region->tangentU, tangentV ) );
}

static float v3BlockGridPairOverflowSupportProjection( const v3BlockGridPairOverflowRegion* region, b3Vec3 axis, bool upper )
{
	float projection = upper ? -FLT_MAX : FLT_MAX;
	for ( int i = 0; i < v3_blockGridPairExtremeCount; ++i )
	{
		if ( region->extremes[i].valid == 0 )
		{
			continue;
		}
		float value = b3Dot( region->extremes[i].point.point, axis );
		projection = upper ? b3MaxFloat( projection, value ) : b3MinFloat( projection, value );
	}
	return projection;
}

static bool v3BlockGridPairOverflowKeyIsLower( const v3BlockGridPairOverflowRegion* candidate, int candidateOrdinal,
											   const v3BlockGridPairOverflowRegion* incumbent, int incumbentOrdinal )
{
	if ( candidate->directedSatKey != incumbent->directedSatKey )
	{
		return candidate->directedSatKey < incumbent->directedSatKey;
	}
	if ( candidate->materialPairKey != incumbent->materialPairKey )
	{
		return candidate->materialPairKey < incumbent->materialPairKey;
	}
	if ( candidate->supportRegionKey != incumbent->supportRegionKey )
	{
		return candidate->supportRegionKey < incumbent->supportRegionKey;
	}
	return candidateOrdinal < incumbentOrdinal;
}

static void v3BlockGridPairReadOverflowRegion( v3BlockGridPairOverflowRegion* destination, const v3BlockGridPairLeaf* source )
{
	memcpy( destination, source, sizeof( *destination ) );
}

static void v3BlockGridPairMarkSupportExtremes( const v3BlockGridPairLeaf* leaves, const int* rootIndices, int regionCount,
												bool supportExtreme[v3_blockGridPairMaxTouchingPairs] )
{
	bool processed[v3_blockGridPairMaxTouchingPairs] = { 0 };
	for ( int ordinal = 0; ordinal < regionCount; ++ordinal )
	{
		if ( processed[ordinal] )
		{
			continue;
		}

		v3BlockGridPairOverflowRegion first;
		v3BlockGridPairReadOverflowRegion( &first, leaves + rootIndices[ordinal] );
		int groupCount = 0;
		int groupOrdinals[v3_blockGridPairMaxTouchingPairs];
		for ( int candidate = ordinal; candidate < regionCount; ++candidate )
		{
			v3BlockGridPairOverflowRegion region;
			v3BlockGridPairReadOverflowRegion( &region, leaves + rootIndices[candidate] );
			if ( region.directedSatKey == first.directedSatKey && region.materialPairKey == first.materialPairKey )
			{
				processed[candidate] = true;
				groupOrdinals[groupCount++] = candidate;
			}
		}

		for ( int axisIndex = 0; axisIndex < 4; ++axisIndex )
		{
			b3Vec3 axis = v3BlockGridPairOverflowSupportAxis( &first, axisIndex );
			int lowerOrdinal = B3_NULL_INDEX;
			int upperOrdinal = B3_NULL_INDEX;
			float lowerProjection = FLT_MAX;
			float upperProjection = -FLT_MAX;
			for ( int groupIndex = 0; groupIndex < groupCount; ++groupIndex )
			{
				int candidateOrdinal = groupOrdinals[groupIndex];
				v3BlockGridPairOverflowRegion candidate;
				v3BlockGridPairReadOverflowRegion( &candidate, leaves + rootIndices[candidateOrdinal] );
				float candidateLower = v3BlockGridPairOverflowSupportProjection( &candidate, axis, false );
				float candidateUpper = v3BlockGridPairOverflowSupportProjection( &candidate, axis, true );
				v3BlockGridPairOverflowRegion lower;
				v3BlockGridPairOverflowRegion upper;
				if ( lowerOrdinal != B3_NULL_INDEX )
				{
					v3BlockGridPairReadOverflowRegion( &lower, leaves + rootIndices[lowerOrdinal] );
					v3BlockGridPairReadOverflowRegion( &upper, leaves + rootIndices[upperOrdinal] );
				}
				if ( lowerOrdinal == B3_NULL_INDEX || candidateLower < lowerProjection ||
					 ( candidateLower == lowerProjection &&
					   v3BlockGridPairOverflowKeyIsLower( &candidate, candidateOrdinal, &lower, lowerOrdinal ) ) )
				{
					lowerOrdinal = candidateOrdinal;
					lowerProjection = candidateLower;
				}
				if ( upperOrdinal == B3_NULL_INDEX || candidateUpper > upperProjection ||
					 ( candidateUpper == upperProjection &&
					   v3BlockGridPairOverflowKeyIsLower( &candidate, candidateOrdinal, &upper, upperOrdinal ) ) )
				{
					upperOrdinal = candidateOrdinal;
					upperProjection = candidateUpper;
				}
			}
			if ( lowerOrdinal != B3_NULL_INDEX )
			{
				supportExtreme[lowerOrdinal] = true;
				supportExtreme[upperOrdinal] = true;
			}
		}
	}
}

static bool v3BlockGridPairRegionIsPreferred( const v3BlockGridPairOverflowRegion* candidate, int candidateOrdinal,
											  bool candidateIsExtreme, const v3BlockGridPairOverflowRegion* incumbent,
											  int incumbentOrdinal, bool incumbentIsExtreme )
{
	if ( candidateIsExtreme != incumbentIsExtreme )
	{
		return candidateIsExtreme;
	}
	if ( candidate->deepest.point.separation != incumbent->deepest.point.separation )
	{
		return candidate->deepest.point.separation < incumbent->deepest.point.separation;
	}
	return v3BlockGridPairOverflowKeyIsLower( candidate, candidateOrdinal, incumbent, incumbentOrdinal );
}

static int v3BlockGridPairSelectOverflowRegions( const v3BlockGridPairLeaf* leaves, const int* rootIndices, int regionCount,
												 int selectedOrdinals[v3_blockGridPairStandardMaxRegions] )
{
	bool supportExtreme[v3_blockGridPairMaxTouchingPairs] = { 0 };
	bool selected[v3_blockGridPairMaxTouchingPairs] = { 0 };
	v3BlockGridPairMarkSupportExtremes( leaves, rootIndices, regionCount, supportExtreme );

	int selectedCount = b3MinInt( regionCount, v3_blockGridPairStandardMaxRegions );
	for ( int slot = 0; slot < selectedCount; ++slot )
	{
		int bestOrdinal = B3_NULL_INDEX;
		for ( int candidateOrdinal = 0; candidateOrdinal < regionCount; ++candidateOrdinal )
		{
			if ( selected[candidateOrdinal] )
			{
				continue;
			}
			v3BlockGridPairOverflowRegion candidate;
			v3BlockGridPairReadOverflowRegion( &candidate, leaves + rootIndices[candidateOrdinal] );
			v3BlockGridPairOverflowRegion best;
			if ( bestOrdinal != B3_NULL_INDEX )
			{
				v3BlockGridPairReadOverflowRegion( &best, leaves + rootIndices[bestOrdinal] );
			}
			if ( bestOrdinal == B3_NULL_INDEX ||
				 v3BlockGridPairRegionIsPreferred( &candidate, candidateOrdinal, supportExtreme[candidateOrdinal], &best,
												   bestOrdinal, supportExtreme[bestOrdinal] ) )
			{
				bestOrdinal = candidateOrdinal;
			}
		}
		B3_ASSERT( bestOrdinal != B3_NULL_INDEX );
		selected[bestOrdinal] = true;
		selectedOrdinals[slot] = bestOrdinal;
	}
	return selectedCount;
}

static void v3BlockGridPairStoreOverflowRegion( v3BlockGridPairLeaf* destination, const v3BlockGridPairRegion* source )
{
	v3BlockGridPairOverflowRegion summary = {
		.normal = source->normal,
		.tangentU = source->tangentU,
		.deepest = source->deepest,
		.materialPairKey = source->materialPairKey,
		.supportRegionKey = source->supportRegionKey,
		.directedSatKey = source->directedSatKey,
	};
	memcpy( summary.extremes, source->extremes, sizeof( summary.extremes ) );
	memcpy( destination, &summary, sizeof( summary ) );
}

static void v3BlockGridPairLoadOverflowRegion( v3BlockGridPairRegion* destination, const v3BlockGridPairLeaf* sourceSlot )
{
	v3BlockGridPairOverflowRegion source;
	v3BlockGridPairReadOverflowRegion( &source, sourceSlot );
	memset( destination, 0, sizeof( *destination ) );
	destination->normal = source.normal;
	destination->tangentU = source.tangentU;
	destination->tangentV = b3Cross( source.normal, source.tangentU );
	memcpy( destination->extremes, source.extremes, sizeof( destination->extremes ) );
	destination->deepest = source.deepest;
	destination->materialPairKey = source.materialPairKey;
	destination->supportRegionKey = source.supportRegionKey;
	destination->directedSatKey = source.directedSatKey;
}

static v3BlockGridPairReducedPatch v3BlockGridPairFinishRegion( const v3BlockGridPairRegion* region )
{
	v3BlockGridPairReducerPoint selected[4];
	int count = v3BlockGridPairSelectReducerPoints( selected, region );
	qsort( selected, (size_t)count, sizeof( selected[0] ), v3BlockGridPairCompareReducerPoints );

	v3BlockGridPairReducedPatch patch = {
		.normal = region->normal,
		.materialPairKey = region->materialPairKey,
		.supportRegionKey = region->supportRegionKey,
		.directedSatKey = region->directedSatKey,
		.pointCount = (uint8_t)count,
	};
	for ( int i = 0; i < count; ++i )
	{
		patch.points[i] = selected[i].point;
		patch.hitboxPairKeys[i] = selected[i].hitboxPairKey;
	}
	return patch;
}

size_t v3BlockGridPairBuildScratchByteCount( void )
{
	size_t cursor = 0;
	if ( v3BlockGridPairAddBytes( &cursor, _Alignof( v3BlockGridPairBuilder ) - 1 ) == false ||
		 v3BlockGridPairAddBytes( &cursor, sizeof( v3BlockGridPairBuilder ) ) == false ||
		 v3BlockGridPairAddBytes( &cursor, _Alignof( v3BlockGridPairLeaf ) - 1 ) == false ||
		 v3BlockGridPairAddBytes( &cursor, (size_t)v3_blockGridPairMaxTouchingPairs * sizeof( v3BlockGridPairLeaf ) ) == false )
	{
		return 0;
	}
	return cursor;
}

v3BlockGridPairBuilder* v3BlockGridPairBuildBegin( void* memory, size_t byteCapacity, v3BlockGridPairBuildStatus* status )
{
	if ( status != NULL )
	{
		*status = v3_blockGridPairBuildInvalid;
	}
	if ( memory == NULL )
	{
		return NULL;
	}

	uintptr_t begin = (uintptr_t)memory;
	if ( begin > UINTPTR_MAX - ( _Alignof( v3BlockGridPairBuilder ) - 1 ) )
	{
		return NULL;
	}
	uintptr_t builderAddress =
		( begin + _Alignof( v3BlockGridPairBuilder ) - 1 ) & ~(uintptr_t)( _Alignof( v3BlockGridPairBuilder ) - 1 );
	if ( builderAddress > UINTPTR_MAX - sizeof( v3BlockGridPairBuilder ) )
	{
		return NULL;
	}
	uintptr_t afterBuilder = builderAddress + sizeof( v3BlockGridPairBuilder );
	if ( afterBuilder > UINTPTR_MAX - ( _Alignof( v3BlockGridPairLeaf ) - 1 ) )
	{
		return NULL;
	}
	uintptr_t leafAddress =
		( afterBuilder + _Alignof( v3BlockGridPairLeaf ) - 1 ) & ~(uintptr_t)( _Alignof( v3BlockGridPairLeaf ) - 1 );
	size_t leafBytes = (size_t)v3_blockGridPairMaxTouchingPairs * sizeof( v3BlockGridPairLeaf );
	if ( leafAddress < begin || leafAddress - begin > byteCapacity || leafBytes > byteCapacity - (size_t)( leafAddress - begin ) )
	{
		return NULL;
	}

	v3BlockGridPairBuilder* builder = (v3BlockGridPairBuilder*)builderAddress;
	memset( builder, 0, sizeof( *builder ) );
	builder->leaves = (v3BlockGridPairLeaf*)leafAddress;
	builder->leafCapacity = v3_blockGridPairMaxTouchingPairs;
	builder->status = v3_blockGridPairBuildCollecting;
	if ( status != NULL )
	{
		*status = v3_blockGridPairBuildCollecting;
	}
	return builder;
}

static v3BlockGridPairBuildStatus v3BlockGridPairBuildComponents( v3BlockGridPairBuilder* builder,
														   v3BlockGridPairLeaf* leaves, int count );
static v3BlockGridPairBuildStatus v3BlockGridPairFlushBatch( v3BlockGridPairBuilder* builder );

v3BlockGridPairBuildStatus v3BlockGridPairBuildAdd( v3BlockGridPairBuilder* builder, int hitboxIndexA, int hitboxIndexB,
													uint32_t materialPairKey, const v3BlockGridPairManifold* manifold )
{
	if ( builder == NULL )
	{
		return v3_blockGridPairBuildInvalid;
	}
	if ( builder->status != v3_blockGridPairBuildCollecting )
	{
		return (v3BlockGridPairBuildStatus)builder->status;
	}
	if ( hitboxIndexA < 0 || hitboxIndexA >= UINT16_MAX || hitboxIndexB < 0 || hitboxIndexB >= UINT16_MAX ||
		 v3BlockGridPairManifoldIsValid( manifold ) == false )
	{
		builder->status = v3_blockGridPairBuildInvalid;
		return v3_blockGridPairBuildInvalid;
	}
	if ( builder->touchingPairCount == UINT64_MAX )
	{
		builder->status = v3_blockGridPairBuildInvalid;
		return v3_blockGridPairBuildInvalid;
	}
	if ( builder->leafCount == builder->leafCapacity &&
		 v3BlockGridPairFlushBatch( builder ) != v3_blockGridPairBuildCompleted )
	{
		builder->status = v3_blockGridPairBuildInvalid;
		return v3_blockGridPairBuildInvalid;
	}
	B3_ASSERT( builder->leafCount < builder->leafCapacity );

	v3BlockGridPairLeaf* leaf = builder->leaves + builder->retainedRegionCount + builder->leafCount++;
	memset( leaf, 0, sizeof( *leaf ) );
	leaf->manifold = *manifold;
	leaf->materialPairKey = materialPairKey;
	leaf->hitboxPairKey = v3BlockGridPairHitboxPairKey( hitboxIndexA, hitboxIndexB );
	leaf->parent = B3_NULL_INDEX;
	leaf->regionIndex = B3_NULL_INDEX;
	builder->touchingPairCount += 1;
	return v3_blockGridPairBuildCollecting;
}

static v3BlockGridPairBuildStatus v3BlockGridPairBuildComponents( v3BlockGridPairBuilder* builder,
														   v3BlockGridPairLeaf* leaves, int count )
{
	qsort( leaves, (size_t)count, sizeof( leaves[0] ), v3BlockGridPairCompareLeafPair );
	for ( int i = 1; i < count; ++i )
	{
		if ( leaves[i - 1].hitboxPairKey == leaves[i].hitboxPairKey )
		{
			return v3_blockGridPairBuildInvalid;
		}
	}

	qsort( leaves, (size_t)count, sizeof( leaves[0] ), v3BlockGridPairCompareLeafPartition );

	int partitionBegin = 0;
	while ( partitionBegin < count )
	{
		int partitionEnd = partitionBegin + 1;
		while ( partitionEnd < count &&
				leaves[partitionEnd].manifold.directedSatKey == leaves[partitionBegin].manifold.directedSatKey &&
				leaves[partitionEnd].materialPairKey == leaves[partitionBegin].materialPairKey )
		{
			partitionEnd += 1;
		}

		b3Vec3 normal = leaves[partitionBegin].manifold.normal;
		b3Vec3 tangentU;
		b3Vec3 tangentV;
		v3BlockGridPairBuildTangentBasis( normal, &tangentU, &tangentV );
		b3Vec2 partitionLower = { FLT_MAX, FLT_MAX };
		b3Vec2 partitionUpper = { -FLT_MAX, -FLT_MAX };
		for ( int i = partitionBegin; i < partitionEnd; ++i )
		{
			if ( v3BlockGridPairProjectLeaf( leaves + i, normal, tangentU, tangentV ) == false )
			{
				return v3_blockGridPairBuildInvalid;
			}
			partitionLower = v3BlockGridPairMin2( partitionLower, leaves[i].lower );
			partitionUpper = v3BlockGridPairMax2( partitionUpper, leaves[i].upper );
		}
		if ( partitionUpper.y - partitionLower.y > partitionUpper.x - partitionLower.x )
		{
			// Sweep the wider tangent axis to avoid comparing every pad in a long row
			for ( int i = partitionBegin; i < partitionEnd; ++i )
			{
				B3_SWAP( leaves[i].lower.x, leaves[i].lower.y );
				B3_SWAP( leaves[i].upper.x, leaves[i].upper.y );
				for ( int j = 0; j < leaves[i].manifold.footprint.pointCount; ++j )
				{
					B3_SWAP( leaves[i].projectedFootprint[j].x, leaves[i].projectedFootprint[j].y );
				}
			}
		}
		qsort( leaves + partitionBegin, (size_t)( partitionEnd - partitionBegin ), sizeof( leaves[0] ),
			   v3BlockGridPairCompareLeafSweep );
		for ( int i = partitionBegin; i < partitionEnd; ++i )
		{
			leaves[i].parent = i;
			leaves[i].regionIndex = B3_NULL_INDEX;
		}

		for ( int i = partitionBegin; i < partitionEnd; ++i )
		{
			for ( int j = i + 1; j < partitionEnd && leaves[j].lower.x <= leaves[i].upper.x + B3_OVERLAP_SLOP; ++j )
			{
					// One batch contains at most 4,096 leaves, so this loop performs at most
					// 4,096 * 4,095 / 2 comparisons before the batch is reduced.
					if ( builder->correlationTestCount == UINT64_MAX )
				{
					return v3_blockGridPairBuildInvalid;
				}
				builder->correlationTestCount += 1;
				if ( leaves[i].upper.y + B3_OVERLAP_SLOP < leaves[j].lower.y ||
					 leaves[j].upper.y + B3_OVERLAP_SLOP < leaves[i].lower.y )
				{
					continue;
				}
				if ( v3BlockGridPairFootprintsCorrelate( leaves + i, leaves + j ) )
				{
					v3BlockGridPairUnion( leaves, i, j );
				}
			}
		}
		partitionBegin = partitionEnd;
	}

	int regionCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		if ( v3BlockGridPairFindRoot( leaves, i ) == i )
		{
			regionCount += 1;
		}
	}
	builder->regionCount = regionCount;
	if ( regionCount > v3_blockGridPairStandardMaxRegions )
	{
		int rootIndices[v3_blockGridPairMaxTouchingPairs];
		bool rootFlags[v3_blockGridPairMaxTouchingPairs] = { 0 };
		int nextRegion = 0;
		for ( int i = 0; i < count; ++i )
		{
			int root = v3BlockGridPairFindRoot( leaves, i );
			if ( leaves[root].regionIndex == B3_NULL_INDEX )
			{
				B3_ASSERT( nextRegion < regionCount );
				leaves[root].regionIndex = nextRegion;
				rootIndices[nextRegion] = root;
				rootFlags[root] = true;
				nextRegion += 1;
			}
			leaves[i].regionIndex = leaves[root].regionIndex;
		}
		B3_ASSERT( nextRegion == regionCount );

		// The compact summaries fit in the existing leaf slots. Keep each summary in its root slot
		// until every leaf has contributed, then copy the bounded selection into the ordinary region array.
		for ( int ordinal = 0; ordinal < regionCount; ++ordinal )
		{
			int root = rootIndices[ordinal];
			v3BlockGridPairRegion summary;
			v3BlockGridPairResetRegion( &summary, leaves + root );
			for ( int pointIndex = 0; pointIndex < leaves[root].manifold.pointCount; ++pointIndex )
			{
				v3BlockGridPairAddReducerPoint( &summary, (v3BlockGridPairReducerPoint){
															  .point = leaves[root].manifold.points[pointIndex],
															  .hitboxPairKey = leaves[root].hitboxPairKey,
															  .valid = 1,
														  } );
			}
			v3BlockGridPairStoreOverflowRegion( leaves + root, &summary );
		}
		for ( int i = 0; i < count; ++i )
		{
			if ( rootFlags[i] )
			{
				continue;
			}
			int root = rootIndices[leaves[i].regionIndex];
			v3BlockGridPairRegion summary;
			v3BlockGridPairLoadOverflowRegion( &summary, leaves + root );
			for ( int pointIndex = 0; pointIndex < leaves[i].manifold.pointCount; ++pointIndex )
			{
				v3BlockGridPairAddReducerPoint( &summary, (v3BlockGridPairReducerPoint){
															  .point = leaves[i].manifold.points[pointIndex],
															  .hitboxPairKey = leaves[i].hitboxPairKey,
															  .valid = 1,
														  } );
			}
			v3BlockGridPairStoreOverflowRegion( leaves + root, &summary );
		}

		int selectedOrdinals[v3_blockGridPairStandardMaxRegions];
		int selectedCount = v3BlockGridPairSelectOverflowRegions( leaves, rootIndices, regionCount, selectedOrdinals );
		for ( int i = 0; i < selectedCount; ++i )
		{
			v3BlockGridPairRegion summary;
			v3BlockGridPairLoadOverflowRegion( &summary, leaves + rootIndices[selectedOrdinals[i]] );
			builder->regions[i] = summary;
		}
		builder->regionCount = selectedCount;
		builder->reductionEngaged = 1;
		return v3_blockGridPairBuildCompleted;
	}

	int nextRegion = 0;
	for ( int i = 0; i < count; ++i )
	{
		int root = v3BlockGridPairFindRoot( leaves, i );
		if ( leaves[root].regionIndex == B3_NULL_INDEX )
		{
			leaves[root].regionIndex = nextRegion;
			v3BlockGridPairResetRegion( builder->regions + nextRegion, leaves + root );
			nextRegion += 1;
		}

		v3BlockGridPairRegion* region = builder->regions + leaves[root].regionIndex;
		for ( int j = 0; j < leaves[i].manifold.pointCount; ++j )
		{
			v3BlockGridPairReducerPoint point = {
				.point = leaves[i].manifold.points[j],
				.hitboxPairKey = leaves[i].hitboxPairKey,
				.valid = 1,
			};
			v3BlockGridPairAddReducerPoint( region, point );
		}
	}
	B3_ASSERT( nextRegion == regionCount );
	return v3_blockGridPairBuildCompleted;
}

// A completed batch contributes at most 120 real region summaries. Prior summaries occupy the
// beginning of the same leaf allocation, so each later batch has at least 3,976 leaf slots. The
// selector therefore sees at most 240 summaries and the builder scratch byte count stays unchanged.
static v3BlockGridPairBuildStatus v3BlockGridPairFlushBatch( v3BlockGridPairBuilder* builder )
{
	if ( builder->leafCount == 0 )
	{
		return v3_blockGridPairBuildCompleted;
	}

	int retainedCount = builder->retainedRegionCount;
	v3BlockGridPairLeaf* batch = builder->leaves + retainedCount;
	v3BlockGridPairBuildStatus status = v3BlockGridPairBuildComponents( builder, batch, builder->leafCount );
	if ( status != v3_blockGridPairBuildCompleted )
	{
		return status;
	}

	int currentCount = builder->regionCount;
	B3_ASSERT( retainedCount + currentCount <= 2 * v3_blockGridPairStandardMaxRegions );
	for ( int i = 0; i < currentCount; ++i )
	{
		v3BlockGridPairStoreOverflowRegion( builder->leaves + retainedCount + i, builder->regions + i );
	}

	int combinedCount = retainedCount + currentCount;
	int selectedCount = combinedCount;
	if ( combinedCount > v3_blockGridPairStandardMaxRegions )
	{
		int rootIndices[2 * v3_blockGridPairStandardMaxRegions];
		int selectedOrdinals[v3_blockGridPairStandardMaxRegions];
		for ( int i = 0; i < combinedCount; ++i )
		{
			rootIndices[i] = i;
		}
		selectedCount = v3BlockGridPairSelectOverflowRegions( builder->leaves, rootIndices, combinedCount, selectedOrdinals );
		for ( int i = 0; i < selectedCount; ++i )
		{
			v3BlockGridPairLoadOverflowRegion( builder->regions + i, builder->leaves + selectedOrdinals[i] );
		}
	}
	else
	{
		for ( int i = 0; i < selectedCount; ++i )
		{
			v3BlockGridPairLoadOverflowRegion( builder->regions + i, builder->leaves + i );
		}
	}
	for ( int i = 0; i < selectedCount; ++i )
	{
		v3BlockGridPairStoreOverflowRegion( builder->leaves + i, builder->regions + i );
	}

	builder->retainedRegionCount = (uint8_t)selectedCount;
	builder->leafCount = 0;
	builder->leafCapacity = v3_blockGridPairMaxTouchingPairs - selectedCount;
	builder->regionCount = 0;
	builder->reductionEngaged = 1;
	return v3_blockGridPairBuildCompleted;
}

v3BlockGridPairBuildResult v3BlockGridPairBuildFinish( v3BlockGridPairBuilder* builder, v3BlockGridPairReducedPatch* patches,
													   int patchCapacity )
{
	v3BlockGridPairBuildResult result = { .status = v3_blockGridPairBuildInvalid };
	if ( builder == NULL )
	{
		return result;
	}
	result.touchingPairCount = builder->touchingPairCount;
	if ( builder->status != v3_blockGridPairBuildCollecting )
	{
		result.status = (v3BlockGridPairBuildStatus)builder->status;
		return result;
	}

	v3BlockGridPairBuildStatus status;
	if ( builder->retainedRegionCount == 0 )
	{
		status = v3BlockGridPairBuildComponents( builder, builder->leaves, builder->leafCount );
	}
	else
	{
		status = v3BlockGridPairFlushBatch( builder );
		if ( status == v3_blockGridPairBuildCompleted )
		{
			builder->regionCount = builder->retainedRegionCount;
			for ( int i = 0; i < builder->regionCount; ++i )
			{
				v3BlockGridPairLoadOverflowRegion( builder->regions + i, builder->leaves + i );
			}
		}
	}
	if ( status != v3_blockGridPairBuildCompleted )
	{
		builder->status = status;
		result.status = status;
		return result;
	}
	result.reductionEngaged = builder->reductionEngaged != 0;
	if ( patchCapacity < builder->regionCount || ( builder->regionCount > 0 && patches == NULL ) )
	{
		builder->status = v3_blockGridPairBuildInvalid;
		return result;
	}

	qsort( builder->regions, (size_t)builder->regionCount, sizeof( builder->regions[0] ), v3BlockGridPairCompareRegions );
	int pointCount = 0;
	for ( int i = 0; i < builder->regionCount; ++i )
	{
		patches[i] = v3BlockGridPairFinishRegion( builder->regions + i );
		pointCount += patches[i].pointCount;
	}
	B3_ASSERT( pointCount <= v3_blockGridPairStandardMaxPoints );
	(void)pointCount;

	result.patchCount = builder->regionCount;
	result.status = v3_blockGridPairBuildCompleted;
	builder->status = v3_blockGridPairBuildCompleted;
	return result;
}

v3BlockGridPairPointMatchMetrics v3BlockGridPairMeasureReducedPointMatch( const v3BlockGridPairReducedPatch* current,
																		  int currentPoint,
																		  const v3BlockGridPairReducedPatch* previous,
																		  int previousPoint )
{
	const b3LocalManifoldPoint* currentLocalPoint = current->points + currentPoint;
	const b3LocalManifoldPoint* previousLocalPoint = previous->points + previousPoint;
	b3Vec3 currentAnchorA = b3MulSub( currentLocalPoint->point, 0.5f * currentLocalPoint->separation, current->normal );
	b3Vec3 currentAnchorB = b3MulAdd( currentLocalPoint->point, 0.5f * currentLocalPoint->separation, current->normal );
	b3Vec3 previousAnchorA = b3MulSub( previousLocalPoint->point, 0.5f * previousLocalPoint->separation, previous->normal );
	b3Vec3 previousAnchorB = b3MulAdd( previousLocalPoint->point, 0.5f * previousLocalPoint->separation, previous->normal );
	float distanceSquaredA = b3DistanceSquared( currentAnchorA, previousAnchorA );
	float distanceSquaredB = b3DistanceSquared( currentAnchorB, previousAnchorB );
	return (v3BlockGridPairPointMatchMetrics){
		.anchorDistanceSquaredA = distanceSquaredA,
		.anchorDistanceSquaredB = distanceSquaredB,
		.maximumAnchorDistanceSquared = distanceSquaredA > distanceSquaredB ? distanceSquaredA : distanceSquaredB,
		.summedAnchorDistanceSquared = distanceSquaredA + distanceSquaredB,
		.normalDot = b3Dot( current->normal, previous->normal ),
	};
}

bool v3BlockGridPairReducedPointHasCompatibleGeometry( const v3BlockGridPairReducedPatch* current, int currentPoint,
													   const v3BlockGridPairReducedPatch* previous, int previousPoint )
{
	if ( current == NULL || previous == NULL || currentPoint < 0 || currentPoint >= current->pointCount || previousPoint < 0 ||
		 previousPoint >= previous->pointCount )
	{
		return false;
	}

	v3BlockGridPairPointMatchMetrics metrics =
		v3BlockGridPairMeasureReducedPointMatch( current, currentPoint, previous, previousPoint );
	// Box3D stores a quaternion half angle threshold, while normals use the full angle
	float normalDotThreshold = 2.0f * B3_CONTACT_RECYCLE_ANGULAR_DISTANCE - 1.0f;
	float distance = B3_CONTACT_RECYCLE_DISTANCE;
	return metrics.normalDot >= normalDotThreshold && metrics.anchorDistanceSquaredA <= distance * distance &&
		   metrics.anchorDistanceSquaredB <= distance * distance;
}
