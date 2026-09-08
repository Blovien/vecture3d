// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdlib.h>

#if defined( _WIN32 )
#include <malloc.h>
#endif

typedef struct CastFixture
{
	b3WorldId gridWorld;
	b3WorldId oracleWorld;
	b3ShapeId gridShape;
	b3CompoundData* compound;
} CastFixture;

typedef struct CastChildIndex
{
	int value;
	bool hit;
} CastChildIndex;

static int castAllocationCount;

static void* CastAlloc( int32_t size, int32_t alignment )
{
	castAllocationCount += 1;
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void CastFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static float CaptureChildIndex( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								int triangleIndex, int childIndex, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	(void)userMaterialId;
	(void)triangleIndex;
	CastChildIndex* captured = context;
	captured->value = childIndex;
	captured->hit = true;
	return fraction;
}

static CastFixture CreateCastFixture( bool reverseBlocks )
{
	v3BlockGridBox cellBox = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock blocks[4];
	for ( int i = 0; i < 4; ++i )
	{
		int x = reverseBlocks ? 3 - i : i;
		blocks[i] = (v3BlockGridBlock){ .x = x, .userData = (uint64_t)( 100 + x ), .boxes = &cellBox, .boxCount = 1 };
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.userMaterialId = 73;
	v3BlockGridCookDef cookDef = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = 4 };
	v3BlockGridCookResult cooked = v3CookBlockGrid( &cookDef );

	b3WorldDef worldDef = b3DefaultWorldDef();
	CastFixture fixture = { .gridWorld = b3CreateWorld( &worldDef ), .oracleWorld = b3CreateWorld( &worldDef ) };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId gridBody = b3CreateBody( fixture.gridWorld, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 2;
	fixture.gridShape = v3CreateBlockGridShape( gridBody, &shapeDef, cooked.data );
	v3DestroyBlockGridData( cooked.data );

	b3BoxHull boxes[4];
	b3CompoundHullDef hulls[4];
	for ( int x = 0; x < 4; ++x )
	{
		boxes[x] = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		hulls[x] = (b3CompoundHullDef){
			.hull = &boxes[x].base,
			.transform = { .p = { (float)x + 0.5f, 0.5f, 0.5f }, .q = b3Quat_identity },
			.material = material,
		};
	}
	b3CompoundDef compoundDef = { .hulls = hulls, .hullCount = 4 };
	fixture.compound = b3CreateCompound( &compoundDef );
	b3BodyId oracleBody = b3CreateBody( fixture.oracleWorld, &bodyDef );
	b3CreateBakedCompoundShape( oracleBody, &shapeDef, fixture.compound );
	return fixture;
}

static void DestroyCastFixture( CastFixture fixture )
{
	b3DestroyWorld( fixture.gridWorld );
	b3DestroyWorld( fixture.oracleWorld );
	b3DestroyCompound( fixture.compound );
}

static int EnsureSameCast( v3ClosestCastResult grid, v3ClosestCastResult oracle )
{
	ENSURE( grid.hit && oracle.hit );
	ENSURE_SMALL( grid.fraction - oracle.fraction, 2.0e-4f );
	ENSURE_SMALL( grid.normal.x - oracle.normal.x, 2.0e-4f );
	ENSURE_SMALL( grid.normal.y - oracle.normal.y, 2.0e-4f );
	ENSURE_SMALL( grid.normal.z - oracle.normal.z, 2.0e-4f );
	ENSURE( grid.isBlockGrid && !oracle.isBlockGrid );
	ENSURE( B3_ID_EQUALS( grid.bodyId, b3Shape_GetBody( grid.shapeId ) ) );
	ENSURE( b3Shape_IsValid( grid.shapeId ) );
	ENSURE( grid.cellX == 1 && grid.cellY == 0 && grid.cellZ == 0 );
	ENSURE( grid.cellBoxIndex == 0 && grid.materialIndex == 0 );
	ENSURE( grid.userMaterialId == 73 && grid.userData == 101 );
	return 0;
}

static int AllCastKindsMatchCompound( void )
{
	CastFixture fixture = CreateCastFixture( false );
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 2;
	b3Pos origin = { 1.25, 3.0, 0.5 };
	b3Vec3 translation = { 0.0f, -4.0f, 0.0f };
	ENSURE( EnsureSameCast( v3World_CastRayClosest( fixture.gridWorld, origin, translation, filter ),
							v3World_CastRayClosest( fixture.oracleWorld, origin, translation, filter ) ) == 0 );

	b3Sphere sphere = { .center = b3Vec3_zero, .radius = 0.2f };
	ENSURE( EnsureSameCast( v3World_CastSphereClosest( fixture.gridWorld, origin, &sphere, translation, filter ),
							v3World_CastSphereClosest( fixture.oracleWorld, origin, &sphere, translation, filter ) ) == 0 );
	b3Capsule capsule = { .center1 = { -0.15f, 0.0f, 0.0f }, .center2 = { 0.15f, 0.0f, 0.0f }, .radius = 0.15f };
	ENSURE( EnsureSameCast( v3World_CastCapsuleClosest( fixture.gridWorld, origin, &capsule, translation, filter ),
							v3World_CastCapsuleClosest( fixture.oracleWorld, origin, &capsule, translation, filter ) ) == 0 );
	ENSURE( EnsureSameCast( v3World_CastBoxClosest( fixture.gridWorld, origin, b3Vec3_zero, (b3Vec3){ 0.2f, 0.2f, 0.2f },
													translation, filter ),
							v3World_CastBoxClosest( fixture.oracleWorld, origin, b3Vec3_zero, (b3Vec3){ 0.2f, 0.2f, 0.2f },
													translation, filter ) ) == 0 );
	b3Vec3 points[4] = { { -0.2f, -0.2f, -0.2f }, { 0.2f, -0.2f, -0.2f }, { 0.0f, -0.2f, 0.2f }, { 0.0f, 0.2f, 0.0f } };
	b3HullData* hull = b3CreateHull( points, 4, 4 );
	ENSURE( EnsureSameCast( v3World_CastHullClosest( fixture.gridWorld, origin, hull, translation, filter ),
							v3World_CastHullClosest( fixture.oracleWorld, origin, hull, translation, filter ) ) == 0 );
	CastChildIndex rayChild = { .value = B3_NULL_INDEX };
	b3World_CastRay( fixture.gridWorld, origin, translation, filter, CaptureChildIndex, &rayChild );
	ENSURE( rayChild.hit && rayChild.value == 0 );
	CastChildIndex shapeChild = { .value = B3_NULL_INDEX };
	b3ShapeProxy sphereProxy = { .points = &sphere.center, .count = 1, .radius = sphere.radius };
	b3World_CastShape( fixture.gridWorld, origin, &sphereProxy, translation, filter, CaptureChildIndex, &shapeChild );
	ENSURE( shapeChild.hit && shapeChild.value == 0 );
	b3DestroyHull( hull );
	DestroyCastFixture( fixture );
	return 0;
}

static int FilterOverlapPermutationAndAllocation( void )
{
	b3SetAllocator( CastAlloc, CastFree );
	CastFixture ordered = CreateCastFixture( false );
	CastFixture reversed = CreateCastFixture( true );
	b3Pos origin = { 1.25, 3.0, 0.5 };
	b3Vec3 translation = { 0.0f, -4.0f, 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = 1;
	ENSURE( !v3World_CastRayClosest( ordered.gridWorld, origin, translation, filter ).hit );
	filter.maskBits = 2;
	v3ClosestCastResult first = v3World_CastRayClosest( ordered.gridWorld, origin, translation, filter );
	v3ClosestCastResult second = v3World_CastRayClosest( reversed.gridWorld, origin, translation, filter );
	ENSURE( first.hit && second.hit && first.cellX == second.cellX && first.cellBoxIndex == second.cellBoxIndex &&
			first.userData == second.userData && first.fraction == second.fraction );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId tiedBody = b3CreateBody( ordered.gridWorld, &bodyDef );
	b3ShapeDef tiedDef = b3DefaultShapeDef();
	tiedDef.filter.categoryBits = 2;
	b3BoxHull tiedBox = b3MakeOffsetBoxHull( 2.0f, 0.5f, 0.5f, (b3Vec3){ 2.0f, 0.5f, 0.5f } );
	b3CreateHullShape( tiedBody, &tiedDef, &tiedBox.base );
	v3ClosestCastResult tied = v3World_CastRayClosest( ordered.gridWorld, origin, translation, filter );
	ENSURE( tied.hit && B3_ID_EQUALS( tied.shapeId, ordered.gridShape ) );

	b3Pos inside = { 1.25, 0.5, 0.5 };
	b3Sphere sphere = { .center = b3Vec3_zero, .radius = 0.2f };
	ENSURE( !v3World_CastRayClosest( ordered.gridWorld, inside, (b3Vec3){ 0.0f, 2.0f, 0.0f }, filter ).hit );
	v3ClosestCastResult gridOverlap =
		v3World_CastSphereClosest( ordered.gridWorld, inside, &sphere, (b3Vec3){ 0.0f, 2.0f, 0.0f }, filter );
	v3ClosestCastResult oracleOverlap =
		v3World_CastSphereClosest( ordered.oracleWorld, inside, &sphere, (b3Vec3){ 0.0f, 2.0f, 0.0f }, filter );
	ENSURE( gridOverlap.hit == oracleOverlap.hit );
	if ( gridOverlap.hit )
	{
		ENSURE( EnsureSameCast( gridOverlap, oracleOverlap ) == 0 );
	}

	for ( int i = 0; i < 8; ++i )
	{
		(void)v3World_CastRayClosest( ordered.gridWorld, origin, translation, filter );
		(void)v3World_CastSphereClosest( ordered.gridWorld, origin, &sphere, translation, filter );
	}
	castAllocationCount = 0;
	for ( int i = 0; i < 100; ++i )
	{
		(void)v3World_CastRayClosest( ordered.gridWorld, origin, translation, filter );
		(void)v3World_CastSphereClosest( ordered.gridWorld, origin, &sphere, translation, filter );
	}
	int allocations = castAllocationCount;
	DestroyCastFixture( ordered );
	DestroyCastFixture( reversed );
	b3SetAllocator( NULL, NULL );
	ENSURE( allocations == 0 );
	return 0;
}

// Neighboring surfaces differ by 2^-20 in ray fraction. They are distinct hits,
// even though each pair is within the former 1e-6 tie tolerance. Adding a box
// away from the ray or changing definition order cannot change the nearest one.
static int NearestInsetRow( int cellCount, bool distantBox, bool reverse )
{
	v3BlockGridBox cellBox = { .bounds = { { 0.125f, 0.125f, 0.125f }, { 0.875f, 0.875f, 0.875f } } };
	v3BlockGridBlock blocks[66];
	b3CompoundHullDef hulls[66];
	b3BoxHull box = b3MakeBoxHull( 0.375f, 0.375f, 0.375f );
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.userMaterialId = 73;
	int total = cellCount + ( distantBox ? 1 : 0 );
	for ( int i = 0; i < total; ++i )
	{
		int slot = reverse ? total - 1 - i : i;
		int x = i < cellCount ? i : 512;
		int z = i < cellCount ? 0 : 64;
		blocks[slot] = (v3BlockGridBlock){ .x = x, .z = z, .userData = (uint64_t)( 1000 + i ), .boxes = &cellBox, .boxCount = 1 };
		hulls[slot] = (b3CompoundHullDef){
			.hull = &box.base,
			.transform = { .p = { (float)x + 0.5f, 0.5f, (float)z + 0.5f }, .q = b3Quat_identity },
			.material = material,
		};
	}
	v3BlockGridCookDef cookDef = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = total };
	v3BlockGridCookResult cooked = v3CookBlockGrid( &cookDef );
	ENSURE( cooked.status == v3_blockGridCookOk );
	b3CompoundDef compoundDef = { .hulls = hulls, .hullCount = total };
	b3WorldDef worldDef = b3DefaultWorldDef();
	CastFixture fixture = { .gridWorld = b3CreateWorld( &worldDef ),
							.oracleWorld = b3CreateWorld( &worldDef ),
							.compound = b3CreateCompound( &compoundDef ) };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	fixture.gridShape = v3CreateBlockGridShape( b3CreateBody( fixture.gridWorld, &bodyDef ), &shapeDef, cooked.data );
	v3DestroyBlockGridData( cooked.data );
	b3CreateBakedCompoundShape( b3CreateBody( fixture.oracleWorld, &bodyDef ), &shapeDef, fixture.compound );

	b3Pos origin = { 524288.875 + cellCount - 1, 0.5, 0.5 };
	b3Vec3 translation = { -1048576.0f, 0.0f, 0.0f };
	v3ClosestCastResult grid = v3World_CastRayClosest( fixture.gridWorld, origin, translation, b3DefaultQueryFilter() );
	v3ClosestCastResult oracle = v3World_CastRayClosest( fixture.oracleWorld, origin, translation, b3DefaultQueryFilter() );
	DestroyCastFixture( fixture );

	// The first upper face is exactly halfway along this ray. No engine output
	// supplies the expected fraction or logical cell.
	ENSURE( oracle.hit && oracle.fraction == 0.5f );
	ENSURE( grid.hit && grid.isBlockGrid && grid.fraction == 0.5f );
	ENSURE( grid.cellX == cellCount - 1 && grid.cellY == 0 && grid.cellZ == 0 );
	ENSURE( grid.cellBoxIndex == 0 && grid.materialIndex == 0 && grid.userMaterialId == 73 );
	ENSURE( grid.userData == (uint64_t)( 1000 + cellCount - 1 ) );
	ENSURE( grid.point.x == (double)cellCount - 0.125 && grid.point.x == oracle.point.x );
	ENSURE( grid.normal.x == 1.0f && grid.normal.y == 0.0f && grid.normal.z == 0.0f );
	return 0;
}

static int NearHitsKeepTheNearestSurface( void )
{
	for ( int count = 63; count <= 65; ++count )
	{
		for ( int distant = 0; distant <= 1; ++distant )
		{
			for ( int reverse = 0; reverse <= 1; ++reverse )
			{
				ENSURE( NearestInsetRow( count, distant != 0, reverse != 0 ) == 0 );
			}
		}
	}
	return 0;
}

int V3BlockGridCastsTest( void )
{
	RUN_SUBTEST( AllCastKindsMatchCompound );
	RUN_SUBTEST( FilterOverlapPermutationAndAllocation );
	RUN_SUBTEST( NearHitsKeepTheNearestSurface );
	return 0;
}
