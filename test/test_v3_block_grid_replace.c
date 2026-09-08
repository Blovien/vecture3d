// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

// Transactional publication of BlockGrid revisions at the public seam: cook a new revision,
// replace the one a live shape holds, and read identity, bounds, mass, contacts, counters and
// allocator accounting back through the published API. Nothing here reaches into the shape.

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#include <malloc.h>
#endif

#define V3_REPLACE_ENSURE( C )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( C ) == false )                                                                                                    \
		{                                                                                                                        \
			printf( "condition false at %s:%d: " #C "\n", __FILE__, __LINE__ );                                                  \
			status = 1;                                                                                                          \
			goto cleanup;                                                                                                        \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static const float replaceTimeStep = 1.0f / 60.0f;

enum
{
	replaceSubStepCount = 4,
	settleSteps = 180,
	observationSteps = 60,
	maximumCells = 512,
};

typedef struct ReplaceCell
{
	int x, y, z;
} ReplaceCell;

// Every cell is one full unit cube, so a revision is described by its occupied cells alone and
// two revisions differ exactly where a block was added or removed.
static v3BlockGridData* CookCells( const ReplaceCell* cells, int cellCount, float friction )
{
	static v3BlockGridBox boxes[maximumCells];
	static v3BlockGridBlock blocks[maximumCells];
	if ( cellCount <= 0 || cellCount > maximumCells )
	{
		return NULL;
	}

	for ( int i = 0; i < cellCount; ++i )
	{
		boxes[i] = (v3BlockGridBox){ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
		blocks[i] = (v3BlockGridBlock){
			.x = cells[i].x,
			.y = cells[i].y,
			.z = cells[i].z,
			// The user data doubles as the block identity the assertions read back.
			.userData = 0x1000u + (uint64_t)i,
			.boxes = boxes + i,
			.boxCount = 1,
		};
	}

	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = friction;
	material.restitution = 0.0f;
	v3BlockGridCookDef def = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = cellCount,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static int FillSlab( ReplaceCell* cells, int start, int x0, int x1, int z0, int z1 )
{
	int count = start;
	for ( int z = z0; z <= z1; ++z )
	{
		for ( int x = x0; x <= x1; ++x )
		{
			cells[count++] = (ReplaceCell){ x, 0, z };
		}
	}
	return count;
}

// FNV-1a over the raw pose and velocity bytes of the given bodies, accumulated every step, so any
// divergence anywhere in the window changes the value.
static uint64_t HashBytes( uint64_t hash, const void* bytes, size_t byteCount )
{
	const uint8_t* cursor = bytes;
	for ( size_t i = 0; i < byteCount; ++i )
	{
		hash ^= cursor[i];
		hash *= 0x100000001B3ull;
	}
	return hash;
}

static uint64_t HashBodies( uint64_t hash, const b3BodyId* bodies, int bodyCount )
{
	for ( int i = 0; i < bodyCount; ++i )
	{
		b3Pos position = b3Body_GetPosition( bodies[i] );
		b3Quat rotation = b3Body_GetRotation( bodies[i] );
		b3Vec3 linearVelocity = b3Body_GetLinearVelocity( bodies[i] );
		b3Vec3 angularVelocity = b3Body_GetAngularVelocity( bodies[i] );
		hash = HashBytes( hash, &position, sizeof( position ) );
		hash = HashBytes( hash, &rotation, sizeof( rotation ) );
		hash = HashBytes( hash, &linearVelocity, sizeof( linearVelocity ) );
		hash = HashBytes( hash, &angularVelocity, sizeof( angularVelocity ) );
	}
	return hash;
}

static uint64_t StepAndHash( b3WorldId worldId, const b3BodyId* bodies, int bodyCount, int stepCount )
{
	uint64_t hash = 0xCBF29CE484222325ull;
	for ( int step = 0; step < stepCount; ++step )
	{
		b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
		hash = HashBodies( hash, bodies, bodyCount );
	}
	return hash;
}

static b3WorldId CreateReplaceWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	return b3CreateWorld( &worldDef );
}

static int TouchingPointCount( b3ShapeId shapeId, b3ShapeId otherShapeId )
{
	b3ContactData contacts[64];
	int contactCount = b3Shape_GetContactData( shapeId, contacts, 64 );
	int pointCount = 0;
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		bool samePair = ( B3_ID_EQUALS( contact->shapeIdA, shapeId ) && B3_ID_EQUALS( contact->shapeIdB, otherShapeId ) ) ||
						( B3_ID_EQUALS( contact->shapeIdA, otherShapeId ) && B3_ID_EQUALS( contact->shapeIdB, shapeId ) );
		if ( samePair == false || contact->manifolds == NULL )
		{
			continue;
		}
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			pointCount += contact->manifolds[manifoldIndex].pointCount;
		}
	}
	return pointCount;
}

static b3ContactId TouchingContactId( b3ShapeId shapeId, b3ShapeId otherShapeId )
{
	b3ContactData contacts[64];
	int contactCount = b3Shape_GetContactData( shapeId, contacts, 64 );
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		bool samePair = ( B3_ID_EQUALS( contact->shapeIdA, shapeId ) && B3_ID_EQUALS( contact->shapeIdB, otherShapeId ) ) ||
						( B3_ID_EQUALS( contact->shapeIdA, otherShapeId ) && B3_ID_EQUALS( contact->shapeIdB, shapeId ) );
		if ( samePair && contact->manifoldCount > 0 )
		{
			return contact->contactId;
		}
	}
	return b3_nullContactId;
}

// A shape identifier, its user data, and every other b3ShapeDef property survive a publication;
// the bounds, the mass, and the material table follow the new revision.
static int ReplacePreservesShapeIdentity( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wide = NULL;
	v3BlockGridData* narrow = NULL;

	ReplaceCell cells[maximumCells];
	int wideCount = FillSlab( cells, 0, 0, 5, 0, 1 );
	wide = CookCells( cells, wideCount, 0.7f );
	int narrowCount = FillSlab( cells, 0, 0, 1, 0, 1 );
	narrow = CookCells( cells, narrowCount, 0.7f );
	V3_REPLACE_ENSURE( wide != NULL && narrow != NULL );

	worldId = CreateReplaceWorld();
	V3_REPLACE_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0, 4.0, 0.0 };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	void* userData = (void*)(uintptr_t)0x5EED0044u;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 2.5f;
	shapeDef.userData = userData;
	shapeDef.filter = (b3Filter){ .categoryBits = 0x4u, .maskBits = 0x9u, .groupIndex = -7 };
	shapeDef.name = "shipHull";
	shapeDef.enableContactEvents = true;
	b3ShapeId shapeId = v3CreateBlockGridShape( bodyId, &shapeDef, wide );
	V3_REPLACE_ENSURE( B3_IS_NON_NULL( shapeId ) );

	b3AABB boundsBefore = b3Shape_GetAABB( shapeId );
	float massBefore = b3Body_GetMass( bodyId );

	// Mass follows the revision only when the caller asks for it.
	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeId, narrow, false ) == v3_blockGridReplaceOk );
	V3_REPLACE_ENSURE( b3Body_GetMass( bodyId ) == massBefore );

	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeId, narrow, true ) == v3_blockGridReplaceOk );

	b3AABB boundsAfter = b3Shape_GetAABB( shapeId );
	float massAfter = b3Body_GetMass( bodyId );
	printf( "  replace bounds x [%.3f, %.3f] -> [%.3f, %.3f] mass %.3f -> %.3f\n", (double)boundsBefore.lowerBound.x,
			(double)boundsBefore.upperBound.x, (double)boundsAfter.lowerBound.x, (double)boundsAfter.upperBound.x,
			(double)massBefore, (double)massAfter );

	// The identifier is the same handle, not a new one that merely compares valid.
	V3_REPLACE_ENSURE( b3Shape_IsValid( shapeId ) );
	V3_REPLACE_ENSURE( b3Shape_GetUserData( shapeId ) == userData );
	V3_REPLACE_ENSURE( b3Shape_GetDensity( shapeId ) == 2.5f );
	b3Filter filter = b3Shape_GetFilter( shapeId );
	V3_REPLACE_ENSURE( filter.categoryBits == 0x4u && filter.maskBits == 0x9u && filter.groupIndex == -7 );
	V3_REPLACE_ENSURE( strcmp( b3Shape_GetName( shapeId ), "shipHull" ) == 0 );
	V3_REPLACE_ENSURE( b3Shape_GetType( shapeId ) == v3_blockGridShape );
	b3ShapeId attached = b3_nullShapeId;
	V3_REPLACE_ENSURE( b3Body_GetShapes( bodyId, &attached, 1 ) == 1 && B3_ID_EQUALS( attached, shapeId ) );

	// Six cells wide became two, so both the bounds and the mass shrink by the same factor.
	V3_REPLACE_ENSURE( boundsAfter.upperBound.x < boundsBefore.upperBound.x );
	V3_REPLACE_ENSURE( massAfter < massBefore );
	V3_REPLACE_ENSURE( massBefore > 0.0f && massAfter > 0.0f );

	status = 0;

cleanup:
	if ( B3_IS_NON_NULL( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( wide );
	v3DestroyBlockGridData( narrow );
	return status;
}

static int allocationCount;
static int allocationFailureIndex;

static void* FailingAlloc( int32_t size, int32_t alignment )
{
	if ( allocationCount++ == allocationFailureIndex )
	{
		return NULL;
	}
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void FailingFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

// Every rejected publication leaves the previous revision attached and stepping exactly as it
// would have without the call, which the behavior hash over the following window proves.
static int FailedReplaceLeavesRevisionStepping( void )
{
	int status = 1;
	b3WorldId controlWorldId = b3_nullWorldId;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* terrain = NULL;
	v3BlockGridData* other = NULL;

	ReplaceCell cells[maximumCells];
	int terrainCount = FillSlab( cells, 0, 0, 5, 0, 5 );
	terrain = CookCells( cells, terrainCount, 0.8f );
	int otherCount = FillSlab( cells, 0, 0, 1, 0, 1 );
	other = CookCells( cells, otherCount, 0.8f );
	V3_REPLACE_ENSURE( terrain != NULL && other != NULL );

	uint64_t hashes[2] = { 0, 0 };
	for ( int run = 0; run < 2; ++run )
	{
		b3WorldId runWorldId = CreateReplaceWorld();
		V3_REPLACE_ENSURE( b3World_IsValid( runWorldId ) );
		if ( run == 0 )
		{
			controlWorldId = runWorldId;
		}
		else
		{
			worldId = runWorldId;
		}

		b3BodyDef terrainDef = b3DefaultBodyDef();
		terrainDef.type = b3_staticBody;
		b3BodyId terrainBodyId = b3CreateBody( runWorldId, &terrainDef );
		b3ShapeDef terrainShapeDef = b3DefaultShapeDef();
		b3ShapeId terrainShapeId = v3CreateBlockGridShape( terrainBodyId, &terrainShapeDef, terrain );
		V3_REPLACE_ENSURE( B3_IS_NON_NULL( terrainShapeId ) );

		b3BodyDef shipDef = b3DefaultBodyDef();
		shipDef.type = b3_dynamicBody;
		shipDef.position = (b3Pos){ 1.5, 3.0, 1.5 };
		b3BodyId shipBodyId = b3CreateBody( runWorldId, &shipDef );
		b3ShapeDef shipShapeDef = b3DefaultShapeDef();
		shipShapeDef.density = 1.0f;
		b3ShapeId shipShapeId = v3CreateBlockGridShape( shipBodyId, &shipShapeDef, other );
		V3_REPLACE_ENSURE( B3_IS_NON_NULL( shipShapeId ) );

		b3BodyId bodies[2] = { terrainBodyId, shipBodyId };
		for ( int step = 0; step < settleSteps; ++step )
		{
			b3World_Step( runWorldId, replaceTimeStep, replaceSubStepCount );
		}

		if ( run == 1 )
		{
			// NULL data.
			V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( terrainShapeId, NULL, true ) == v3_blockGridReplaceInvalidData );

			// A live shape that is not a BlockGrid shape.
			b3BodyDef sphereBodyDef = b3DefaultBodyDef();
			sphereBodyDef.type = b3_staticBody;
			sphereBodyDef.position = (b3Pos){ 40.0, 40.0, 40.0 };
			b3BodyId sphereBodyId = b3CreateBody( runWorldId, &sphereBodyDef );
			b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );
			V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( sphereShapeId, other, true ) == v3_blockGridReplaceWrongShapeType );

			// A stale identifier, and the null identifier.
			b3DestroyBody( sphereBodyId );
			V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( sphereShapeId, other, true ) == v3_blockGridReplaceInvalidShape );
			V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( b3_nullShapeId, other, true ) == v3_blockGridReplaceInvalidShape );

			// An allocator that fails the first allocation the publication asks for. The
			// publication takes exactly one, before it has mutated anything.
			int byteCountBeforeFailure = b3GetByteCount();
			allocationCount = 0;
			allocationFailureIndex = 0;
			b3SetAllocator( FailingAlloc, FailingFree );
			v3BlockGridReplaceStatus failed = v3ReplaceBlockGridShape( terrainShapeId, other, true );
			b3SetAllocator( NULL, NULL );
			V3_REPLACE_ENSURE( failed == v3_blockGridReplaceOutOfMemory );
			V3_REPLACE_ENSURE( allocationCount == 1 );
			V3_REPLACE_ENSURE( b3GetByteCount() == byteCountBeforeFailure );
		}

		hashes[run] = StepAndHash( runWorldId, bodies, 2, observationSteps );
	}

	printf( "  behavior hash control 0x%016llx failed-replace 0x%016llx\n", (unsigned long long)hashes[0],
			(unsigned long long)hashes[1] );
	V3_REPLACE_ENSURE( hashes[0] == hashes[1] );

	status = 0;

cleanup:
	if ( B3_IS_NON_NULL( controlWorldId ) )
	{
		b3DestroyWorld( controlWorldId );
	}
	if ( B3_IS_NON_NULL( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( terrain );
	v3DestroyBlockGridData( other );
	return status;
}

static void* watchedPointer;
static int watchedFreeCount;

static void* WatchingAlloc( int32_t size, int32_t alignment )
{
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void WatchingFree( void* memory )
{
	if ( memory != NULL && memory == watchedPointer )
	{
		watchedFreeCount += 1;
	}
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static int measuredAllocationCount;
static int measuredFreeCount;

static void* MeasuringAlloc( int32_t size, int32_t alignment )
{
	measuredAllocationCount += 1;
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void MeasuringFree( void* memory )
{
	measuredFreeCount += 1;
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

// Cooked data is one allocation, so counting the frees of its address counts the releases of the
// revision. Two shapes in one world, two worlds, a replacement, and teardown in either order all
// have to end with exactly one.
static int SharedRevisionReleasedExactlyOnce( void )
{
	int status = 1;
	b3SetAllocator( WatchingAlloc, WatchingFree );

	ReplaceCell cells[maximumCells];

	for ( int scenario = 0; scenario < 2; ++scenario )
	{
		int sharedCount = FillSlab( cells, 0, 0, 2, 0, 2 );
		v3BlockGridData* sharedGrid = CookCells( cells, sharedCount, 0.6f );
		watchedPointer = sharedGrid;
		watchedFreeCount = 0;
		V3_REPLACE_ENSURE( sharedGrid != NULL );

		int replacementCount = FillSlab( cells, 0, 0, 1, 0, 1 );
		v3BlockGridData* replacementGrid = CookCells( cells, replacementCount, 0.6f );
		V3_REPLACE_ENSURE( replacementGrid != NULL );

		b3WorldId worldA = CreateReplaceWorld();
		b3WorldId worldB = CreateReplaceWorld();
		V3_REPLACE_ENSURE( b3World_IsValid( worldA ) && b3World_IsValid( worldB ) );

		// Two shapes in one world and one shape in a second world, all on the same revision.
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3ShapeId shapeA1 = v3CreateBlockGridShape( b3CreateBody( worldA, &bodyDef ), &shapeDef, sharedGrid );
		b3ShapeId shapeA2 = v3CreateBlockGridShape( b3CreateBody( worldA, &bodyDef ), &shapeDef, sharedGrid );
		b3ShapeId shapeB1 = v3CreateBlockGridShape( b3CreateBody( worldB, &bodyDef ), &shapeDef, sharedGrid );
		V3_REPLACE_ENSURE( B3_IS_NON_NULL( shapeA1 ) && B3_IS_NON_NULL( shapeA2 ) && B3_IS_NON_NULL( shapeB1 ) );

		// One of them moves to another revision; the other two still hold this one.
		V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeA1, replacementGrid, true ) == v3_blockGridReplaceOk );
		V3_REPLACE_ENSURE( watchedFreeCount == 0 );

		// The caller's own reference goes early in one scenario and last in the other.
		if ( scenario == 0 )
		{
			v3DestroyBlockGridData( sharedGrid );
			V3_REPLACE_ENSURE( watchedFreeCount == 0 );
			b3DestroyWorld( worldB );
			V3_REPLACE_ENSURE( watchedFreeCount == 0 );
			b3DestroyWorld( worldA );
		}
		else
		{
			b3DestroyWorld( worldA );
			V3_REPLACE_ENSURE( watchedFreeCount == 0 );
			b3DestroyWorld( worldB );
			V3_REPLACE_ENSURE( watchedFreeCount == 0 );
			v3DestroyBlockGridData( sharedGrid );
		}

		printf( "  shared release scenario %d frees %d\n", scenario, watchedFreeCount );
		V3_REPLACE_ENSURE( watchedFreeCount == 1 );

		watchedPointer = NULL;
		v3DestroyBlockGridData( replacementGrid );
	}

	status = 0;

cleanup:
	watchedPointer = NULL;
	b3SetAllocator( NULL, NULL );
	return status;
}

typedef struct ContactScene
{
	b3WorldId worldId;
	b3ShapeId terrainShapeId;
	b3ShapeId leftShipShapeId;
	b3ShapeId rightShipShapeId;
	b3BodyId leftShipBodyId;
	b3BodyId rightShipBodyId;
} ContactScene;

// Two ships resting on one terrain revision, on platforms far enough apart that neither ship can
// reach the other's blocks.
static bool BuildContactScene( ContactScene* scene, v3BlockGridData* terrain, v3BlockGridData* ship )
{
	scene->worldId = CreateReplaceWorld();
	if ( b3World_IsValid( scene->worldId ) == false )
	{
		return false;
	}

	b3BodyDef terrainDef = b3DefaultBodyDef();
	terrainDef.type = b3_staticBody;
	b3ShapeDef terrainShapeDef = b3DefaultShapeDef();
	scene->terrainShapeId = v3CreateBlockGridShape( b3CreateBody( scene->worldId, &terrainDef ), &terrainShapeDef, terrain );

	b3BodyDef shipDef = b3DefaultBodyDef();
	shipDef.type = b3_dynamicBody;
	b3ShapeDef shipShapeDef = b3DefaultShapeDef();
	shipShapeDef.density = 1.0f;

	shipDef.position = (b3Pos){ 0.5, 2.0, 0.5 };
	scene->leftShipBodyId = b3CreateBody( scene->worldId, &shipDef );
	scene->leftShipShapeId = v3CreateBlockGridShape( scene->leftShipBodyId, &shipShapeDef, ship );

	shipDef.position = (b3Pos){ 8.5, 2.0, 0.5 };
	scene->rightShipBodyId = b3CreateBody( scene->worldId, &shipDef );
	scene->rightShipShapeId = v3CreateBlockGridShape( scene->rightShipBodyId, &shipShapeDef, ship );

	return B3_IS_NON_NULL( scene->terrainShapeId ) && B3_IS_NON_NULL( scene->leftShipShapeId ) &&
		   B3_IS_NON_NULL( scene->rightShipShapeId );
}

// The fourth criterion. Removing the blocks under one ship ends that ship's contact in the step
// after the publication, while the untouched ship's contact is back in that same step. The warm
// start behind it is rebuilt rather than carried, and the measurement below is what says that
// costs nothing observable: the untouched ship's pose and velocity are compared step by step
// against an identical world that was never replaced.
static int ReplaceEndsRemovedBlockContacts( void )
{
	int status = 1;
	ContactScene replaced = { .worldId = b3_nullWorldId };
	ContactScene control = { .worldId = b3_nullWorldId };
	v3BlockGridData* bothPlatforms = NULL;
	v3BlockGridData* rightPlatformOnly = NULL;
	v3BlockGridData* ship = NULL;

	ReplaceCell cells[maximumCells];
	int bothCount = FillSlab( cells, 0, 0, 3, 0, 3 );
	bothCount = FillSlab( cells, bothCount, 8, 11, 0, 3 );
	bothPlatforms = CookCells( cells, bothCount, 0.8f );

	int rightCount = FillSlab( cells, 0, 8, 11, 0, 3 );
	rightPlatformOnly = CookCells( cells, rightCount, 0.8f );

	int shipCount = FillSlab( cells, 0, 0, 2, 0, 2 );
	ship = CookCells( cells, shipCount, 0.8f );
	V3_REPLACE_ENSURE( bothPlatforms != NULL && rightPlatformOnly != NULL && ship != NULL );

	V3_REPLACE_ENSURE( BuildContactScene( &replaced, bothPlatforms, ship ) );
	V3_REPLACE_ENSURE( BuildContactScene( &control, bothPlatforms, ship ) );

	for ( int step = 0; step < settleSteps; ++step )
	{
		b3World_Step( replaced.worldId, replaceTimeStep, replaceSubStepCount );
		b3World_Step( control.worldId, replaceTimeStep, replaceSubStepCount );
	}

	int leftBefore = TouchingPointCount( replaced.leftShipShapeId, replaced.terrainShapeId );
	int rightBefore = TouchingPointCount( replaced.rightShipShapeId, replaced.terrainShapeId );
	V3_REPLACE_ENSURE( leftBefore > 0 && rightBefore > 0 );

	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( replaced.terrainShapeId, rightPlatformOnly, true ) == v3_blockGridReplaceOk );

	// The publication itself must not leave a touching contact behind on the removed blocks.
	V3_REPLACE_ENSURE( TouchingPointCount( replaced.leftShipShapeId, replaced.terrainShapeId ) == 0 );

	b3World_Step( replaced.worldId, replaceTimeStep, replaceSubStepCount );
	b3World_Step( control.worldId, replaceTimeStep, replaceSubStepCount );

	int leftAfter = TouchingPointCount( replaced.leftShipShapeId, replaced.terrainShapeId );
	int rightAfter = TouchingPointCount( replaced.rightShipShapeId, replaced.terrainShapeId );
	printf( "  contact points left %d -> %d, right %d -> %d\n", leftBefore, leftAfter, rightBefore, rightAfter );
	V3_REPLACE_ENSURE( leftAfter == 0 );
	V3_REPLACE_ENSURE( rightAfter > 0 );

	// The untouched ship over the next window, replaced world against control world.
	double maximumPositionDelta = 0.0;
	double maximumVelocityDelta = 0.0;
	for ( int step = 0; step < observationSteps; ++step )
	{
		b3World_Step( replaced.worldId, replaceTimeStep, replaceSubStepCount );
		b3World_Step( control.worldId, replaceTimeStep, replaceSubStepCount );

		b3Pos replacedPosition = b3Body_GetPosition( replaced.rightShipBodyId );
		b3Pos controlPosition = b3Body_GetPosition( control.rightShipBodyId );
		b3Vec3 replacedVelocity = b3Body_GetLinearVelocity( replaced.rightShipBodyId );
		b3Vec3 controlVelocity = b3Body_GetLinearVelocity( control.rightShipBodyId );

		double positionDelta = b3Length( b3SubPos( replacedPosition, controlPosition ) );
		double velocityDelta = b3Length( b3Sub( replacedVelocity, controlVelocity ) );
		maximumPositionDelta = positionDelta > maximumPositionDelta ? positionDelta : maximumPositionDelta;
		maximumVelocityDelta = velocityDelta > maximumVelocityDelta ? velocityDelta : maximumVelocityDelta;

		V3_REPLACE_ENSURE( TouchingPointCount( replaced.rightShipShapeId, replaced.terrainShapeId ) > 0 );
	}

	printf( "  warm start rebuild over %d steps: position delta %.9f m, velocity delta %.9f m/s, slop %.9f m\n", observationSteps,
			maximumPositionDelta, maximumVelocityDelta, (double)B3_LINEAR_SLOP );

	// The measured decision: rebuilding the contacts moves the resting ship by less than the
	// linear slop, so the Box3D destroy-and-recreate path stands and no pair state is carried.
	V3_REPLACE_ENSURE( maximumPositionDelta < (double)B3_LINEAR_SLOP );
	V3_REPLACE_ENSURE( maximumVelocityDelta < (double)B3_LINEAR_SLOP );

	status = 0;

cleanup:
	if ( B3_IS_NON_NULL( replaced.worldId ) )
	{
		b3DestroyWorld( replaced.worldId );
	}
	if ( B3_IS_NON_NULL( control.worldId ) )
	{
		b3DestroyWorld( control.worldId );
	}
	v3DestroyBlockGridData( bothPlatforms );
	v3DestroyBlockGridData( rightPlatformOnly );
	v3DestroyBlockGridData( ship );
	return status;
}

// A warmed aggregate pair keeps its native storage through a successful publication. Each
// replacement still closes the old public lifetime and starts a fresh one on the next step.
static int RepeatedReplacementReusesNativeCapacity( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* terrain = NULL;
	v3BlockGridData* ship = NULL;

	ReplaceCell cells[maximumCells];
	int terrainCount = FillSlab( cells, 0, 0, 5, 0, 5 );
	terrain = CookCells( cells, terrainCount, 0.8f );
	int shipCount = FillSlab( cells, 0, 0, 2, 0, 2 );
	ship = CookCells( cells, shipCount, 0.8f );
	V3_REPLACE_ENSURE( terrain != NULL && ship != NULL );

	worldId = CreateReplaceWorld();
	V3_REPLACE_ENSURE( b3World_IsValid( worldId ) );
	b3BodyDef terrainBodyDef = b3DefaultBodyDef();
	terrainBodyDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	b3ShapeId terrainShape = v3CreateBlockGridShape( b3CreateBody( worldId, &terrainBodyDef ), &shapeDef, terrain );

	b3BodyDef shipBodyDef = b3DefaultBodyDef();
	shipBodyDef.type = b3_dynamicBody;
	shipBodyDef.position = (b3Pos){ 1.5, 2.0, 1.5 };
	shapeDef.density = 1.0f;
	b3ShapeId shipShape = v3CreateBlockGridShape( b3CreateBody( worldId, &shipBodyDef ), &shapeDef, ship );
	V3_REPLACE_ENSURE( B3_IS_NON_NULL( terrainShape ) && B3_IS_NON_NULL( shipShape ) );

	for ( int step = 0; step < settleSteps; ++step )
	{
		b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
	}

	for ( int replacement = 0; replacement < 3; ++replacement )
	{
		b3ContactId oldContact = TouchingContactId( shipShape, terrainShape );
		V3_REPLACE_ENSURE( B3_IS_NON_NULL( oldContact ) && b3Contact_IsValid( oldContact ) );
		V3_REPLACE_ENSURE( TouchingPointCount( shipShape, terrainShape ) > 0 );

		V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( terrainShape, terrain, true ) == v3_blockGridReplaceOk );
		V3_REPLACE_ENSURE( b3Contact_IsValid( oldContact ) == false );
		V3_REPLACE_ENSURE( TouchingPointCount( shipShape, terrainShape ) == 0 );

		measuredAllocationCount = 0;
		measuredFreeCount = 0;
		b3SetAllocator( MeasuringAlloc, MeasuringFree );
		b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
		b3SetAllocator( NULL, NULL );

		b3ContactId newContact = TouchingContactId( shipShape, terrainShape );
		b3ContactEvents nativeEvents = b3World_GetContactEvents( worldId );
		v3BlockContactEvents logicalEvents = v3World_GetBlockContactEvents( worldId );
		printf( "  replacement %d native allocations/frees %d/%d, native begin/end %d/%d, logical begin/end %d/%d\n", replacement,
				measuredAllocationCount, measuredFreeCount, nativeEvents.beginCount, nativeEvents.endCount,
				logicalEvents.beginCount, logicalEvents.endCount );

		V3_REPLACE_ENSURE( B3_IS_NON_NULL( newContact ) && b3Contact_IsValid( newContact ) );
		V3_REPLACE_ENSURE( newContact.index1 == oldContact.index1 && newContact.generation != oldContact.generation );
		V3_REPLACE_ENSURE( nativeEvents.beginCount == 1 && nativeEvents.endCount == 1 );
		V3_REPLACE_ENSURE( B3_ID_EQUALS( nativeEvents.beginEvents[0].contactId, newContact ) );
		V3_REPLACE_ENSURE( B3_ID_EQUALS( nativeEvents.endEvents[0].contactId, oldContact ) );
		V3_REPLACE_ENSURE( logicalEvents.beginCount > 0 && logicalEvents.endCount > 0 && logicalEvents.truncated == false );
		V3_REPLACE_ENSURE( measuredAllocationCount == 0 && measuredFreeCount == 0 );
	}

	status = 0;

cleanup:
	b3SetAllocator( NULL, NULL );
	if ( B3_IS_NON_NULL( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( terrain );
	v3DestroyBlockGridData( ship );
	return status;
}

// The counter reports the publications made since the previous step, so two replacements between
// two steps read as two and the step after that reads as none.
static int ReplacementCounterPublishes( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* first = NULL;
	v3BlockGridData* second = NULL;

	ReplaceCell cells[maximumCells];
	int firstCount = FillSlab( cells, 0, 0, 3, 0, 3 );
	first = CookCells( cells, firstCount, 0.8f );
	int secondCount = FillSlab( cells, 0, 0, 2, 0, 2 );
	second = CookCells( cells, secondCount, 0.8f );
	V3_REPLACE_ENSURE( first != NULL && second != NULL );

	worldId = CreateReplaceWorld();
	V3_REPLACE_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = v3CreateBlockGridShape( b3CreateBody( worldId, &bodyDef ), &shapeDef, first );
	V3_REPLACE_ENSURE( B3_IS_NON_NULL( shapeId ) );

	b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
	V3_REPLACE_ENSURE( v3World_GetBlockGridPairCounters( worldId ).replacementPublishedCount == 0 );

	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeId, second, true ) == v3_blockGridReplaceOk );
	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeId, first, true ) == v3_blockGridReplaceOk );

	// A failed publication must not count.
	V3_REPLACE_ENSURE( v3ReplaceBlockGridShape( shapeId, NULL, true ) == v3_blockGridReplaceInvalidData );

	b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
	V3_REPLACE_ENSURE( v3World_GetBlockGridPairCounters( worldId ).replacementPublishedCount == 2 );

	b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
	V3_REPLACE_ENSURE( v3World_GetBlockGridPairCounters( worldId ).replacementPublishedCount == 0 );

	status = 0;

cleanup:
	if ( B3_IS_NON_NULL( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( first );
	v3DestroyBlockGridData( second );
	return status;
}

typedef struct LockedProbe
{
	b3ShapeId shapeId;
	v3BlockGridData* grid;
	v3BlockGridReplaceStatus status;
	int callCount;
} LockedProbe;

static bool LockedFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	LockedProbe* probe = context;
	MAYBE_UNUSED( shapeIdA );
	MAYBE_UNUSED( shapeIdB );
	if ( probe->callCount == 0 )
	{
		probe->status = v3ReplaceBlockGridShape( probe->shapeId, probe->grid, true );
	}
	probe->callCount += 1;
	return true;
}

// The world is locked for the whole step, so a publication attempted from inside a callback is
// reported rather than performed.
static int ReplaceRejectsLockedWorld( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* terrain = NULL;
	v3BlockGridData* ship = NULL;

	ReplaceCell cells[maximumCells];
	int terrainCount = FillSlab( cells, 0, 0, 3, 0, 3 );
	terrain = CookCells( cells, terrainCount, 0.8f );
	int shipCount = FillSlab( cells, 0, 0, 1, 0, 1 );
	ship = CookCells( cells, shipCount, 0.8f );
	V3_REPLACE_ENSURE( terrain != NULL && ship != NULL );

	worldId = CreateReplaceWorld();
	V3_REPLACE_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef terrainDef = b3DefaultBodyDef();
	terrainDef.type = b3_staticBody;
	b3ShapeDef terrainShapeDef = b3DefaultShapeDef();
	b3ShapeId terrainShapeId = v3CreateBlockGridShape( b3CreateBody( worldId, &terrainDef ), &terrainShapeDef, terrain );

	b3BodyDef shipDef = b3DefaultBodyDef();
	shipDef.type = b3_dynamicBody;
	shipDef.position = (b3Pos){ 1.0, 2.0, 1.0 };
	b3ShapeDef shipShapeDef = b3DefaultShapeDef();
	shipShapeDef.density = 1.0f;
	shipShapeDef.enableCustomFiltering = true;
	b3ShapeId shipShapeId = v3CreateBlockGridShape( b3CreateBody( worldId, &shipDef ), &shipShapeDef, ship );
	V3_REPLACE_ENSURE( B3_IS_NON_NULL( terrainShapeId ) && B3_IS_NON_NULL( shipShapeId ) );

	LockedProbe probe = { .shapeId = terrainShapeId, .grid = ship, .status = v3_blockGridReplaceOk };
	b3World_SetCustomFilterCallback( worldId, LockedFilter, &probe );

	for ( int step = 0; step < settleSteps && probe.callCount == 0; ++step )
	{
		b3World_Step( worldId, replaceTimeStep, replaceSubStepCount );
	}

	V3_REPLACE_ENSURE( probe.callCount > 0 );
	V3_REPLACE_ENSURE( probe.status == v3_blockGridReplaceWorldLocked );

	// The rejected call left the terrain revision in place.
	V3_REPLACE_ENSURE( b3Shape_GetType( terrainShapeId ) == v3_blockGridShape );
	b3World_SetCustomFilterCallback( worldId, NULL, NULL );

	status = 0;

cleanup:
	if ( B3_IS_NON_NULL( worldId ) )
	{
		b3World_SetCustomFilterCallback( worldId, NULL, NULL );
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( terrain );
	v3DestroyBlockGridData( ship );
	return status;
}

int V3BlockGridReplaceTest( void )
{
	RUN_SUBTEST( ReplacePreservesShapeIdentity );
	RUN_SUBTEST( FailedReplaceLeavesRevisionStepping );
	RUN_SUBTEST( SharedRevisionReleasedExactlyOnce );
	RUN_SUBTEST( ReplaceEndsRemovedBlockContacts );
	RUN_SUBTEST( RepeatedReplacementReusesNativeCapacity );
	RUN_SUBTEST( ReplacementCounterPublishes );
	RUN_SUBTEST( ReplaceRejectsLockedWorld );
	return 0;
}
