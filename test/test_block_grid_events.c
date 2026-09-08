// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"

#include <limits.h>
#include <stdlib.h>

#if defined( _WIN32 )
#include <malloc.h>
#endif

static v3BlockGridData* CookEventSlab( void )
{
	v3BlockGridBox box = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock blocks[4];
	for ( int i = 0; i < 4; ++i )
	{
		blocks[i] = (v3BlockGridBlock){ .x = i - 2, .userData = 991, .boxes = &box, .boxCount = 1 };
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.userMaterialId = 73;
	material.friction = 0.0f;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = 4 };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.data;
}

static b3WorldId CreateEventWorld( void )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	def.enableSleep = false;
	def.hitEventThreshold = 0.1f;
	return b3CreateWorld( &def );
}

static b3ShapeId CreateEventTerrain( b3WorldId worldId, v3BlockGridData* grid )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId body = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	return v3CreateBlockGridShape( body, &shapeDef, grid );
}

static b3ShapeId CreateEventSphere( b3WorldId worldId, b3BodyId* body )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -0.25, 2.0, 0.5 };
	*body = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	shapeDef.baseMaterial.friction = 0.0f;
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	return b3CreateSphereShape( *body, &shapeDef, &sphere );
}

static bool EventMatchesSphere( const v3BlockContactEvent* event, b3ShapeId terrain, b3ShapeId sphere, b3BodyId sphereBody )
{
	const v3BlockContactSide* grid = event->sideA.isBlockGrid ? &event->sideA : &event->sideB;
	const v3BlockContactSide* convex = event->sideA.isBlockGrid ? &event->sideB : &event->sideA;
	bool matches = grid->isBlockGrid && !convex->isBlockGrid && B3_ID_EQUALS( grid->shapeId, terrain ) &&
				   B3_ID_EQUALS( convex->shapeId, sphere ) && B3_ID_EQUALS( convex->bodyId, sphereBody ) &&
				   B3_ID_EQUALS( grid->bodyId, b3Shape_GetBody( terrain ) ) && grid->cellX == -1 && grid->cellY == 0 &&
				   grid->cellZ == 0 && grid->subHitboxIndex == 0 && grid->materialIndex == 0 && grid->userMaterialId == 73 &&
				   grid->userData == 991;
	if ( !matches )
	{
		printf( "grid cell=(%d,%d,%d), Cell Box=%d, material=%llu, data=%llu\n", grid->cellX, grid->cellY, grid->cellZ,
				grid->subHitboxIndex, (unsigned long long)grid->userMaterialId, (unsigned long long)grid->userData );
	}
	return matches;
}

// The sphere's vertical centre line is strictly inside cell (-1, 0, 0).
// All four cells share caller data, so that value cannot identify the cell.
static int SphereEventNamesMergedCell( void )
{
	v3BlockGridData* grid = CookEventSlab();
	ENSURE( grid != NULL );
	v3BlockGridCookStats stats = v3BlockGrid_GetCookStats( grid );
	bool valid = stats.boxCount == 1 && stats.blockCount == 4;
	b3WorldId world = CreateEventWorld();
	b3ShapeId terrain = CreateEventTerrain( world, grid );
	v3DestroyBlockGridData( grid );
	b3BodyId sphereBody;
	b3ShapeId sphere = CreateEventSphere( world, &sphereBody );
	bool began = false;
	bool hit = false;
	for ( int step = 0; step < 180 && valid; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
		b3ContactEvents native = b3World_GetContactEvents( world );
		valid = !events.truncated && events.endCount == 0 && ( events.hitCount > 0 ) == ( native.hitCount > 0 );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			valid = valid && !began && EventMatchesSphere( events.beginEvents + i, terrain, sphere, sphereBody );
			began = true;
		}
		for ( int i = 0; i < events.hitCount; ++i )
		{
			const v3BlockContactEvent* event = events.hitEvents + i;
			valid = valid && EventMatchesSphere( event, terrain, sphere, sphereBody ) && event->normalImpulse > 0.0f &&
					event->approachSpeed > 0.1f;
			hit = true;
		}
	}
	b3DestroyWorld( world );
	ENSURE( valid && began && hit );
	return 0;
}

static const v3BlockContactSide* GridSide( const v3BlockContactEvent* event )
{
	return event->sideA.isBlockGrid ? &event->sideA : &event->sideB;
}

static int DestructionPreservesEndIdentity( void )
{
	v3BlockGridData* grid = CookEventSlab();
	ENSURE( grid != NULL );
	b3WorldId world = CreateEventWorld();
	b3ShapeId terrain = CreateEventTerrain( world, grid );
	b3BodyId terrainBody = b3Shape_GetBody( terrain );
	v3DestroyBlockGridData( grid );
	b3BodyId sphereBody;
	b3ShapeId sphere = CreateEventSphere( world, &sphereBody );
	bool began = false;
	for ( int step = 0; step < 180; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		began = began || v3World_GetBlockContactEvents( world ).beginCount > 0;
	}
	b3DestroyBody( terrainBody );
	bool valid = began && !b3Shape_IsValid( terrain ) && !b3Body_IsValid( terrainBody );
	b3World_Step( world, 1.0f / 60.0f, 4 );
	v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
	valid = valid && events.endCount > 0 && !events.truncated;
	for ( int i = 0; i < events.endCount; ++i )
	{
		const v3BlockContactEvent* event = events.endEvents + i;
		const v3BlockContactSide* side = GridSide( event );
		const v3BlockContactSide* convex = event->sideA.isBlockGrid ? &event->sideB : &event->sideA;
		valid = valid && B3_ID_EQUALS( side->bodyId, terrainBody ) && B3_ID_EQUALS( side->shapeId, terrain ) &&
				B3_ID_EQUALS( convex->shapeId, sphere ) && side->cellX == -1 && side->cellY == 0 && side->cellZ == 0 &&
				side->userData == 991 && side->userMaterialId == 73;
	}
	b3World_Step( world, 1.0f / 60.0f, 4 );
	valid = valid && v3World_GetBlockContactEvents( world ).endCount == 0;
	b3DestroyWorld( world );
	ENSURE( valid );
	return 0;
}

static int GridPairUsesEachLocalCell( void )
{
	v3BlockGridData* terrainGrid = CookEventSlab();
	ENSURE( terrainGrid != NULL );
	b3WorldId world = CreateEventWorld();
	b3ShapeId terrain = CreateEventTerrain( world, terrainGrid );
	v3DestroyBlockGridData( terrainGrid );
	v3BlockGridBox boxes[2] = {
		{ .bounds = { { 0.25f, 0.7f, 0.25f }, { 0.75f, 1.0f, 0.75f } } },
		{ .bounds = { { 0.25f, 0.0f, 0.25f }, { 0.75f, 0.3f, 0.75f } } },
	};
	v3BlockGridBlock block = { .x = 4, .y = 7, .z = -3, .userData = 812, .boxes = boxes, .boxCount = 2 };
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.userMaterialId = 91;
	v3BlockGridCookDef cookDef = { .materials = &material, .materialCount = 1, .blocks = &block, .blockCount = 1 };
	v3BlockGridCookResult cooked = v3CookBlockGrid( &cookDef );
	if ( cooked.data == NULL )
	{
		b3DestroyWorld( world );
		return 1;
	}
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -4.0, -5.0, 3.0 };
	b3BodyId body = b3CreateBody( world, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	b3ShapeId hull = v3CreateBlockGridShape( body, &shapeDef, cooked.data );
	v3DestroyBlockGridData( cooked.data );
	bool began = false;
	bool hit = false;
	bool valid = true;
	for ( int step = 0; step < 90 && valid; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
		valid = !events.truncated;
		for ( int kind = 0; kind < 2; ++kind )
		{
			const v3BlockContactEvent* array = kind == 0 ? events.beginEvents : events.hitEvents;
			int count = kind == 0 ? events.beginCount : events.hitCount;
			for ( int i = 0; i < count; ++i )
			{
				const v3BlockContactSide* ship = B3_ID_EQUALS( array[i].sideA.shapeId, hull ) ? &array[i].sideA : &array[i].sideB;
				const v3BlockContactSide* ground =
					B3_ID_EQUALS( array[i].sideA.shapeId, hull ) ? &array[i].sideB : &array[i].sideA;
				valid = valid && ship->isBlockGrid && ground->isBlockGrid && B3_ID_EQUALS( ship->shapeId, hull ) &&
						B3_ID_EQUALS( ground->shapeId, terrain ) && ship->cellX == 4 && ship->cellY == 7 && ship->cellZ == -3 &&
						ship->subHitboxIndex == 1 && ship->userData == 812 && ship->userMaterialId == 91 && ground->cellX == 0 &&
						ground->cellY == 0 && ground->cellZ == 0 && ground->subHitboxIndex == 0 && ground->userData == 991 &&
						ground->userMaterialId == 73;
				began = began || kind == 0;
				hit = hit || kind == 1;
			}
		}
	}
	b3DestroyWorld( world );
	ENSURE( valid && began && hit );
	return 0;
}

static int ReplacementEndsOldCell( void )
{
	v3BlockGridData* grid = CookEventSlab();
	ENSURE( grid != NULL );
	b3WorldId world = CreateEventWorld();
	b3ShapeId terrain = CreateEventTerrain( world, grid );
	v3DestroyBlockGridData( grid );
	b3BodyId sphereBody;
	b3ShapeId sphere = CreateEventSphere( world, &sphereBody );
	bool began = false;
	for ( int step = 0; step < 180; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		began = began || v3World_GetBlockContactEvents( world ).beginCount > 0;
	}
	v3BlockGridBox box = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock block = { .x = 8, .userData = 123, .boxes = &box, .boxCount = 1 };
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.userMaterialId = 17;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = &block, .blockCount = 1 };
	v3BlockGridCookResult replacement = v3CookBlockGrid( &def );
	bool valid = began && replacement.data != NULL;
	if ( replacement.data != NULL )
	{
		valid = valid && v3ReplaceBlockGridShape( terrain, replacement.data, false ) == v3_blockGridReplaceOk;
		v3DestroyBlockGridData( replacement.data );
	}
	b3World_Step( world, 1.0f / 60.0f, 4 );
	v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
	valid = valid && events.endCount > 0 && events.beginCount == 0 && !events.truncated;
	for ( int i = 0; i < events.endCount; ++i )
	{
		valid = valid && EventMatchesSphere( events.endEvents + i, terrain, sphere, sphereBody );
	}
	b3World_Step( world, 1.0f / 60.0f, 4 );
	valid = valid && v3World_GetBlockContactEvents( world ).endCount == 0;
	b3DestroyWorld( world );
	ENSURE( valid );
	return 0;
}

static int eventAllocationCount;

static void* EventAlloc( int32_t size, int32_t alignment )
{
	eventAllocationCount += 1;
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void EventFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static bool SlideEventRoundTrip( b3WorldId world, b3BodyId sphereBody )
{
	bool valid = true;
	for ( int direction = 0; direction < 2; ++direction )
	{
		int oldCellX = direction == 0 ? -1 : 0;
		int newCellX = direction == 0 ? 0 : -1;
		bool endedOld = false;
		bool beganNew = false;
		b3Body_SetLinearVelocity( sphereBody, (b3Vec3){ direction == 0 ? 0.5f : -0.5f, 0.0f, 0.0f } );
		for ( int step = 0; step < 60; ++step )
		{
			b3World_Step( world, 1.0f / 60.0f, 4 );
			v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
			valid = valid && !events.truncated;
			for ( int i = 0; i < events.endCount; ++i )
			{
				const v3BlockContactSide* side = GridSide( events.endEvents + i );
				valid = valid && side->cellX == oldCellX && side->cellY == 0 && side->cellZ == 0 && side->userData == 991;
				endedOld = true;
			}
			for ( int i = 0; i < events.beginCount; ++i )
			{
				const v3BlockContactSide* side = GridSide( events.beginEvents + i );
				valid = valid && side->cellX == newCellX && side->cellY == 0 && side->cellZ == 0 && side->userData == 991;
				beganNew = true;
			}
		}
		valid = valid && endedOld && beganNew;
		b3Body_SetLinearVelocity( sphereBody, b3Vec3_zero );
		for ( int step = 0; step < 30; ++step )
		{
			b3World_Step( world, 1.0f / 60.0f, 4 );
			v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
			valid = valid && !events.truncated && events.beginCount == 0 && events.endCount == 0;
		}
	}
	return valid;
}

static int SlidingReportsCellTransitionsWithoutAllocations( void )
{
	b3SetAllocator( EventAlloc, EventFree );
	v3BlockGridData* grid = CookEventSlab();
	if ( grid == NULL )
	{
		b3SetAllocator( NULL, NULL );
		return 1;
	}
	b3WorldId world = CreateEventWorld();
	CreateEventTerrain( world, grid );
	v3DestroyBlockGridData( grid );
	b3BodyId sphereBody;
	CreateEventSphere( world, &sphereBody );
	for ( int step = 0; step < 180; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
	}
	bool valid = SlideEventRoundTrip( world, sphereBody );
	valid &= SlideEventRoundTrip( world, sphereBody );
	eventAllocationCount = 0;
	valid &= SlideEventRoundTrip( world, sphereBody );
	int allocations = eventAllocationCount;
	b3DestroyWorld( world );
	b3SetAllocator( NULL, NULL );
	ENSURE( valid );
	ENSURE( allocations == 0 );
	return 0;
}

static int CheckOverflowRecovery( bool removeTracked )
{
	v3BlockGridData* grid = CookEventSlab();
	ENSURE( grid != NULL );
	b3WorldId world = CreateEventWorld();
	b3ShapeId terrain = CreateEventTerrain( world, grid );
	v3DestroyBlockGridData( grid );
	int capacity = v3World_GetBlockContactEvents( world ).capacity;
	if ( capacity <= 0 || capacity == INT_MAX )
	{
		b3DestroyWorld( world );
		return 1;
	}
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -0.25, 1.19, 0.5 };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	// These overlapping spheres contact the grid but never one another.
	shapeDef.filter.groupIndex = -1;
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	for ( int i = 0; i <= capacity; ++i )
	{
		b3BodyId body = b3CreateBody( world, &bodyDef );
		b3CreateSphereShape( body, &shapeDef, &sphere );
	}
	b3World_Step( world, 1.0f / 60.0f, 4 );
	v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
	b3ContactEvents native = b3World_GetContactEvents( world );
	bool valid = native.beginCount == capacity + 1 && events.beginCount <= capacity && events.truncated &&
				 events.droppedBeginCount > 0 &&
				 (uint32_t)events.beginCount + events.droppedBeginCount == (uint32_t)native.beginCount;

	b3BodyId tracked = b3_nullBodyId;
	b3BodyId omitted = b3_nullBodyId;
	for ( int i = 0; i < native.beginCount; ++i )
	{
		b3ShapeId shape = B3_ID_EQUALS( native.beginEvents[i].shapeIdA, terrain ) ? native.beginEvents[i].shapeIdB
																				  : native.beginEvents[i].shapeIdA;
		bool reported = false;
		for ( int j = 0; j < events.beginCount; ++j )
		{
			reported = reported || B3_ID_EQUALS( events.beginEvents[j].sideA.shapeId, shape ) ||
					   B3_ID_EQUALS( events.beginEvents[j].sideB.shapeId, shape );
		}
		if ( reported )
			tracked = b3Shape_GetBody( shape );
		else
			omitted = b3Shape_GetBody( shape );
	}
	valid = valid && B3_IS_NON_NULL( tracked ) && B3_IS_NON_NULL( omitted );
	if ( valid )
	{
		b3DestroyBody( removeTracked ? tracked : omitted );
		b3World_Step( world, 1.0f / 60.0f, 4 );
		events = v3World_GetBlockContactEvents( world );
		native = b3World_GetContactEvents( world );
		valid = native.beginCount == 0 && native.endCount == 1 && events.beginCount == 0 &&
				events.endCount == ( removeTracked ? 1 : 0 ) && events.truncated;
		b3World_Step( world, 1.0f / 60.0f, 4 );
		events = v3World_GetBlockContactEvents( world );
		valid = valid && events.beginCount == 0 && events.endCount == 0 && !events.truncated;
		if ( removeTracked )
		{
			// The formerly omitted contact is now tracked and must end normally.
			b3DestroyBody( omitted );
			b3World_Step( world, 1.0f / 60.0f, 4 );
			events = v3World_GetBlockContactEvents( world );
			valid = valid && events.beginCount == 0 && events.endCount == 1 && !events.truncated;
		}
	}
	b3DestroyWorld( world );
	ENSURE( valid );
	return 0;
}

static int EventOverflowIsReported( void )
{
	ENSURE( CheckOverflowRecovery( true ) == 0 );
	ENSURE( CheckOverflowRecovery( false ) == 0 );
	return 0;
}

int V3BlockGridEventsTest( void )
{
	RUN_SUBTEST( SphereEventNamesMergedCell );
	RUN_SUBTEST( GridPairUsesEachLocalCell );
	RUN_SUBTEST( DestructionPreservesEndIdentity );
	RUN_SUBTEST( ReplacementEndsOldCell );
	RUN_SUBTEST( SlidingReportsCellTransitionsWithoutAllocations );
	RUN_SUBTEST( EventOverflowIsReported );
	return 0;
}
