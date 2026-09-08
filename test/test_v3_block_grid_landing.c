// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "shape.h"
#include "test_macros.h"
#include "v3_block_grid.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <stdio.h>

#define V3_LANDING_ENSURE( C )                                                                                                   \
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

enum
{
	terrainExtent = 12,
	shipLength = 12,
	shipWidth = 6,
	shipHeight = 3,
	bulletCount = 8,
	landingSteps = 180,
	postLandingSteps = 240,
	steadyWindowSteps = 60,
	fragmentedTerrainWidth = 16,
	fragmentedTerrainDepth = 10,
	fragmentedTerrainCellCount = fragmentedTerrainWidth * fragmentedTerrainDepth,
	fragmentedRegionCapacity = 120,
	fragmentedLandingSteps = 600,
};

static const float landingTimeStep = 1.0f / 60.0f;

// Without pair-state recycling every resting step rebuilds the manifold, so the settled ship keeps a
// small residual speed instead of the exactly zero one the frozen anchors used to produce. Measured
// 0.0695 m/s at 1/60 s with four sub-steps; the bound stays far below any real ship motion.
static const float steadyShipSpeedBound = 0.1f;

static v3BlockGridCookResult CookFragmentedLandingTerrain( void )
{
	v3BlockGridBox box = { .bounds = { { 0.2f, 0.0f, 0.2f }, { 0.8f, 1.0f, 0.8f } } };
	v3BlockGridBlock blocks[fragmentedTerrainCellCount];
	for ( int z = 0; z < fragmentedTerrainDepth; ++z )
	{
		for ( int x = 0; x < fragmentedTerrainWidth; ++x )
		{
			int index = z * fragmentedTerrainWidth + x;
			blocks[index] = (v3BlockGridBlock){
				.x = x,
				.z = z,
				.userData = (uint64_t)index + 1,
				.boxes = &box,
				.boxCount = 1,
			};
		}
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	v3BlockGridCookDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = fragmentedTerrainCellCount,
	};
	return v3CookBlockGrid( &definition );
}

static v3BlockGridCookResult CookFragmentedLandingBody( void )
{
	v3BlockGridBox box = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock blocks[fragmentedTerrainCellCount];
	for ( int z = 0; z < fragmentedTerrainDepth; ++z )
	{
		for ( int x = 0; x < fragmentedTerrainWidth; ++x )
		{
			int index = z * fragmentedTerrainWidth + x;
			blocks[index] = (v3BlockGridBlock){
				.x = x,
				.z = z,
				.userData = (uint64_t)index + 1000,
				.boxes = &box,
				.boxCount = 1,
			};
		}
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridCookDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = fragmentedTerrainCellCount,
	};
	return v3CookBlockGrid( &definition );
}

static v3BlockGridCookResult CookMixedFragmentedLandingTerrain( void )
{
	v3BlockGridBox boxes[fragmentedTerrainCellCount];
	v3BlockGridBlock blocks[fragmentedTerrainCellCount];
	for ( int z = 0; z < fragmentedTerrainDepth; ++z )
	{
		for ( int x = 0; x < fragmentedTerrainWidth; ++x )
		{
			int index = z * fragmentedTerrainWidth + x;
			boxes[index] = (v3BlockGridBox){ .bounds = { { 0.2f, 0.0f, 0.2f }, { 0.8f, 1.0f, 0.8f } } };
			blocks[index] = (v3BlockGridBlock){
				.x = x,
				.z = z,
				.userData = (uint64_t)index + 1,
				.boxes = boxes + index,
				.boxCount = 1,
			};
		}
	}
	boxes[0].bounds.upperBound.x = 1.0f;
	boxes[1].bounds.lowerBound.x = 0.0f;
	boxes[1].bounds.lowerBound.z = 0.3f;
	boxes[1].bounds.upperBound.z = 0.7f;
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	v3BlockGridCookDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = fragmentedTerrainCellCount,
	};
	return v3CookBlockGrid( &definition );
}

// A fragmented support surface can exceed the ordinary region capacity. The reduced contact must
// remain active while an awake body rests on every fragment.
static int RunFragmentedLanding( v3BlockGridCookResult terrain, int stepCount )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridCookResult moving = CookFragmentedLandingBody();
	V3_LANDING_ENSURE( terrain.status == v3_blockGridCookOk );
	V3_LANDING_ENSURE( terrain.stats.boxCount == fragmentedTerrainCellCount );
	V3_LANDING_ENSURE( moving.status == v3_blockGridCookOk );
	V3_LANDING_ENSURE( moving.stats.boxCount == 1 );

	b3WorldDef worldDefinition = b3DefaultWorldDef();
	worldDefinition.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDefinition.enableSleep = false;
	worldDefinition.enableContinuous = false;
	worldDefinition.workerCount = 1;
	worldId = b3CreateWorld( &worldDefinition );
	V3_LANDING_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDefinition = b3DefaultBodyDef();
	b3BodyId terrainBodyId = b3CreateBody( worldId, &bodyDefinition );
	b3ShapeDef shapeDefinition = b3DefaultShapeDef();
	b3ShapeId terrainShapeId = v3CreateBlockGridShape( terrainBodyId, &shapeDefinition, terrain.data );
	V3_LANDING_ENSURE( b3Shape_IsValid( terrainShapeId ) );

	bodyDefinition.type = b3_dynamicBody;
	bodyDefinition.position = (b3Pos){ 0.0, 1.0, 0.0 };
	bodyDefinition.enableSleep = false;
	b3BodyId movingBodyId = b3CreateBody( worldId, &bodyDefinition );
	shapeDefinition.density = 1.0f;
	b3ShapeId movingShapeId = v3CreateBlockGridShape( movingBodyId, &shapeDefinition, moving.data );
	V3_LANDING_ENSURE( b3Shape_IsValid( movingShapeId ) );
	v3DestroyBlockGridData( terrain.data );
	terrain.data = NULL;
	v3DestroyBlockGridData( moving.data );
	moving.data = NULL;

	bool reductionEngaged = false;
	for ( int step = 0; step < stepCount; ++step )
	{
		b3World_Step( worldId, landingTimeStep, 4 );
		v3BlockGridPairCounters counters = v3World_GetBlockGridPairCounters( worldId );
		V3_LANDING_ENSURE( b3Body_IsAwake( movingBodyId ) );
		V3_LANDING_ENSURE( b3AbsFloat( (float)b3Body_GetPosition( movingBodyId ).y - 1.0f ) < B3_LINEAR_SLOP );
		if ( counters.touchingPairCount > fragmentedRegionCapacity )
		{
			reductionEngaged = true;
			V3_LANDING_ENSURE( counters.contactReductionCount == 1 );
		}
		else
		{
			V3_LANDING_ENSURE( counters.contactReductionCount == 0 );
		}
	}
	V3_LANDING_ENSURE( reductionEngaged );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( terrain.data );
	v3DestroyBlockGridData( moving.data );
	return status;
}

static int V3BlockGridFragmentedLandingReducesContact( void )
{
	return RunFragmentedLanding( CookFragmentedLandingTerrain(), fragmentedLandingSteps );
}

// Two touching terrain hitboxes form one component among more than 120 regions. Overflow storage
// must preserve the other leaf while it accumulates that component's points.
static int V3BlockGridMixedFragmentedLandingReducesContact( void )
{
	return RunFragmentedLanding( CookMixedFragmentedLandingTerrain(), 120 );
}

typedef struct LandingContactObservation
{
	int pointCount;
	int persistedPointCount;
} LandingContactObservation;

static bool LandingSameShapePair( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3ShapeId expectedA, b3ShapeId expectedB )
{
	return ( B3_ID_EQUALS( shapeIdA, expectedA ) && B3_ID_EQUALS( shapeIdB, expectedB ) ) ||
		   ( B3_ID_EQUALS( shapeIdA, expectedB ) && B3_ID_EQUALS( shapeIdB, expectedA ) );
}

static LandingContactObservation ObserveLandingContact( b3ShapeId shapeId, b3ShapeId otherShapeId )
{
	b3ContactData contacts[64];
	int contactCount = b3Shape_GetContactData( shapeId, contacts, 64 );
	LandingContactObservation observation = { 0 };
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		if ( LandingSameShapePair( contact->shapeIdA, contact->shapeIdB, shapeId, otherShapeId ) == false ||
			 contact->manifolds == NULL )
		{
			continue;
		}
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			const b3Manifold* manifold = contact->manifolds + manifoldIndex;
			observation.pointCount += manifold->pointCount;
			for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
			{
				observation.persistedPointCount += manifold->points[pointIndex].persisted;
			}
		}
	}
	return observation;
}

static v3BlockGridData* CreateLandingTerrain( void )
{
	v3BlockGridBlockDef blocks[terrainExtent * terrainExtent];
	v3BlockGridBoxDef boxes[terrainExtent * terrainExtent];
	int count = 0;
	for ( int z = 0; z < terrainExtent; ++z )
	{
		for ( int x = 0; x < terrainExtent; ++x )
		{
			blocks[count] = (v3BlockGridBlockDef){ .x = x, .z = z, .sourceId = (uint64_t)count + 1 };
			boxes[count] = (v3BlockGridBoxDef){
				.center = { (float)x + 0.5f, 0.5f, (float)z + 0.5f },
				.halfExtent = { 0.5f, 0.5f, 0.5f },
				.ownerBlockIndex = (uint32_t)count,
				.sourceId = 1000u + (uint64_t)count,
			};
			count += 1;
		}
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	material.restitution = 0.0f;
	v3BlockGridDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = count,
		.boxes = boxes,
		.boxCount = count,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	return v3BlockGrid_Create( &definition, &createStatus );
}

static v3BlockGridData* CreateLandingShip( void )
{
	v3BlockGridBlockDef blocks[shipLength * shipWidth * shipHeight];
	v3BlockGridBoxDef boxes[shipLength * shipWidth * shipHeight];
	int count = 0;
	for ( int y = 0; y < shipHeight; ++y )
	{
		for ( int z = 0; z < shipWidth; ++z )
		{
			for ( int x = 0; x < shipLength; ++x )
			{
				bool hull = x == 0 || x == shipLength - 1 || z == 0 || z == shipWidth - 1;
				bool deck = y == 0;
				if ( hull == false && deck == false )
				{
					continue;
				}

				b3Vec3 center = { (float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f };
				b3Vec3 halfExtent = { 0.5f, 0.5f, 0.5f };
				if ( deck && hull == false )
				{
					center.y = (float)y + 0.25f;
					halfExtent.y = 0.25f;
				}
				else if ( hull && y == shipHeight - 1 )
				{
					halfExtent.x = 0.1f;
					halfExtent.z = 0.1f;
				}

				blocks[count] = (v3BlockGridBlockDef){
					.x = x,
					.y = y,
					.z = z,
					.sourceId = (uint64_t)count + 1,
				};
				boxes[count] = (v3BlockGridBoxDef){
					.center = center,
					.halfExtent = halfExtent,
					.ownerBlockIndex = (uint32_t)count,
					.sourceId = 2000u + (uint64_t)count,
				};
				count += 1;
			}
		}
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.9f;
	material.restitution = 0.0f;
	v3BlockGridDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = count,
		.boxes = boxes,
		.boxCount = count,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	return v3BlockGrid_Create( &definition, &createStatus );
}

static bool SpawnLandingBullet( b3WorldId worldId, b3Pos shipPosition, int index, b3BodyId* bodyId, b3ShapeId* shapeId,
								float* launchY )
{
	int row = index / 4;
	int column = index % 4;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ shipPosition.x - 3.0, shipPosition.y + 0.85 + 0.75 * row, shipPosition.z + 0.75 + 1.5 * column };
	bodyDef.linearVelocity = (b3Vec3){ 24.0f, 0.0f, 0.0f };
	bodyDef.isBullet = true;
	bodyDef.enableSleep = false;
	*bodyId = b3CreateBody( worldId, &bodyDef );
	if ( B3_IS_NULL( *bodyId ) )
	{
		return false;
	}
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 4.0f;
	shapeDef.baseMaterial.friction = 0.0f;
	shapeDef.baseMaterial.restitution = 0.8f;
	*shapeId = b3CreateSphereShape( *bodyId, &shapeDef, &sphere );
	*launchY = (float)bodyDef.position.y;
	return B3_IS_NON_NULL( *shapeId );
}

// The Pair Landing ship must land on terrain, keep persistent solver contact through the bullet
// volley, and settle. Discrete contacts and Speculative Contact are the only motion model here.
static int V3BlockGridLandingSettlesAfterBulletVolley( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* terrainGrid = NULL;
	v3BlockGridData* shipGrid = NULL;
	b3BodyId shipBodyId = b3_nullBodyId;
	b3ShapeId terrainShapeId = b3_nullShapeId;
	b3ShapeId shipShapeId = b3_nullShapeId;
	b3BodyId bulletBodyIds[bulletCount] = { 0 };
	float bulletLaunchY[bulletCount] = { 0 };
	bool bulletBounced[bulletCount] = { false };
	float steadyShipSpeedMaximum = 0.0f;

	terrainGrid = CreateLandingTerrain();
	shipGrid = CreateLandingShip();
	V3_LANDING_ENSURE( terrainGrid != NULL && shipGrid != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableContinuous = true;
	worldDef.enableSleep = false;
	worldDef.maximumLinearSpeed = 1000.0f;
	worldDef.workerCount = 1;
	worldId = b3CreateWorld( &worldDef );
	V3_LANDING_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ -0.5 * terrainExtent, 0.0, -0.5 * terrainExtent };
	b3BodyId terrainBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	terrainShapeId = v3CreateBlockGridShape( terrainBodyId, &shapeDef, terrainGrid );
	V3_LANDING_ENSURE( B3_IS_NON_NULL( terrainShapeId ) );
	v3BlockGrid_Release( terrainGrid );
	terrainGrid = NULL;

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -0.5 * shipLength, 12.0, -0.5 * shipWidth };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 5.2f * B3_DEG_TO_RAD );
	bodyDef.enableSleep = false;
	shipBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shipShapeId = v3CreateBlockGridShape( shipBodyId, &shapeDef, shipGrid );
	V3_LANDING_ENSURE( B3_IS_NON_NULL( shipShapeId ) );
	v3BlockGrid_Release( shipGrid );
	shipGrid = NULL;

	for ( int step = 0; step < landingSteps; ++step )
	{
		b3World_Step( worldId, landingTimeStep, 4 );
	}
	LandingContactObservation landed = ObserveLandingContact( shipShapeId, terrainShapeId );
	b3Pos landedPosition = b3Body_GetPosition( shipBodyId );
	V3_LANDING_ENSURE( landed.pointCount > 0 && landed.persistedPointCount > 0 );

	b3Pos shipPosition = b3Body_GetPosition( shipBodyId );
	for ( int bulletIndex = 0; bulletIndex < bulletCount; ++bulletIndex )
	{
		b3ShapeId bulletShapeId = b3_nullShapeId;
		V3_LANDING_ENSURE( SpawnLandingBullet( worldId, shipPosition, bulletIndex, bulletBodyIds + bulletIndex, &bulletShapeId,
											   bulletLaunchY + bulletIndex ) );
	}

	for ( int step = 0; step < postLandingSteps; ++step )
	{
		b3World_Step( worldId, landingTimeStep, 4 );
		LandingContactObservation postLandingContact = ObserveLandingContact( shipShapeId, terrainShapeId );
		V3_LANDING_ENSURE( postLandingContact.pointCount > 0 );
		if ( step >= postLandingSteps - steadyWindowSteps )
		{
			V3_LANDING_ENSURE( postLandingContact.persistedPointCount > 0 );
			float shipSpeed = b3Length( b3Body_GetLinearVelocity( shipBodyId ) );
			steadyShipSpeedMaximum = shipSpeed > steadyShipSpeedMaximum ? shipSpeed : steadyShipSpeedMaximum;
		}
		for ( int bulletIndex = 0; bulletIndex < bulletCount; ++bulletIndex )
		{
			if ( bulletBounced[bulletIndex] == false && b3Body_GetLinearVelocity( bulletBodyIds[bulletIndex] ).x < -0.5f )
			{
				b3Pos bouncePosition = b3Body_GetPosition( bulletBodyIds[bulletIndex] );
				bulletBounced[bulletIndex] = bouncePosition.x > shipPosition.x - 0.5 && bouncePosition.x < shipPosition.x + 0.5 &&
											 bouncePosition.y > shipPosition.y;
			}
		}
	}

	LandingContactObservation resting = ObserveLandingContact( shipShapeId, terrainShapeId );
	b3Pos restingPosition = b3Body_GetPosition( shipBodyId );
	V3_LANDING_ENSURE( resting.pointCount > 0 && resting.persistedPointCount > 0 );
	V3_LANDING_ENSURE( b3Length( b3SubPos( restingPosition, landedPosition ) ) < 0.05f );
	V3_LANDING_ENSURE( steadyShipSpeedMaximum < steadyShipSpeedBound );
	for ( int bulletIndex = 0; bulletIndex < bulletCount; ++bulletIndex )
	{
		V3_LANDING_ENSURE( bulletBounced[bulletIndex] );
		V3_LANDING_ENSURE( (float)b3Body_GetPosition( bulletBodyIds[bulletIndex] ).y < bulletLaunchY[bulletIndex] - 1.0f );
	}

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3BlockGrid_Release( terrainGrid );
	v3BlockGrid_Release( shipGrid );
	return status;
}

static v3BlockGridData* CreateRestingUnitGrid( b3SurfaceMaterial material )
{
	v3BlockGridBlockDef block = { .sourceId = 1 };
	v3BlockGridBoxDef box = {
		.center = { 0.5f, 0.5f, 0.5f },
		.halfExtent = { 0.5f, 0.5f, 0.5f },
		.ownerBlockIndex = 0,
		.sourceId = 2,
	};
	v3BlockGridDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = &block,
		.blockCount = 1,
		.boxes = &box,
		.boxCount = 1,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	return v3BlockGrid_Create( &definition, &createStatus );
}

static bool CreateRestingPair( b3WorldId worldId, b3SurfaceMaterial staticMaterial, b3BodyId* dynamicBodyId,
							   b3ShapeId* staticShapeId, b3ShapeId* dynamicShapeId )
{
	v3BlockGridData* staticGrid = CreateRestingUnitGrid( staticMaterial );
	v3BlockGridData* dynamicGrid = CreateRestingUnitGrid( b3DefaultSurfaceMaterial() );
	if ( staticGrid == NULL || dynamicGrid == NULL )
	{
		v3BlockGrid_Release( staticGrid );
		v3BlockGrid_Release( dynamicGrid );
		return false;
	}

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BodyDef staticDef = b3DefaultBodyDef();
	b3BodyId staticBodyId = b3CreateBody( worldId, &staticDef );
	*staticShapeId = v3CreateBlockGridShape( staticBodyId, &shapeDef, staticGrid );

	b3BodyDef dynamicDef = b3DefaultBodyDef();
	dynamicDef.type = b3_dynamicBody;
	dynamicDef.position = (b3Pos){ 0.0, 1.0, 0.0 };
	dynamicDef.enableSleep = false;
	*dynamicBodyId = b3CreateBody( worldId, &dynamicDef );
	*dynamicShapeId = v3CreateBlockGridShape( *dynamicBodyId, &shapeDef, dynamicGrid );
	v3BlockGrid_Release( staticGrid );
	v3BlockGrid_Release( dynamicGrid );
	return B3_IS_NON_NULL( staticBodyId ) && B3_IS_NON_NULL( *staticShapeId ) && B3_IS_NON_NULL( *dynamicBodyId ) &&
		   B3_IS_NON_NULL( *dynamicShapeId );
}

// Migrated from the pair-counter suite: a resting BlockGrid contact keeps its points and its
// solved impulses from one step to the next, now that every step rebuilds the manifold.
static int V3BlockGridRestingContactPersistsAcrossSteps( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId dynamicBodyId = b3_nullBodyId;
	b3ShapeId staticShapeId = b3_nullShapeId;
	b3ShapeId dynamicShapeId = b3_nullShapeId;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	worldId = b3CreateWorld( &worldDef );
	V3_LANDING_ENSURE( b3World_IsValid( worldId ) );
	V3_LANDING_ENSURE(
		CreateRestingPair( worldId, b3DefaultSurfaceMaterial(), &dynamicBodyId, &staticShapeId, &dynamicShapeId ) );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, landingTimeStep, 4 );
	}
	LandingContactObservation settled = ObserveLandingContact( dynamicShapeId, staticShapeId );
	V3_LANDING_ENSURE( settled.pointCount > 0 );
	V3_LANDING_ENSURE( settled.persistedPointCount == settled.pointCount );

	b3World_Step( worldId, landingTimeStep, 4 );
	LandingContactObservation next = ObserveLandingContact( dynamicShapeId, staticShapeId );
	V3_LANDING_ENSURE( next.pointCount == settled.pointCount );
	V3_LANDING_ENSURE( next.persistedPointCount == settled.pointCount );
	V3_LANDING_ENSURE( b3Length( b3Body_GetLinearVelocity( dynamicBodyId ) ) < 0.05f );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	return status;
}

// Migrated from the pair-counter suite: a conveyor tangent velocity cooked into the resting
// surface's material table reaches the resting body, so a BlockGrid contact resolves its
// material from the cooked table on every step instead of holding a stale value at rest.
static int V3BlockGridRestingContactRefreshesTangentVelocity( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId dynamicBodyId = b3_nullBodyId;
	b3ShapeId staticShapeId = b3_nullShapeId;
	b3ShapeId dynamicShapeId = b3_nullShapeId;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	worldId = b3CreateWorld( &worldDef );
	V3_LANDING_ENSURE( b3World_IsValid( worldId ) );
	b3SurfaceMaterial conveyor = b3DefaultSurfaceMaterial();
	conveyor.tangentVelocity = (b3Vec3){ 4.0f, 0.0f, 0.0f };
	V3_LANDING_ENSURE( CreateRestingPair( worldId, conveyor, &dynamicBodyId, &staticShapeId, &dynamicShapeId ) );

	b3World_Step( worldId, landingTimeStep, 4 );
	b3Body_SetLinearVelocity( dynamicBodyId, b3Vec3_zero );
	V3_LANDING_ENSURE( b3AbsFloat( b3Body_GetLinearVelocity( dynamicBodyId ).x ) < 1.0e-6f );

	b3World_Step( worldId, landingTimeStep, 4 );
	V3_LANDING_ENSURE( b3AbsFloat( b3Body_GetLinearVelocity( dynamicBodyId ).x ) > 0.01f );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	return status;
}

// Translating the cell lattice changes the local centroid, not mass, inertia or support height.
static int V3BlockGridTranslatedHullRests( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* grid = NULL;
	const int origins[] = { -3, 0, 3 };
	for ( int originIndex = 0; originIndex < 3; ++originIndex )
	{
		int origin = origins[originIndex];
		v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, 1, 1 } } };
		v3BlockGridBlock blocks[144];
		int count = 0;
		for ( int x = 0; x < 6; ++x )
		{
			for ( int y = 0; y < 4; ++y )
			{
				for ( int z = 0; z < 6; ++z )
				{
					blocks[count++] = (v3BlockGridBlock){
						.x = x + origin,
						.y = y,
						.z = z + origin,
						.boxes = &box,
						.boxCount = 1,
					};
				}
			}
		}
		b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
		v3BlockGridCookDef cook = {
			.materials = &material,
			.materialCount = 1,
			.blocks = blocks,
			.blockCount = count,
		};
		v3BlockGridCookResult result = v3CookBlockGrid( &cook );
		grid = result.data;
		V3_LANDING_ENSURE( result.status == v3_blockGridCookOk );
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.enableSleep = false;
		worldId = b3CreateWorld( &worldDef );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = (b3Pos){ 0, -0.5, 0 };
		b3BodyId floor = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 1.0f;
		b3BoxHull floorHull = b3MakeBoxHull( 30, 0.5f, 30 );
		b3CreateHullShape( floor, &shapeDef, &floorHull.base );
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -origin, 2, -origin };
		b3BodyId body = b3CreateBody( worldId, &bodyDef );
		b3ShapeId shape = v3CreateBlockGridShape( body, &shapeDef, grid );
		V3_LANDING_ENSURE( b3Shape_IsValid( shape ) );
		b3MassData mass = b3Body_GetMassData( body );
		V3_LANDING_ENSURE( b3AbsFloat( mass.mass - 144.0f ) < 0.001f );
		V3_LANDING_ENSURE( b3AbsFloat( mass.center.x - ( origin + 3.0f ) ) < 0.001f );
		V3_LANDING_ENSURE( b3AbsFloat( mass.center.y - 2.0f ) < 0.001f );
		V3_LANDING_ENSURE( b3AbsFloat( mass.center.z - ( origin + 3.0f ) ) < 0.001f );
		// A uniform 6 by 4 by 6 solid has principal inertia m * (a*a + b*b) / 12.
		V3_LANDING_ENSURE( b3Length( b3Sub( mass.inertia.cx, (b3Vec3){ 624, 0, 0 } ) ) < 0.001f );
		V3_LANDING_ENSURE( b3Length( b3Sub( mass.inertia.cy, (b3Vec3){ 0, 864, 0 } ) ) < 0.001f );
		V3_LANDING_ENSURE( b3Length( b3Sub( mass.inertia.cz, (b3Vec3){ 0, 0, 624 } ) ) < 0.001f );
		for ( int step = 0; step < 300; ++step )
		{
			b3World_Step( worldId, landingTimeStep, 4 );
			if ( step >= 60 )
			{
				V3_LANDING_ENSURE( b3AbsFloat( (float)b3Body_GetPosition( body ).y ) < B3_LINEAR_SLOP );
			}
		}
		b3DestroyWorld( worldId );
		worldId = b3_nullWorldId;
		v3DestroyBlockGridData( grid );
		grid = NULL;
	}
	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( grid );
	return status;
}

int V3BlockGridLandingTest( void )
{
	RUN_SUBTEST( V3BlockGridFragmentedLandingReducesContact );
	RUN_SUBTEST( V3BlockGridMixedFragmentedLandingReducesContact );
	RUN_SUBTEST( V3BlockGridTranslatedHullRests );
	RUN_SUBTEST( V3BlockGridLandingSettlesAfterBulletVolley );
	RUN_SUBTEST( V3BlockGridRestingContactPersistsAcrossSteps );
	RUN_SUBTEST( V3BlockGridRestingContactRefreshesTangentVelocity );
	return 0;
}
