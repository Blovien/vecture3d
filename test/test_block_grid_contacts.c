// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

// BlockGrid pair tests use shape creation, stepping, contact queries, counters, and recordings.
// Cooked-layout assertions also use private accessors.

#include "test_macros.h"
#include "block_grid/block_grid.h"

#include "box3d/box3d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define V3_CONTACTS_ENSURE( C )                                                                                                  \
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

typedef struct ContactGridOptions
{
	int width;
	bool giantBox;
	bool distinctMaterials;
	uint64_t userMaterialBase;
} ContactGridOptions;

static v3BlockGridData* CreateContactGrid( ContactGridOptions options )
{
	int blockCount = options.giantBox ? 1 : options.width;
	int materialCount = options.distinctMaterials ? options.width : 1;
	b3SurfaceMaterial* materials = calloc( (size_t)materialCount, sizeof( b3SurfaceMaterial ) );
	v3BlockGridBlockDef* blocks = calloc( (size_t)blockCount, sizeof( v3BlockGridBlockDef ) );
	v3BlockGridBoxDef* boxes = calloc( (size_t)blockCount, sizeof( v3BlockGridBoxDef ) );
	if ( materials == NULL || blocks == NULL || boxes == NULL )
	{
		free( materials );
		free( blocks );
		free( boxes );
		return NULL;
	}

	uint64_t userMaterialBase = options.userMaterialBase != 0 ? options.userMaterialBase : 1;
	for ( int materialIndex = 0; materialIndex < materialCount; ++materialIndex )
	{
		materials[materialIndex] = b3DefaultSurfaceMaterial();
		materials[materialIndex].friction = 0.8f;
		materials[materialIndex].rollingResistance = 0.2f;
		materials[materialIndex].tangentVelocity = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		materials[materialIndex].userMaterialId = userMaterialBase + (uint64_t)materialIndex;
	}

	for ( int blockIndex = 0; blockIndex < blockCount; ++blockIndex )
	{
		uint32_t materialIndex = options.distinctMaterials ? (uint32_t)blockIndex : 0;
		blocks[blockIndex] = (v3BlockGridBlockDef){
			.x = blockIndex,
			.materialIndex = materialIndex,
			.sourceId = (uint64_t)blockIndex + 1,
		};
		boxes[blockIndex] = (v3BlockGridBoxDef){
			.center = options.giantBox ? (b3Vec3){ 0.5f * options.width, 0.5f, 0.5f }
									   : (b3Vec3){ (float)blockIndex + 0.5f, 0.5f, 0.5f },
			.halfExtent = options.giantBox ? (b3Vec3){ 0.5f * options.width, 0.5f, 0.5f } : (b3Vec3){ 0.5f, 0.5f, 0.5f },
			.ownerBlockIndex = (uint32_t)blockIndex,
			.materialIndex = materialIndex,
			.sourceId = (uint64_t)blockCount + (uint64_t)blockIndex + 1,
		};
	}

	v3BlockGridDef definition = {
		.materials = materials,
		.materialCount = materialCount,
		.blocks = blocks,
		.blockCount = blockCount,
		.boxes = boxes,
		.boxCount = blockCount,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	v3BlockGridData* grid = v3BlockGrid_Create( &definition, &createStatus );
	free( materials );
	free( blocks );
	free( boxes );
	return createStatus == v3_blockGridCreateOk ? grid : NULL;
}

typedef struct ContactFixture
{
	b3WorldId worldId;
	b3BodyId movingBodyId;
	b3ShapeId fixedShapeId;
	b3ShapeId movingShapeId;
} ContactFixture;

static bool AttachContactPair( b3WorldId worldId, ContactGridOptions fixedOptions, ContactGridOptions movingOptions,
							   b3Pos fixedPosition, b3Pos movingPosition, ContactFixture* fixture )
{
	v3BlockGridData* fixedGrid = CreateContactGrid( fixedOptions );
	v3BlockGridData* movingGrid = CreateContactGrid( movingOptions );
	if ( fixedGrid == NULL || movingGrid == NULL )
	{
		v3BlockGrid_Release( fixedGrid );
		v3BlockGrid_Release( movingGrid );
		return false;
	}

	b3BodyDef fixedBodyDef = b3DefaultBodyDef();
	fixedBodyDef.position = fixedPosition;
	b3BodyId fixedBodyId = b3CreateBody( worldId, &fixedBodyDef );
	b3ShapeDef fixedShapeDef = b3DefaultShapeDef();
	fixedShapeDef.baseMaterial.friction = 0.8f;
	fixture->fixedShapeId = v3CreateBlockGridShape( fixedBodyId, &fixedShapeDef, fixedGrid );

	b3BodyDef movingBodyDef = b3DefaultBodyDef();
	movingBodyDef.type = b3_dynamicBody;
	movingBodyDef.position = movingPosition;
	movingBodyDef.enableSleep = false;
	fixture->movingBodyId = b3CreateBody( worldId, &movingBodyDef );
	b3ShapeDef movingShapeDef = b3DefaultShapeDef();
	movingShapeDef.density = 1.0f;
	movingShapeDef.baseMaterial.friction = 0.8f;
	fixture->movingShapeId = v3CreateBlockGridShape( fixture->movingBodyId, &movingShapeDef, movingGrid );
	v3BlockGrid_Release( fixedGrid );
	v3BlockGrid_Release( movingGrid );
	fixture->worldId = worldId;
	return B3_IS_NON_NULL( fixedBodyId ) && B3_IS_NON_NULL( fixture->fixedShapeId ) && B3_IS_NON_NULL( fixture->movingBodyId ) &&
		   B3_IS_NON_NULL( fixture->movingShapeId );
}

static bool CreateContactFixture( ContactGridOptions fixedOptions, ContactGridOptions movingOptions, b3Pos movingPosition,
								  ContactFixture* fixture )
{
	*fixture = (ContactFixture){ .worldId = b3_nullWorldId };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	if ( B3_IS_NULL( worldId ) )
	{
		return false;
	}
	fixture->worldId = worldId;
	return AttachContactPair( worldId, fixedOptions, movingOptions, b3Pos_zero, movingPosition, fixture );
}

static void DestroyContactFixture( ContactFixture* fixture )
{
	if ( b3World_IsValid( fixture->worldId ) )
	{
		b3DestroyWorld( fixture->worldId );
	}
	*fixture = (ContactFixture){ .worldId = b3_nullWorldId };
}

static bool SameContactShapePair( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3ShapeId expectedA, b3ShapeId expectedB )
{
	return ( B3_ID_EQUALS( shapeIdA, expectedA ) && B3_ID_EQUALS( shapeIdB, expectedB ) ) ||
		   ( B3_ID_EQUALS( shapeIdA, expectedB ) && B3_ID_EQUALS( shapeIdB, expectedA ) );
}

typedef struct ContactObservation
{
	float normalImpulseSum;
	int manifoldCount;
	int pointCount;
	int persistedPointCount;
	int frictionPatchCount;
	int twistPatchCount;
	int rollingPatchCount;
} ContactObservation;

static bool ObservePairContact( const ContactFixture* fixture, ContactObservation* observation )
{
	*observation = (ContactObservation){ 0 };
	b3ContactData contacts[4];
	int contactCount = b3Shape_GetContactData( fixture->movingShapeId, contacts, 4 );
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		if ( SameContactShapePair( contact->shapeIdA, contact->shapeIdB, fixture->fixedShapeId, fixture->movingShapeId ) ==
				 false ||
			 contact->manifolds == NULL )
		{
			continue;
		}

		observation->manifoldCount = contact->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			const b3Manifold* manifold = contact->manifolds + manifoldIndex;
			observation->pointCount += manifold->pointCount;
			observation->frictionPatchCount += b3LengthSquared( manifold->frictionImpulse ) > 1.0e-10f;
			observation->twistPatchCount += fabsf( manifold->twistImpulse ) > 1.0e-5f;
			observation->rollingPatchCount += b3LengthSquared( manifold->rollingImpulse ) > 1.0e-10f;
			for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
			{
				const b3ManifoldPoint* point = manifold->points + pointIndex;
				observation->normalImpulseSum += point->normalImpulse;
				observation->persistedPointCount += point->persisted;
			}
		}
		return observation->pointCount > 0;
	}
	return false;
}

// A settled contact that slides onto the next cell keeps its impulses instead of restarting them.
static int V3BlockGridContactPersistsAcrossCellBoundary( void )
{
	int status = 1;
	ContactFixture fixture = { .worldId = b3_nullWorldId };
	V3_CONTACTS_ENSURE( CreateContactFixture( (ContactGridOptions){ .width = 2 }, (ContactGridOptions){ .width = 1 },
											  (b3Pos){ -0.04, 1.0, 0.0 }, &fixture ) );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	ContactObservation previous;
	V3_CONTACTS_ENSURE( ObservePairContact( &fixture, &previous ) );
	V3_CONTACTS_ENSURE( previous.normalImpulseSum > 0.0f );

	b3Body_SetTransform( fixture.movingBodyId, (b3Pos){ 0.005, 1.0, 0.0 }, b3Quat_identity );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	ContactObservation current;
	V3_CONTACTS_ENSURE( ObservePairContact( &fixture, &current ) );
	V3_CONTACTS_ENSURE( current.persistedPointCount > 0 );
	V3_CONTACTS_ENSURE( current.normalImpulseSum > 0.0f );

	status = 0;
cleanup:
	DestroyContactFixture( &fixture );
	return status;
}

// A manifold that splits into two patches claims each retained point and patch impulse once.
static int V3BlockGridSplitPatchClaimsImpulsesOnce( void )
{
	int status = 1;
	ContactFixture fixture = { .worldId = b3_nullWorldId };
	V3_CONTACTS_ENSURE( CreateContactFixture( (ContactGridOptions){ .width = 2, .distinctMaterials = true },
											  (ContactGridOptions){ .width = 1 }, (b3Pos){ -0.04, 1.0, 0.0 }, &fixture ) );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	b3Body_SetTransform( fixture.movingBodyId, (b3Pos){ -0.04, 1.0, 0.0 }, b3Quat_identity );
	b3Body_SetAngularVelocity( fixture.movingBodyId, (b3Vec3){ 0.5f, 1.0f, 0.5f } );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	ContactObservation previous;
	V3_CONTACTS_ENSURE( ObservePairContact( &fixture, &previous ) );
	V3_CONTACTS_ENSURE( previous.manifoldCount == 1 );
	V3_CONTACTS_ENSURE( previous.normalImpulseSum > 0.0f );
	V3_CONTACTS_ENSURE( previous.frictionPatchCount == 1 );
	V3_CONTACTS_ENSURE( previous.twistPatchCount == 1 );
	V3_CONTACTS_ENSURE( previous.rollingPatchCount == 1 );

	b3Body_SetTransform( fixture.movingBodyId, (b3Pos){ 0.005, 1.0, 0.0 }, b3Quat_identity );
	b3World_Step( fixture.worldId, 0.0f, 4 );
	ContactObservation current;
	V3_CONTACTS_ENSURE( ObservePairContact( &fixture, &current ) );
	V3_CONTACTS_ENSURE( current.manifoldCount == 2 );
	V3_CONTACTS_ENSURE( current.persistedPointCount > 0 );
	V3_CONTACTS_ENSURE( current.persistedPointCount <= previous.pointCount );
	V3_CONTACTS_ENSURE( current.normalImpulseSum <= previous.normalImpulseSum + 1.0e-5f );
	V3_CONTACTS_ENSURE( current.frictionPatchCount == 1 );
	V3_CONTACTS_ENSURE( current.twistPatchCount == 1 );
	V3_CONTACTS_ENSURE( current.rollingPatchCount == 1 );

	status = 0;
cleanup:
	DestroyContactFixture( &fixture );
	return status;
}

static bool RecordPairVariation( bool fixedIsGiant, b3Recording* recording )
{
	ContactFixture fixture = { .worldId = b3_nullWorldId };
	if ( CreateContactFixture( (ContactGridOptions){ .width = 6, .giantBox = fixedIsGiant },
							   (ContactGridOptions){ .width = 6, .giantBox = !fixedIsGiant }, (b3Pos){ 0.0, 1.0, 0.0 },
							   &fixture ) == false )
	{
		DestroyContactFixture( &fixture );
		return false;
	}

	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	b3World_StartRecording( fixture.worldId, recording );
	for ( int step = 0; step < 8; ++step )
	{
		b3Pos position = b3Body_GetPosition( fixture.movingBodyId );
		position.x += 0.01;
		b3Body_SetTransform( fixture.movingBodyId, position, b3Quat_identity );
		b3World_Step( fixture.worldId, 0.0f, 4 );
	}
	b3World_StopRecording( fixture.worldId );
	DestroyContactFixture( &fixture );
	return b3Recording_GetSize( recording ) > 0;
}

// A resting hull recorded in both traversal directions replays to the same state on one and four workers.
static int V3BlockGridRecordingReplayIsExact( void )
{
	int status = 1;
	b3Recording* aToB = b3CreateRecording( 0 );
	b3Recording* bToA = b3CreateRecording( 0 );
	V3_CONTACTS_ENSURE( aToB != NULL );
	V3_CONTACTS_ENSURE( bToA != NULL );
	V3_CONTACTS_ENSURE( RecordPairVariation( true, aToB ) );
	V3_CONTACTS_ENSURE( RecordPairVariation( false, bToA ) );
	V3_CONTACTS_ENSURE( b3ValidateReplay( b3Recording_GetData( aToB ), b3Recording_GetSize( aToB ), 1 ) );
	V3_CONTACTS_ENSURE( b3ValidateReplay( b3Recording_GetData( aToB ), b3Recording_GetSize( aToB ), 4 ) );
	V3_CONTACTS_ENSURE( b3ValidateReplay( b3Recording_GetData( bToA ), b3Recording_GetSize( bToA ), 1 ) );
	V3_CONTACTS_ENSURE( b3ValidateReplay( b3Recording_GetData( bToA ), b3Recording_GetSize( bToA ), 4 ) );

	status = 0;
cleanup:
	b3DestroyRecording( aToB );
	b3DestroyRecording( bToA );
	return status;
}

enum
{
	poisonedUserMaterialId = 11,
	healthyUserMaterialId = 22,
};

static float PoisonedFrictionCallback( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB )
{
	(void)frictionA;
	(void)frictionB;
	if ( userMaterialIdA == poisonedUserMaterialId || userMaterialIdB == poisonedUserMaterialId )
	{
		return NAN;
	}
	return 0.5f;
}

// A surface material callback that returns an unusable value drops the contact that reads it for
// that step, the way an ordinary Box3D contact drops its manifold, and leaves every other pair alone.
static int V3BlockGridBadMaterialDropsOnlyItsPair( void )
{
	int status = 1;
	ContactFixture poisoned = { .worldId = b3_nullWorldId };
	ContactFixture healthy = { .worldId = b3_nullWorldId };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	V3_CONTACTS_ENSURE( b3World_IsValid( worldId ) );

	V3_CONTACTS_ENSURE( AttachContactPair( worldId,
										   (ContactGridOptions){ .width = 2, .userMaterialBase = poisonedUserMaterialId },
										   (ContactGridOptions){ .width = 1, .userMaterialBase = poisonedUserMaterialId },
										   b3Pos_zero, (b3Pos){ 0.0, 1.0, 0.0 }, &poisoned ) );
	V3_CONTACTS_ENSURE( AttachContactPair( worldId, (ContactGridOptions){ .width = 2, .userMaterialBase = healthyUserMaterialId },
										   (ContactGridOptions){ .width = 1, .userMaterialBase = healthyUserMaterialId },
										   (b3Pos){ 40.0, 0.0, 0.0 }, (b3Pos){ 40.0, 1.0, 0.0 }, &healthy ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ContactObservation observation;
	V3_CONTACTS_ENSURE( ObservePairContact( &poisoned, &observation ) && observation.pointCount > 0 );
	V3_CONTACTS_ENSURE( ObservePairContact( &healthy, &observation ) && observation.pointCount > 0 );

	b3World_SetFrictionCallback( worldId, PoisonedFrictionCallback );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	V3_CONTACTS_ENSURE( ObservePairContact( &poisoned, &observation ) == false );
	V3_CONTACTS_ENSURE( observation.pointCount == 0 );
	V3_CONTACTS_ENSURE( ObservePairContact( &healthy, &observation ) );
	V3_CONTACTS_ENSURE( observation.pointCount > 0 );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	return status;
}

enum
{
	slidingUserMaterialBase = 31,
	floorUserMaterialId = 33,
	wallUserMaterialBase = 41,
	slugUserMaterialId = 51,
};

// One hull with two Hitboxes stacked in y, each carrying its own surface material, so the same
// cooked grid can be rested on either face.
static v3BlockGridData* CreateTwoMaterialColumn( float lowerFriction, float upperFriction )
{
	b3SurfaceMaterial materials[2];
	v3BlockGridBlockDef blocks[2];
	v3BlockGridBoxDef boxes[2];
	for ( int i = 0; i < 2; ++i )
	{
		materials[i] = b3DefaultSurfaceMaterial();
		materials[i].friction = i == 0 ? lowerFriction : upperFriction;
		materials[i].userMaterialId = slidingUserMaterialBase + (uint64_t)i;
		blocks[i] = (v3BlockGridBlockDef){ .y = i, .materialIndex = (uint32_t)i, .sourceId = (uint64_t)i + 1 };
		boxes[i] = (v3BlockGridBoxDef){
			.center = { 0.5f, (float)i + 0.5f, 0.5f },
			.halfExtent = { 0.5f, 0.5f, 0.5f },
			.ownerBlockIndex = (uint32_t)i,
			.materialIndex = (uint32_t)i,
			.sourceId = (uint64_t)i + 3,
		};
	}

	v3BlockGridDef definition = {
		.materials = materials,
		.materialCount = 2,
		.blocks = blocks,
		.blockCount = 2,
		.boxes = boxes,
		.boxCount = 2,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	v3BlockGridData* grid = v3BlockGrid_Create( &definition, &createStatus );
	return createStatus == v3_blockGridCreateOk ? grid : NULL;
}

// One flat static floor of unit Hitboxes sharing a single material, long enough for the slide window
static v3BlockGridData* CreateFloorGrid( float friction, int cellCount )
{
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = friction;
	material.userMaterialId = floorUserMaterialId;
	v3BlockGridBlockDef* blocks = calloc( (size_t)cellCount, sizeof( v3BlockGridBlockDef ) );
	v3BlockGridBoxDef* boxes = calloc( (size_t)cellCount, sizeof( v3BlockGridBoxDef ) );
	if ( blocks == NULL || boxes == NULL )
	{
		free( blocks );
		free( boxes );
		return NULL;
	}

	for ( int i = 0; i < cellCount; ++i )
	{
		blocks[i] = (v3BlockGridBlockDef){ .x = i, .sourceId = (uint64_t)i + 1 };
		boxes[i] = (v3BlockGridBoxDef){
			.center = { (float)i + 0.5f, 0.5f, 0.5f },
			.halfExtent = { 0.5f, 0.5f, 0.5f },
			.ownerBlockIndex = (uint32_t)i,
			.sourceId = (uint64_t)cellCount + (uint64_t)i + 1,
		};
	}

	v3BlockGridDef definition = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = cellCount,
		.boxes = boxes,
		.boxCount = cellCount,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus = v3_blockGridCreateInvalidDefinition;
	v3BlockGridData* grid = v3BlockGrid_Create( &definition, &createStatus );
	free( blocks );
	free( boxes );
	return createStatus == v3_blockGridCreateOk ? grid : NULL;
}

// Rests one face of the column on a floor and slides it for the window, reporting the speed left.
// Rotation is locked so the column cannot pitch and the contact carries the full weight throughout.
// A null floorGrid gives a box hull floor and the grid-to-convex contact path; a floor grid gives a
// BlockGrid pair contact, so the same expectations cover both material paths.
static bool SlideColumnOnFloor( v3BlockGridData* grid, v3BlockGridData* floorGrid, bool upperFaceDown, float floorFriction,
								int stepCount, float timeStep, float* remainingSpeed )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	if ( b3World_IsValid( worldId ) == false )
	{
		return false;
	}

	// Either floor puts its top face at y = 0 and reaches well past the end of the slide
	b3BodyDef floorBodyDef = b3DefaultBodyDef();
	floorBodyDef.position = floorGrid != NULL ? (b3Pos){ -2.0, -1.0, 0.0 } : (b3Pos){ 0.0, -0.5, 0.0 };
	b3BodyId floorBodyId = b3CreateBody( worldId, &floorBodyDef );
	b3ShapeDef floorShapeDef = b3DefaultShapeDef();
	floorShapeDef.baseMaterial.friction = floorFriction;
	b3BoxHull floorHull = b3MakeBoxHull( 200.0f, 0.5f, 200.0f );
	b3ShapeId floorShapeId = floorGrid != NULL ? v3CreateBlockGridShape( floorBodyId, &floorShapeDef, floorGrid )
											   : b3CreateHullShape( floorBodyId, &floorShapeDef, &floorHull.base );

	// A half turn about z puts the upper Hitbox on the floor without changing the cooked hull
	b3BodyDef columnBodyDef = b3DefaultBodyDef();
	columnBodyDef.type = b3_dynamicBody;
	columnBodyDef.enableSleep = false;
	columnBodyDef.motionLocks = (b3MotionLocks){ .angularX = true, .angularY = true, .angularZ = true };
	columnBodyDef.position = upperFaceDown ? (b3Pos){ 0.0, 2.0, 0.0 } : b3Pos_zero;
	columnBodyDef.rotation = upperFaceDown ? b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_PI ) : b3Quat_identity;
	b3BodyId columnBodyId = b3CreateBody( worldId, &columnBodyDef );
	b3ShapeDef columnShapeDef = b3DefaultShapeDef();
	columnShapeDef.density = 1.0f;
	b3ShapeId columnShapeId = v3CreateBlockGridShape( columnBodyId, &columnShapeDef, grid );

	bool created = B3_IS_NON_NULL( floorShapeId ) && B3_IS_NON_NULL( columnShapeId );
	if ( created )
	{
		b3Body_SetLinearVelocity( columnBodyId, (b3Vec3){ 8.0f, 0.0f, 0.0f } );
		for ( int step = 0; step < stepCount; ++step )
		{
			b3World_Step( worldId, timeStep, 4 );
		}
		*remainingSpeed = b3Body_GetLinearVelocity( columnBodyId ).x;
	}
	b3DestroyWorld( worldId );
	return created;
}

// Two Hitboxes of one hull carry different friction, and each face slides at its own rate.
//
// Derivation, no value taken from a run. The floor is level, rotation is locked, and the column
// rests on one Hitbox, so the contact carries the whole weight and the normal impulse per substep
// is m*g*h. Coulomb friction saturates while the body slides, so the tangential impulse is
// mu*m*g*h and the deceleration is a = mu*g, independent of mass. Box3D's default friction
// callback is the geometric mean, so mu = sqrt(faceFriction * floorFriction). With the floor at
// 1.0, the lower face at 0.25 and the upper face at 1.0, mu is 0.5 and 1.0, so after t = 0.5 s at
// g = 10 the speed left of v0 = 8 m/s is 8 - 0.5*10*0.5 = 5.5 and 8 - 1.0*10*0.5 = 3.0 m/s.
// Tolerance is three steps of the larger deceleration, 3 * 10 * (1/60) = 0.5 m/s: the first step
// may resolve a speculative point before friction saturates, and the solver applies friction
// against the previous substep's normal impulse, which lags by one substep.
static int V3BlockGridHitboxMaterialsSlideIndependently( void )
{
	int status = 1;
	v3BlockGridData* column = CreateTwoMaterialColumn( 0.25f, 1.0f );
	V3_CONTACTS_ENSURE( column != NULL );

	const int stepCount = 30;
	const float timeStep = 1.0f / 60.0f;
	const float tolerance = 0.5f;
	float lowerFaceSpeed = 0.0f;
	float upperFaceSpeed = 0.0f;
	V3_CONTACTS_ENSURE( SlideColumnOnFloor( column, NULL, false, 1.0f, stepCount, timeStep, &lowerFaceSpeed ) );
	V3_CONTACTS_ENSURE( SlideColumnOnFloor( column, NULL, true, 1.0f, stepCount, timeStep, &upperFaceSpeed ) );
	V3_CONTACTS_ENSURE( fabsf( lowerFaceSpeed - 5.5f ) <= tolerance );
	V3_CONTACTS_ENSURE( fabsf( upperFaceSpeed - 3.0f ) <= tolerance );

	status = 0;
cleanup:
	v3BlockGrid_Release( column );
	return status;
}

// Uses a BlockGrid floor so v3BlockGridPairResolvePatchMaterial resolves both hitbox materials.
// The fixture uses the preceding friction derivation, expected speeds, and tolerance.
// The floor spans ten cells from x = -2, beyond the 4 m the column can travel during the test.
static int V3BlockGridHitboxMaterialsSlideOnGridFloor( void )
{
	int status = 1;
	v3BlockGridData* column = CreateTwoMaterialColumn( 0.25f, 1.0f );
	v3BlockGridData* floor = CreateFloorGrid( 1.0f, 10 );
	V3_CONTACTS_ENSURE( column != NULL );
	V3_CONTACTS_ENSURE( floor != NULL );

	const int stepCount = 30;
	const float timeStep = 1.0f / 60.0f;
	const float tolerance = 0.5f;
	float lowerFaceSpeed = 0.0f;
	float upperFaceSpeed = 0.0f;
	V3_CONTACTS_ENSURE( SlideColumnOnFloor( column, floor, false, 1.0f, stepCount, timeStep, &lowerFaceSpeed ) );
	V3_CONTACTS_ENSURE( SlideColumnOnFloor( column, floor, true, 1.0f, stepCount, timeStep, &upperFaceSpeed ) );
	V3_CONTACTS_ENSURE( fabsf( lowerFaceSpeed - 5.5f ) <= tolerance );
	V3_CONTACTS_ENSURE( fabsf( upperFaceSpeed - 3.0f ) <= tolerance );

	status = 0;
cleanup:
	v3BlockGrid_Release( column );
	v3BlockGrid_Release( floor );
	return status;
}

// A hit event names the struck Hitbox's surface material, not the shape's first one.
static int V3BlockGridHitEventNamesStruckHitboxMaterial( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wall = CreateContactGrid(
		(ContactGridOptions){ .width = 2, .distinctMaterials = true, .userMaterialBase = wallUserMaterialBase } );
	v3BlockGridData* slug = CreateContactGrid( (ContactGridOptions){ .width = 1, .userMaterialBase = slugUserMaterialId } );
	V3_CONTACTS_ENSURE( wall != NULL );
	V3_CONTACTS_ENSURE( slug != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldId = b3CreateWorld( &worldDef );
	V3_CONTACTS_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef wallBodyDef = b3DefaultBodyDef();
	b3BodyId wallBodyId = b3CreateBody( worldId, &wallBodyDef );
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	wallShapeDef.enableHitEvents = true;
	b3ShapeId wallShapeId = v3CreateBlockGridShape( wallBodyId, &wallShapeDef, wall );

	// The slug sits over the second cell only, so the second Hitbox is the one it strikes
	b3BodyDef slugBodyDef = b3DefaultBodyDef();
	slugBodyDef.type = b3_dynamicBody;
	slugBodyDef.enableSleep = false;
	slugBodyDef.position = (b3Pos){ 1.2, 0.999, 0.0 };
	b3BodyId slugBodyId = b3CreateBody( worldId, &slugBodyDef );
	b3ShapeDef slugShapeDef = b3DefaultShapeDef();
	slugShapeDef.density = 1.0f;
	slugShapeDef.enableHitEvents = true;
	b3ShapeId slugShapeId = v3CreateBlockGridShape( slugBodyId, &slugShapeDef, slug );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( wallShapeId ) );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( slugShapeId ) );

	// The slug already overlaps the second Hitbox, so the first step resolves the impact
	b3Body_SetLinearVelocity( slugBodyId, (b3Vec3){ 0.0f, -40.0f, 0.0f } );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3ContactEvents events = b3World_GetContactEvents( worldId );
	int matchCount = 0;
	for ( int i = 0; i < events.hitCount; ++i )
	{
		const b3ContactHitEvent* hit = events.hitEvents + i;
		if ( SameContactShapePair( hit->shapeIdA, hit->shapeIdB, wallShapeId, slugShapeId ) == false )
		{
			continue;
		}
		matchCount += 1;
		uint64_t wallMaterialId = B3_ID_EQUALS( hit->shapeIdA, wallShapeId ) ? hit->userMaterialIdA : hit->userMaterialIdB;
		uint64_t slugMaterialId = B3_ID_EQUALS( hit->shapeIdA, wallShapeId ) ? hit->userMaterialIdB : hit->userMaterialIdA;
		V3_CONTACTS_ENSURE( wallMaterialId == wallUserMaterialBase + 1 );
		V3_CONTACTS_ENSURE( slugMaterialId == slugUserMaterialId );
	}
	V3_CONTACTS_ENSURE( matchCount == 1 );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3BlockGrid_Release( wall );
	v3BlockGrid_Release( slug );
	return status;
}

static v3BlockGridData* CreateCounterGrid( void )
{
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridBlockDef block = { .sourceId = 1 };
	v3BlockGridBoxDef box = {
		.center = { 0.5f, 0.5f, 0.5f },
		.halfExtent = { 0.5f, 0.5f, 0.5f },
		.ownerBlockIndex = 0,
		.sourceId = 2,
	};
	v3BlockGridDef def = {
		.materials = &material,
		.materialCount = 1,
		.blocks = &block,
		.blockCount = 1,
		.boxes = &box,
		.boxCount = 1,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus createStatus;
	return v3BlockGrid_Create( &def, &createStatus );
}

static b3ShapeId AttachCounterGrid( b3BodyId bodyId, v3BlockGridData* grid )
{
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	return v3CreateBlockGridShape( bodyId, &shapeDef, grid );
}

// The published counters report the pairs of the latest step and no pair storage outlives its body.
static int V3BlockGridPairCountersBehavior( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId bodyIdA = b3_nullBodyId;
	v3BlockGridData* gridA = NULL;
	v3BlockGridData* gridB = NULL;

	v3BlockGridPairCounters invalid = v3World_GetBlockGridPairCounters( b3_nullWorldId );
	V3_CONTACTS_ENSURE( invalid.candidateHitboxPairCount == 0 );
	V3_CONTACTS_ENSURE( invalid.touchingPairCount == 0 );
	V3_CONTACTS_ENSURE( invalid.contactCount == 0 );
	V3_CONTACTS_ENSURE( invalid.projectileSweepCount == 0 );
	V3_CONTACTS_ENSURE( invalid.capExhaustionCount == 0 );
	V3_CONTACTS_ENSURE( invalid.replacementPublishedCount == 0 );
	V3_CONTACTS_ENSURE( invalid.scratchPeakBytes == 0 );

	gridA = CreateCounterGrid();
	gridB = CreateCounterGrid();
	V3_CONTACTS_ENSURE( gridA != NULL );
	V3_CONTACTS_ENSURE( gridB != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	worldDef.enableSleep = false;
	worldId = b3CreateWorld( &worldDef );
	V3_CONTACTS_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	bodyDef.gravityScale = 0.0f;
	bodyIdA = b3CreateBody( worldId, &bodyDef );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( bodyIdA ) );

	bodyDef.position = (b3Pos){ 1.0f, 0.0f, 0.0f };
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( bodyIdB ) );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( AttachCounterGrid( bodyIdA, gridA ) ) );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( AttachCounterGrid( bodyIdB, gridB ) ) );
	v3BlockGrid_Release( gridA );
	gridA = NULL;
	v3BlockGrid_Release( gridB );
	gridB = NULL;

	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	v3BlockGridPairCounters afterStep = v3World_GetBlockGridPairCounters( worldId );
	V3_CONTACTS_ENSURE( afterStep.contactCount == 1 );
	V3_CONTACTS_ENSURE( afterStep.candidateHitboxPairCount == 1 );
	V3_CONTACTS_ENSURE( afterStep.touchingPairCount == 1 );
	V3_CONTACTS_ENSURE( afterStep.scratchPeakBytes > 0 );

	// This fixture performs no projectile sweeps or grid replacements, so these counts are zero.
	V3_CONTACTS_ENSURE( afterStep.projectileSweepCount == 0 );
	V3_CONTACTS_ENSURE( afterStep.capExhaustionCount == 0 );
	V3_CONTACTS_ENSURE( afterStep.replacementPublishedCount == 0 );

	// A resting pair rebuilds its manifold from the current poses and reports the same work
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	v3BlockGridPairCounters afterRestingStep = v3World_GetBlockGridPairCounters( worldId );
	V3_CONTACTS_ENSURE( afterRestingStep.contactCount == 1 );
	V3_CONTACTS_ENSURE( afterRestingStep.candidateHitboxPairCount == afterStep.candidateHitboxPairCount );
	V3_CONTACTS_ENSURE( afterRestingStep.touchingPairCount == afterStep.touchingPairCount );
	V3_CONTACTS_ENSURE( afterRestingStep.scratchPeakBytes == afterStep.scratchPeakBytes );

	// Counters describe the latest completed step, so destroying a body does not move them
	b3DestroyBody( bodyIdA );
	bodyIdA = b3_nullBodyId;
	v3BlockGridPairCounters afterDestroy = v3World_GetBlockGridPairCounters( worldId );
	V3_CONTACTS_ENSURE( afterDestroy.contactCount == afterRestingStep.contactCount );
	V3_CONTACTS_ENSURE( afterDestroy.candidateHitboxPairCount == afterRestingStep.candidateHitboxPairCount );
	V3_CONTACTS_ENSURE( afterDestroy.touchingPairCount == afterRestingStep.touchingPairCount );
	V3_CONTACTS_ENSURE( afterDestroy.scratchPeakBytes == afterRestingStep.scratchPeakBytes );

	status = 0;
cleanup:
	v3BlockGrid_Release( gridA );
	v3BlockGrid_Release( gridB );
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	return status;
}

// Cooked geometry reduction: cook, attach, step, and compare statistics and resulting poses.
// Expected values are derived from the fixture definitions in the comments below.

// One cell of a reduction fixture: its coordinate, its material, and whether it is a full cube or
// the two-halves control that reduction must leave alone.
typedef struct ReductionCell
{
	int x, y, z;
	uint32_t materialIndex;
} ReductionCell;

enum
{
	// The control splits each cell into two boxes stacked in y. Their union is exactly the unit
	// cube, so the solid is unchanged, but the cell no longer holds one box filling it and
	// reduction has to leave every box where it is. This is the unmerged cook the exactness cases
	// compare against, expressed through the public contract alone.
	reductionBoxesPerControlCell = 2,
};

static v3BlockGridData* CookReductionGrid( const ReductionCell* cells, int cellCount, int materialCount, bool control,
										   v3BlockGridCookStats* stats )
{
	int boxesPerCell = control ? reductionBoxesPerControlCell : 1;
	b3SurfaceMaterial* materials = calloc( (size_t)materialCount, sizeof( b3SurfaceMaterial ) );
	v3BlockGridBlock* blocks = calloc( (size_t)cellCount, sizeof( v3BlockGridBlock ) );
	v3BlockGridBox* boxes = calloc( (size_t)cellCount * boxesPerCell, sizeof( v3BlockGridBox ) );
	if ( materials == NULL || blocks == NULL || boxes == NULL )
	{
		free( materials );
		free( blocks );
		free( boxes );
		return NULL;
	}

	// Every material carries the same surface values, so two cells differing only in material
	// index are the same physics and any behavior difference is reduction, not friction.
	for ( int i = 0; i < materialCount; ++i )
	{
		materials[i] = b3DefaultSurfaceMaterial();
		materials[i].friction = 0.8f;
		materials[i].userMaterialId = (uint64_t)i + 1;
	}

	for ( int i = 0; i < cellCount; ++i )
	{
		v3BlockGridBox* cellBoxes = boxes + (size_t)i * boxesPerCell;
		if ( control )
		{
			cellBoxes[0] = (v3BlockGridBox){ { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.5f, 1.0f } }, cells[i].materialIndex };
			cellBoxes[1] = (v3BlockGridBox){ { { 0.0f, 0.5f, 0.0f }, { 1.0f, 1.0f, 1.0f } }, cells[i].materialIndex };
		}
		else
		{
			cellBoxes[0] = (v3BlockGridBox){ { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } }, cells[i].materialIndex };
		}
		blocks[i] = (v3BlockGridBlock){
			.x = cells[i].x,
			.y = cells[i].y,
			.z = cells[i].z,
			// Caller data follows the cell, not the array slot, so two permutations of one field carry
			// the same caller data per cell and any difference left in the cooked bytes is the cooker's.
			.userData = ( ( (uint64_t)cells[i].x * 4096 + (uint64_t)cells[i].y ) * 4096 + (uint64_t)cells[i].z ) + 1,
			.boxes = cellBoxes,
			.boxCount = boxesPerCell,
		};
	}

	v3BlockGridCookDef definition = {
		.materials = materials,
		.materialCount = materialCount,
		.blocks = blocks,
		.blockCount = cellCount,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &definition );
	free( materials );
	free( blocks );
	free( boxes );
	if ( result.status != v3_blockGridCookOk )
	{
		return NULL;
	}
	if ( stats != NULL )
	{
		*stats = result.stats;
	}
	return result.data;
}

// Fills cells with the boundary of a box, the hollow hull shape every alpha-scale scene uses.
static int FillHullShell( ReductionCell* cells, int sizeX, int sizeY, int sizeZ )
{
	int count = 0;
	for ( int x = 0; x < sizeX; ++x )
	{
		for ( int y = 0; y < sizeY; ++y )
		{
			for ( int z = 0; z < sizeZ; ++z )
			{
				if ( x > 0 && x < sizeX - 1 && y > 0 && y < sizeY - 1 && z > 0 && z < sizeZ - 1 )
				{
					continue;
				}
				cells[count++] = (ReductionCell){ x, y, z, 0 };
			}
		}
	}
	return count;
}

// Casts one ray at a cooked grid through the public world query and returns what it reports. Two
// cooks of one field have to answer the same query the same way, whatever order they were listed
// in and however many cells share a Hitbox.
static bool CastRayAtGrid( v3BlockGridData* grid, b3Pos origin, b3Vec3 translation, float* fraction, b3Vec3* normal )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	if ( B3_IS_NULL( worldId ) )
	{
		return false;
	}
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	bool ok = B3_IS_NON_NULL( v3CreateBlockGridShape( bodyId, &shapeDef, grid ) );
	b3RayResult hit = b3World_CastRayClosest( worldId, origin, translation, b3DefaultQueryFilter() );
	*fraction = hit.fraction;
	*normal = hit.normal;
	ok = ok && hit.hit;
	b3DestroyWorld( worldId );
	return ok;
}

// Drops one small dynamic BlockGrid onto the fixture and returns where it comes to rest. Two cooks
// of the same solid have to answer this the same way, which is what merge exactness means.
static bool SettleProbeOnGrid( v3BlockGridData* grid, b3Pos gridPosition, b3Pos probePosition, int steps, b3Pos* restPosition,
							   b3Vec3* restVelocity )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	if ( B3_IS_NULL( worldId ) )
	{
		return false;
	}

	b3BodyDef staticDef = b3DefaultBodyDef();
	staticDef.position = gridPosition;
	b3BodyId staticBody = b3CreateBody( worldId, &staticDef );
	b3ShapeDef staticShape = b3DefaultShapeDef();
	staticShape.baseMaterial.friction = 0.8f;
	bool ok = B3_IS_NON_NULL( v3CreateBlockGridShape( staticBody, &staticShape, grid ) );

	ReductionCell probeCell = { 0, 0, 0, 0 };
	v3BlockGridData* probe = CookReductionGrid( &probeCell, 1, 1, false, NULL );
	if ( probe == NULL )
	{
		b3DestroyWorld( worldId );
		return false;
	}
	b3BodyDef probeDef = b3DefaultBodyDef();
	probeDef.type = b3_dynamicBody;
	probeDef.position = probePosition;
	probeDef.enableSleep = false;
	b3BodyId probeBody = b3CreateBody( worldId, &probeDef );
	b3ShapeDef probeShape = b3DefaultShapeDef();
	probeShape.density = 1.0f;
	probeShape.baseMaterial.friction = 0.8f;
	ok = ok && B3_IS_NON_NULL( v3CreateBlockGridShape( probeBody, &probeShape, probe ) );
	v3BlockGrid_Release( probe );

	for ( int step = 0; step < steps; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	*restPosition = b3Body_GetPosition( probeBody );
	*restVelocity = b3Body_GetLinearVelocity( probeBody );
	b3DestroyWorld( worldId );
	return ok;
}

// Cooked geometry is a function of the field, never of the order the caller listed it in.
//
// Derivation: cooking sorts blocks by cell coordinate and derives every identity, every index and
// every merged run from that order, so two permutations of one hull cannot differ in a single
// cooked byte. The comparison is therefore exact, not toleranced, on the payload, on a cast and
// on a settled pose.
static int V3BlockGridShuffledDefinitionCooksIdentically( void )
{
	int status = 1;
	v3BlockGridData* ordered = NULL;
	v3BlockGridData* shuffled = NULL;
	ReductionCell* cells = calloc( 36 * 12 * 20, sizeof( ReductionCell ) );
	ReductionCell* permuted = calloc( 36 * 12 * 20, sizeof( ReductionCell ) );
	V3_CONTACTS_ENSURE( cells != NULL && permuted != NULL );

	// The alpha-scale hull: a 36 x 12 x 20 shell is 2 * ( 36*12 + 36*20 + 12*20 ) minus the
	// double-counted edges, which the fill computes directly as 2520 boundary cells.
	int cellCount = FillHullShell( cells, 36, 12, 20 );
	V3_CONTACTS_ENSURE( cellCount == 2520 );

	// A fixed permutation, not a random one: stepping by a stride coprime with the count visits
	// every cell exactly once, so the second definition is the same field in a different order and
	// the test repeats bit for bit on every machine.
	for ( int i = 0; i < cellCount; ++i )
	{
		permuted[i] = cells[( (int64_t)i * 1013 + 7 ) % cellCount];
	}

	v3BlockGridCookStats orderedStats = { 0 };
	v3BlockGridCookStats shuffledStats = { 0 };
	ordered = CookReductionGrid( cells, cellCount, 1, false, &orderedStats );
	shuffled = CookReductionGrid( permuted, cellCount, 1, false, &shuffledStats );
	V3_CONTACTS_ENSURE( ordered != NULL && shuffled != NULL );

	V3_CONTACTS_ENSURE( orderedStats.byteCount == shuffledStats.byteCount );
	V3_CONTACTS_ENSURE( orderedStats.blockCount == shuffledStats.blockCount );
	V3_CONTACTS_ENSURE( orderedStats.boxCount == shuffledStats.boxCount );
	V3_CONTACTS_ENSURE( orderedStats.inputBoxCount == shuffledStats.inputBoxCount );
	V3_CONTACTS_ENSURE( orderedStats.culledBoxCount == shuffledStats.culledBoxCount );
	V3_CONTACTS_ENSURE( orderedStats.mergedBoxCount == shuffledStats.mergedBoxCount );

	// Compare the entire cooked allocation before attachment or retention. Both mutable headers
	// have one reference and the same cook origin, so they can be compared along with the content.
	V3_CONTACTS_ENSURE( memcmp( ordered, shuffled, (size_t)orderedStats.byteCount ) == 0 );

	// A hull is hollow, so the shell keeps every cell: nothing is enclosed and the input is the
	// 2520 Cell Boxes the definition listed.
	V3_CONTACTS_ENSURE( orderedStats.inputBoxCount == 2520 );
	V3_CONTACTS_ENSURE( orderedStats.culledBoxCount == 0 );
	V3_CONTACTS_ENSURE( orderedStats.boxCount + orderedStats.culledBoxCount + orderedStats.mergedBoxCount == 2520 );

	// Same solid, same answers: one ray at each cook, same origin and direction, has to report the
	// same entry fraction and the same face. The hull top face sits at y = 12 over the whole footprint,
	// so a ray fired straight down from y = 20 at ( 4, 4 ) travels 8 of its 20 units before it lands
	// on that face and reports 0.4 with an up normal.
	float orderedFraction = -1.0f;
	float shuffledFraction = -1.0f;
	b3Vec3 orderedNormal = b3Vec3_zero;
	b3Vec3 shuffledNormal = b3Vec3_zero;
	V3_CONTACTS_ENSURE(
		CastRayAtGrid( ordered, (b3Pos){ 4.5, 20.0, 4.5 }, (b3Vec3){ 0.0f, -20.0f, 0.0f }, &orderedFraction, &orderedNormal ) );
	V3_CONTACTS_ENSURE( CastRayAtGrid( shuffled, (b3Pos){ 4.5, 20.0, 4.5 }, (b3Vec3){ 0.0f, -20.0f, 0.0f }, &shuffledFraction,
									   &shuffledNormal ) );
	V3_CONTACTS_ENSURE( orderedFraction == shuffledFraction );
	V3_CONTACTS_ENSURE( orderedNormal.x == shuffledNormal.x && orderedNormal.y == shuffledNormal.y &&
						orderedNormal.z == shuffledNormal.z );
	V3_CONTACTS_ENSURE( fabsf( orderedFraction - 0.4f ) <= 1e-6f );
	V3_CONTACTS_ENSURE( orderedNormal.y > 0.9f );

	// Same solid, same behavior: a probe dropped on either cook settles in the same place.
	b3Pos orderedRest, shuffledRest;
	b3Vec3 orderedVelocity, shuffledVelocity;
	V3_CONTACTS_ENSURE( SettleProbeOnGrid( ordered, b3Pos_zero, (b3Pos){ 4.0, 13.0, 4.0 }, 90, &orderedRest, &orderedVelocity ) );
	V3_CONTACTS_ENSURE(
		SettleProbeOnGrid( shuffled, b3Pos_zero, (b3Pos){ 4.0, 13.0, 4.0 }, 90, &shuffledRest, &shuffledVelocity ) );
	V3_CONTACTS_ENSURE( orderedRest.x == shuffledRest.x && orderedRest.y == shuffledRest.y && orderedRest.z == shuffledRest.z );
	V3_CONTACTS_ENSURE( orderedVelocity.x == shuffledVelocity.x && orderedVelocity.y == shuffledVelocity.y &&
						orderedVelocity.z == shuffledVelocity.z );

	status = 0;
cleanup:
	v3BlockGrid_Release( ordered );
	v3BlockGrid_Release( shuffled );
	free( cells );
	free( permuted );
	return status;
}

// A cell nothing can reach carries no collision geometry, and mass never notices.
//
// Derivation on a solid 3 x 3 x 3: only ( 1, 1, 1 ) has all six face neighbours present, so
// exactly one of the 27 Cell Boxes is culled and 26 survive to be merged. Mass comes from the 27
// logical cells and one density either way, so the merged cook and the two-halves control, which
// reduction cannot touch because neither of its boxes fills its cell, must report the same mass,
// the same centre of mass and the same inertia to the last bit.
static int V3BlockGridEnclosedCellsProduceNoHitbox( void )
{
	int status = 1;
	v3BlockGridData* merged = NULL;
	v3BlockGridData* control = NULL;
	b3WorldId worldId = b3_nullWorldId;

	ReductionCell cells[27];
	int cellCount = 0;
	for ( int x = 0; x < 3; ++x )
	{
		for ( int y = 0; y < 3; ++y )
		{
			for ( int z = 0; z < 3; ++z )
			{
				cells[cellCount++] = (ReductionCell){ x, y, z, 0 };
			}
		}
	}
	V3_CONTACTS_ENSURE( cellCount == 27 );

	v3BlockGridCookStats mergedStats = { 0 };
	v3BlockGridCookStats controlStats = { 0 };
	merged = CookReductionGrid( cells, cellCount, 1, false, &mergedStats );
	control = CookReductionGrid( cells, cellCount, 1, true, &controlStats );
	V3_CONTACTS_ENSURE( merged != NULL && control != NULL );

	V3_CONTACTS_ENSURE( mergedStats.inputBoxCount == 27 );
	V3_CONTACTS_ENSURE( mergedStats.culledBoxCount == 1 );
	V3_CONTACTS_ENSURE( mergedStats.boxCount + mergedStats.culledBoxCount + mergedStats.mergedBoxCount == 27 );

	// The control's cells hold two half boxes each, so no cell is a full cube: nothing is enclosed,
	// nothing merges, and all 54 boxes survive as their own collision geometry.
	V3_CONTACTS_ENSURE( controlStats.inputBoxCount == 27 * reductionBoxesPerControlCell );
	V3_CONTACTS_ENSURE( controlStats.culledBoxCount == 0 );
	V3_CONTACTS_ENSURE( controlStats.mergedBoxCount == 0 );
	V3_CONTACTS_ENSURE( controlStats.boxCount == 27 * reductionBoxesPerControlCell );

	// Both cooks describe 27 logical cells, so both weigh the same and spin the same.
	V3_CONTACTS_ENSURE( mergedStats.blockCount == 27 && controlStats.blockCount == 27 );
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldId = b3CreateWorld( &worldDef );
	V3_CONTACTS_ENSURE( b3World_IsValid( worldId ) );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId mergedBody = b3CreateBody( worldId, &bodyDef );
	b3BodyId controlBody = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 3.0f;
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( mergedBody, &shapeDef, merged ) ) );
	V3_CONTACTS_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( controlBody, &shapeDef, control ) ) );

	b3MassData mergedMass = b3Body_GetMassData( mergedBody );
	b3MassData controlMass = b3Body_GetMassData( controlBody );
	V3_CONTACTS_ENSURE( mergedMass.mass == controlMass.mass );
	V3_CONTACTS_ENSURE( mergedMass.center.x == controlMass.center.x && mergedMass.center.y == controlMass.center.y &&
						mergedMass.center.z == controlMass.center.z );
	V3_CONTACTS_ENSURE( memcmp( &mergedMass.inertia, &controlMass.inertia, sizeof( mergedMass.inertia ) ) == 0 );

	// 27 unit cells of density 3 weigh 81, and a symmetric cube's centre of mass is its middle.
	V3_CONTACTS_ENSURE( mergedMass.mass == 81.0f );
	V3_CONTACTS_ENSURE( mergedMass.center.x == 1.5f && mergedMass.center.y == 1.5f && mergedMass.center.z == 1.5f );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3BlockGrid_Release( merged );
	v3BlockGrid_Release( control );
	return status;
}

// Full cubes collapse into slabs along one axis then the next, capped, split by material, and the
// solid they describe does not move.
//
// Derivation on a 40 x 1 x 3 plate of one material. Cells are visited in cooked block order, which
// is x then y then z, and the run that starts at ( 0, 0, 0 ) grows along x to the 32-cell cap,
// then fails to take a row along y because the plate is one cell thick, then takes both remaining
// layers along z. That is a 32 x 1 x 3 slab of 96 cells. The next unclaimed cell is ( 32, 0, 0 ),
// which grows the same way over the 8 columns that are left: an 8 x 1 x 3 slab of 24 cells. So
// 120 Cell Boxes become 2 Hitboxes, 118 of them merged away, and their covered cell ranges are
// x 0..31 and x 32..39, both over y 0..0 and z 0..2.
static int V3BlockGridFullCubesMergeIntoSlabs( void )
{
	int status = 1;
	v3BlockGridData* plate = NULL;
	v3BlockGridData* control = NULL;
	v3BlockGridData* striped = NULL;
	ReductionCell* cells = calloc( 40 * 3, sizeof( ReductionCell ) );
	V3_CONTACTS_ENSURE( cells != NULL );

	int cellCount = 0;
	for ( int x = 0; x < 40; ++x )
	{
		for ( int z = 0; z < 3; ++z )
		{
			cells[cellCount++] = (ReductionCell){ x, 0, z, 0 };
		}
	}
	V3_CONTACTS_ENSURE( cellCount == 120 );

	v3BlockGridCookStats plateStats = { 0 };
	plate = CookReductionGrid( cells, cellCount, 1, false, &plateStats );
	V3_CONTACTS_ENSURE( plate != NULL );
	V3_CONTACTS_ENSURE( plateStats.inputBoxCount == 120 );
	V3_CONTACTS_ENSURE( plateStats.culledBoxCount == 0 );
	V3_CONTACTS_ENSURE( plateStats.mergedBoxCount == 118 );
	V3_CONTACTS_ENSURE( plateStats.boxCount == 2 );

	// The covered cell range names the run, which is the only way a hit can still resolve to a
	// cell once thirty-two of them share one box. Hitboxes are cooked in stable identity order,
	// and identity follows the cooked block order, so the run starting at ( 0, 0, 0 ) is first.
	int firstMin[3] = { -1, -1, -1 };
	int firstMax[3] = { -1, -1, -1 };
	int secondMin[3] = { -1, -1, -1 };
	int secondMax[3] = { -1, -1, -1 };
	v3BlockGrid_GetHitboxCellRange( plate, 0, firstMin, firstMax );
	v3BlockGrid_GetHitboxCellRange( plate, 1, secondMin, secondMax );
	V3_CONTACTS_ENSURE( firstMin[0] == 0 && firstMax[0] == 31 );
	V3_CONTACTS_ENSURE( firstMin[1] == 0 && firstMax[1] == 0 );
	V3_CONTACTS_ENSURE( firstMin[2] == 0 && firstMax[2] == 2 );
	V3_CONTACTS_ENSURE( secondMin[0] == 32 && secondMax[0] == 39 );
	V3_CONTACTS_ENSURE( secondMin[1] == 0 && secondMax[1] == 0 );
	V3_CONTACTS_ENSURE( secondMin[2] == 0 && secondMax[2] == 2 );

	// Two materials of identical surface values still cannot merge across their boundary, because
	// a merged Hitbox carries one material and a Hitbox is what names the surface a contact uses.
	// Cells 0..19 carry material 0 and 20..39 material 1, so the run from ( 0, 0, 0 ) stops at the
	// material change after 20 columns rather than at the 32-cell cap, and the run from ( 20, 0, 0 )
	// takes the other 20. Two Hitboxes again, but covering x 0..19 and x 20..39 instead of x 0..31
	// and x 32..39, which is what tells the two apart.
	for ( int i = 0; i < cellCount; ++i )
	{
		cells[i].materialIndex = cells[i].x < 20 ? 0u : 1u;
	}
	v3BlockGridCookStats stripedStats = { 0 };
	striped = CookReductionGrid( cells, cellCount, 2, false, &stripedStats );
	V3_CONTACTS_ENSURE( striped != NULL );
	V3_CONTACTS_ENSURE( stripedStats.boxCount == 2 );
	V3_CONTACTS_ENSURE( stripedStats.mergedBoxCount == 118 );
	v3BlockGrid_GetHitboxCellRange( striped, 0, firstMin, firstMax );
	v3BlockGrid_GetHitboxCellRange( striped, 1, secondMin, secondMax );
	V3_CONTACTS_ENSURE( firstMin[0] == 0 && firstMax[0] == 19 );
	V3_CONTACTS_ENSURE( secondMin[0] == 20 && secondMax[0] == 39 );

	// The unmerged control is the same solid built so that reduction cannot touch it: two half
	// boxes per cell, 240 of them, none of which fills a cell. A probe dropped on the plate has to
	// settle in the same place on both, inside one linear slop.
	for ( int i = 0; i < cellCount; ++i )
	{
		cells[i].materialIndex = 0;
	}
	v3BlockGridCookStats controlStats = { 0 };
	control = CookReductionGrid( cells, cellCount, 1, true, &controlStats );
	V3_CONTACTS_ENSURE( control != NULL );
	V3_CONTACTS_ENSURE( controlStats.boxCount == 240 && controlStats.mergedBoxCount == 0 );

	b3Pos mergedRest, controlRest;
	b3Vec3 mergedVelocity, controlVelocity;
	V3_CONTACTS_ENSURE( SettleProbeOnGrid( plate, b3Pos_zero, (b3Pos){ 31.0, 1.6, 1.0 }, 120, &mergedRest, &mergedVelocity ) );
	V3_CONTACTS_ENSURE(
		SettleProbeOnGrid( control, b3Pos_zero, (b3Pos){ 31.0, 1.6, 1.0 }, 120, &controlRest, &controlVelocity ) );
	V3_CONTACTS_ENSURE( fabs( mergedRest.x - controlRest.x ) <= B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabs( mergedRest.y - controlRest.y ) <= B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabs( mergedRest.z - controlRest.z ) <= B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabsf( mergedVelocity.x - controlVelocity.x ) <= B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabsf( mergedVelocity.y - controlVelocity.y ) <= B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabsf( mergedVelocity.z - controlVelocity.z ) <= B3_LINEAR_SLOP );

	// The probe rests on the plate's top face, one cell up, not inside it.
	V3_CONTACTS_ENSURE( mergedRest.y >= 1.0 - B3_LINEAR_SLOP && mergedRest.y <= 1.0 + B3_LINEAR_SLOP );

	status = 0;
cleanup:
	v3BlockGrid_Release( plate );
	v3BlockGrid_Release( control );
	v3BlockGrid_Release( striped );
	free( cells );
	return status;
}

// Two overlapping boxes in one cell stay two boxes on one logical collision source.
//
// Derivation on three cells in a row, where the middle one carries a unit cube and a smaller box
// inside it. A cell is only a merge candidate when its whole box set is exactly the unit cube, and
// the middle cell's is not: it holds two boxes, so reduction leaves both alone and cannot fold the
// cell into a run. The outer two cells are full cubes but are not adjacent to each other, so each
// becomes its own single-cell run. Four Cell Boxes in, four Hitboxes out, nothing culled and
// nothing merged. Identity comes from the cooked block order, so the middle cell's two boxes are
// Hitboxes 1 and 2 and both name block 1, and both report the one cell they sit in.
static int V3BlockGridOverlappingBoxesStayOneSource( void )
{
	int status = 1;
	v3BlockGridData* grid = NULL;

	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	const v3BlockGridBox unitCube = { { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } }, 0 };
	const v3BlockGridBox innerBox = { { { 0.25f, 0.25f, 0.25f }, { 0.75f, 0.75f, 0.75f } }, 0 };
	v3BlockGridBox overlappingPair[2] = { unitCube, innerBox };
	v3BlockGridBlock blocks[3] = {
		{ .x = 0, .userData = 1, .boxes = &unitCube, .boxCount = 1 },
		{ .x = 1, .userData = 2, .boxes = overlappingPair, .boxCount = 2 },
		{ .x = 2, .userData = 3, .boxes = &unitCube, .boxCount = 1 },
	};
	v3BlockGridCookDef definition = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = 3 };
	v3BlockGridCookResult cooked = v3CookBlockGrid( &definition );
	V3_CONTACTS_ENSURE( cooked.status == v3_blockGridCookOk );
	grid = cooked.data;

	V3_CONTACTS_ENSURE( cooked.stats.inputBoxCount == 4 );
	V3_CONTACTS_ENSURE( cooked.stats.culledBoxCount == 0 );
	V3_CONTACTS_ENSURE( cooked.stats.mergedBoxCount == 0 );
	V3_CONTACTS_ENSURE( cooked.stats.boxCount == 4 );

	// Both of the middle cell's boxes survive, and both still name that one cell.
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxOwnerBlock( grid, 0 ) == 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxOwnerBlock( grid, 1 ) == 1 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxOwnerBlock( grid, 2 ) == 1 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxOwnerBlock( grid, 3 ) == 2 );
	for ( int hitboxIndex = 1; hitboxIndex <= 2; ++hitboxIndex )
	{
		int cellMin[3] = { -1, -1, -1 };
		int cellMax[3] = { -1, -1, -1 };
		v3BlockGrid_GetHitboxCellRange( grid, hitboxIndex, cellMin, cellMax );
		V3_CONTACTS_ENSURE( cellMin[0] == 1 && cellMax[0] == 1 );
		V3_CONTACTS_ENSURE( cellMin[1] == 0 && cellMax[1] == 0 );
		V3_CONTACTS_ENSURE( cellMin[2] == 0 && cellMax[2] == 0 );
	}

	// Neither box was resized: the cube keeps its half cell and the inner box its quarter.
	b3Vec3 cubeCenter, cubeExtent, innerCenter, innerExtent;
	v3BlockGrid_GetHitboxBounds( grid, 1, &cubeCenter, &cubeExtent );
	v3BlockGrid_GetHitboxBounds( grid, 2, &innerCenter, &innerExtent );
	V3_CONTACTS_ENSURE( cubeExtent.x == 0.5f && cubeExtent.y == 0.5f && cubeExtent.z == 0.5f );
	V3_CONTACTS_ENSURE( innerExtent.x == 0.25f && innerExtent.y == 0.25f && innerExtent.z == 0.25f );
	V3_CONTACTS_ENSURE( cubeCenter.x == innerCenter.x && cubeCenter.y == innerCenter.y && cubeCenter.z == innerCenter.z );

	status = 0;
cleanup:
	v3BlockGrid_Release( grid );
	return status;
}
// ---------------------------------------------------------------------------------------------
// Buried-face coverage: which Hitbox faces the cook proves buried, and the contacts that follow.
// Face bit b is 2 * axis + positive, so bit 0 is -x, bit 1 is +x, bit 2 is -y, bit 3 is +y, bit 4
// is -z and bit 5 is +z. Every mask below is derived from the cell field alone and printed before
// it is asserted, so a run records what each fixture culls even when an expectation fails.
// ---------------------------------------------------------------------------------------------

enum
{
	buriedFaceMinusX = 1u << 0,
	buriedFacePlusX = 1u << 1,
	buriedFaceMinusY = 1u << 2,
	buriedFacePlusY = 1u << 3,
	buriedFaceMinusZ = 1u << 4,
	buriedFacePlusZ = 1u << 5,
};

// The Hitbox covering exactly this cell range, or -1. Runs are ordered by identity, and identity
// follows the cooked block order, so naming a run by the cells it covers is the one address that
// does not move when a fixture gains a cell.
static int FindHitboxByCellRange( v3BlockGridData* grid, int minX, int minY, int minZ, int maxX, int maxY, int maxZ )
{
	int count = v3BlockGrid_GetHitboxCount( grid );
	for ( int i = 0; i < count; ++i )
	{
		int cellMin[3] = { -1, -1, -1 };
		int cellMax[3] = { -1, -1, -1 };
		v3BlockGrid_GetHitboxCellRange( grid, i, cellMin, cellMax );
		if ( cellMin[0] == minX && cellMin[1] == minY && cellMin[2] == minZ && cellMax[0] == maxX && cellMax[1] == maxY &&
			 cellMax[2] == maxZ )
		{
			return i;
		}
	}
	return -1;
}

static void PrintHitboxFaceMasks( const char* label, v3BlockGridData* grid )
{
	int count = v3BlockGrid_GetHitboxCount( grid );
	printf( "buried-face fixture %s: %d hitboxes\n", label, count );
	for ( int i = 0; i < count; ++i )
	{
		int cellMin[3] = { -1, -1, -1 };
		int cellMax[3] = { -1, -1, -1 };
		v3BlockGrid_GetHitboxCellRange( grid, i, cellMin, cellMax );
		printf( "  hitbox %d cells x %d..%d y %d..%d z %d..%d mask 0x%02x\n", i, cellMin[0], cellMax[0], cellMin[1], cellMax[1],
				cellMin[2], cellMax[2], (unsigned)v3BlockGrid_GetHitboxFaceMask( grid, i ) );
	}
}

typedef struct SeamProbeResult
{
	b3Pos position;
	b3Vec3 velocity;
	int pointCount;
	int manifoldCount;
	int peakLateralManifoldCount;
	int peakPointCount;
} SeamProbeResult;

// Drives one unit-cube probe grid over a fixture and records both where it ends up and every
// manifold it ever built. A manifold whose normal is not within 30 degrees of vertical is a
// lateral one, which on a flat support can only come from a seam face between two Hitboxes.
static bool RunSeamProbe( v3BlockGridData* grid, b3Pos probePosition, b3Vec3 probeVelocity, b3Vec3 gravity, int steps,
						  SeamProbeResult* result )
{
	*result = (SeamProbeResult){ 0 };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = gravity;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	if ( B3_IS_NULL( worldId ) )
	{
		return false;
	}

	b3BodyDef staticDef = b3DefaultBodyDef();
	b3BodyId staticBody = b3CreateBody( worldId, &staticDef );
	b3ShapeDef staticShape = b3DefaultShapeDef();
	staticShape.baseMaterial.friction = 0.8f;
	b3ShapeId staticShapeId = v3CreateBlockGridShape( staticBody, &staticShape, grid );
	bool ok = B3_IS_NON_NULL( staticShapeId );

	ReductionCell probeCell = { 0, 0, 0, 0 };
	v3BlockGridData* probe = CookReductionGrid( &probeCell, 1, 1, false, NULL );
	if ( probe == NULL )
	{
		b3DestroyWorld( worldId );
		return false;
	}
	b3BodyDef probeDef = b3DefaultBodyDef();
	probeDef.type = b3_dynamicBody;
	probeDef.position = probePosition;
	probeDef.linearVelocity = probeVelocity;
	probeDef.enableSleep = false;
	b3BodyId probeBody = b3CreateBody( worldId, &probeDef );
	b3ShapeDef probeShape = b3DefaultShapeDef();
	probeShape.density = 1.0f;
	probeShape.baseMaterial.friction = 0.8f;
	b3ShapeId probeShapeId = v3CreateBlockGridShape( probeBody, &probeShape, probe );
	ok = ok && B3_IS_NON_NULL( probeShapeId );
	v3BlockGrid_Release( probe );

	for ( int step = 0; step < steps; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactData contacts[16];
		int contactCount = b3Shape_GetContactData( probeShapeId, contacts, 16 );
		int lateral = 0;
		int points = 0;
		int manifolds = 0;
		for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
		{
			const b3ContactData* contact = contacts + contactIndex;
			if ( SameContactShapePair( contact->shapeIdA, contact->shapeIdB, staticShapeId, probeShapeId ) == false ||
				 contact->manifolds == NULL )
			{
				continue;
			}
			manifolds += contact->manifoldCount;
			for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
			{
				const b3Manifold* manifold = contact->manifolds + manifoldIndex;
				points += manifold->pointCount;
				lateral += manifold->pointCount > 0 && fabsf( manifold->normal.y ) < 0.866f;
			}
		}
		result->pointCount = points;
		result->manifoldCount = manifolds;
		result->peakPointCount = points > result->peakPointCount ? points : result->peakPointCount;
		result->peakLateralManifoldCount =
			lateral > result->peakLateralManifoldCount ? lateral : result->peakLateralManifoldCount;
	}

	result->position = b3Body_GetPosition( probeBody );
	result->velocity = b3Body_GetLinearVelocity( probeBody );
	b3DestroyWorld( worldId );
	return ok;
}

// Fills a plate of full cubes, one cell thick in y, with an optional short column beyond a step.
static int FillCoveragePlate( ReductionCell* cells, int fullDepth, int shortDepth, bool stripeMaterials )
{
	int count = 0;
	for ( int x = 0; x < 40; ++x )
	{
		int depth = x < 32 ? fullDepth : shortDepth;
		for ( int z = 0; z < depth; ++z )
		{
			uint32_t material = stripeMaterials && x >= 32 ? (uint32_t)( 1 + z ) : 0u;
			cells[count++] = (ReductionCell){ x, 0, z, material };
		}
	}
	return count;
}

// The four coverage cases the deleted per-contact search used to answer, now answered by the cooked
// mask.
//
// Complete: a 40x1x3 plate reduces to runs x 0..31 and x 32..39 (the 32-cell cap). The first run's
// +x face is the 1x3 cell slab at x = 32, every cell of which the second run holds, so it is
// buried and nothing else on either run is. Tiled: give the cells beyond the cap one material per
// z, so the same face is covered by three separate runs instead of one; a face is covered by its
// cells, not by how many Hitboxes hold them, so the first run's mask is unchanged. Partial: drop
// the z = 2 column beyond x = 32 and the face loses one of its three cells, so the conservative
// rule leaves it exposed. Overflow: a 32x1x2 run carrying sixty-four one-cell runs on top has its
// +y face covered by sixty-four Hitboxes, far past the thirty-two the deleted search could hold on
// the stack, and the cell rule culls it anyway.
static int V3BlockGridBuriedFaceMasksAreCooked( void )
{
	int status = 1;
	v3BlockGridData* complete = NULL;
	v3BlockGridData* tiled = NULL;
	v3BlockGridData* partial = NULL;
	v3BlockGridData* overflow = NULL;
	v3BlockGridData* subCell = NULL;
	ReductionCell* cells = calloc( 40 * 3 + 64 * 2, sizeof( ReductionCell ) );
	V3_CONTACTS_ENSURE( cells != NULL );

	int completeCount = FillCoveragePlate( cells, 3, 3, false );
	V3_CONTACTS_ENSURE( completeCount == 120 );
	complete = CookReductionGrid( cells, completeCount, 1, false, NULL );
	V3_CONTACTS_ENSURE( complete != NULL );
	PrintHitboxFaceMasks( "complete", complete );

	int tiledCount = FillCoveragePlate( cells, 3, 3, true );
	V3_CONTACTS_ENSURE( tiledCount == 120 );
	tiled = CookReductionGrid( cells, tiledCount, 4, false, NULL );
	V3_CONTACTS_ENSURE( tiled != NULL );
	PrintHitboxFaceMasks( "tiled", tiled );

	int partialCount = FillCoveragePlate( cells, 3, 2, false );
	V3_CONTACTS_ENSURE( partialCount == 112 );
	partial = CookReductionGrid( cells, partialCount, 1, false, NULL );
	V3_CONTACTS_ENSURE( partial != NULL );
	PrintHitboxFaceMasks( "partial", partial );

	int overflowCount = 0;
	for ( int x = 0; x < 32; ++x )
	{
		for ( int z = 0; z < 2; ++z )
		{
			cells[overflowCount++] = (ReductionCell){ x, 0, z, 0 };
			cells[overflowCount++] = (ReductionCell){ x, 1, z, (uint32_t)( 1 + z * 32 + x ) };
		}
	}
	V3_CONTACTS_ENSURE( overflowCount == 128 );
	overflow = CookReductionGrid( cells, overflowCount, 65, false, NULL );
	V3_CONTACTS_ENSURE( overflow != NULL );
	printf( "buried-face fixture overflow: %d hitboxes\n", v3BlockGrid_GetHitboxCount( overflow ) );

	// One full cube beside a cell holding two half boxes whose union is exactly the shared face.
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	const v3BlockGridBox unitCube = { { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } }, 0 };
	const v3BlockGridBox halves[2] = {
		{ { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.5f, 1.0f } }, 0 },
		{ { { 0.0f, 0.5f, 0.0f }, { 1.0f, 1.0f, 1.0f } }, 0 },
	};
	v3BlockGridBlock subCellBlocks[2] = {
		{ .x = 0, .userData = 1, .boxes = &unitCube, .boxCount = 1 },
		{ .x = 1, .userData = 2, .boxes = halves, .boxCount = 2 },
	};
	v3BlockGridCookDef subCellDef = { .materials = &material, .materialCount = 1, .blocks = subCellBlocks, .blockCount = 2 };
	v3BlockGridCookResult subCellCooked = v3CookBlockGrid( &subCellDef );
	V3_CONTACTS_ENSURE( subCellCooked.status == v3_blockGridCookOk );
	subCell = subCellCooked.data;
	PrintHitboxFaceMasks( "sub-cell", subCell );

	// Complete coverage. Only the seam faces are buried, and nothing on the plate's outside is.
	int completeFirst = FindHitboxByCellRange( complete, 0, 0, 0, 31, 0, 2 );
	int completeSecond = FindHitboxByCellRange( complete, 32, 0, 0, 39, 0, 2 );
	V3_CONTACTS_ENSURE( completeFirst >= 0 && completeSecond >= 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxCount( complete ) == 2 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( complete, completeFirst ) == buriedFacePlusX );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( complete, completeSecond ) == buriedFaceMinusX );

	// Tiled coverage. The same seam face, held by three runs instead of one, is still buried; each
	// of those runs is buried against the plate and against its own two z neighbours.
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxCount( tiled ) == 4 );
	int tiledFirst = FindHitboxByCellRange( tiled, 0, 0, 0, 31, 0, 2 );
	int tiledStripLow = FindHitboxByCellRange( tiled, 32, 0, 0, 39, 0, 0 );
	int tiledStripMid = FindHitboxByCellRange( tiled, 32, 0, 1, 39, 0, 1 );
	int tiledStripHigh = FindHitboxByCellRange( tiled, 32, 0, 2, 39, 0, 2 );
	V3_CONTACTS_ENSURE( tiledFirst >= 0 && tiledStripLow >= 0 && tiledStripMid >= 0 && tiledStripHigh >= 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( tiled, tiledFirst ) == buriedFacePlusX );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( tiled, tiledStripLow ) == ( buriedFaceMinusX | buriedFacePlusZ ) );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( tiled, tiledStripMid ) ==
						( buriedFaceMinusX | buriedFaceMinusZ | buriedFacePlusZ ) );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( tiled, tiledStripHigh ) == ( buriedFaceMinusX | buriedFaceMinusZ ) );

	// Partial coverage. The step is one cell short, so the long run keeps its whole +x face.
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxCount( partial ) == 2 );
	int partialFirst = FindHitboxByCellRange( partial, 0, 0, 0, 31, 0, 2 );
	int partialSecond = FindHitboxByCellRange( partial, 32, 0, 0, 39, 0, 1 );
	V3_CONTACTS_ENSURE( partialFirst >= 0 && partialSecond >= 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( partial, partialFirst ) == 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( partial, partialSecond ) == buriedFaceMinusX );

	// Overflow coverage. Sixty-five Hitboxes: the 32x1x2 base and sixty-four one-cell runs, each of
	// which the base carries. The base's +y face is covered cell by cell whatever the count.
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxCount( overflow ) == 65 );
	int overflowBase = FindHitboxByCellRange( overflow, 0, 0, 0, 31, 0, 1 );
	int overflowMiddle = FindHitboxByCellRange( overflow, 15, 1, 0, 15, 1, 0 );
	V3_CONTACTS_ENSURE( overflowBase >= 0 && overflowMiddle >= 0 );
	printf( "buried-face fixture overflow: base mask 0x%02x middle mask 0x%02x\n",
			(unsigned)v3BlockGrid_GetHitboxFaceMask( overflow, overflowBase ),
			(unsigned)v3BlockGrid_GetHitboxFaceMask( overflow, overflowMiddle ) );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( overflow, overflowBase ) == buriedFacePlusY );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( overflow, overflowMiddle ) ==
						( buriedFaceMinusX | buriedFacePlusX | buriedFaceMinusY | buriedFacePlusZ ) );

	// Sub-cell coverage. Two half boxes tile the shared face exactly, but a cell holding them is
	// not solid, so the conservative rule keeps the cube's +x face and gives the halves no mask at
	// all.
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxCount( subCell ) == 3 );
	int subCellCube = FindHitboxByCellRange( subCell, 0, 0, 0, 0, 0, 0 );
	V3_CONTACTS_ENSURE( subCellCube >= 0 );
	V3_CONTACTS_ENSURE( v3BlockGrid_GetHitboxFaceMask( subCell, subCellCube ) == 0 );
	for ( int i = 0; i < 3; ++i )
	{
		V3_CONTACTS_ENSURE( i == subCellCube || v3BlockGrid_GetHitboxFaceMask( subCell, i ) == 0 );
	}

	status = 0;
cleanup:
	v3BlockGrid_Release( complete );
	v3BlockGrid_Release( tiled );
	v3BlockGrid_Release( partial );
	v3BlockGrid_Release( overflow );
	v3BlockGrid_Release( subCell );
	free( cells );
	return status;
}

// The three behaviors the mask has to preserve, all on the plate whose seam sits at x = 32.
//
// Flat support, no lateral seams: a probe settling astride the seam touches the two runs' top
// faces only. The seam faces themselves are buried, so no manifold may point sideways; one that
// did would push the probe along the plate for no physical reason.
//
// Conservative buried-face culling: on the partial plate the step at x = 32 is a real exposed
// surface, and a probe driven into it has to stop against it rather than pass through.
//
// Flat motor drag: a probe launched along the plate crosses the seam and coasts to a stop under
// friction alone. Deceleration is mu * g = 8 m/s^2, so 3 m/s carries it 3^2 / 16 = 0.5625 m; a
// seam that caught would stop it short of that and would show a lateral manifold on the way.
static int V3BlockGridBuriedFaceCullingKeepsFlatSupport( void )
{
	int status = 1;
	v3BlockGridData* plate = NULL;
	v3BlockGridData* partial = NULL;
	ReductionCell* cells = calloc( 40 * 3, sizeof( ReductionCell ) );
	V3_CONTACTS_ENSURE( cells != NULL );

	int plateCount = FillCoveragePlate( cells, 3, 3, false );
	V3_CONTACTS_ENSURE( plateCount == 120 );
	plate = CookReductionGrid( cells, plateCount, 1, false, NULL );
	V3_CONTACTS_ENSURE( plate != NULL );

	int partialCount = FillCoveragePlate( cells, 3, 2, false );
	V3_CONTACTS_ENSURE( partialCount == 112 );
	partial = CookReductionGrid( cells, partialCount, 1, false, NULL );
	V3_CONTACTS_ENSURE( partial != NULL );

	const b3Vec3 gravity = { 0.0f, -10.0f, 0.0f };
	const b3Vec3 weightless = { 0.0f, 0.0f, 0.0f };

	SeamProbeResult support;
	V3_CONTACTS_ENSURE( RunSeamProbe( plate, (b3Pos){ 32.0, 1.6, 1.0 }, weightless, gravity, 120, &support ) );
	printf( "buried-face support: rest ( %.6f, %.6f, %.6f ) points %d manifolds %d peak points %d peak lateral %d\n",
			support.position.x, support.position.y, support.position.z, support.pointCount, support.manifoldCount,
			support.peakPointCount, support.peakLateralManifoldCount );
	V3_CONTACTS_ENSURE( support.peakLateralManifoldCount == 0 );
	V3_CONTACTS_ENSURE( support.position.y >= 1.0 - B3_LINEAR_SLOP && support.position.y <= 1.0 + B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabs( support.position.x - 32.0 ) <= B3_LINEAR_SLOP );

	SeamProbeResult step;
	V3_CONTACTS_ENSURE(
		RunSeamProbe( partial, (b3Pos){ 34.5, 0.5, 2.5 }, (b3Vec3){ -2.0f, 0.0f, 0.0f }, weightless, 120, &step ) );
	printf( "buried-face step: rest x %.6f velocity x %.6f points %d peak points %d peak lateral %d\n", step.position.x,
			step.velocity.x, step.pointCount, step.peakPointCount, step.peakLateralManifoldCount );
	V3_CONTACTS_ENSURE( step.position.x >= 32.0 - B3_LINEAR_SLOP && step.position.x <= 32.5 + B3_LINEAR_SLOP );
	V3_CONTACTS_ENSURE( fabsf( step.velocity.x ) <= 0.05f );
	V3_CONTACTS_ENSURE( step.peakLateralManifoldCount > 0 );

	SeamProbeResult drag;
	V3_CONTACTS_ENSURE( RunSeamProbe( plate, (b3Pos){ 30.0, 1.0, 1.0 }, (b3Vec3){ 3.0f, 0.0f, 0.0f }, gravity, 120, &drag ) );
	printf( "buried-face drag: rest x %.6f velocity x %.6f peak lateral %d\n", drag.position.x, drag.velocity.x,
			drag.peakLateralManifoldCount );
	V3_CONTACTS_ENSURE( drag.peakLateralManifoldCount == 0 );
	V3_CONTACTS_ENSURE( fabsf( drag.velocity.x ) <= 0.05f );
	V3_CONTACTS_ENSURE( drag.position.x >= 30.4 && drag.position.x <= 30.7 );

	status = 0;
cleanup:
	v3BlockGrid_Release( plate );
	v3BlockGrid_Release( partial );
	free( cells );
	return status;
}

typedef struct SupportLossGridOptions
{
	int cellCount;
	int materialCount;
	uint64_t userMaterialBase;
	bool fragmented;
	bool reversed;
	bool separatedIslands;
} SupportLossGridOptions;

static v3BlockGridCookResult CookSupportLossGrid( SupportLossGridOptions options )
{
	b3SurfaceMaterial* materials = calloc( (size_t)options.materialCount, sizeof( *materials ) );
	v3BlockGridBlock* blocks = calloc( (size_t)options.cellCount, sizeof( *blocks ) );
	v3BlockGridBox* boxes = calloc( (size_t)options.cellCount, sizeof( *boxes ) );
	if ( materials == NULL || blocks == NULL || boxes == NULL )
	{
		free( materials );
		free( blocks );
		free( boxes );
		return (v3BlockGridCookResult){ .status = v3_blockGridCookOutOfMemory };
	}

	for ( int material = 0; material < options.materialCount; ++material )
	{
		materials[material] = b3DefaultSurfaceMaterial();
		materials[material].friction = 0.8f;
		materials[material].userMaterialId = options.userMaterialBase + (uint64_t)material;
	}
	for ( int slot = 0; slot < options.cellCount; ++slot )
	{
		int cell = options.reversed ? options.cellCount - 1 - slot : slot;
		int islandSize = options.cellCount / 2;
		int island = options.separatedIslands && cell >= islandSize;
		int localCell = island ? cell - islandSize : cell;
		uint32_t materialIndex = (uint32_t)( cell % options.materialCount );
		boxes[slot] = (v3BlockGridBox){
			.bounds = options.fragmented ? (b3AABB){ { 0.2f, 0.0f, 0.2f }, { 0.8f, 1.0f, 0.8f } }
										 : (b3AABB){ { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } },
			.materialIndex = materialIndex,
		};
		blocks[slot] = (v3BlockGridBlock){
			.x = localCell % 65 + 100 * island,
			.z = localCell / 65,
			.userData = (uint64_t)cell + 1,
			.boxes = boxes + slot,
			.boxCount = 1,
		};
	}

	v3BlockGridCookDef definition = {
		.materials = materials,
		.materialCount = options.materialCount,
		.blocks = blocks,
		.blockCount = options.cellCount,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &definition );
	free( materials );
	free( blocks );
	free( boxes );
	return result;
}

typedef struct SupportLossResult
{
	b3Pos position;
	b3Vec3 velocity;
	v3BlockGridPairCounters counters;
	uint64_t contactHash;
	float minimumAnchorX;
	float maximumAnchorX;
	int supportSteps;
	int reductionSteps;
	int warmAllocations;
	int warmFrees;
	bool worldAdvanced;
} SupportLossResult;

static bool supportLossCountWarm;
static int supportLossWarmAllocations;
static int supportLossWarmFrees;
static unsigned supportLossMaterialMask;

static void* SupportLossAlloc( int32_t size, int32_t alignment )
{
	supportLossWarmAllocations += supportLossCountWarm;
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void SupportLossFree( void* memory )
{
	supportLossWarmFrees += supportLossCountWarm;
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static float SupportLossMaterialCallback( float frictionA, uint64_t materialA, float frictionB, uint64_t materialB )
{
	(void)frictionA;
	(void)frictionB;
	if ( materialA == 7001 || materialB == 7001 )
	{
		supportLossMaterialMask |= 1;
	}
	if ( materialA == 7002 || materialB == 7002 )
	{
		supportLossMaterialMask |= 2;
	}
	return 0.8f;
}

static uint64_t HashSupportLossContact( uint64_t hash, const void* data, size_t size )
{
	const unsigned char* bytes = data;
	for ( size_t i = 0; i < size; ++i )
	{
		hash = ( hash ^ bytes[i] ) * UINT64_C( 1099511628211 );
	}
	return hash;
}

static bool ObserveSupportLossContact( const ContactFixture* fixture, SupportLossResult* result )
{
	b3ContactData contacts[4];
	int contactCount = b3Shape_GetContactData( fixture->movingShapeId, contacts, 4 );
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		if ( !SameContactShapePair( contact->shapeIdA, contact->shapeIdB, fixture->fixedShapeId, fixture->movingShapeId ) ||
			 contact->manifolds == NULL )
		{
			continue;
		}
		result->contactHash = HashSupportLossContact( result->contactHash, &contact->manifoldCount, sizeof( contact->manifoldCount ) );
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			const b3Manifold* manifold = contact->manifolds + manifoldIndex;
			result->contactHash = HashSupportLossContact( result->contactHash, &manifold->normal, sizeof( manifold->normal ) );
			result->contactHash = HashSupportLossContact( result->contactHash, &manifold->pointCount, sizeof( manifold->pointCount ) );
			for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
			{
				const b3ManifoldPoint* point = manifold->points + pointIndex;
				result->contactHash = HashSupportLossContact( result->contactHash, &point->anchorA, sizeof( point->anchorA ) );
				result->contactHash = HashSupportLossContact( result->contactHash, &point->anchorB, sizeof( point->anchorB ) );
				result->contactHash = HashSupportLossContact( result->contactHash, &point->separation, sizeof( point->separation ) );
				result->contactHash = HashSupportLossContact( result->contactHash, &point->featureId, sizeof( point->featureId ) );
				result->minimumAnchorX = b3MinFloat( result->minimumAnchorX, b3MinFloat( point->anchorA.x, point->anchorB.x ) );
				result->maximumAnchorX = b3MaxFloat( result->maximumAnchorX, b3MaxFloat( point->anchorA.x, point->anchorB.x ) );
			}
		}
		return contact->manifoldCount > 0;
	}
	return false;
}

static bool CreateSupportLossFixture( SupportLossGridOptions fixedOptions, SupportLossGridOptions movingOptions,
										 ContactFixture* fixture )
{
	*fixture = (ContactFixture){ .worldId = b3_nullWorldId };
	v3BlockGridCookResult fixed = CookSupportLossGrid( fixedOptions );
	v3BlockGridCookResult moving = CookSupportLossGrid( movingOptions );
	if ( fixed.status != v3_blockGridCookOk || moving.status != v3_blockGridCookOk ||
		 ( fixedOptions.fragmented && fixed.stats.boxCount != fixedOptions.cellCount ) )
	{
		v3BlockGrid_Release( fixed.data );
		v3BlockGrid_Release( moving.data );
		return false;
	}

	b3WorldDef worldDefinition = b3DefaultWorldDef();
	worldDefinition.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDefinition.enableSleep = false;
	worldDefinition.workerCount = 1;
	fixture->worldId = b3CreateWorld( &worldDefinition );
	if ( !b3World_IsValid( fixture->worldId ) )
	{
		v3BlockGrid_Release( fixed.data );
		v3BlockGrid_Release( moving.data );
		return false;
	}

	b3BodyDef bodyDefinition = b3DefaultBodyDef();
	b3BodyId fixedBodyId = b3CreateBody( fixture->worldId, &bodyDefinition );
	b3ShapeDef shapeDefinition = b3DefaultShapeDef();
	shapeDefinition.baseMaterial.friction = 0.8f;
	fixture->fixedShapeId = v3CreateBlockGridShape( fixedBodyId, &shapeDefinition, fixed.data );

	bodyDefinition.type = b3_dynamicBody;
	bodyDefinition.position = (b3Pos){ 0.0, 1.0, 0.0 };
	bodyDefinition.enableSleep = false;
	fixture->movingBodyId = b3CreateBody( fixture->worldId, &bodyDefinition );
	shapeDefinition.density = 1.0f;
	fixture->movingShapeId = v3CreateBlockGridShape( fixture->movingBodyId, &shapeDefinition, moving.data );
	v3BlockGrid_Release( fixed.data );
	v3BlockGrid_Release( moving.data );
	return b3Shape_IsValid( fixture->fixedShapeId ) && b3Shape_IsValid( fixture->movingShapeId );
}

static bool RunSupportLossCase( int touchingPairCount, bool reversed, int materialCount, bool separatedIslands,
								bool countWarmAllocations, SupportLossResult* result )
{
	*result = (SupportLossResult){
		.contactHash = UINT64_C( 1469598103934665603 ),
		.minimumAnchorX = FLT_MAX,
		.maximumAnchorX = -FLT_MAX,
	};
	SupportLossGridOptions fixedOptions = {
		.cellCount = touchingPairCount,
		.materialCount = materialCount,
		.userMaterialBase = 7001,
		.fragmented = true,
		.reversed = reversed,
		.separatedIslands = separatedIslands,
	};
	SupportLossGridOptions movingOptions = fixedOptions;
	movingOptions.materialCount = 1;
	movingOptions.userMaterialBase = 8001;
	movingOptions.fragmented = false;
	if ( countWarmAllocations )
	{
		supportLossCountWarm = false;
		supportLossWarmAllocations = 0;
		supportLossWarmFrees = 0;
		b3SetAllocator( SupportLossAlloc, SupportLossFree );
	}
	ContactFixture fixture;
	bool valid = CreateSupportLossFixture( fixedOptions, movingOptions, &fixture );
	b3BodyId controlBodyId = b3_nullBodyId;
	if ( touchingPairCount == 8193 && valid )
	{
		b3BodyDef controlBodyDef = b3DefaultBodyDef();
		controlBodyDef.type = b3_dynamicBody;
		controlBodyDef.position = (b3Pos){ -100.0, 100.0, -100.0 };
		controlBodyDef.gravityScale = 0.0f;
		controlBodyDef.enableSleep = false;
		controlBodyId = b3CreateBody( fixture.worldId, &controlBodyDef );
		valid = B3_IS_NON_NULL( controlBodyId );
		if ( valid )
		{
			b3ShapeDef controlShapeDef = b3DefaultShapeDef();
			controlShapeDef.density = 1.0f;
			b3Sphere controlSphere = { b3Vec3_zero, 0.1f };
			valid = B3_IS_NON_NULL( b3CreateSphereShape( controlBodyId, &controlShapeDef, &controlSphere ) );
			b3Body_SetLinearVelocity( controlBodyId, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
		}
	}
	if ( materialCount > 1 && valid )
	{
		supportLossMaterialMask = 0;
		b3World_SetFrictionCallback( fixture.worldId, SupportLossMaterialCallback );
	}
	for ( int step = 0; step < 12; ++step )
	{
		if ( !valid )
		{
			break;
		}
		b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
		result->supportSteps += ObserveSupportLossContact( &fixture, result );
		result->counters = v3World_GetBlockGridPairCounters( fixture.worldId );
		result->reductionSteps += result->counters.contactReductionCount != 0;
		valid = result->counters.candidateHitboxPairCount == (uint64_t)touchingPairCount &&
				result->counters.touchingPairCount == (uint64_t)touchingPairCount;
		if ( step == 0 && countWarmAllocations )
		{
			supportLossWarmAllocations = 0;
			supportLossWarmFrees = 0;
			supportLossCountWarm = true;
		}
	}
	if ( valid )
	{
		result->position = b3Body_GetPosition( fixture.movingBodyId );
		result->velocity = b3Body_GetLinearVelocity( fixture.movingBodyId );
		if ( B3_IS_NON_NULL( controlBodyId ) )
		{
			b3Pos controlPosition = b3Body_GetPosition( controlBodyId );
			result->worldAdvanced = fabs( controlPosition.x - ( -99.8 ) ) <= 1.0e-6 && controlPosition.y == 100.0 &&
									controlPosition.z == -100.0;
		}
	}
	result->warmAllocations = supportLossWarmAllocations;
	result->warmFrees = supportLossWarmFrees;
	supportLossCountWarm = false;
	DestroyContactFixture( &fixture );
	if ( countWarmAllocations )
	{
		b3SetAllocator( NULL, NULL );
	}
	return valid;
}

static bool SupportLossResultIsSupported( const SupportLossResult* result )
{
	return result->supportSteps == 12 && result->reductionSteps == 12 &&
		   result->position.y >= 1.0 - B3_LINEAR_SLOP && result->position.y <= 1.0 + B3_LINEAR_SLOP &&
		   fabsf( result->velocity.y ) <= 0.05f;
}

// The 4,097th real touching pair must continue into bounded reduction. Losing the contact gives
// the independently derived free-fall pose y = 0.795833333 after these twelve steps.
static int V3BlockGridTouchingPairBatchKeepsSupport( void )
{
	int status = 1;
	SupportLossResult result;
	V3_CONTACTS_ENSURE( RunSupportLossCase( 4097, false, 1, false, false, &result ) );
	printf( "touching-pair batch 4097: y %.9f vy %.9f\n", result.position.y, result.velocity.y );
	V3_CONTACTS_ENSURE( SupportLossResultIsSupported( &result ) );
	status = 0;
cleanup:
	return status;
}

static int V3BlockGridTouchingPairBatchBoundariesAndOrder( void )
{
	int status = 1;
	const int counts[] = { 4095, 4096, 4097, 4225, 8193 };
	SupportLossResult ordered[5];
	for ( int i = 0; i < ARRAY_COUNT( counts ); ++i )
	{
		V3_CONTACTS_ENSURE( RunSupportLossCase( counts[i], false, 1, false, counts[i] == 8193, ordered + i ) );
		V3_CONTACTS_ENSURE( SupportLossResultIsSupported( ordered + i ) );
		printf( "touching-pair boundary %d: y %.9f candidates %llu touching %llu reduction %d warm %d/%d\n", counts[i],
				ordered[i].position.y, (unsigned long long)ordered[i].counters.candidateHitboxPairCount,
				(unsigned long long)ordered[i].counters.touchingPairCount, ordered[i].reductionSteps,
				ordered[i].warmAllocations, ordered[i].warmFrees );
	}
	V3_CONTACTS_ENSURE( ordered[4].warmAllocations == 0 && ordered[4].warmFrees == 0 );
	V3_CONTACTS_ENSURE( ordered[4].worldAdvanced );

	SupportLossResult reversed;
	V3_CONTACTS_ENSURE( RunSupportLossCase( 4225, true, 1, false, false, &reversed ) );
	V3_CONTACTS_ENSURE( SupportLossResultIsSupported( &reversed ) );
	V3_CONTACTS_ENSURE( reversed.contactHash == ordered[3].contactHash );
	V3_CONTACTS_ENSURE( memcmp( &reversed.position, &ordered[3].position, sizeof( reversed.position ) ) == 0 );
	V3_CONTACTS_ENSURE( memcmp( &reversed.velocity, &ordered[3].velocity, sizeof( reversed.velocity ) ) == 0 );
	status = 0;
cleanup:
	return status;
}

static int V3BlockGridTouchingPairBatchPreservesMaterialsAndIslands( void )
{
	int status = 1;
	SupportLossResult materials;
	V3_CONTACTS_ENSURE( RunSupportLossCase( 4225, false, 2, false, false, &materials ) );
	V3_CONTACTS_ENSURE( SupportLossResultIsSupported( &materials ) );
	V3_CONTACTS_ENSURE( supportLossMaterialMask == 3 );

	SupportLossResult islands;
	V3_CONTACTS_ENSURE( RunSupportLossCase( 4098, false, 1, true, false, &islands ) );
	V3_CONTACTS_ENSURE( SupportLossResultIsSupported( &islands ) );
	V3_CONTACTS_ENSURE( islands.maximumAnchorX - islands.minimumAnchorX > 80.0f );
	status = 0;
cleanup:
	return status;
}

static bool GetSupportLossContactId( const ContactFixture* fixture, b3ContactId* contactId )
{
	b3ContactData contacts[4];
	int contactCount = b3Shape_GetContactData( fixture->movingShapeId, contacts, 4 );
	for ( int i = 0; i < contactCount; ++i )
	{
		if ( SameContactShapePair( contacts[i].shapeIdA, contacts[i].shapeIdB, fixture->fixedShapeId,
								  fixture->movingShapeId ) )
		{
			*contactId = contacts[i].contactId;
			return b3Contact_IsValid( *contactId );
		}
	}
	return false;
}

static int V3BlockGridTouchingPairBatchRebuildsAfterReplacement( void )
{
	int status = 1;
	ContactFixture fixture = { .worldId = b3_nullWorldId };
	SupportLossGridOptions fixedOptions = {
		.cellCount = 4097,
		.materialCount = 1,
		.userMaterialBase = 7001,
		.fragmented = true,
	};
	SupportLossGridOptions movingOptions = fixedOptions;
	movingOptions.userMaterialBase = 8001;
	movingOptions.fragmented = false;
	V3_CONTACTS_ENSURE( CreateSupportLossFixture( fixedOptions, movingOptions, &fixture ) );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	b3ContactId firstContact = b3_nullContactId;
	V3_CONTACTS_ENSURE( GetSupportLossContactId( &fixture, &firstContact ) );
	V3_CONTACTS_ENSURE( v3World_GetBlockGridPairCounters( fixture.worldId ).touchingPairCount == 4097 );

	fixedOptions.cellCount = 4096;
	fixedOptions.reversed = true;
	v3BlockGridCookResult removed = CookSupportLossGrid( fixedOptions );
	V3_CONTACTS_ENSURE( removed.status == v3_blockGridCookOk );
	V3_CONTACTS_ENSURE( v3ReplaceBlockGridShape( fixture.fixedShapeId, removed.data, false ) == v3_blockGridReplaceOk );
	v3BlockGrid_Release( removed.data );
	removed.data = NULL;
	V3_CONTACTS_ENSURE( !b3Contact_IsValid( firstContact ) );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	b3ContactId secondContact = b3_nullContactId;
	V3_CONTACTS_ENSURE( GetSupportLossContactId( &fixture, &secondContact ) );
	V3_CONTACTS_ENSURE( !B3_ID_EQUALS( firstContact, secondContact ) );
	V3_CONTACTS_ENSURE( v3World_GetBlockGridPairCounters( fixture.worldId ).touchingPairCount == 4096 );

	fixedOptions.cellCount = 4097;
	v3BlockGridCookResult restored = CookSupportLossGrid( fixedOptions );
	V3_CONTACTS_ENSURE( restored.status == v3_blockGridCookOk );
	V3_CONTACTS_ENSURE( v3ReplaceBlockGridShape( fixture.fixedShapeId, restored.data, false ) == v3_blockGridReplaceOk );
	v3BlockGrid_Release( restored.data );
	restored.data = NULL;
	V3_CONTACTS_ENSURE( !b3Contact_IsValid( secondContact ) );
	b3World_Step( fixture.worldId, 1.0f / 60.0f, 4 );
	b3ContactId thirdContact = b3_nullContactId;
	V3_CONTACTS_ENSURE( GetSupportLossContactId( &fixture, &thirdContact ) );
	V3_CONTACTS_ENSURE( !B3_ID_EQUALS( secondContact, thirdContact ) );
	V3_CONTACTS_ENSURE( v3World_GetBlockGridPairCounters( fixture.worldId ).touchingPairCount == 4097 );
	b3Pos position = b3Body_GetPosition( fixture.movingBodyId );
	V3_CONTACTS_ENSURE( position.y >= 1.0 - B3_LINEAR_SLOP && position.y <= 1.0 + B3_LINEAR_SLOP );

	status = 0;
cleanup:
	DestroyContactFixture( &fixture );
	return status;
}

int V3BlockGridContactsTest( void )
{
	RUN_SUBTEST( V3BlockGridContactPersistsAcrossCellBoundary );
	RUN_SUBTEST( V3BlockGridSplitPatchClaimsImpulsesOnce );
	RUN_SUBTEST( V3BlockGridRecordingReplayIsExact );
	RUN_SUBTEST( V3BlockGridBadMaterialDropsOnlyItsPair );
	RUN_SUBTEST( V3BlockGridHitboxMaterialsSlideIndependently );
	RUN_SUBTEST( V3BlockGridHitboxMaterialsSlideOnGridFloor );
	RUN_SUBTEST( V3BlockGridHitEventNamesStruckHitboxMaterial );
	RUN_SUBTEST( V3BlockGridPairCountersBehavior );
	RUN_SUBTEST( V3BlockGridShuffledDefinitionCooksIdentically );
	RUN_SUBTEST( V3BlockGridEnclosedCellsProduceNoHitbox );
	RUN_SUBTEST( V3BlockGridFullCubesMergeIntoSlabs );
	RUN_SUBTEST( V3BlockGridOverlappingBoxesStayOneSource );
	RUN_SUBTEST( V3BlockGridBuriedFaceMasksAreCooked );
	RUN_SUBTEST( V3BlockGridBuriedFaceCullingKeepsFlatSupport );
	RUN_SUBTEST( V3BlockGridTouchingPairBatchKeepsSupport );
	RUN_SUBTEST( V3BlockGridTouchingPairBatchBoundariesAndOrder );
	RUN_SUBTEST( V3BlockGridTouchingPairBatchPreservesMaterialsAndIslands );
	RUN_SUBTEST( V3BlockGridTouchingPairBatchRebuildsAfterReplacement );
	return 0;
}

#undef V3_CONTACTS_ENSURE
