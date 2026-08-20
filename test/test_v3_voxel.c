// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include "body.h"
#include "physics_world.h"
#include "recording.h"

#include <math.h>
#include <stdio.h>

#define V3_DENSE_VOXEL_HULL_COUNT 32

#define V3_ENSURE( C )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( C ) == false )                                                                                                    \
		{                                                                                                                        \
			printf( "condition false at %s:%d: " #C "\n", __FILE__, __LINE__ );                                                 \
			status = 1;                                                                                                         \
			goto cleanup;                                                                                                      \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static b3CompoundData* CreateHullVoxel( int hullCount )
{
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3CompoundHullDef hulls[V3_DENSE_VOXEL_HULL_COUNT];
	for ( int i = 0; i < hullCount; ++i )
	{
		hulls[i] = (b3CompoundHullDef){ .hull = &box.base, .transform = b3Transform_identity, .material = material };
	}
	b3CompoundDef def = { .hulls = hulls, .hullCount = hullCount };
	return b3CreateCompound( &def );
}

static int V3VoxelPublicShape( void )
{
	int status = 1;
	b3CompoundData* voxel = NULL;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId staticBodyId = b3_nullBodyId;
	b3BodyId dynamicBodyId = b3_nullBodyId;
	b3ShapeId staticShapeId = b3_nullShapeId;
	b3ShapeId dynamicShapeId = b3_nullShapeId;
	b3WorldDef worldDef;
	b3BodyDef staticBodyDef;
	b3BodyDef dynamicBodyDef;
	b3ShapeDef shapeDef;

	voxel = CreateHullVoxel( 1 );
	V3_ENSURE( voxel != NULL );

	// The borrowed voxel data must outlive both shape-creation attempts.
	worldDef = b3DefaultWorldDef();
	worldId = b3CreateWorld( &worldDef );
	V3_ENSURE( b3World_IsValid( worldId ) );

	staticBodyDef = b3DefaultBodyDef();
	staticBodyDef.type = b3_staticBody;
	staticBodyId = b3CreateBody( worldId, &staticBodyDef );
	V3_ENSURE( B3_IS_NON_NULL( staticBodyId ) );
	shapeDef = b3DefaultShapeDef();
	staticShapeId = b3CreateVoxelShape( staticBodyId, &shapeDef, voxel );
	V3_ENSURE( B3_IS_NON_NULL( staticShapeId ) );
	V3_ENSURE( b3Shape_GetType( staticShapeId ) == b3_voxelShape );

	dynamicBodyDef = b3DefaultBodyDef();
	dynamicBodyDef.type = b3_dynamicBody;
	dynamicBodyId = b3CreateBody( worldId, &dynamicBodyDef );
	V3_ENSURE( B3_IS_NON_NULL( dynamicBodyId ) );
	dynamicShapeId = b3CreateVoxelShape( dynamicBodyId, &shapeDef, voxel );
	V3_ENSURE( B3_IS_NULL( dynamicShapeId ) );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	if ( voxel != NULL )
	{
		b3DestroyCompound( voxel );
	}
	return status;
}

static int V3VoxelMassCenter( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId bodyId = b3_nullBodyId;
	b3ShapeId shapeId = b3_nullShapeId;
	b3MeshData* mesh = NULL;
	b3World* world = NULL;
	b3Body* body = NULL;
	b3BodySim* bodySim = NULL;
	b3WorldDef worldDef;
	b3BodyDef bodyDef;
	b3ShapeDef shapeDef;
	b3Vec3 originVelocityBefore;
	b3Matrix3 worldInertiaBefore;
	float minExtentBefore;
	b3Vec3 maxExtentBefore;
	b3MassData massData;
	b3Vec3 originVelocityAfter;
	b3Matrix3 worldInertiaAfter;
	float minExtentAfter;
	b3Vec3 maxExtentAfter;

	worldDef = b3DefaultWorldDef();
	worldId = b3CreateWorld( &worldDef );
	V3_ENSURE( b3World_IsValid( worldId ) );
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyId = b3CreateBody( worldId, &bodyDef );
	V3_ENSURE( B3_IS_NON_NULL( bodyId ) );

	mesh = b3CreateBoxMesh( (b3Vec3){ 0.5f, 0.0f, 0.0f }, (b3Vec3){ 1.5f, 2.0f, 3.0f }, false );
	V3_ENSURE( mesh != NULL );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeId = b3CreateMeshShape( bodyId, &shapeDef, mesh, b3Vec3_one );
	V3_ENSURE( B3_IS_NON_NULL( shapeId ) );

	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 1.0f, 2.0f, 3.0f } );
	b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 0.0f, 2.0f } );
	originVelocityBefore = b3Body_GetLocalPointVelocity( bodyId, b3Vec3_zero );
	worldInertiaBefore = b3Body_GetWorldInverseRotationalInertia( bodyId );

	world = b3GetWorld( bodyId.world0 );
	body = b3GetBodyFullId( world, bodyId );
	bodySim = b3GetBodySim( world, body );
	minExtentBefore = bodySim->minExtent;
	maxExtentBefore = bodySim->maxExtent;

	massData = (b3MassData){ .mass = 5.0f, .center = { 1.0f, 1.0f, 0.5f }, .inertia = b3Mat3_identity };
	massData.inertia.cx.x = 1.0f;
	massData.inertia.cy.y = 2.0f;
	massData.inertia.cz.z = 3.0f;
	b3Body_SetMassData( bodyId, massData );

	originVelocityAfter = b3Body_GetLocalPointVelocity( bodyId, b3Vec3_zero );
	V3_ENSURE( fabsf( originVelocityAfter.x - originVelocityBefore.x ) < 1e-5f );
	V3_ENSURE( fabsf( originVelocityAfter.y - originVelocityBefore.y ) < 1e-5f );
	V3_ENSURE( fabsf( originVelocityAfter.z - originVelocityBefore.z ) < 1e-5f );

	worldInertiaAfter = b3Body_GetWorldInverseRotationalInertia( bodyId );
	V3_ENSURE( fabsf( worldInertiaAfter.cx.x - worldInertiaBefore.cx.x ) > 1e-5f );

	bodySim = b3GetBodySim( world, body );
	minExtentAfter = bodySim->minExtent;
	maxExtentAfter = bodySim->maxExtent;
	V3_ENSURE( fabsf( minExtentAfter - minExtentBefore ) > 1e-5f );
	V3_ENSURE( fabsf( maxExtentAfter.x - maxExtentBefore.x ) > 1e-5f );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	if ( mesh != NULL )
	{
		b3DestroyMesh( mesh );
	}
	return status;
}

static int V3VoxelBroadPhaseOverflow( void )
{
	int status = 1;
	b3CompoundData* voxel = NULL;
	b3WorldId worldId = b3_nullWorldId;
	b3BodyId staticBodyId = b3_nullBodyId;
	b3BodyId dynamicBodyId = b3_nullBodyId;
	b3ShapeId voxelShapeId = b3_nullShapeId;
	b3ShapeId dynamicShapeId = b3_nullShapeId;
	b3WorldDef worldDef;
	b3BodyDef staticBodyDef;
	b3BodyDef dynamicBodyDef;
	b3ShapeDef voxelShapeDef;
	b3BoxHull dynamicBox;
	b3ShapeDef dynamicShapeDef;
	b3Counters counters;
	b3ContactData contacts[V3_DENSE_VOXEL_HULL_COUNT];
	int contactCount = 0;

	voxel = CreateHullVoxel( V3_DENSE_VOXEL_HULL_COUNT );
	V3_ENSURE( voxel != NULL );

	worldDef = b3DefaultWorldDef();
	worldId = b3CreateWorld( &worldDef );
	V3_ENSURE( b3World_IsValid( worldId ) );
	staticBodyDef = b3DefaultBodyDef();
	staticBodyDef.type = b3_staticBody;
	staticBodyId = b3CreateBody( worldId, &staticBodyDef );
	V3_ENSURE( B3_IS_NON_NULL( staticBodyId ) );
	voxelShapeDef = b3DefaultShapeDef();
	voxelShapeId = b3CreateVoxelShape( staticBodyId, &voxelShapeDef, voxel );
	V3_ENSURE( B3_IS_NON_NULL( voxelShapeId ) );

	dynamicBodyDef = b3DefaultBodyDef();
	dynamicBodyDef.type = b3_dynamicBody;
	dynamicBodyId = b3CreateBody( worldId, &dynamicBodyDef );
	V3_ENSURE( B3_IS_NON_NULL( dynamicBodyId ) );
	dynamicBox = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	dynamicShapeDef = b3DefaultShapeDef();
	dynamicShapeDef.density = 1.0f;
	dynamicShapeId = b3CreateHullShape( dynamicBodyId, &dynamicShapeDef, &dynamicBox.base );
	V3_ENSURE( B3_IS_NON_NULL( dynamicShapeId ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	counters = b3World_GetCounters( worldId );
	V3_ENSURE( counters.heapMovePairCount > 0 );

	contactCount = b3Shape_GetContactData( dynamicShapeId, contacts, V3_DENSE_VOXEL_HULL_COUNT );
	V3_ENSURE( contactCount == V3_DENSE_VOXEL_HULL_COUNT );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	if ( voxel != NULL )
	{
		b3DestroyCompound( voxel );
	}
	return status;
}

static int V3VoxelSnapshot( void )
{
	int status = 1;
	b3CompoundData* voxel = NULL;
	b3WorldId worldId = b3_nullWorldId;
	b3WorldId restoredWorldId = b3_nullWorldId;
	b3BodyId bodyId = b3_nullBodyId;
	b3BodyId restoredBodyId = b3_nullBodyId;
	b3ShapeId shapeId = b3_nullShapeId;
	b3ShapeId restoredShapes[1] = { b3_nullShapeId };
	b3Recording* recording = NULL;
	b3RecPlayer* player = NULL;
	b3WorldDef worldDef;
	b3BodyDef bodyDef;
	b3ShapeDef shapeDef;
	b3WorldCastOutput directCast;
	b3RayResult worldCast;

	voxel = CreateHullVoxel( 1 );
	V3_ENSURE( voxel != NULL );

	worldDef = b3DefaultWorldDef();
	worldId = b3CreateWorld( &worldDef );
	V3_ENSURE( b3World_IsValid( worldId ) );
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyId = b3CreateBody( worldId, &bodyDef );
	V3_ENSURE( B3_IS_NON_NULL( bodyId ) );
	shapeDef = b3DefaultShapeDef();
	shapeId = b3CreateVoxelShape( bodyId, &shapeDef, voxel );
	V3_ENSURE( B3_IS_NON_NULL( shapeId ) );

	recording = b3CreateRecording( 0 );
	V3_ENSURE( recording != NULL );
	b3World_StartRecording( worldId, recording );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3World_StopRecording( worldId );
	V3_ENSURE( b3Recording_GetSize( recording ) > 0 );

	player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
	V3_ENSURE( player != NULL );
	restoredWorldId = b3RecPlayer_GetWorldId( player );
	V3_ENSURE( b3World_IsValid( restoredWorldId ) );
	V3_ENSURE( b3RecPlayer_GetBodyCount( player ) == 1 );
	restoredBodyId = b3RecPlayer_GetBodyId( player, 0 );
	V3_ENSURE( B3_IS_NON_NULL( restoredBodyId ) );
	V3_ENSURE( b3Body_GetWorld( restoredBodyId ).index1 == restoredWorldId.index1 );
	V3_ENSURE( b3Body_GetShapeCount( restoredBodyId ) == 1 );
	V3_ENSURE( b3Body_GetShapes( restoredBodyId, restoredShapes, 1 ) == 1 );
	V3_ENSURE( b3Shape_GetType( restoredShapes[0] ) == b3_voxelShape );

	directCast = b3Shape_RayCast( restoredShapes[0], (b3Pos){ 0.0f, 5.0f, 0.0f }, (b3Vec3){ 0.0f, -10.0f, 0.0f } );
	V3_ENSURE( directCast.hit );
	worldCast = b3World_CastRayClosest( restoredWorldId, (b3Pos){ 0.0f, 5.0f, 0.0f },
			(b3Vec3){ 0.0f, -10.0f, 0.0f }, b3DefaultQueryFilter() );
	V3_ENSURE( worldCast.hit );
	V3_ENSURE( B3_ID_EQUALS( worldCast.shapeId, restoredShapes[0] ) );

	status = 0;
cleanup:
	// The player owns restoredWorldId and every restored body and shape.
	if ( player != NULL )
	{
		b3DestroyPlayer( player );
	}
	if ( recording != NULL )
	{
		b3DestroyRecording( recording );
	}
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	if ( voxel != NULL )
	{
		b3DestroyCompound( voxel );
	}
	return status;
}

int V3VoxelTest( void )
{
	RUN_SUBTEST( V3VoxelPublicShape );
	RUN_SUBTEST( V3VoxelMassCenter );
	RUN_SUBTEST( V3VoxelBroadPhaseOverflow );
	RUN_SUBTEST( V3VoxelSnapshot );
	return 0;
}

#undef V3_ENSURE
