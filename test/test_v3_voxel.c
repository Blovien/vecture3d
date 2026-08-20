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

#define V3_DENSE_VOXEL_HULL_COUNT 32

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
	b3CompoundData* voxel = CreateHullVoxel( 1 );
	ENSURE( voxel != NULL );

	// The borrowed voxel data must outlive both shape-creation attempts.
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef staticBodyDef = b3DefaultBodyDef();
	staticBodyDef.type = b3_staticBody;
	b3BodyId staticBodyId = b3CreateBody( worldId, &staticBodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId staticShapeId = b3CreateVoxelShape( staticBodyId, &shapeDef, voxel );
	ENSURE( B3_IS_NON_NULL( staticShapeId ) );
	ENSURE( b3Shape_GetType( staticShapeId ) == b3_voxelShape );

	b3BodyDef dynamicBodyDef = b3DefaultBodyDef();
	dynamicBodyDef.type = b3_dynamicBody;
	b3BodyId dynamicBodyId = b3CreateBody( worldId, &dynamicBodyDef );
	b3ShapeId dynamicShapeId = b3CreateVoxelShape( dynamicBodyId, &shapeDef, voxel );
	ENSURE( B3_IS_NULL( dynamicShapeId ) );

	b3DestroyWorld( worldId );
	b3DestroyCompound( voxel );
	return 0;
}

static int V3VoxelMassCenter( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeBoxHull( 1.0f, 2.0f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 1.0f, 2.0f, 3.0f } );
	b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 0.0f, 2.0f } );
	b3Vec3 originVelocityBefore = b3Body_GetLocalPointVelocity( bodyId, b3Vec3_zero );
	b3Matrix3 worldInertiaBefore = b3Body_GetWorldInverseRotationalInertia( bodyId );

	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3BodySim* bodySim = b3GetBodySim( world, body );
	float minExtentBefore = bodySim->minExtent;
	b3Vec3 maxExtentBefore = bodySim->maxExtent;

	b3MassData massData = { .mass = 5.0f, .center = { 2.0f, 0.0f, 0.0f }, .inertia = b3Mat3_identity };
	massData.inertia.cx.x = 1.0f;
	massData.inertia.cy.y = 2.0f;
	massData.inertia.cz.z = 3.0f;
	b3Body_SetMassData( bodyId, massData );

	b3Vec3 originVelocityAfter = b3Body_GetLocalPointVelocity( bodyId, b3Vec3_zero );
	ENSURE_SMALL( originVelocityAfter.x - originVelocityBefore.x, 1e-5f );
	ENSURE_SMALL( originVelocityAfter.y - originVelocityBefore.y, 1e-5f );
	ENSURE_SMALL( originVelocityAfter.z - originVelocityBefore.z, 1e-5f );

	b3Matrix3 worldInertiaAfter = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE( fabsf( worldInertiaAfter.cx.x - worldInertiaBefore.cx.x ) > 1e-5f );

	bodySim = b3GetBodySim( world, body );
	ENSURE( fabsf( bodySim->minExtent - minExtentBefore ) > 1e-5f ||
			fabsf( bodySim->maxExtent.x - maxExtentBefore.x ) > 1e-5f ||
			fabsf( bodySim->maxExtent.y - maxExtentBefore.y ) > 1e-5f ||
			fabsf( bodySim->maxExtent.z - maxExtentBefore.z ) > 1e-5f );

	b3DestroyWorld( worldId );
	return 0;
}

static int V3VoxelBroadPhaseOverflow( void )
{
	b3CompoundData* voxel = CreateHullVoxel( V3_DENSE_VOXEL_HULL_COUNT );
	ENSURE( voxel != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef staticBodyDef = b3DefaultBodyDef();
	staticBodyDef.type = b3_staticBody;
	b3BodyId staticBodyId = b3CreateBody( worldId, &staticBodyDef );
	b3ShapeDef voxelShapeDef = b3DefaultShapeDef();
	b3ShapeId voxelShapeId = b3CreateVoxelShape( staticBodyId, &voxelShapeDef, voxel );
	ENSURE( B3_IS_NON_NULL( voxelShapeId ) );

	b3BodyDef dynamicBodyDef = b3DefaultBodyDef();
	dynamicBodyDef.type = b3_dynamicBody;
	b3BodyId dynamicBodyId = b3CreateBody( worldId, &dynamicBodyDef );
	b3BoxHull dynamicBox = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	b3ShapeDef dynamicShapeDef = b3DefaultShapeDef();
	dynamicShapeDef.density = 1.0f;
	b3ShapeId dynamicShapeId = b3CreateHullShape( dynamicBodyId, &dynamicShapeDef, &dynamicBox.base );
	ENSURE( B3_IS_NON_NULL( dynamicShapeId ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3Counters counters = b3World_GetCounters( worldId );
	ENSURE( counters.heapMovePairCount > 0 );

	b3ContactData contacts[V3_DENSE_VOXEL_HULL_COUNT];
	int contactCount = b3Shape_GetContactData( dynamicShapeId, contacts, V3_DENSE_VOXEL_HULL_COUNT );
	ENSURE( contactCount == V3_DENSE_VOXEL_HULL_COUNT );

	b3DestroyWorld( worldId );
	b3DestroyCompound( voxel );
	return 0;
}

static int V3VoxelSnapshot( void )
{
	b3CompoundData* voxel = CreateHullVoxel( 1 );
	ENSURE( voxel != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = b3CreateVoxelShape( bodyId, &shapeDef, voxel );
	ENSURE( B3_IS_NON_NULL( shapeId ) );

	b3Recording* recording = b3CreateRecording( 0 );
	ENSURE( recording != NULL );
	b3World_StartRecording( worldId, recording );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3World_StopRecording( worldId );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
	ENSURE( player != NULL );
	b3WorldId restoredWorldId = b3RecPlayer_GetWorldId( player );
	ENSURE( b3World_IsValid( restoredWorldId ) );
	ENSURE( b3RecPlayer_GetBodyCount( player ) == 1 );
	b3BodyId restoredBodyId = b3RecPlayer_GetBodyId( player, 0 );
	ENSURE( b3Body_GetWorld( restoredBodyId ).index1 == restoredWorldId.index1 );
	ENSURE( b3Body_GetShapeCount( restoredBodyId ) == 1 );
	b3ShapeId restoredShapes[1];
	ENSURE( b3Body_GetShapes( restoredBodyId, restoredShapes, 1 ) == 1 );
	ENSURE( b3Shape_GetType( restoredShapes[0] ) == b3_voxelShape );

	b3WorldCastOutput cast = b3Shape_RayCast( restoredShapes[0], (b3Pos){ 0.0f, 5.0f, 0.0f },
			(b3Vec3){ 0.0f, -10.0f, 0.0f } );
	ENSURE( cast.hit );

	b3DestroyPlayer( player );
	b3DestroyRecording( recording );
	b3DestroyWorld( worldId );
	b3DestroyCompound( voxel );
	return 0;
}

int V3VoxelTest( void )
{
	RUN_SUBTEST( V3VoxelPublicShape );
	RUN_SUBTEST( V3VoxelMassCenter );
	RUN_SUBTEST( V3VoxelBroadPhaseOverflow );
	RUN_SUBTEST( V3VoxelSnapshot );
	return 0;
}
