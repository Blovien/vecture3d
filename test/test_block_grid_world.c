// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"

#include <math.h>

#define CHECK( condition )                                                                                                       \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( condition ) )                                                                                                    \
		{                                                                                                                        \
			printf( "condition false at %s:%d: %s\n", __FILE__, __LINE__, #condition );                                          \
			status = 1;                                                                                                          \
			goto cleanup;                                                                                                        \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static b3WorldId CreateWorld( int workers )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.gravity = b3Vec3_zero;
	def.enableSleep = false;
	def.workerCount = workers;
	def.maximumLinearSpeed = 1000.0f;
	return b3CreateWorld( &def );
}

static b3ShapeId AttachCells( b3BodyId body, const int cells[][3], int count )
{
	v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, 1, 1 } } };
	v3BlockGridBlock blocks[5];
	if ( count < 1 || count > ARRAY_COUNT( blocks ) )
	{
		return b3_nullShapeId;
	}
	for ( int i = 0; i < count; ++i )
	{
		blocks[i] = (v3BlockGridBlock){
			.x = cells[i][0],
			.y = cells[i][1],
			.z = cells[i][2],
			.boxes = &box,
			.boxCount = 1,
		};
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.0f;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult cooked = v3CookBlockGrid( &def );
	if ( cooked.status != v3_blockGridCookOk )
	{
		return b3_nullShapeId;
	}
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.0f;
	b3ShapeId shape = v3CreateBlockGridShape( body, &shapeDef, cooked.data );
	v3DestroyBlockGridData( cooked.data );
	return shape;
}

static const int unitCell[1][3] = { { 0, 0, 0 } };

typedef struct HullContact
{
	b3WorldId world;
	b3BodyId gridBody, hullBody;
	b3ShapeId grid, hull;
} HullContact;

// The two unit cubes overlap by 0.001 along x, giving a four point face contact.
static bool CreateHullContact( HullContact* fixture, int workers )
{
	fixture->world = CreateWorld( workers );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	fixture->gridBody = b3CreateBody( fixture->world, &bodyDef );
	fixture->grid = AttachCells( fixture->gridBody, unitCell, 1 );
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 1.499, 0.5, 0.5 };
	fixture->hullBody = b3CreateBody( fixture->world, &bodyDef );
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.0f;
	fixture->hull = b3CreateHullShape( fixture->hullBody, &shapeDef, &box.base );
	return b3Shape_IsValid( fixture->grid ) && b3Shape_IsValid( fixture->hull );
}

static bool SamePair( b3ShapeId a, b3ShapeId b, b3ShapeId expectedA, b3ShapeId expectedB )
{
	return ( B3_ID_EQUALS( a, expectedA ) && B3_ID_EQUALS( b, expectedB ) ) ||
		   ( B3_ID_EQUALS( b, expectedA ) && B3_ID_EQUALS( a, expectedB ) );
}

static int ContactPoints( b3ShapeId shape, b3ShapeId other )
{
	b3ContactData contacts[2];
	int count = b3Shape_GetContactData( shape, contacts, ARRAY_COUNT( contacts ) );
	if ( count != 1 || !SamePair( contacts[0].shapeIdA, contacts[0].shapeIdB, shape, other ) )
	{
		return 0;
	}
	int points = 0;
	for ( int i = 0; i < contacts[0].manifoldCount; ++i )
	{
		points += contacts[0].manifolds[i].pointCount;
	}
	return points;
}

static void DriveGrid( b3BodyId body )
{
	b3Body_SetLinearVelocity( body, (b3Vec3){ 120, 0, 0 } );
	b3Body_SetAngularVelocity( body, (b3Vec3){ 0, 0, 15 } );
}

// Preserve the old face and corner contact observations, without counting private work.
static int GridPairContacts( void )
{
	int status = 0;
	b3WorldId world = b3_nullWorldId;
	const int corner[2][3] = { { 1, 0, 0 }, { 0, 1, 0 } };
	for ( int twoFaces = 0; twoFaces < 2; ++twoFaces )
	{
		world = CreateWorld( 1 );
		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		b3BodyId moving = b3CreateBody( world, &def );
		b3ShapeId movingShape = AttachCells( moving, unitCell, 1 );
		def.type = twoFaces ? b3_kinematicBody : b3_dynamicBody;
		def.position.x = twoFaces ? 0 : 1;
		b3BodyId obstacle = b3CreateBody( world, &def );
		b3ShapeId obstacleShape = AttachCells( obstacle, twoFaces ? corner : unitCell, twoFaces ? 2 : 1 );
		CHECK( b3Shape_IsValid( movingShape ) && b3Shape_IsValid( obstacleShape ) );
		b3World_Step( world, 1.0f / 60.0f, 1 );
		int points = ContactPoints( movingShape, obstacleShape );
		CHECK( twoFaces ? points >= 2 : points == 4 );
		b3Body_SetLinearVelocity( moving, (b3Vec3){ 120, twoFaces ? 120 : 0, 0 } );
		b3Body_SetAngularVelocity( moving, (b3Vec3){ 0, 0, twoFaces ? 0 : 15 } );
		b3World_Step( world, 1.0f / 60.0f, 1 );
		CHECK( ContactPoints( movingShape, obstacleShape ) > 0 );
		b3DestroyWorld( world );
		world = b3_nullWorldId;
	}
cleanup:
	if ( b3World_IsValid( world ) )
	{
		b3DestroyWorld( world );
	}
	return status;
}

static int CurrentContactAllowsKinematicMotion( void )
{
	int status = 0;
	HullContact fixture = { 0 };
	CHECK( CreateHullContact( &fixture, 1 ) );
	b3Body_SetType( fixture.gridBody, b3_kinematicBody );
	b3Body_SetType( fixture.hullBody, b3_dynamicBody );
	b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
	CHECK( ContactPoints( fixture.grid, fixture.hull ) > 0 );
	b3Pos before = b3Body_GetPosition( fixture.gridBody );
	DriveGrid( fixture.gridBody );
	b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
	CHECK( b3Body_GetPosition( fixture.gridBody ).x > before.x + 0.05 );
cleanup:
	if ( b3World_IsValid( fixture.world ) )
	{
		b3DestroyWorld( fixture.world );
	}
	return status;
}

static bool CountPreSolve( b3ShapeId a, b3ShapeId b, b3Pos point, b3Vec3 normal, void* context )
{
	(void)a;
	(void)b;
	(void)point;
	(void)normal;
	*(int*)context += 1;
	return true;
}

static int ContactEventsAreNotRepeated( void )
{
	int status = 0;
	int preSolveCount = 0;
	HullContact fixture = { 0 };
	CHECK( CreateHullContact( &fixture, 1 ) );
	b3Shape_EnableContactEvents( fixture.grid, true );
	b3Shape_EnableHitEvents( fixture.grid, true );
	b3Shape_EnablePreSolveEvents( fixture.grid, true );
	b3World_SetHitEventThreshold( fixture.world, 0.0f );
	b3World_SetPreSolveCallback( fixture.world, CountPreSolve, &preSolveCount );
	b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
	b3ContactEvents first = b3World_GetContactEvents( fixture.world );
	CHECK( ContactPoints( fixture.grid, fixture.hull ) == 4 );
	CHECK( first.beginCount == 1 && first.hitCount == 0 && first.endCount == 0 );
	CHECK( SamePair( first.beginEvents[0].shapeIdA, first.beginEvents[0].shapeIdB, fixture.grid, fixture.hull ) );
	CHECK( preSolveCount > 0 );
	int admittedPreSolveCount = preSolveCount;
	DriveGrid( fixture.gridBody );
	b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
	b3ContactEvents driven = b3World_GetContactEvents( fixture.world );
	CHECK( driven.beginCount == 0 && driven.hitCount == 1 && driven.endCount == 0 );
	CHECK( SamePair( driven.hitEvents[0].shapeIdA, driven.hitEvents[0].shapeIdB, fixture.grid, fixture.hull ) );
	CHECK( preSolveCount == admittedPreSolveCount );
cleanup:
	if ( b3World_IsValid( fixture.world ) )
	{
		b3DestroyWorld( fixture.world );
	}
	return status;
}

static bool SameVector( b3Vec3 a, b3Vec3 b )
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

static int DrivenContactIsDeterministic( void )
{
	int status = 0;
	HullContact fixture = { 0 };
	b3WorldTransform reference = { 0 };
	b3Vec3 linear = b3Vec3_zero, angular = b3Vec3_zero;
	const int workers[] = { 1, 1, 4 };
	for ( int run = 0; run < ARRAY_COUNT( workers ); ++run )
	{
		CHECK( CreateHullContact( &fixture, workers[run] ) );
		CHECK( b3World_GetWorkerCount( fixture.world ) == workers[run] );
		b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
		CHECK( ContactPoints( fixture.grid, fixture.hull ) == 4 );
		DriveGrid( fixture.gridBody );
		b3World_Step( fixture.world, 1.0f / 60.0f, 1 );
		CHECK( ContactPoints( fixture.grid, fixture.hull ) == 4 );
		b3WorldTransform pose = b3Body_GetTransform( fixture.gridBody );
		b3Vec3 velocity = b3Body_GetLinearVelocity( fixture.gridBody );
		b3Vec3 spin = b3Body_GetAngularVelocity( fixture.gridBody );
		if ( run == 0 )
		{
			reference = pose;
			linear = velocity;
			angular = spin;
		}
		else
		{
			CHECK( pose.p.x == reference.p.x && pose.p.y == reference.p.y && pose.p.z == reference.p.z );
			CHECK( SameVector( pose.q.v, reference.q.v ) && pose.q.s == reference.q.s );
			CHECK( SameVector( velocity, linear ) && SameVector( spin, angular ) );
		}
		b3DestroyWorld( fixture.world );
		fixture.world = b3_nullWorldId;
	}
cleanup:
	if ( b3World_IsValid( fixture.world ) )
	{
		b3DestroyWorld( fixture.world );
	}
	return status;
}

static int MovingContainerCarriesSphere( void )
{
	int status = 0;
	b3WorldId world = CreateWorld( 1 );
	b3World_SetGravity( world, (b3Vec3){ 0, -10, 0 } );
	const int cells[5][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 0, 1, 0 }, { 2, 1, 0 } };
	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_kinematicBody;
	def.linearVelocity = (b3Vec3){ 0, 1, 0 };
	b3BodyId container = b3CreateBody( world, &def );
	b3ShapeId grid = AttachCells( container, cells, ARRAY_COUNT( cells ) );
	CHECK( b3Shape_IsValid( grid ) );
	def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = (b3Pos){ 1.5, 1.25, 0.5 };
	b3BodyId body = b3CreateBody( world, &def );
	b3Sphere sphere = { .radius = 0.25f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shape = b3CreateSphereShape( body, &shapeDef, &sphere );
	CHECK( b3Shape_IsValid( shape ) );
	for ( int step = 0; step < 120; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
	}
	b3Pos position = b3Body_GetPosition( body );
	b3Vec3 velocity = b3Body_GetLinearVelocity( body );
	// Two seconds at 1 m/s raises the floor from y=1 to y=3. The radius adds 0.25.
	CHECK( fabs( b3Body_GetPosition( container ).y - 2.0 ) < 1.0e-5 );
	CHECK( fabs( position.y - 3.25 ) < B3_LINEAR_SLOP );
	CHECK( fabs( position.x - 1.5 ) < B3_LINEAR_SLOP && fabs( position.z - 0.5 ) < B3_LINEAR_SLOP );
	CHECK( fabsf( velocity.y - 1.0f ) < 0.01f );
	CHECK( ContactPoints( shape, grid ) > 0 );
cleanup:
	b3DestroyWorld( world );
	return status;
}

static int ContinuousFlagControlsCrossing( void )
{
	int status = 0;
	b3WorldId world = b3_nullWorldId;
	for ( int enabled = 0; enabled < 2; ++enabled )
	{
		world = CreateWorld( 1 );
		b3World_EnableContinuous( world, enabled != 0 );
		b3BodyDef def = b3DefaultBodyDef();
		b3BodyId wall = b3CreateBody( world, &def );
		CHECK( b3Shape_IsValid( AttachCells( wall, unitCell, 1 ) ) );
		def.type = b3_dynamicBody;
		def.position = (b3Pos){ -2, 0.5, 0.5 };
		def.linearVelocity = (b3Vec3){ 240, 0, 0 };
		b3BodyId body = b3CreateBody( world, &def );
		b3Sphere sphere = { .radius = 0.25f };
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		CHECK( b3Shape_IsValid( b3CreateSphereShape( body, &shapeDef, &sphere ) ) );
		b3World_Step( world, 1.0f / 60.0f, 4 );
		double x = b3Body_GetPosition( body ).x;
		// Free motion travels four metres. The swept sphere meets the wall at x=-radius.
		CHECK( enabled ? fabs( x + 0.25 ) < 2.0 * B3_LINEAR_SLOP : fabs( x - 2.0 ) < 1.0e-5 );
		b3DestroyWorld( world );
		world = b3_nullWorldId;
	}
cleanup:
	if ( b3World_IsValid( world ) )
	{
		b3DestroyWorld( world );
	}
	return status;
}

// The recording starts with attached geometry, so queries must use the restored snapshot.
static int SnapshotRestoresQueryableGrid( void )
{
	int status = 0;
	b3WorldId world = CreateWorld( 1 );
	b3Recording* recording = NULL;
	b3RecPlayer* player = NULL;
	b3BodyDef def = b3DefaultBodyDef();
	b3BodyId body = b3CreateBody( world, &def );
	CHECK( b3Shape_IsValid( AttachCells( body, unitCell, 1 ) ) );
	recording = b3CreateRecording( 0 );
	CHECK( recording != NULL );
	b3World_StartRecording( world, recording );
	b3World_Step( world, 1.0f / 60.0f, 4 );
	b3World_StopRecording( world );
	CHECK( b3Recording_GetSize( recording ) > 0 );
	b3DestroyWorld( world );
	world = b3_nullWorldId;

	player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
	CHECK( player != NULL );
	CHECK( b3RecPlayer_GetBodyCount( player ) == 1 );
	b3BodyId restoredBody = b3RecPlayer_GetBodyId( player, 0 );
	b3ShapeId shape;
	CHECK( b3Body_GetShapeCount( restoredBody ) == 1 );
	CHECK( b3Body_GetShapes( restoredBody, &shape, 1 ) == 1 );
	CHECK( b3Shape_GetType( shape ) == v3_blockGridShape );
	b3Pos origin = { 0.5, 5, 0.5 };
	b3Vec3 translation = { 0, -10, 0 };
	b3WorldCastOutput direct = b3Shape_RayCast( shape, origin, translation );
	b3RayResult hit = b3World_CastRayClosest( b3RecPlayer_GetWorldId( player ), origin, translation, b3DefaultQueryFilter() );
	// The top face is y=1, four metres into a ten metre ray.
	CHECK( direct.hit && fabsf( direct.fraction - 0.4f ) < 1.0e-6f );
	CHECK( hit.hit && fabsf( hit.fraction - 0.4f ) < 1.0e-6f );
	CHECK( B3_ID_EQUALS( hit.shapeId, shape ) );
	CHECK( b3RecPlayer_GetFrameCount( player ) == 1 );
	CHECK( b3RecPlayer_StepFrame( player ) );
	CHECK( !b3RecPlayer_StepFrame( player ) );
	CHECK( b3RecPlayer_IsAtEnd( player ) && !b3RecPlayer_HasDiverged( player ) );
cleanup:
	if ( player != NULL )
	{
		b3DestroyPlayer( player );
	}
	if ( recording != NULL )
	{
		b3DestroyRecording( recording );
	}
	if ( b3World_IsValid( world ) )
	{
		b3DestroyWorld( world );
	}
	return status;
}

int V3BlockGridWorldTest( void )
{
	RUN_SUBTEST( GridPairContacts );
	RUN_SUBTEST( CurrentContactAllowsKinematicMotion );
	RUN_SUBTEST( ContactEventsAreNotRepeated );
	RUN_SUBTEST( DrivenContactIsDeterministic );
	RUN_SUBTEST( MovingContainerCarriesSphere );
	RUN_SUBTEST( ContinuousFlagControlsCrossing );
	RUN_SUBTEST( SnapshotRestoresQueryableGrid );
	return 0;
}

#undef CHECK
