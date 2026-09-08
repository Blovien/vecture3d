// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdio.h>

#define V3_LIMITS_ENSURE( C )                                                                                                    \
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
	// The wall is one cell thick along x, so one cell per step is the documented threshold
	wallHeight = 9,
	wallDepth = 9,
	hullExtent = 2,
	tunnelSteps = 30,
};

static const float limitsTimeStep = 1.0f / 60.0f;
static const float cellSize = 1.0f;

// A projectile that starts this far in front of the wall and travels toward +x
static const double hullStartX = -6.0;
static const double wallX = 0.0;

// Cook one solid box per cell of a lattice, the way a game cooks terrain or a hull
static v3BlockGridData* CookLimitsGrid( int extentX, int extentY, int extentZ )
{
	static v3BlockGridBlock blocks[wallHeight * wallDepth * hullExtent];
	static v3BlockGridBox boxes[wallHeight * wallDepth * hullExtent];
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.5f;
	material.restitution = 0.0f;

	int count = 0;
	for ( int y = 0; y < extentY; ++y )
	{
		for ( int z = 0; z < extentZ; ++z )
		{
			for ( int x = 0; x < extentX; ++x )
			{
				boxes[count] = (v3BlockGridBox){
					.bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } },
					.materialIndex = 0,
				};
				blocks[count] = (v3BlockGridBlock){
					.x = x,
					.y = y,
					.z = z,
					.userData = (uint64_t)count + 1,
					.boxes = boxes + count,
					.boxCount = 1,
				};
				count += 1;
			}
		}
	}

	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

typedef struct TunnelObservation
{
	double finalX, advance, normalImpulse;
	float peakSpeed;
	bool crossed;
} TunnelObservation;

// Observe movement and a physical impulse as well as no far-side crossing.
static int RunTunnelScene( float hullSpeed, float maximumLinearSpeed, TunnelObservation* observation )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wallData = NULL;
	v3BlockGridData* hullData = NULL;

	wallData = CookLimitsGrid( 1, wallHeight, wallDepth );
	hullData = CookLimitsGrid( hullExtent, hullExtent, hullExtent );
	V3_LIMITS_ENSURE( wallData != NULL && hullData != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	// Discrete contacts and Speculative Contact are the only motion model this lane certifies
	worldDef.enableContinuous = false;
	worldDef.workerCount = 1;
	if ( maximumLinearSpeed > 0.0f )
	{
		worldDef.maximumLinearSpeed = maximumLinearSpeed;
	}
	worldId = b3CreateWorld( &worldDef );
	V3_LIMITS_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef wallBodyDef = b3DefaultBodyDef();
	wallBodyDef.type = b3_staticBody;
	wallBodyDef.position = (b3Pos){ wallX, -0.5 * wallHeight, -0.5 * wallDepth };
	b3BodyId wallBodyId = b3CreateBody( worldId, &wallBodyDef );
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	V3_LIMITS_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( wallBodyId, &wallShapeDef, wallData ) ) );

	b3BodyDef hullBodyDef = b3DefaultBodyDef();
	hullBodyDef.type = b3_dynamicBody;
	hullBodyDef.position = (b3Pos){ hullStartX, -0.5 * hullExtent, -0.5 * hullExtent };
	hullBodyDef.linearVelocity = (b3Vec3){ hullSpeed, 0.0f, 0.0f };
	hullBodyDef.enableSleep = false;
	b3BodyId hullBodyId = b3CreateBody( worldId, &hullBodyDef );
	b3ShapeDef hullShapeDef = b3DefaultShapeDef();
	hullShapeDef.density = 1.0f;
	V3_LIMITS_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( hullBodyId, &hullShapeDef, hullData ) ) );

	*observation = (TunnelObservation){ 0 };
	for ( int step = 0; step < tunnelSteps; ++step )
	{
		b3World_Step( worldId, limitsTimeStep, 4 );
		float speed = b3Length( b3Body_GetLinearVelocity( hullBodyId ) );
		observation->peakSpeed = b3MaxFloat( observation->peakSpeed, speed );
		double advance = b3Body_GetPosition( hullBodyId ).x - hullStartX;
		if ( advance > observation->advance )
			observation->advance = advance;
		b3ContactData contacts[4];
		int count = b3Body_GetContactData( hullBodyId, contacts, 4 );
		for ( int i = 0; i < count; ++i )
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
				for ( int k = 0; k < contacts[i].manifolds[j].pointCount; ++k )
					observation->normalImpulse += contacts[i].manifolds[j].points[k].totalNormalImpulse;
		// The hull origin is its minimum corner, so it is past a wall that spans x in [0, 1]
		if ( b3Body_GetPosition( hullBodyId ).x > wallX + (double)cellSize )
		{
			observation->crossed = true;
		}
	}
	observation->finalX = b3Body_GetPosition( hullBodyId ).x;

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( wallData );
	v3DestroyBlockGridData( hullData );
	return status;
}

// Motion-derived admission removes the old three-cell tunnelling characterization.
// This finite sweep checks entry while separate observed speeds protect the configured clamp.
static int V3WorldLimitsSpeculativeEntryUsesConfiguredCaps( void )
{
	int status = 1;
	const float oneCellPerStep = cellSize / limitsTimeStep;
	float twoCellSpeed = 0;
	for ( int cellsPerStep = 1; cellsPerStep <= 6; ++cellsPerStep )
	{
		TunnelObservation observation;
		V3_LIMITS_ENSURE( RunTunnelScene( cellsPerStep * oneCellPerStep, 0, &observation ) == 0 );
		printf( "world limits entry: %d cells per step, x %.6f, crossed %d, advance %.6f, impulse %.6f, peak speed %.6f\n",
				cellsPerStep, observation.finalX, observation.crossed, observation.advance, observation.normalImpulse,
				observation.peakSpeed );
		V3_LIMITS_ENSURE( !observation.crossed && observation.finalX < wallX );
		V3_LIMITS_ENSURE( observation.advance > 1e-4 && observation.normalImpulse > 0 );
		if ( cellsPerStep == 2 )
			twoCellSpeed = observation.peakSpeed;
	}
	TunnelObservation limited;
	V3_LIMITS_ENSURE( RunTunnelScene( 6 * oneCellPerStep, oneCellPerStep, &limited ) == 0 );
	V3_LIMITS_ENSURE( !limited.crossed && limited.finalX < wallX );
	V3_LIMITS_ENSURE( limited.advance > 1e-4 && limited.normalImpulse > 0 );
	// Before it reaches the wall, the default world permits two cells per step.
	// The explicitly limited world permits one even when six were requested.
	V3_LIMITS_ENSURE( fabsf( twoCellSpeed - 2 * oneCellPerStep ) < 1e-4f );
	V3_LIMITS_ENSURE( fabsf( limited.peakSpeed - oneCellPerStep ) < 1e-4f );
	status = 0;
cleanup:
	return status;
}

// Spin a BlockGrid body far above the clamp and read the angular speed one step later
static int RunSpinScene( float spinSpeed, float maximumAngularSpeed, float runtimeAngularSpeed, float timeStep,
						 float* observedSpeed, float* observedLimit )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* hullData = NULL;

	hullData = CookLimitsGrid( hullExtent, hullExtent, hullExtent );
	V3_LIMITS_ENSURE( hullData != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	worldDef.maximumAngularSpeed = maximumAngularSpeed;
	worldId = b3CreateWorld( &worldDef );
	V3_LIMITS_ENSURE( b3World_IsValid( worldId ) );
	V3_LIMITS_ENSURE( b3World_GetMaximumAngularSpeed( worldId ) == maximumAngularSpeed );

	if ( runtimeAngularSpeed >= 0.0f )
	{
		b3World_SetMaximumAngularSpeed( worldId, runtimeAngularSpeed );
		V3_LIMITS_ENSURE( b3World_GetMaximumAngularSpeed( worldId ) == runtimeAngularSpeed );
	}

	b3BodyDef hullBodyDef = b3DefaultBodyDef();
	hullBodyDef.type = b3_dynamicBody;
	hullBodyDef.position = (b3Pos){ -0.5 * hullExtent, -0.5 * hullExtent, -0.5 * hullExtent };
	// The lattice is a cube, so y is a principal axis and the spin carries no gyroscopic torque
	hullBodyDef.angularVelocity = (b3Vec3){ 0.0f, spinSpeed, 0.0f };
	hullBodyDef.enableSleep = false;
	b3BodyId hullBodyId = b3CreateBody( worldId, &hullBodyDef );
	b3ShapeDef hullShapeDef = b3DefaultShapeDef();
	hullShapeDef.density = 1.0f;
	V3_LIMITS_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( hullBodyId, &hullShapeDef, hullData ) ) );

	b3World_Step( worldId, timeStep, 4 );
	*observedSpeed = b3Length( b3Body_GetAngularVelocity( hullBodyId ) );
	float effective = runtimeAngularSpeed >= 0.0f ? runtimeAngularSpeed : maximumAngularSpeed;
	*observedLimit = effective > 0.0f ? effective : B3_MAX_ROTATION / timeStep;

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( hullData );
	return status;
}

static bool NearlyEqual( float observed, float expected )
{
	return fabsf( observed - expected ) <= 1e-4f * ( 1.0f + fabsf( expected ) );
}

// The zero sentinel must resolve to Box3D's own per-step clamp at every step length
static int V3WorldLimitsDefaultAngularClampMatchesBox3D( void )
{
	int status = 1;
	float sixtyHertzSpeed = 0.0f;
	float sixtyHertzLimit = 0.0f;
	float thirtyHertzSpeed = 0.0f;
	float thirtyHertzLimit = 0.0f;

	V3_LIMITS_ENSURE( RunSpinScene( 400.0f, 0.0f, -1.0f, limitsTimeStep, &sixtyHertzSpeed, &sixtyHertzLimit ) == 0 );
	V3_LIMITS_ENSURE( RunSpinScene( 400.0f, 0.0f, -1.0f, 1.0f / 30.0f, &thirtyHertzSpeed, &thirtyHertzLimit ) == 0 );

	printf( "world limits default spin: 1/60 %.6f of %.6f, 1/30 %.6f of %.6f\n", sixtyHertzSpeed, sixtyHertzLimit,
			thirtyHertzSpeed, thirtyHertzLimit );

	V3_LIMITS_ENSURE( NearlyEqual( sixtyHertzSpeed, B3_MAX_ROTATION * 60.0f ) );
	V3_LIMITS_ENSURE( NearlyEqual( thirtyHertzSpeed, B3_MAX_ROTATION * 30.0f ) );

	status = 0;
cleanup:
	return status;
}

static int V3WorldLimitsAngularSpeedClampsBlockGridBody( void )
{
	int status = 1;
	float creationSpeed = 0.0f;
	float creationLimit = 0.0f;
	float runtimeSpeed = 0.0f;
	float runtimeLimit = 0.0f;

	V3_LIMITS_ENSURE( RunSpinScene( 400.0f, 4.0f, -1.0f, limitsTimeStep, &creationSpeed, &creationLimit ) == 0 );
	V3_LIMITS_ENSURE( RunSpinScene( 400.0f, 4.0f, 1.5f, limitsTimeStep, &runtimeSpeed, &runtimeLimit ) == 0 );

	printf( "world limits clamped spin: creation %.6f of %.6f, runtime %.6f of %.6f\n", creationSpeed, creationLimit,
			runtimeSpeed, runtimeLimit );

	V3_LIMITS_ENSURE( NearlyEqual( creationSpeed, 4.0f ) );
	V3_LIMITS_ENSURE( NearlyEqual( runtimeSpeed, 1.5f ) );

	status = 0;
cleanup:
	return status;
}

// The cap is stored and exposed here; the Projectile sweep consumes it later
static int V3WorldLimitsStoreProjectileCandidateCap( void )
{
	int status = 1;
	b3WorldId defaultWorldId = b3_nullWorldId;
	b3WorldId cappedWorldId = b3_nullWorldId;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	V3_LIMITS_ENSURE( worldDef.projectileCandidateCap == 0 );
	V3_LIMITS_ENSURE( worldDef.maximumAngularSpeed == 0.0f );

	defaultWorldId = b3CreateWorld( &worldDef );
	V3_LIMITS_ENSURE( b3World_IsValid( defaultWorldId ) );
	V3_LIMITS_ENSURE( b3World_GetProjectileCandidateCap( defaultWorldId ) == 0 );
	V3_LIMITS_ENSURE( b3World_GetMaximumAngularSpeed( defaultWorldId ) == 0.0f );

	worldDef.projectileCandidateCap = 64;
	cappedWorldId = b3CreateWorld( &worldDef );
	V3_LIMITS_ENSURE( b3World_IsValid( cappedWorldId ) );
	V3_LIMITS_ENSURE( b3World_GetProjectileCandidateCap( cappedWorldId ) == 64 );

	b3World_SetProjectileCandidateCap( cappedWorldId, 8 );
	V3_LIMITS_ENSURE( b3World_GetProjectileCandidateCap( cappedWorldId ) == 8 );

	b3World_SetProjectileCandidateCap( cappedWorldId, 0 );
	V3_LIMITS_ENSURE( b3World_GetProjectileCandidateCap( cappedWorldId ) == 0 );

	status = 0;
cleanup:
	if ( b3World_IsValid( defaultWorldId ) )
	{
		b3DestroyWorld( defaultWorldId );
	}
	if ( b3World_IsValid( cappedWorldId ) )
	{
		b3DestroyWorld( cappedWorldId );
	}
	return status;
}

int V3WorldLimitsTest( void )
{
	RUN_SUBTEST( V3WorldLimitsStoreProjectileCandidateCap );
	RUN_SUBTEST( V3WorldLimitsDefaultAngularClampMatchesBox3D );
	RUN_SUBTEST( V3WorldLimitsAngularSpeedClampsBlockGridBody );
	RUN_SUBTEST( V3WorldLimitsSpeculativeEntryUsesConfiguredCaps );
	return 0;
}
