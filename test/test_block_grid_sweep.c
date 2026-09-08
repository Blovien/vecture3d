// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT
//
// The bounded Projectile sweep against a BlockGrid: what it stops, in what order it
// looks, and what it does when the candidate cap runs out.
//
// Tests observe the body's final pose and velocity and the counters returned by
// v3World_GetBlockGridPairCounters.

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdio.h>

#define V3_SWEEP_ENSURE( C )                                                                                                     \
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
	// A wall one cell thick along x. Sixteen by sixteen cells cook to enough Hitboxes
	// that ordering and the candidate cap both have something to do.
	sweepWallSpan = 16,
	sweepWallCells = sweepWallSpan * sweepWallSpan,

	// The cell the sphere is aimed at, in wall lattice coordinates
	sweepTargetY = 6,
	sweepTargetZ = 9,
};

static const float sweepTimeStep = 1.0f / 60.0f;
static const float sweepSphereRadius = 0.12f;

// Fast enough to clear the wall in one step if nothing stops it: 2 m of travel through
// a wall one metre thick.
static const float sweepSpeed = 120.0f;

// The wall's near face in world x. The wall body sits at the origin and its cells span
// x in [0, 1], so the sphere meets the face at x = 0.
static const double sweepWallFaceX = 0.0;
static const double sweepSphereStartX = -1.0;

// The cooker merges neighbouring cells whose boxes are identical, so a plain flat wall
// arrives as one or two Hitboxes and leaves ordering and the cap nothing to work on. A
// greebled wall varies each cell's far face, which no two neighbours share, so every cell
// keeps its own Hitbox. The near face stays at the cell boundary either way, which is what
// the analytical impact is measured against.
static float SweepCellDepth( int y, int z, bool greeble )
{
	return greeble ? 0.5f + 0.02f * (float)( ( y * 7 + z * 3 ) % 21 ) : 1.0f;
}

// Cook a wall of unit cells. keepY and keepZ select the one cell a reduced cook keeps; a
// negative keepY cooks the whole wall. boxCount reports the Hitboxes the cook produced.
static v3BlockGridData* CookSweepWall( int keepY, int keepZ, bool greeble, int* boxCount )
{
	static v3BlockGridBlock blocks[sweepWallCells];
	static v3BlockGridBox boxes[sweepWallCells];
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.5f;
	material.restitution = 0.0f;

	int count = 0;
	for ( int y = 0; y < sweepWallSpan; ++y )
	{
		for ( int z = 0; z < sweepWallSpan; ++z )
		{
			if ( keepY >= 0 && ( y != keepY || z != keepZ ) )
			{
				continue;
			}

			boxes[count] = (v3BlockGridBox){
				.bounds = { { 0.0f, 0.0f, 0.0f }, { SweepCellDepth( y, z, greeble ), 1.0f, 1.0f } },
				.materialIndex = 0,
			};
			blocks[count] = (v3BlockGridBlock){
				.x = 0,
				.y = keepY >= 0 ? 0 : y,
				.z = keepY >= 0 ? 0 : z,
				.userData = (uint64_t)count + 1,
				.boxes = boxes + count,
				.boxCount = 1,
			};
			count += 1;
		}
	}

	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	if ( boxCount != NULL )
	{
		*boxCount = result.stats.boxCount;
	}
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

typedef struct SweepScene
{
	b3BodyType wallType;
	bool bulletSphere;

	// Cook one cell instead of the whole wall, placed where the whole wall's target cell
	// sits. A single Hitbox leaves the traversal nothing to order and nothing to prune,
	// so its answer is the exhaustive one.
	bool reducedWall;

	// Zero is the world's unlimited sentinel
	int candidateCap;

	// Radians per second about z, for the rotating dynamic grid case
	float wallSpin;

	// Zero takes the defaults. A sphere wide enough to reach many cells at once gives the
	// traversal several candidates it cannot prune, which is what the cap cases need.
	float sphereRadius;
	double sphereStartX;

	// Half length of a capsule along z instead of a sphere. A long thin Projectile is
	// bounded by a wide sphere about its centroid, so many Hitboxes share the smallest
	// entry bound and the traversal has to evaluate all of them.
	float capsuleHalfLength;

	int steps;
} SweepScene;

typedef struct SweepResult
{
	double sphereX;
	b3Vec3 sphereVelocity;
	uint64_t sweepCount;
	uint64_t exhaustionCount;
	int maximumTreeHeight;
	int wallBoxCount;
	bool finite;
} SweepResult;

static int RunSweepScene( const SweepScene* scene, SweepResult* out )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wallData = NULL;

	int wallBoxCount = 0;
	wallData = scene->reducedWall ? CookSweepWall( sweepTargetY, sweepTargetZ, true, &wallBoxCount )
								  : CookSweepWall( -1, -1, true, &wallBoxCount );
	V3_SWEEP_ENSURE( wallData != NULL );
	out->wallBoxCount = wallBoxCount;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	worldDef.projectileCandidateCap = scene->candidateCap;
	worldId = b3CreateWorld( &worldDef );
	V3_SWEEP_ENSURE( b3World_IsValid( worldId ) );

	// The whole wall sits with its lattice origin at the world origin; the reduced cook
	// keeps one cell and its body is moved so that cell lands where the whole wall's
	// target cell is. Both therefore present the same box to the sphere.
	b3BodyDef wallBodyDef = b3DefaultBodyDef();
	wallBodyDef.type = scene->wallType;
	wallBodyDef.position =
		scene->reducedWall ? (b3Pos){ 0.0, (double)sweepTargetY, (double)sweepTargetZ } : (b3Pos){ 0.0, 0.0, 0.0 };
	wallBodyDef.enableSleep = false;
	b3BodyId wallBodyId = b3CreateBody( worldId, &wallBodyDef );
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( wallBodyId, &wallShapeDef, wallData ) ) );

	if ( scene->wallSpin != 0.0f )
	{
		// Both cooks must turn about the same world point, or the two walls would put the
		// target cell in different places and the comparison would be about mass, not order.
		b3MassData wallMass = {
			.mass = 1000.0f,
			.center = scene->reducedWall ? (b3Vec3){ 0.5f, 0.5f, 0.5f }
										 : (b3Vec3){ 0.5f, (float)sweepTargetY + 0.5f, (float)sweepTargetZ + 0.5f },
			.inertia = { { 4.0e5f, 0.0f, 0.0f }, { 0.0f, 4.0e5f, 0.0f }, { 0.0f, 0.0f, 4.0e5f } },
		};
		b3Body_SetMassData( wallBodyId, wallMass );
		b3Body_SetAngularVelocity( wallBodyId, (b3Vec3){ 0.0f, 0.0f, scene->wallSpin } );
	}

	float radius = scene->sphereRadius > 0.0f ? scene->sphereRadius : sweepSphereRadius;
	double startX = scene->sphereStartX != 0.0 ? scene->sphereStartX : sweepSphereStartX;

	b3BodyDef sphereBodyDef = b3DefaultBodyDef();
	sphereBodyDef.type = b3_dynamicBody;
	sphereBodyDef.position = (b3Pos){ startX, (double)sweepTargetY + 0.5, (double)sweepTargetZ + 0.5 };
	sphereBodyDef.linearVelocity = (b3Vec3){ sweepSpeed, 0.0f, 0.0f };
	sphereBodyDef.isBullet = scene->bulletSphere;
	sphereBodyDef.enableSleep = false;
	b3BodyId sphereBodyId = b3CreateBody( worldId, &sphereBodyDef );
	b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
	sphereShapeDef.density = 1.0f;
	if ( scene->capsuleHalfLength > 0.0f )
	{
		b3Capsule capsule = {
			{ 0.0f, 0.0f, -scene->capsuleHalfLength },
			{ 0.0f, 0.0f, scene->capsuleHalfLength },
			radius,
		};
		V3_SWEEP_ENSURE( B3_IS_NON_NULL( b3CreateCapsuleShape( sphereBodyId, &sphereShapeDef, &capsule ) ) );
	}
	else
	{
		b3Sphere sphere = { b3Vec3_zero, radius };
		V3_SWEEP_ENSURE( B3_IS_NON_NULL( b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere ) ) );
	}

	*out = (SweepResult){ 0 };
	int steps = scene->steps > 0 ? scene->steps : 1;
	for ( int step = 0; step < steps; ++step )
	{
		b3World_Step( worldId, sweepTimeStep, 4 );

		v3BlockGridPairCounters counters = v3World_GetBlockGridPairCounters( worldId );
		out->sweepCount += counters.projectileSweepCount;
		out->exhaustionCount += counters.capExhaustionCount;

		b3Counters worldCounters = b3World_GetCounters( worldId );
		out->maximumTreeHeight = b3MaxInt( out->maximumTreeHeight, worldCounters.treeHeight );
	}

	b3Pos position = b3Body_GetPosition( sphereBodyId );
	out->sphereX = position.x;
	out->sphereVelocity = b3Body_GetLinearVelocity( sphereBodyId );
	out->finite =
		isfinite( position.x ) && isfinite( position.y ) && isfinite( position.z ) && b3IsValidVec3( out->sphereVelocity );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( wallData );
	return status;
}

// For a stationary face at x=0, physical touch occurs at centre x=-.12,
// fraction (1-.12)/2=.44. The public result is a final pose, with one slop
// allowed past touch and the speculative distance allowed before it. This is
// not a measurement of physical TOI against a moving target.
static int V3BlockGridSweepStopsFastBodyOnEveryBodyType( void )
{
	int status = 1;
	const double expectedX = sweepWallFaceX - (double)sweepSphereRadius;
	const double physicalFraction = ( expectedX - sweepSphereStartX ) / ( (double)sweepSpeed * sweepTimeStep );
	V3_SWEEP_ENSURE( fabs( physicalFraction - 0.44 ) < 1.0e-7 );

	const b3BodyType types[3] = { b3_staticBody, b3_kinematicBody, b3_dynamicBody };
	const char* names[3] = { "static", "kinematic", "dynamic" };

	for ( int t = 0; t < 3; ++t )
	{
		for ( int bullet = 0; bullet < 2; ++bullet )
		{
			SweepScene scene = {
				.wallType = types[t],
				.bulletSphere = bullet != 0,
				.steps = 1,
			};
			SweepResult result;
			V3_SWEEP_ENSURE( RunSweepScene( &scene, &result ) == 0 );

			printf( "sweep thin target: wall %s, bullet %d, x %.6f, expected %.6f, sweeps %llu\n", names[t], bullet,
					result.sphereX, expectedX, (unsigned long long)result.sweepCount );

			V3_SWEEP_ENSURE( result.finite );

			// Never reports a miss for a sweep that crosses a Hitbox
			V3_SWEEP_ENSURE( result.sphereX < sweepWallFaceX + 1.0 );

			V3_SWEEP_ENSURE( result.sphereX <= expectedX + (double)B3_LINEAR_SLOP + 1.0e-6 );
			V3_SWEEP_ENSURE( result.sphereX >= expectedX - (double)B3_SPECULATIVE_DISTANCE - 1.0e-6 );

			// The sweep stops the body; it does not brake it
			V3_SWEEP_ENSURE( fabsf( result.sphereVelocity.x - sweepSpeed ) <= 1.0f );

			// Every sweep is counted, and an unlimited cap exhausts nothing
			V3_SWEEP_ENSURE( result.sweepCount >= 1 );
			V3_SWEEP_ENSURE( result.exhaustionCount == 0 );
		}
	}

	status = 0;
cleanup:
	return status;
}

// Ordering and pruning only reorder work. A wall of 256 cells and a wall of the one cell
// the sphere is aimed at must stop it in the same place: the 255 cells the traversal
// orders behind the target, or prunes without touching, change nothing. The rotating
// dynamic wall is here because the sweep's grid-side terms guard the general sweep
// contract and are zero under the current finalize ordering, which advances the wall
// body before the sweep runs: the case checks the ordered result against the exhaustive
// one on a wall swept at its end pose.
static int V3BlockGridSweepOrderedTraversalMatchesExhaustive( void )
{
	int status = 1;
	const double tolerance = 2.0 * (double)B3_LINEAR_SLOP;

	const b3BodyType types[4] = { b3_staticBody, b3_kinematicBody, b3_dynamicBody, b3_dynamicBody };
	const float spins[4] = { 0.0f, 0.0f, 0.0f, 6.0f };
	const char* names[4] = { "static", "kinematic", "dynamic", "dynamic spinning" };

	for ( int t = 0; t < 4; ++t )
	{
		SweepScene ordered = {
			.wallType = types[t],
			.bulletSphere = true,
			.wallSpin = spins[t],
			.steps = 1,
		};
		SweepScene exhaustive = ordered;
		exhaustive.reducedWall = true;

		SweepResult orderedResult;
		SweepResult exhaustiveResult;
		V3_SWEEP_ENSURE( RunSweepScene( &ordered, &orderedResult ) == 0 );
		V3_SWEEP_ENSURE( RunSweepScene( &exhaustive, &exhaustiveResult ) == 0 );

		printf( "sweep ordering: wall %s, ordered x %.6f, exhaustive x %.6f\n", names[t], orderedResult.sphereX,
				exhaustiveResult.sphereX );

		V3_SWEEP_ENSURE( orderedResult.finite && exhaustiveResult.finite );
		V3_SWEEP_ENSURE( fabs( orderedResult.sphereX - exhaustiveResult.sphereX ) <= tolerance );

		// Neither crosses the wall
		V3_SWEEP_ENSURE( orderedResult.sphereX < sweepWallFaceX + 1.0 );
		V3_SWEEP_ENSURE( exhaustiveResult.sphereX < sweepWallFaceX + 1.0 );
	}

	status = 0;
cleanup:
	return status;
}

// The number of Hitbox evaluations one sweep runs is not published, so measure it: raise
// the cap until the sweep stops reporting an exhaustion. The smallest cap that exhausts
// nothing is exactly the number of evaluations the ordered traversal needs.
static int MeasureSweepEvaluationCount( const SweepScene* base, int* evaluationCount )
{
	int status = 1;
	*evaluationCount = 0;

	for ( int cap = 1; cap <= sweepWallCells; ++cap )
	{
		SweepScene scene = *base;
		scene.candidateCap = cap;
		SweepResult result;
		V3_SWEEP_ENSURE( RunSweepScene( &scene, &result ) == 0 );

		// The cap is a bound on evaluations, so a capped sweep never runs more of them
		// than the cap allows; an exhaustion is the traversal saying it stopped early.
		if ( result.exhaustionCount == 0 )
		{
			*evaluationCount = cap;
			break;
		}
	}

	V3_SWEEP_ENSURE( *evaluationCount > 0 );
	status = 0;
cleanup:
	return status;
}

static int V3BlockGridSweepCandidateCapHoldsTheProjectile( void )
{
	int status = 1;
	int evaluationCount = 0;

	// A four metre capsule is bounded by a sphere of radius two about its centroid, so
	// every cell within two metres of its path shares the smallest entry bound and gets
	// evaluated. A small sphere needs a single evaluation, which would make the
	// exact-cap and one-less-cap cases the same run.
	SweepScene unlimited = {
		.wallType = b3_staticBody,
		.bulletSphere = true,
		.sphereRadius = 0.1f,
		.capsuleHalfLength = 2.0f,
		.steps = 1,
	};

	V3_SWEEP_ENSURE( MeasureSweepEvaluationCount( &unlimited, &evaluationCount ) == 0 );
	printf( "sweep cap: the ordered traversal needs %d evaluations\n", evaluationCount );
	V3_SWEEP_ENSURE( evaluationCount >= 2 );
	SweepResult unlimitedResult;
	V3_SWEEP_ENSURE( RunSweepScene( &unlimited, &unlimitedResult ) == 0 );

	SweepScene exactCap = unlimited;
	exactCap.candidateCap = evaluationCount;
	SweepResult exactResult;
	V3_SWEEP_ENSURE( RunSweepScene( &exactCap, &exactResult ) == 0 );

	SweepScene oneLess = unlimited;
	oneLess.candidateCap = evaluationCount - 1;
	SweepResult oneLessResult;
	V3_SWEEP_ENSURE( RunSweepScene( &oneLess, &oneLessResult ) == 0 );

	printf( "sweep cap: unlimited x %.6f, cap %d x %.6f exhausted %llu, cap %d x %.6f exhausted %llu\n", unlimitedResult.sphereX,
			evaluationCount, exactResult.sphereX, (unsigned long long)exactResult.exhaustionCount, evaluationCount - 1,
			oneLessResult.sphereX, (unsigned long long)oneLessResult.exhaustionCount );

	// At the exact cap the traversal finishes, so the answer is the unlimited one
	V3_SWEEP_ENSURE( exactResult.exhaustionCount == 0 );
	V3_SWEEP_ENSURE( fabs( exactResult.sphereX - unlimitedResult.sphereX ) <= 1.0e-9 );

	// One below it the traversal runs out with a candidate still reachable, which is
	// counted rather than missed
	V3_SWEEP_ENSURE( oneLessResult.exhaustionCount >= 1 );
	V3_SWEEP_ENSURE( oneLessResult.sweepCount >= 1 );

	// The hold is at or before the impact the full traversal found, never past it, and it
	// never puts the body through the wall
	V3_SWEEP_ENSURE( oneLessResult.sphereX <= unlimitedResult.sphereX + 1.0e-6 );
	V3_SWEEP_ENSURE( oneLessResult.sphereX < sweepWallFaceX + 1.0 );

	// The hold is at the start pose: the body did not advance at all
	V3_SWEEP_ENSURE( fabs( oneLessResult.sphereX - sweepSphereStartX ) <= 1.0e-9 );

	// A hold keeps the Projectile's velocity: it is a refusal to advance, not an impact
	V3_SWEEP_ENSURE( oneLessResult.sphereVelocity.x == sweepSpeed );
	V3_SWEEP_ENSURE( oneLessResult.finite );

	// A cap of one is the smallest bound there is and still holds rather than tunnels
	SweepScene single = unlimited;
	single.candidateCap = 1;
	SweepResult singleResult;
	V3_SWEEP_ENSURE( RunSweepScene( &single, &singleResult ) == 0 );
	printf( "sweep cap: cap 1 x %.6f exhausted %llu\n", singleResult.sphereX, (unsigned long long)singleResult.exhaustionCount );
	V3_SWEEP_ENSURE( singleResult.exhaustionCount >= 1 );
	V3_SWEEP_ENSURE( singleResult.sphereX < sweepWallFaceX + 1.0 );
	V3_SWEEP_ENSURE( fabs( singleResult.sphereX - sweepSphereStartX ) <= 1.0e-9 );
	V3_SWEEP_ENSURE( singleResult.sphereVelocity.x == sweepSpeed );

	status = 0;
cleanup:
	return status;
}

// A Projectile that starts inside a Hitbox has no entry face on it, so the general solve
// reports fraction zero and the sweep falls back to a point proxy. What must not happen
// is that the overlapped Hitbox swallows the rest of the sweep: a second wall further
// along still has to stop the Projectile.
static int V3BlockGridSweepHandlesInitialOverlap( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wallData = NULL;

	// The second wall's near face. Two metres of travel would carry the sphere past it.
	const double secondWallX = 1.5;

	wallData = CookSweepWall( -1, -1, true, NULL );
	V3_SWEEP_ENSURE( wallData != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	worldId = b3CreateWorld( &worldDef );
	V3_SWEEP_ENSURE( b3World_IsValid( worldId ) );

	// One cooked payload, two shapes: the grid is reference counted and immutable
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	b3BodyDef wallBodyDef = b3DefaultBodyDef();
	wallBodyDef.type = b3_staticBody;
	b3BodyId nearWallId = b3CreateBody( worldId, &wallBodyDef );
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( nearWallId, &wallShapeDef, wallData ) ) );

	wallBodyDef.position = (b3Pos){ secondWallX, 0.0, 0.0 };
	b3BodyId farWallId = b3CreateBody( worldId, &wallBodyDef );
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( farWallId, &wallShapeDef, wallData ) ) );

	// Start at the centre of a cell of the near wall, so the sweep begins overlapping it
	b3BodyDef sphereBodyDef = b3DefaultBodyDef();
	sphereBodyDef.type = b3_dynamicBody;
	sphereBodyDef.position = (b3Pos){ 0.5, (double)sweepTargetY + 0.5, (double)sweepTargetZ + 0.5 };
	sphereBodyDef.linearVelocity = (b3Vec3){ sweepSpeed, 0.0f, 0.0f };
	sphereBodyDef.isBullet = true;
	sphereBodyDef.enableSleep = false;
	b3BodyId sphereBodyId = b3CreateBody( worldId, &sphereBodyDef );
	b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
	sphereShapeDef.density = 1.0f;
	b3Sphere sphere = { b3Vec3_zero, sweepSphereRadius };
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere ) ) );

	b3World_Step( worldId, sweepTimeStep, 4 );

	b3Pos position = b3Body_GetPosition( sphereBodyId );
	v3BlockGridPairCounters counters = v3World_GetBlockGridPairCounters( worldId );
	printf( "sweep initial overlap: x %.6f, sweeps %llu, exhausted %llu\n", position.x,
			(unsigned long long)counters.projectileSweepCount, (unsigned long long)counters.capExhaustionCount );

	V3_SWEEP_ENSURE( isfinite( position.x ) );

	// Both walls are swept against, the overlapped one included
	V3_SWEEP_ENSURE( counters.projectileSweepCount >= 2 );
	V3_SWEEP_ENSURE( counters.capExhaustionCount == 0 );

	// The far wall stops it. Without that the step would end past secondWallX + 1.
	V3_SWEEP_ENSURE( position.x < secondWallX );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( wallData );
	return status;
}

// The counters are per step, like every other BlockGrid pair counter: a step that sweeps
// reports its sweeps, and the step after the sphere has stopped reports none.
static int V3BlockGridSweepCountersAreRebuiltEachStep( void )
{
	int status = 1;
	b3WorldId worldId = b3_nullWorldId;
	v3BlockGridData* wallData = NULL;

	wallData = CookSweepWall( -1, -1, true, NULL );
	V3_SWEEP_ENSURE( wallData != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	worldId = b3CreateWorld( &worldDef );
	V3_SWEEP_ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef wallBodyDef = b3DefaultBodyDef();
	wallBodyDef.type = b3_staticBody;
	b3BodyId wallBodyId = b3CreateBody( worldId, &wallBodyDef );
	b3ShapeDef wallShapeDef = b3DefaultShapeDef();
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( v3CreateBlockGridShape( wallBodyId, &wallShapeDef, wallData ) ) );

	// Nothing is moving yet, so the first step sweeps nothing
	b3World_Step( worldId, sweepTimeStep, 4 );
	v3BlockGridPairCounters idle = v3World_GetBlockGridPairCounters( worldId );
	V3_SWEEP_ENSURE( idle.projectileSweepCount == 0 );
	V3_SWEEP_ENSURE( idle.capExhaustionCount == 0 );

	b3BodyDef sphereBodyDef = b3DefaultBodyDef();
	sphereBodyDef.type = b3_dynamicBody;
	sphereBodyDef.position = (b3Pos){ sweepSphereStartX, (double)sweepTargetY + 0.5, (double)sweepTargetZ + 0.5 };
	sphereBodyDef.linearVelocity = (b3Vec3){ sweepSpeed, 0.0f, 0.0f };
	sphereBodyDef.isBullet = true;
	sphereBodyDef.enableSleep = false;
	b3BodyId sphereBodyId = b3CreateBody( worldId, &sphereBodyDef );
	b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
	sphereShapeDef.density = 1.0f;
	b3Sphere sphere = { b3Vec3_zero, sweepSphereRadius };
	V3_SWEEP_ENSURE( B3_IS_NON_NULL( b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere ) ) );

	b3World_Step( worldId, sweepTimeStep, 4 );
	v3BlockGridPairCounters swept = v3World_GetBlockGridPairCounters( worldId );
	printf( "sweep counters: idle %llu, moving %llu\n", (unsigned long long)idle.projectileSweepCount,
			(unsigned long long)swept.projectileSweepCount );
	V3_SWEEP_ENSURE( swept.projectileSweepCount >= 1 );
	V3_SWEEP_ENSURE( swept.capExhaustionCount == 0 );

	// Stop the sphere dead. The next step has no fast body, so the counters go back to zero
	// rather than carrying the previous step's tally.
	b3Body_SetLinearVelocity( sphereBodyId, b3Vec3_zero );
	b3World_Step( worldId, sweepTimeStep, 4 );
	v3BlockGridPairCounters settled = v3World_GetBlockGridPairCounters( worldId );
	V3_SWEEP_ENSURE( settled.projectileSweepCount == 0 );

	status = 0;
cleanup:
	if ( b3World_IsValid( worldId ) )
	{
		b3DestroyWorld( worldId );
	}
	v3DestroyBlockGridData( wallData );
	return status;
}

int V3BlockGridSweepTest( void )
{
	RUN_SUBTEST( V3BlockGridSweepStopsFastBodyOnEveryBodyType );
	RUN_SUBTEST( V3BlockGridSweepOrderedTraversalMatchesExhaustive );
	RUN_SUBTEST( V3BlockGridSweepCandidateCapHoldsTheProjectile );
	RUN_SUBTEST( V3BlockGridSweepHandlesInitialOverlap );
	RUN_SUBTEST( V3BlockGridSweepCountersAreRebuiltEachStep );
	return 0;
}
