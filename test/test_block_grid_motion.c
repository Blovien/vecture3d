// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// 600 fixed steps are ten seconds, or 300 game ticks at 30 TPS with two steps/tick.
// Each fixed step has four solver substeps, not four collision refreshes.
static const float motionStep = 1.0f / 60.0f;
static const double motionTolerance = 1.0e-5;
static const uint64_t hashOffset = UINT64_C( 14695981039346656037 );

#define MOTION_CHECK( C )                                                                                                        \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( C ) )                                                                                                            \
		{                                                                                                                        \
			printf( "motion rep=%d step=%d line=%d: %s\n", rep, step, __LINE__, #C );                                            \
			goto cleanup;                                                                                                        \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static void HashBytes( uint64_t* hash, const void* data, size_t size )
{
	const unsigned char* bytes = data;
	for ( size_t i = 0; i < size; ++i )
	{
		*hash ^= bytes[i];
		*hash *= UINT64_C( 1099511628211 );
	}
}
#define HASH( H, V ) HashBytes( H, &( V ), sizeof( V ) )
#define HASH_VECTOR( H, V )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		HASH( H, ( V ).x );                                                                                                      \
		HASH( H, ( V ).y );                                                                                                      \
		HASH( H, ( V ).z );                                                                                                      \
	}                                                                                                                            \
	while ( false )
#define HASH_TRANSFORM( H, T )                                                                                                   \
	do                                                                                                                           \
	{                                                                                                                            \
		HASH_VECTOR( H, ( T ).p );                                                                                               \
		HASH_VECTOR( H, ( T ).q.v );                                                                                             \
		HASH( H, ( T ).q.s );                                                                                                    \
	}                                                                                                                            \
	while ( false )
// Only world identity is omitted. Keep slot and generation, including destroyed IDs
// in end events. Never hash padding, pointers, or reorder an event array.
#define HASH_ID( H, ID )                                                                                                         \
	do                                                                                                                           \
	{                                                                                                                            \
		HASH( H, ( ID ).index1 );                                                                                                \
		HASH( H, ( ID ).generation );                                                                                            \
	}                                                                                                                            \
	while ( false )

typedef struct MotionTrace
{
	uint64_t state, events, work;
} MotionTrace;
typedef struct EventCounts
{
	int begin, end, hit, move;
	int blockBegin, blockEnd, blockHit;
	bool blockEventsIncomplete;
} EventCounts;

static void HashBody( uint64_t* hash, b3BodyId body )
{
	HASH_ID( hash, body );
	if ( B3_IS_NULL( body ) )
	{
		return;
	}
	b3WorldTransform transform = b3Body_GetTransform( body );
	b3Vec3 velocity = b3Body_GetLinearVelocity( body ), angular = b3Body_GetAngularVelocity( body );
	bool awake = b3Body_IsAwake( body );
	HASH_TRANSFORM( hash, transform );
	HASH_VECTOR( hash, velocity );
	HASH_VECTOR( hash, angular );
	HASH( hash, awake );
	b3ShapeId shape = b3_nullShapeId;
	int count = b3Body_GetShapes( body, &shape, 1 );
	HASH( hash, count );
	HASH_ID( hash, shape );
}

// Logical payload stays in published side and array order, including cached ends.
static void HashBlockEvent( uint64_t* hash, const v3BlockContactEvent* event )
{
	const v3BlockContactSide* sides[2] = { &event->sideA, &event->sideB };
	for ( int i = 0; i < 2; ++i )
	{
		const v3BlockContactSide* side = sides[i];
		HASH_ID( hash, side->bodyId );
		HASH_ID( hash, side->shapeId );
		HASH( hash, side->isBlockGrid );
		HASH( hash, side->cellX );
		HASH( hash, side->cellY );
		HASH( hash, side->cellZ );
		HASH( hash, side->subHitboxIndex );
		HASH( hash, side->materialIndex );
		HASH( hash, side->userMaterialId );
		HASH( hash, side->userData );
	}
	HASH_VECTOR( hash, event->point );
	HASH_VECTOR( hash, event->normal );
	HASH( hash, event->normalImpulse );
	HASH( hash, event->approachSpeed );
}

static MotionTrace StepTrace( b3WorldId world, EventCounts* totals )
{
	MotionTrace trace = { hashOffset, hashOffset, hashOffset };
	uint64_t* hash = &trace.events;
	b3ContactEvents contacts = b3World_GetContactEvents( world );
	HASH( hash, contacts.beginCount );
	HASH( hash, contacts.endCount );
	HASH( hash, contacts.hitCount );
	totals->begin += contacts.beginCount;
	totals->end += contacts.endCount;
	totals->hit += contacts.hitCount;
	for ( int i = 0; i < contacts.beginCount; ++i )
	{
		b3ContactBeginTouchEvent e = contacts.beginEvents[i];
		HASH_ID( hash, e.shapeIdA );
		HASH_ID( hash, e.shapeIdB );
		HASH_ID( hash, e.contactId );
	}
	for ( int i = 0; i < contacts.endCount; ++i )
	{
		b3ContactEndTouchEvent e = contacts.endEvents[i];
		HASH_ID( hash, e.shapeIdA );
		HASH_ID( hash, e.shapeIdB );
		HASH_ID( hash, e.contactId );
	}
	for ( int i = 0; i < contacts.hitCount; ++i )
	{
		b3ContactHitEvent e = contacts.hitEvents[i];
		HASH_ID( hash, e.shapeIdA );
		HASH_ID( hash, e.shapeIdB );
		HASH_ID( hash, e.contactId );
		HASH_VECTOR( hash, e.point );
		HASH_VECTOR( hash, e.normal );
		HASH( hash, e.approachSpeed );
		HASH( hash, e.userMaterialIdA );
		HASH( hash, e.userMaterialIdB );
	}
	v3BlockContactEvents blocks = v3World_GetBlockContactEvents( world );
	HASH( hash, blocks.beginCount );
	HASH( hash, blocks.hitCount );
	HASH( hash, blocks.endCount );
	HASH( hash, blocks.droppedBeginCount );
	HASH( hash, blocks.droppedHitCount );
	HASH( hash, blocks.droppedEndCount );
	HASH( hash, blocks.capacity );
	HASH( hash, blocks.truncated );
	totals->blockBegin += blocks.beginCount;
	totals->blockHit += blocks.hitCount;
	totals->blockEnd += blocks.endCount;
	totals->blockEventsIncomplete |=
		blocks.truncated || blocks.droppedBeginCount > 0 || blocks.droppedHitCount > 0 || blocks.droppedEndCount > 0;
	for ( int i = 0; i < blocks.beginCount; ++i )
	{
		HashBlockEvent( hash, blocks.beginEvents + i );
	}
	for ( int i = 0; i < blocks.hitCount; ++i )
	{
		HashBlockEvent( hash, blocks.hitEvents + i );
	}
	for ( int i = 0; i < blocks.endCount; ++i )
	{
		HashBlockEvent( hash, blocks.endEvents + i );
	}
	b3BodyEvents bodies = b3World_GetBodyEvents( world );
	HASH( hash, bodies.moveCount );
	totals->move += bodies.moveCount;
	for ( int i = 0; i < bodies.moveCount; ++i )
	{
		b3BodyMoveEvent e = bodies.moveEvents[i];
		HASH_ID( hash, e.bodyId );
		HASH_TRANSFORM( hash, e.transform );
		HASH( hash, e.fellAsleep );
	}
	v3BlockGridPairCounters work = v3World_GetBlockGridPairCounters( world );
	hash = &trace.work;
	HASH( hash, work.candidateHitboxPairCount );
	HASH( hash, work.touchingPairCount );
	HASH( hash, work.contactCount );
	HASH( hash, work.projectileSweepCount );
	HASH( hash, work.capExhaustionCount );
	HASH( hash, work.replacementPublishedCount );
	HASH( hash, work.scratchPeakBytes );
	HASH( hash, work.contactReductionCount );
	return trace;
}

static bool SameTrace( MotionTrace a, MotionTrace b )
{
	if ( a.state == b.state && a.events == b.events && a.work == b.work )
	{
		return true;
	}
	printf( "trace state=%016llx/%016llx events=%016llx/%016llx work=%016llx/%016llx\n", (unsigned long long)a.state,
			(unsigned long long)b.state, (unsigned long long)a.events, (unsigned long long)b.events, (unsigned long long)a.work,
			(unsigned long long)b.work );
	return false;
}

// These dimensions describe the input unit cells. No engine bounds feed the oracle.
static v3BlockGridData* CookMotionGrid( int nx, int ny, int nz, bool shell, int expectedCells )
{
	v3BlockGridBlock* blocks = calloc( (size_t)nx * ny * nz, sizeof( *blocks ) );
	if ( blocks == NULL )
	{
		return NULL;
	}
	v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, 1, 1 } } };
	int count = 0;
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
			{
				if ( shell && x > 0 && x < nx - 1 && y > 0 && y < ny - 1 && z > 0 && z < nz - 1 )
				{
					continue;
				}
				blocks[count] = (v3BlockGridBlock){ .x = x - nx / 2,
													.y = y - ny / 2,
													.z = z - nz / 2,
													.userData = (uint64_t)count + 1,
													.boxes = &box,
													.boxCount = 1 };
				++count;
			}
	if ( count != expectedCells )
	{
		free( blocks );
		return NULL;
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = .8f;
	material.restitution = 0;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	free( blocks );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static b3WorldId MotionWorld( bool sleep )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.workerCount = 1;
	def.enableSleep = sleep;
	def.gravity = (b3Vec3){ 0, -10, 0 };
	def.enableContinuous = true;
	def.maximumLinearSpeed = 120;
	def.maximumAngularSpeed = .25f;
	def.projectileCandidateCap = 0;
	return b3CreateWorld( &def );
}

static b3BodyId MotionGridBody( b3WorldId world, v3BlockGridData* grid, b3BodyType type, b3Pos position )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type = type;
	def.position = position;
	b3BodyId body = b3CreateBody( world, &def );
	b3ShapeDef shape = b3DefaultShapeDef();
	shape.density = 1000;
	shape.enableContactEvents = true;
	shape.enableHitEvents = true;
	if ( B3_IS_NULL( v3CreateBlockGridShape( body, &shape, grid ) ) )
	{
		b3DestroyBody( body );
		return b3_nullBodyId;
	}
	return body;
}

// Count actual points for this particular pair. Upward normals distinguish support
// from a side contact. No old fixed manifold or point count is assumed.
static int PairPoints( b3BodyId body, b3BodyId other, bool support )
{
	b3ContactData contacts[32];
	if ( b3Body_GetContactCapacity( body ) > 32 )
	{
		return -1;
	}
	int count = b3Body_GetContactData( body, contacts, 32 ), points = 0;
	for ( int i = 0; i < count; ++i )
	{
		b3BodyId a = b3Shape_GetBody( contacts[i].shapeIdA ), b = b3Shape_GetBody( contacts[i].shapeIdB );
		bool bodyIsA = B3_ID_EQUALS( a, body );
		if ( !( ( bodyIsA && B3_ID_EQUALS( b, other ) ) || ( B3_ID_EQUALS( b, body ) && B3_ID_EQUALS( a, other ) ) ) )
		{
			continue;
		}
		for ( int j = 0; j < contacts[i].manifoldCount; ++j )
		{
			const b3Manifold* m = &contacts[i].manifolds[j];
			float upward = bodyIsA ? -m->normal.y : m->normal.y;
			if ( !support || upward > .9f )
			{
				points += m->pointCount;
			}
		}
	}
	return points;
}

static double BottomHeight( b3BodyId body, b3Vec3 low, b3Vec3 high )
{
	b3WorldTransform transform = b3Body_GetTransform( body );
	double bottom = INFINITY;
	for ( int x = 0; x < 2; ++x )
		for ( int y = 0; y < 2; ++y )
			for ( int z = 0; z < 2; ++z )
			{
				b3Pos p =
					b3TransformWorldPoint( transform, (b3Vec3){ x ? high.x : low.x, y ? high.y : low.y, z ? high.z : low.z } );
				if ( p.y < bottom )
				{
					bottom = p.y;
				}
			}
	return bottom;
}

static int CarrierMotion( bool teleport )
{
	int status = 1, rep = 0, step = -1;
	b3WorldId world = b3_nullWorldId;
	v3BlockGridData* deck = CookMotionGrid( 12, 1, 8, false, 96 );
	v3BlockGridData* cube = CookMotionGrid( 1, 1, 1, false, 1 );
	v3BlockGridData* terrain = CookMotionGrid( 64, 1, 16, false, 1024 );
	MotionTrace reference[540];
	MOTION_CHECK( deck && cube && terrain );
	for ( rep = 0; rep < 3; ++rep )
	{
		world = MotionWorld( true );
		MOTION_CHECK( b3World_IsValid( world ) );
		b3BodyId floor = MotionGridBody( world, terrain, b3_staticBody, (b3Pos){ 0, -1, 0 } );
		b3BodyId carrier = MotionGridBody( world, deck, b3_kinematicBody, (b3Pos){ 0, 1, 0 } );
		b3BodyId cargo[3];
		for ( int i = 0; i < 3; ++i )
		{
			cargo[i] = MotionGridBody( world, cube, b3_dynamicBody, (b3Pos){ 3 * i - 3, 2, 0 } );
		}
		b3BodyId isolated = MotionGridBody( world, cube, b3_dynamicBody, (b3Pos){ 20, 0, 0 } );
		MOTION_CHECK( B3_IS_NON_NULL( floor ) && B3_IS_NON_NULL( carrier ) && B3_IS_NON_NULL( isolated ) );
		for ( int i = 0; i < 3; ++i )
		{
			MOTION_CHECK( B3_IS_NON_NULL( cargo[i] ) );
		}
		EventCounts events = { 0 };
		uint64_t candidates = 0;
		b3Pos isolatedRest = b3Pos_zero;
		int airborne[3] = { 0 };
		for ( step = 0; step < 540; ++step )
		{
			if ( step == 360 )
			{
				isolatedRest = b3Body_GetPosition( isolated );
				MOTION_CHECK( !b3Body_IsAwake( isolated ) );
				MOTION_CHECK( PairPoints( isolated, floor, true ) > 0 );
				for ( int i = 0; i < 3; ++i )
				{
					MOTION_CHECK( !b3Body_IsAwake( cargo[i] ) );
					MOTION_CHECK( PairPoints( cargo[i], carrier, true ) > 0 );
					MOTION_CHECK( fabs( BottomHeight( cargo[i], b3Vec3_zero, (b3Vec3){ 1, 1, 1 } ) - 2 ) <=
								  B3_LINEAR_SLOP + motionTolerance );
				}
				if ( teleport )
				{
					b3Body_SetTransform( carrier, (b3Pos){ 0, 1, 20 }, b3Quat_identity );
					b3Body_SetAwake( carrier, true );
				}
				else
				{
					b3Body_SetTargetTransform( carrier, (b3WorldTransform){ { 0, 1.01, 0 }, b3Quat_identity }, motionStep, true );
				}
			}
			b3World_Step( world, motionStep, 4 );
			MotionTrace trace = StepTrace( world, &events );
			candidates += v3World_GetBlockGridPairCounters( world ).candidateHitboxPairCount;
			HashBody( &trace.state, floor );
			HashBody( &trace.state, carrier );
			for ( int i = 0; i < 3; ++i )
			{
				HashBody( &trace.state, cargo[i] );
				int support = PairPoints( cargo[i], carrier, true );
				HASH( &trace.state, support );
				if ( step == 360 )
				{
					MOTION_CHECK( b3Body_IsAwake( cargo[i] ) );
				}
				if ( step >= 360 )
				{
					double bottom = BottomHeight( cargo[i], b3Vec3_zero, (b3Vec3){ 1, 1, 1 } );
					// The lift imparts upward velocity before the deck stops. Resting
					// height and support are checked again after that transient.
					if ( teleport )
					{
						MOTION_CHECK( support == 0 );
						if ( bottom > .1 && bottom < 1.9 )
						{
							MOTION_CHECK( b3Body_GetLinearVelocity( cargo[i] ).y < 0 );
							++airborne[i];
						}
					}
					else if ( step == 360 )
					{
						MOTION_CHECK( support > 0 );
						MOTION_CHECK( fabs( bottom - 2.01 ) <= B3_LINEAR_SLOP + motionTolerance );
					}
				}
			}
			HashBody( &trace.state, isolated );
			if ( step >= 360 )
			{
				MOTION_CHECK( !b3Body_IsAwake( isolated ) );
				MOTION_CHECK( b3Length( b3SubPos( b3Body_GetPosition( isolated ), isolatedRest ) ) == 0 );
			}
			if ( step == 360 && !teleport )
			{
				b3Body_SetLinearVelocity( carrier, b3Vec3_zero );
			}
			if ( rep == 0 )
			{
				reference[step] = trace;
			}
			else
			{
				MOTION_CHECK( SameTrace( reference[step], trace ) );
			}
		}
		MOTION_CHECK( events.begin >= 4 && events.move > 0 && candidates > 0 );
		MOTION_CHECK( events.blockBegin > 0 && !events.blockEventsIncomplete );
		if ( teleport )
		{
			MOTION_CHECK( events.blockHit > 0 );
		}
		for ( int i = 0; i < 3; ++i )
		{
			double expected = teleport ? 0 : 2.01;
			MOTION_CHECK( fabs( BottomHeight( cargo[i], b3Vec3_zero, (b3Vec3){ 1, 1, 1 } ) - expected ) <=
						  B3_LINEAR_SLOP + motionTolerance );
			MOTION_CHECK( PairPoints( cargo[i], teleport ? floor : carrier, true ) > 0 );
			if ( teleport )
			{
				MOTION_CHECK( airborne[i] > 10 && events.end >= 3 );
			}
		}
		b3Pos carrierEnd = b3Body_GetPosition( carrier );
		MOTION_CHECK( fabs( carrierEnd.y - ( teleport ? 1 : 1.01 ) ) <= motionTolerance );
		MOTION_CHECK( fabs( carrierEnd.z - ( teleport ? 20 : 0 ) ) <= motionTolerance );
		printf( "carrier teleport=%d rep=%d events begin/end/hit/move=%d/%d/%d/%d block begin/end/hit=%d/%d/%d\n", teleport, rep,
				events.begin, events.end, events.hit, events.move, events.blockBegin, events.blockEnd, events.blockHit );
		b3DestroyWorld( world );
		world = b3_nullWorldId;
	}
	status = 0;
cleanup:
	if ( b3World_IsValid( world ) )
	{
		b3DestroyWorld( world );
	}
	v3DestroyBlockGridData( deck );
	v3DestroyBlockGridData( cube );
	v3DestroyBlockGridData( terrain );
	return status;
}

static int CarrierConnectedWake( void )
{
	return CarrierMotion( false );
}
static int CarrierTeleportAndExplicitWake( void )
{
	return CarrierMotion( true );
}

typedef struct MotionShot
{
	b3BodyId body;
	int hitStep, gravitySteps, longestFlight, floorStep;
	bool fell, descendingToFloor;
	double expectedY, expectedVy, previousY;
} MotionShot;

// A clear descending segment, away from the hull's x=-18 face and the y=1 floor.
// Once a segment starts, prediction evolves independently instead of rebasing each
// step on the observed pose. Four semi-implicit Euler substeps give g*h*h*5/8.
static int CheckShot( MotionShot* shot, b3BodyId hull, int step )
{
	b3Pos p = b3Body_GetPosition( shot->body );
	b3Vec3 v = b3Body_GetLinearVelocity( shot->body );
	if ( !isfinite( p.x ) || !isfinite( p.y ) || !isfinite( p.z ) || !b3IsValidVec3( v ) )
	{
		return 1;
	}
	if ( shot->hitStep < 0 && PairPoints( shot->body, hull, false ) > 0 )
	{
		shot->hitStep = step;
	}
	if ( shot->hitStep >= 0 && step > shot->hitStep && p.y < 6 && v.y < 0 )
	{
		shot->fell = true;
	}
	// Slot reuse bounds the time to reach the floor. Until then, the observed
	// sphere centre must descend toward the y=1 floor and its .12 radius.
	if ( shot->descendingToFloor )
	{
		const double floorCenterY = 1.0 + .12;
		if ( p.y <= floorCenterY + B3_LINEAR_SLOP + motionTolerance )
		{
			shot->floorStep = step;
			shot->descendingToFloor = false;
		}
		else
		{
			if ( p.y >= shot->previousY )
			{
				printf( "floor progress step=%d y=%.9f previous=%.9f\n", step, p.y, shot->previousY );
				return 1;
			}
			shot->previousY = p.y;
		}
	}
	if ( shot->gravitySteps > 0 )
	{
		double dt = (double)motionStep / 4;
		for ( int i = 0; i < 4; ++i )
		{
			shot->expectedVy -= 10 * dt;
			shot->expectedY += shot->expectedVy * dt;
		}
		if ( shot->expectedY > 1.5 )
		{
			if ( fabs( p.y - shot->expectedY ) > 1.0e-4 || fabs( v.y - shot->expectedVy ) > 1.0e-4 )
			{
				printf( "gravity step=%d segment=%d y=%.9f expected=%.9f vy=%.9f expectedVy=%.9f\n", step, shot->gravitySteps,
						p.y, shot->expectedY, (double)v.y, shot->expectedVy );
				return 1;
			}
			++shot->gravitySteps;
			if ( shot->gravitySteps > shot->longestFlight )
			{
				shot->longestFlight = shot->gravitySteps;
			}
		}
		else
		{
			shot->gravitySteps = 0;
			shot->descendingToFloor = true;
			shot->previousY = p.y;
		}
	}
	if ( shot->gravitySteps == 0 && shot->hitStep >= 0 && p.x < -18.2 && p.y > 1.6 && v.x < 0 && v.y < 0 )
	{
		shot->expectedY = p.y;
		shot->expectedVy = v.y;
		shot->gravitySteps = 1;
	}
	return 0;
}

static bool ShotCompleted( const MotionShot* shot )
{
	return shot->hitStep >= 0 && shot->fell && shot->longestFlight >= 8 && shot->floorStep >= 0;
}

typedef struct VolleyWindow
{
	uint64_t candidates, sweeps;
	int peakBytes, peakContacts, peakArena;
} VolleyWindow;

static int LandedHullSustainedVolley( void )
{
	int status = 1, rep = 0, step = -1;
	b3WorldId world = b3_nullWorldId;
	const int bytesBefore = b3GetByteCount();
	// 36*12*20 - 34*10*18 = 2520 occupied unit cells, each at 1000 kg/m^3.
	v3BlockGridData* shell = CookMotionGrid( 36, 12, 20, true, 2520 );
	v3BlockGridData* terrain = CookMotionGrid( 96, 1, 64, false, 6144 );
	MotionTrace reference[1020];
	MOTION_CHECK( shell && terrain );
	for ( rep = 0; rep < 3; ++rep )
	{
		world = MotionWorld( false );
		MOTION_CHECK( b3World_IsValid( world ) );
		b3BodyId floor = MotionGridBody( world, terrain, b3_staticBody, b3Pos_zero );
		b3BodyId hull = MotionGridBody( world, shell, b3_dynamicBody, (b3Pos){ 0, 7.01, 0 } );
		MOTION_CHECK( B3_IS_NON_NULL( floor ) && B3_IS_NON_NULL( hull ) );
		MOTION_CHECK( fabs( b3Body_GetMass( hull ) - 2520000.0 ) <= .5 );
		MotionShot shots[8] = { 0 };
		EventCounts events = { 0 };
		VolleyWindow windows[4] = { 0 };
		b3Pos rest = b3Pos_zero;
		int emitted = 0, completed = 0;
		double minGap = INFINITY, maxDrift = 0;
		uint64_t sweeps = 0, candidates = 0;
		for ( step = 0; step < 1020; ++step )
		{
			int fireStep = step - 360;
			if ( step == 360 )
			{
				rest = b3Body_GetPosition( hull );
				MOTION_CHECK( PairPoints( hull, floor, true ) > 0 );
				MOTION_CHECK( fabs( BottomHeight( hull, (b3Vec3){ -18, -6, -10 }, (b3Vec3){ 18, 6, 10 } ) - 1 ) <=
							  B3_LINEAR_SLOP + motionTolerance );
			}
			if ( fireStep >= 0 && fireStep < 600 && fireStep % 15 == 0 )
			{
				MotionShot* shot = &shots[emitted % 8];
				if ( B3_IS_NON_NULL( shot->body ) )
				{
					MOTION_CHECK( ShotCompleted( shot ) );
					++completed;
					b3DestroyBody( shot->body );
				}
				b3BodyDef body = b3DefaultBodyDef();
				body.type = b3_dynamicBody;
				body.isBullet = true;
				body.position = (b3Pos){ -20, 7, -7.5 + 2 * ( emitted % 8 ) };
				body.linearVelocity = (b3Vec3){ 100, 0, 0 };
				b3BodyId id = b3CreateBody( world, &body );
				b3ShapeDef shape = b3DefaultShapeDef();
				shape.density = 1000;
				shape.baseMaterial.friction = 0;
				shape.baseMaterial.restitution = .5f;
				shape.enableContactEvents = true;
				shape.enableHitEvents = true;
				b3Sphere sphere = { b3Vec3_zero, .12f };
				MOTION_CHECK( B3_IS_NON_NULL( b3CreateSphereShape( id, &shape, &sphere ) ) );
				// Sphere volume times the same explicit density gives about 7.24 kg.
				const double expectedMass = 1000.0 * ( 4.0 / 3.0 ) * 3.141592653589793 * .12 * .12 * .12;
				MOTION_CHECK( fabs( b3Body_GetMass( id ) - expectedMass ) < 1.0e-4 );
				*shot = (MotionShot){ .body = id, .hitStep = -1, .floorStep = -1 };
				++emitted;
			}
			b3World_Step( world, motionStep, 4 );
			MotionTrace trace = StepTrace( world, &events );
			HashBody( &trace.state, floor );
			HashBody( &trace.state, hull );
			v3BlockGridPairCounters work = v3World_GetBlockGridPairCounters( world );
			MOTION_CHECK( work.capExhaustionCount == 0 );
			sweeps += work.projectileSweepCount;
			candidates += work.candidateHitboxPairCount;
			if ( step >= 360 )
			{
				int support = PairPoints( hull, floor, true );
				HASH( &trace.state, support );
				double gap = BottomHeight( hull, (b3Vec3){ -18, -6, -10 }, (b3Vec3){ 18, 6, 10 } ) - 1;
				double drift = b3Length( b3SubPos( b3Body_GetPosition( hull ), rest ) );
				if ( gap < minGap )
				{
					minGap = gap;
				}
				if ( drift > maxDrift )
				{
					maxDrift = drift;
				}
				MOTION_CHECK( support > 0 );
				MOTION_CHECK( fabs( gap ) <= B3_LINEAR_SLOP + motionTolerance );
				MOTION_CHECK( drift <= B3_LINEAR_SLOP + motionTolerance );
				for ( int j = 0; j < 8; ++j )
				{
					HashBody( &trace.state, shots[j].body );
					if ( B3_IS_NON_NULL( shots[j].body ) )
					{
						MOTION_CHECK( CheckShot( &shots[j], hull, step ) == 0 );
					}
				}
			}
			// Equal launch phases and eight live slots in each mature 120-step window.
			// Retained bytes are a memory bound, not an allocation count or time bound.
			if ( fireStep >= 120 && fireStep < 600 )
			{
				VolleyWindow* window = &windows[( fireStep - 120 ) / 120];
				b3Counters counters = b3World_GetCounters( world );
				int bytes = b3GetByteCount();
				MOTION_CHECK( counters.bodyCount == 10 && counters.shapeCount == 10 );
				window->candidates += work.candidateHitboxPairCount;
				window->sweeps += work.projectileSweepCount;
				window->peakBytes = b3MaxInt( window->peakBytes, bytes );
				window->peakContacts = b3MaxInt( window->peakContacts, counters.contactCount );
				window->peakArena = b3MaxInt( window->peakArena, counters.arenaCapacity );
			}
			if ( rep == 0 )
			{
				reference[step] = trace;
			}
			else
			{
				MOTION_CHECK( SameTrace( reference[step], trace ) );
			}
		}
		for ( int i = 0; i < 8; ++i )
		{
			MOTION_CHECK( ShotCompleted( &shots[i] ) );
			++completed;
		}
		MOTION_CHECK( emitted == 40 && completed == 40 );
		MOTION_CHECK( events.begin >= 40 && events.end >= 32 && events.hit >= 40 && events.move > 600 );
		MOTION_CHECK( events.blockBegin > 0 && events.blockHit > 0 && !events.blockEventsIncomplete );
		MOTION_CHECK( sweeps >= 40 && candidates > 0 );
		for ( int i = 0; i < 4; ++i )
		{
			printf( "volley rep=%d window=%d-%d candidates=%llu sweeps=%llu peakBytes=%d contacts=%d arena=%d\n", rep,
					120 * ( i + 1 ), 120 * ( i + 2 ) - 1, (unsigned long long)windows[i].candidates,
					(unsigned long long)windows[i].sweeps, windows[i].peakBytes, windows[i].peakContacts, windows[i].peakArena );
			MOTION_CHECK( windows[i].candidates > 0 && windows[i].sweeps >= 8 );
			MOTION_CHECK( windows[i].peakBytes <= windows[0].peakBytes );
			MOTION_CHECK( windows[i].peakContacts <= windows[0].peakContacts );
			MOTION_CHECK( windows[i].peakArena <= windows[0].peakArena );
		}
		printf( "volley rep=%d completed=%d gap=%.9f drift=%.9f events=%d/%d/%d/%d block begin/end/hit=%d/%d/%d\n", rep,
				completed, minGap, maxDrift, events.begin, events.end, events.hit, events.move, events.blockBegin,
				events.blockEnd, events.blockHit );
		b3DestroyWorld( world );
		world = b3_nullWorldId;
	}
	status = 0;
cleanup:
	if ( b3World_IsValid( world ) )
	{
		b3DestroyWorld( world );
	}
	v3DestroyBlockGridData( shell );
	v3DestroyBlockGridData( terrain );
	if ( b3GetByteCount() != bytesBefore )
	{
		printf( "motion leaked Box3D bytes\n" );
		status = 1;
	}
	return status;
}

int V3BlockGridMotionTest( void )
{
	RUN_SUBTEST( CarrierConnectedWake );
	RUN_SUBTEST( CarrierTeleportAndExplicitWake );
	RUN_SUBTEST( LandedHullSustainedVolley );
	return 0;
}
